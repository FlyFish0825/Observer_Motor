/**
 * @file board_adc.c
 * @brief 主循环软件触发电压与温度采样，并发布完整测量快照。
 * ADC1 Rank1-3依次采母线/U/W（6.5周期），Rank4采MCU温度（247.5周期）；ADC2采V。
 * 规则组非PWM同步采样；电压输出为RC滤波后的端电压。
 */
#include "board_adc.h"

#include "adc.h"
#include "arm_math.h"
#include <stddef.h>

/* ADC参考电压与PCB分压参数：100kΩ / 5.1kΩ。 */
/* ADC 模拟参考电压，决定原始计数对应的引脚电压。 */
#define BOARD_ADC_REFERENCE_VOLTAGE    3.3f
/* 12位 ADC 的满量程计数数目，采用2^12而非最大码值4095进行换算。 */
#define BOARD_ADC_FULL_SCALE_COUNTS    4096.0f
/* PCB 电阻分压网络的输入侧/下臂比例，用于恢复被测端电压。 */
#define BOARD_ADC_VOLTAGE_DIVIDER_GAIN ((100.0f + 5.1f) / 5.1f)
/* 将 ADC 计数直接换算为分压器输入侧电压的组合比例系数。 */
#define BOARD_ADC_COUNT_TO_VOLTAGE                                      \
  (BOARD_ADC_REFERENCE_VOLTAGE / BOARD_ADC_FULL_SCALE_COUNTS *          \
   BOARD_ADC_VOLTAGE_DIVIDER_GAIN)

/* 已完成发布的最近一组母线和三相端电压测量快照。 */
static BoardAdcMeasurements_t board_adc_measurements = {0};
/* seqlock 序号：奇数表示主循环正在写入，偶数表示快照可读取。 */
static volatile uint32_t board_adc_sequence = 0U;
/* 快照有效标志，避免系统启动时读到尚未采样的零值。 */
static volatile uint8_t board_adc_valid = 0U;

/**
 * @brief 读取一轮规则组结果，全部成功后才发布整组数据。
 * 局部next暂存本轮结果，途中失败时共享测量值仍保持上一轮完整结果。
 * 每次轮询最多等待10ms；只允许主循环调用，避免阻塞电流控制中断。
 */
HAL_StatusTypeDef BoardAdc_Update(void) {
  /* 暂存本轮完整结果；只有所有 ADC 转换成功才会发布到共享对象。 */
  BoardAdcMeasurements_t next = {0};
  /* ADC1规则组原始计数：母线、U相、W相和MCU内部温度。 */
  uint16_t adc1_values[4];

  /* ADC1规则组间断模式：每次触发只转换一个Rank，读完再推进到下一Rank。
   * 温度通道使用长采样时间，仍按逐Rank轮询确保每个结果与通道对应。
   */
  for (uint32_t rank = 0U; rank < 4U; rank++) {
    /* rank 是当前要触发并读取的 ADC1 规则组序号。 */
    if (HAL_ADC_Start(&hadc1) != HAL_OK) {
      (void)HAL_ADCEx_RegularStop(&hadc1);
      return HAL_ERROR;
    }
    if (HAL_ADC_PollForConversion(&hadc1, 10U) != HAL_OK) {
      /* 仅停止规则组并复位序列位置，不能中断电流注入组的硬件触发。 */
      (void)HAL_ADCEx_RegularStop(&hadc1);
      return HAL_TIMEOUT;
    }
    adc1_values[rank] = (uint16_t)HAL_ADC_GetValue(&hadc1);
  }

  if (HAL_ADC_Start(&hadc2) != HAL_OK) {
    return HAL_ERROR;
  }
  if (HAL_ADC_PollForConversion(&hadc2, 10U) != HAL_OK) {
    return HAL_TIMEOUT;
  }

  next.vbus_raw = adc1_values[0];
  next.phase_u_raw = adc1_values[1];
  next.phase_w_raw = adc1_values[2];
  next.temperature_raw = adc1_values[3];
  next.phase_v_raw = (uint16_t)HAL_ADC_GetValue(&hadc2);

  /* ADC计数先换算引脚电压，再乘分压倍率，恢复分压器输入侧电压(V)。 */
  next.vbus_voltage = (float)next.vbus_raw * BOARD_ADC_COUNT_TO_VOLTAGE;
  next.phase_u_voltage =
      (float)next.phase_u_raw * BOARD_ADC_COUNT_TO_VOLTAGE;
  next.phase_v_voltage =
      (float)next.phase_v_raw * BOARD_ADC_COUNT_TO_VOLTAGE;
  next.phase_w_voltage =
      (float)next.phase_w_raw * BOARD_ADC_COUNT_TO_VOLTAGE;
  next.temperature_c = (float)__HAL_ADC_CALC_TEMPERATURE(
      3300U, next.temperature_raw, ADC_RESOLUTION_12B);

  /* 奇数表示正在写，偶数表示中断可以读取完整快照。 */
  board_adc_sequence++;
  __DMB();
  board_adc_measurements = next;
  __DMB();
  board_adc_sequence++;
  board_adc_valid = 1U;

  return HAL_OK;
}

/**
 * @brief 单次尝试获取一致快照，不在中断里自旋等待主循环。
 * 写入期间序号为奇数，写入结束变偶数；复制前后序号一致才接受。
 * 内存屏障约束数据与序号的访问顺序。若中断打断了写入，立即返回0，
 * 让主循环恢复后完成发布；若等待奇数变偶数则会阻塞被打断的写入者。
 * sequence标识发布批次，不代表采样时间戳；成功返回1，失败不更新输出。
 */
uint8_t BoardAdc_GetSnapshot(BoardAdcMeasurements_t *snapshot,
                             uint32_t *sequence) {
  /* 先复制到局部，避免直接修改调用者输出。 */
  BoardAdcMeasurements_t local;
  /* 复制前读取的 seqlock 序号。 */
  uint32_t sequence_before;
  /* 复制后读取的 seqlock 序号。 */
  uint32_t sequence_after;

  if ((snapshot == NULL) || (sequence == NULL) || (board_adc_valid == 0U)) {
    return 0U;
  }

  sequence_before = board_adc_sequence;
  if ((sequence_before & 1U) != 0U) {
    return 0U;
  }

  __DMB();
  local = board_adc_measurements;
  __DMB();

  sequence_after = board_adc_sequence;
  if ((sequence_before != sequence_after) ||
      ((sequence_after & 1U) != 0U) ||
      (!isfinite(local.vbus_voltage)) ||
      (!isfinite(local.phase_u_voltage)) ||
      (!isfinite(local.phase_v_voltage)) ||
      (!isfinite(local.phase_w_voltage)) ||
      (!isfinite(local.temperature_c))) {
    return 0U;
  }

  *snapshot = local;
  *sequence = sequence_after;

  return 1U;
}

BoardAdcMeasurements_t *BoardAdc_GetMeasurements(void) {
  /* 返回共享快照地址，调用者应结合有效标志/序号保证读取一致性。 */
  return &board_adc_measurements;
}
