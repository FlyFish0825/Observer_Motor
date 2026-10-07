/**
 * @file board_adc.c
 * @brief 主循环软件触发母线电压与温度采样，并发布测量快照。
 * ADC1 Rank1采Vbus，Rank2采MCU温度，Rank3采VREFINT。
 * U/V/W三相端电压已迁移到TIM1同步Injected序列，不再由规则组轮询。
 */
#include "board_adc.h"

#include "adc.h"

/* ADC计数到板端实际电压的统一比例定义在board_adc.h，供规则组和注入组共用。 */

/* 已完成发布的最近一组规则组测量结果。 */
static BoardAdcMeasurements_t board_adc_measurements = {0};
static volatile float board_adc_voltage_scale = 0.0f;

/* STM32G431 DS12589表5：第二标定点为130℃。旧版LL头文件误写110℃，
 * 在板级明确使用正确值，不修改供应商驱动，也不使用整数温度换算宏。 */
#define BOARD_ADC_TS_CAL1_C 30.0f
#define BOARD_ADC_TS_CAL2_C 130.0f
#define BOARD_ADC_DIVIDER_RATIO ((100.0f + 5.1f) / 5.1f)

/**
 * @brief 读取一轮规则组结果，全部成功后才发布整组数据。
 * 局部next暂存本轮结果，途中失败时共享测量值仍保持上一轮完整结果。
 * 每次轮询最多等待10ms；只允许主循环调用，避免阻塞电流控制中断。
 */
HAL_StatusTypeDef BoardAdc_Update(void) {
  /* 暂存本轮完整结果；只有所有 ADC 转换成功才会发布到共享对象。 */
  BoardAdcMeasurements_t next = {0};
  /* 顺序必须与adc.c及Observer.ioc一致：[Vbus, 温度, VREFINT]。 */
  uint16_t adc1_values[3];

  /* ADC1规则组间断模式：每次触发只转换一个Rank，读完再推进到下一Rank。
   * 温度通道使用长采样时间，仍按逐Rank轮询确保每个结果与通道对应。
   * 顺序必须与MX_ADC1_Init()里的规则组Rank配置一致，否则会把温度当母线电压。
   */
  for (uint32_t rank = 0U; rank < 3U; rank++) {
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

  next.vbus_raw = adc1_values[0];
  next.temperature_raw = adc1_values[1];
  next.vrefint_raw = adc1_values[2];

  const uint16_t vref_cal = *VREFINT_CAL_ADDR;
  const uint16_t ts_cal1 = *TEMPSENSOR_CAL1_ADDR;
  const uint16_t ts_cal2 = *TEMPSENSOR_CAL2_ADDR;
  /* 无效采样/标定不得变成除零、假温度或新的电压系数，保留上一组。 */
  if ((next.vrefint_raw == 0U) || (next.vrefint_raw >= 4095U) ||
      (next.temperature_raw == 0U) || (next.temperature_raw >= 4095U) ||
      (vref_cal == 0U) || (vref_cal >= 4095U) ||
      (ts_cal1 == 0U) || (ts_cal2 >= 4095U) || (ts_cal2 <= ts_cal1)) {
    return HAL_ERROR;
  }
  next.vref_mv = (float)VREFINT_CAL_VREF * (float)vref_cal /
                 (float)next.vrefint_raw;
  if ((next.vref_mv < 1710.0f) || (next.vref_mv > 3600.0f)) {
    return HAL_ERROR;
  }

  const float voltage_scale = next.vref_mv * (0.001f / 4096.0f) *
                              BOARD_ADC_DIVIDER_RATIO;
  next.vbus_voltage = (float)next.vbus_raw * voltage_scale;
  const float temperature_at_cal_vref = (float)next.temperature_raw *
      next.vref_mv / (float)TEMPSENSOR_CAL_VREFANALOG;
  next.temperature_c = BOARD_ADC_TS_CAL1_C +
      (temperature_at_cal_vref - (float)ts_cal1) *
      (BOARD_ADC_TS_CAL2_C - BOARD_ADC_TS_CAL1_C) /
      (float)(ts_cal2 - ts_cal1);

  board_adc_measurements = next;
  board_adc_voltage_scale = voltage_scale;

  return HAL_OK;
}

BoardAdcMeasurements_t *BoardAdc_GetMeasurements(void) {
  return &board_adc_measurements;
}

float BoardAdc_GetVoltageScale(void) {
  return board_adc_voltage_scale;
}
