/**
 * @file board_adc.c
 * @brief 主循环软件触发母线电压与温度采样，并发布测量快照。
 * ADC1 Rank1采Vbus（6.5周期），Rank2采MCU温度（247.5周期）。
 * U/V/W三相端电压已迁移到TIM1同步Injected序列，不再由规则组轮询。
 */
#include "board_adc.h"

#include "adc.h"

/* ADC计数到板端实际电压的统一比例定义在board_adc.h，供规则组和注入组共用。 */

/* 已完成发布的最近一组规则组测量结果；当前有效内容为Vbus和MCU温度。 */
static BoardAdcMeasurements_t board_adc_measurements = {0};

/**
 * @brief 读取一轮规则组结果，全部成功后才发布整组数据。
 * 局部next暂存本轮结果，途中失败时共享测量值仍保持上一轮完整结果。
 * 每次轮询最多等待10ms；只允许主循环调用，避免阻塞电流控制中断。
 */
HAL_StatusTypeDef BoardAdc_Update(void) {
  /* 暂存本轮完整结果；只有所有 ADC 转换成功才会发布到共享对象。 */
  BoardAdcMeasurements_t next = {0};
  /* ADC1规则组原始计数：[0]=母线电压(Vbus)，[1]=MCU内部温度，单位count，范围0~4095。 */
  uint16_t adc1_values[2];
  /* 当前要触发并读取的ADC1规则组序号，0=Rank1(Vbus)，1=Rank2(温度)。 */

  /* ADC1规则组间断模式：每次触发只转换一个Rank，读完再推进到下一Rank。
   * 温度通道使用长采样时间，仍按逐Rank轮询确保每个结果与通道对应。
   * 顺序必须与MX_ADC1_Init()里的规则组Rank配置一致，否则会把温度当母线电压。
   */
  for (uint32_t rank = 0U; rank < 2U; rank++) {
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

  /* Vbus恢复100kΩ/5.1kΩ分压倍率；温度按STM32内部标定值换算。 */
  next.vbus_voltage = (float)next.vbus_raw * BOARD_ADC_COUNT_TO_VOLTAGE;
  next.temperature_c = (float)__HAL_ADC_CALC_TEMPERATURE(
      3300U, next.temperature_raw, ADC_RESOLUTION_12B);

  board_adc_measurements = next;

  return HAL_OK;
}

BoardAdcMeasurements_t *BoardAdc_GetMeasurements(void) {
  return &board_adc_measurements;
}
