#ifndef BOARD_ADC_H
#define BOARD_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <stdint.h>

/* 12位ADC计数恢复到100kΩ/5.1kΩ分压输入侧电压的比例，单位V/count。 */
#define BOARD_ADC_COUNT_TO_VOLTAGE \
  (3.3f / 4096.0f * ((100.0f + 5.1f) / 5.1f))

/**
 * @brief PCB规则组ADC测量结果
 *
 * 规则组现在只保留慢速量：
 * - PA0：母线电压
 * - ADC1内部温度传感器
 *
 * U/V/W三相端电压已迁移到TIM1同步Injected序列。
 */
typedef struct {
  /* 单次12位转换原始计数，正常范围0~4095。 */
  uint16_t vbus_raw;
  uint16_t temperature_raw;

  /* 母线电压已恢复100k/5.1k分压倍率，单位V。 */
  float vbus_voltage;
  /* MCU内部温度传感器的换算结果，单位摄氏度。 */
  float temperature_c;
} BoardAdcMeasurements_t;

/**
 * @brief 软件触发一次规则组采样并更新测量值。
 * @note 主循环独占调用；各通道顺序转换，非同步采样，失败保留上一组结果。
 * @return HAL_OK采样并发布成功；HAL_ERROR启动失败；HAL_TIMEOUT转换超时（保留上一组结果）。
 */
HAL_StatusTypeDef BoardAdc_Update(void);

/**
 * @brief 获取最近一次规则组测量结果，供主循环和调试读取。
 * @return 指向共享测量结构体的指针（非NULL）。
 */
BoardAdcMeasurements_t *BoardAdc_GetMeasurements(void);

#ifdef __cplusplus
}
#endif

#endif
