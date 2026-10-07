#ifndef BOARD_ADC_H
#define BOARD_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <stdint.h>

/* 母线和三相端电压共用实测VREF+换算系数，禁止重新写死3.3 V。 */
#define BOARD_ADC_COUNT_TO_VOLTAGE (BoardAdc_GetVoltageScale())

/**
 * @brief PCB规则组ADC测量结果
 *
 * 规则组现在只保留慢速量：
 * - PA0：母线电压
 * - ADC1内部温度传感器
 * - VREFINT：结合出厂标定值计算ADC实际参考电压
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
  uint16_t vrefint_raw;
  /* ADC实际参考电压VREF+，单位mV；不能用数字电源VDD代替。 */
  float vref_mv;
} BoardAdcMeasurements_t;

/**
 * @brief 软件触发一次规则组采样并更新测量值。
 * @note 主循环独占调用；各通道顺序转换，非同步采样，失败保留上一组结果。
 * @return HAL_OK发布成功；HAL_ERROR启动/标定/参考电压无效；HAL_TIMEOUT转换超时。
 *         失败保留上一组结果及电压换算系数。
 */
HAL_StatusTypeDef BoardAdc_Update(void);

/**
 * @brief 获取最近一次规则组测量结果，供主循环和调试读取。
 * @return 指向共享测量结构体的指针（非NULL）。
 */
BoardAdcMeasurements_t *BoardAdc_GetMeasurements(void);

/* 单位V/count，已包含100kΩ/5.1kΩ分压倍率；首次有效采样前为0。
 * 中断只读已发布的单个float，不在25 kHz控制路径重复计算除法。 */
float BoardAdc_GetVoltageScale(void);

#ifdef __cplusplus
}
#endif

#endif
