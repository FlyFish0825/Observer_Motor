#ifndef BOARD_ADC_H
#define BOARD_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <stdint.h>

/**
 * @brief PCB规则组ADC测量结果
 *
 * 引脚对应关系：
 * - PA0：母线电压
 * - PB12：U相端电压
 * - PA4：V相端电压
 * - PB11：W相端电压
 */
typedef struct {
  /* 单次12位转换原始计数，正常范围0~4095。 */
  uint16_t vbus_raw;
  uint16_t phase_u_raw;
  uint16_t phase_v_raw;
  uint16_t phase_w_raw;

  /* 已恢复100k/5.1k分压倍率，单位V；端电压为对地测量值。 */
  float vbus_voltage;
  float phase_u_voltage;
  float phase_v_voltage;
  float phase_w_voltage;
} BoardAdcMeasurements_t;

/**
 * @brief 软件触发一次规则组采样并更新测量值
 * @note 主循环独占调用；各通道顺序转换，非同步采样，失败保留上一组结果。
 */
HAL_StatusTypeDef BoardAdc_Update(void);

/**
 * @brief 无阻塞读取一份完整测量快照
 * @note  若主循环正在更新，函数返回0，调用方应沿用上一份有效数据。
 */
uint8_t BoardAdc_GetSnapshot(BoardAdcMeasurements_t *snapshot,
                             uint32_t *sequence);

/**
 * @brief 获取测量结构地址，供调试控制台观察
 * @note 指针指向共享数据；控制中断需要完整一组数据时使用GetSnapshot。
 */
BoardAdcMeasurements_t *BoardAdc_GetMeasurements(void);

#ifdef __cplusplus
}
#endif

#endif
