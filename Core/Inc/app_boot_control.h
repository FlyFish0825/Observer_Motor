#ifndef APP_BOOT_CONTROL_H
#define APP_BOOT_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

/* Bootloader 控制帧固定使用标准 ID 0x000、经典 CAN、8 字节。 */
#define APP_BOOT_CONTROL_RX_ID    0x000U
#define APP_BOOT_ENTER_CMD        0x04U
#define APP_BOOT_REQUEST_MAGIC    0x544F4F42UL /* 必须与 Bootloader 的 BOOT 一致 */

/* Config 无效时的安全回退值；正常 Boot APP 会读取现有 Config 页 Node ID。 */
#ifndef APP_BOOT_NODE_ID
#define APP_BOOT_NODE_ID          1U
#endif

HAL_StatusTypeDef AppBootControl_Init(FDCAN_HandleTypeDef *hfdcan);
void AppBootControl_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_BOOT_CONTROL_H */
