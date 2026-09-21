#ifndef FDCAN_TEST_H
#define FDCAN_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

HAL_StatusTypeDef CANFD_SendSingleTestFrame(void);

#ifdef __cplusplus
}
#endif

#endif /* FDCAN_TEST_H */
