#ifndef MOTOR_CALIBRATION_H
#define MOTOR_CALIBRATION_H

/**
 * @brief Stationary stator-resistance identification service.
 */

#include "foc_math.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    MOTOR_CALIBRATION_IDLE = 0U,
    MOTOR_CALIBRATION_STARTING,
    MOTOR_CALIBRATION_RUNNING,
    MOTOR_CALIBRATION_STOPPING,
    MOTOR_CALIBRATION_DONE,
    MOTOR_CALIBRATION_ERROR
} MotorCalibrationState_t;

typedef enum
{
    RS_INJECTION_VOLTAGE = 0U,
    RS_INJECTION_CURRENT
} RsInjectionMode_t;

/** @brief Start stationary Rs identification in voltage mode (V).
 *  @return HAL_OK when armed; power outputs are gated before PWM setup.
 */
HAL_StatusTypeDef MotorCalibration_Start(void);
/** @brief Start voltage injection or the optional current A/B mode (A).
 *  @param mode Selects the only controller allowed to write calibration Duty.
 */
HAL_StatusTypeDef MotorCalibration_StartMode(RsInjectionMode_t mode);
/** @brief Request immediate power-output shutdown. */
void MotorCalibration_Stop(void);
/** @brief Consume one paired ADC current sample in the injected ADC ISR.
 *  @param adc_b_raw B-phase oversampled ADC sum, before gain/offset conversion.
 *  @note No printing or fitting occurs here; overcurrent gates MOE immediately.
 */
void MotorCalibration_AdcStep(uint16_t adc_b_raw);
/** @brief Main-loop watchdog, safe timer stop, fit and report after output off. */
void MotorCalibration_Process(void);
uint8_t MotorCalibration_IsActive(void);
MotorCalibrationState_t MotorCalibration_GetState(void);
float MotorCalibration_GetResultOhm(void);
uint8_t MotorCalibration_GetErrorCode(void);

#ifdef __cplusplus
}
#endif

#endif
