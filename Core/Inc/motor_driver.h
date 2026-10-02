#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H
#include "main.h"
#include "car_config.h"
#include "emm42_driver.h"
typedef Emm42Status_t MotorStatus_t;
typedef enum { MOTOR_RF = 1, MOTOR_LF = 2, MOTOR_LR = 3, MOTOR_RR = 4 } CarMotor_t;
typedef struct { CarMotor_t motor; uint8_t address; char roller; } CarMotorMap_t;
extern const CarMotorMap_t car_motor_map[4];
/* Logical positive = wheel rolling forward (+X), before roller coupling.
 * Hardware-confirmed installation correction; applied ONLY in this layer. */
extern const int8_t motor_sign[5];
int16_t Car_Motor_Limit(int16_t logical_rpm);
int16_t Car_Motor_Physical(uint8_t address, int16_t logical_rpm);
HAL_StatusTypeDef car_motor_set(CarMotor_t motor, int16_t logical_rpm);
void Motor_Init(void);
HAL_StatusTypeDef Motor_Enable(uint8_t addr);
HAL_StatusTypeDef Motor_Disable(uint8_t addr);
HAL_StatusTypeDef Motor_EnableAll(void);
HAL_StatusTypeDef Motor_DisableAll(void);
HAL_StatusTypeDef Motor_SetSpeed(uint8_t addr, int16_t rpm);
HAL_StatusTypeDef Motor_SetSpeedSync4(int16_t rpm1, int16_t rpm2, int16_t rpm3, int16_t rpm4);
HAL_StatusTypeDef Motor_EStop(uint8_t addr);
HAL_StatusTypeDef Motor_EStopAll(void);
void Motor_Process(void);
void Motor_ProcessRx(const uint8_t *data, uint16_t length);
const MotorStatus_t *Motor_GetLastStatus(uint8_t addr);
uint8_t Motor_HasFault(void);
void Motor_ClearFault(void);
uint8_t Motor_IsIdle(void);
void Motor_RxCallback(UART_HandleTypeDef *uart);
void Motor_TxCallback(UART_HandleTypeDef *uart);
void Motor_ErrorCallback(UART_HandleTypeDef *uart);
#endif
