#ifndef EMM42_DRIVER_H
#define EMM42_DRIVER_H
#include "main.h"
#include "car_config.h"
typedef struct {
    uint8_t command, response_status, valid;
    uint32_t timestamp;
} Emm42Status_t;
/* UART5 protocol transport. Speeds are PHYSICAL signed RPM; no wheel geometry.
 * Application motion must use motor_driver.h / mecanum.h so limits and
 * installation correction are applied. F3/F6/FE/FF frames are preserved. */

void Emm42_Init(void);
HAL_StatusTypeDef Emm42_Enable(uint8_t addr);
HAL_StatusTypeDef Emm42_Disable(uint8_t addr);
HAL_StatusTypeDef Emm42_EnableAll(void);
HAL_StatusTypeDef Emm42_DisableAll(void);
HAL_StatusTypeDef Emm42_SetSpeed(uint8_t addr, int16_t rpm);
HAL_StatusTypeDef Emm42_SetSpeedSync4(int16_t rpm1, int16_t rpm2, int16_t rpm3, int16_t rpm4);
HAL_StatusTypeDef Emm42_EStop(uint8_t addr);
HAL_StatusTypeDef Emm42_EStopAll(void);
void Emm42_Process(void);
void Emm42_ProcessRx(const uint8_t *data, uint16_t length);
const Emm42Status_t *Emm42_GetLastStatus(uint8_t addr);
/* LastStatus is a cached COMMAND ACK, not a queried servo/encoder state.
 * No validated nonblocking telemetry decoder exists in this transport. */
typedef enum { EMM42_TELEMETRY_UNSUPPORTED = 0 } Emm42TelemetryCapability_t;
Emm42TelemetryCapability_t Emm42_GetTelemetryCapability(void);
uint8_t Emm42_HasFault(void);
void Emm42_ClearFault(void);
uint8_t Emm42_IsIdle(void);
void Emm42_RxCallback(UART_HandleTypeDef *uart);
void Emm42_TxCallback(UART_HandleTypeDef *uart);
void Emm42_ErrorCallback(UART_HandleTypeDef *uart);
#endif
