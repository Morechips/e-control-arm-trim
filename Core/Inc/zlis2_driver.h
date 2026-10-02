#ifndef ZLIS2_DRIVER_H
#define ZLIS2_DRIVER_H

#include <stddef.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ZLIS2_BAUD_RATE 115200U
#define ZLIS2_TX_TIMEOUT_MS 40U
/* Capacity choice, not a protocol limit on IDs or action numbers. */
#define ZLIS2_MAX_SERVO_COMMANDS 24U
#define ZLIS2_SERVO_COMMAND_LENGTH 15U
#define ZLIS2_MAX_TX_LENGTH (2U + ZLIS2_MAX_SERVO_COMMANDS * ZLIS2_SERVO_COMMAND_LENGTH)

typedef enum {
    ZLIS2_OK = 0,
    ZLIS2_ERROR,
    ZLIS2_INVALID_PARAM,
    ZLIS2_NOT_INITIALIZED,
    ZLIS2_BUFFER_OVERFLOW,
    ZLIS2_UART_ERROR
} ZLIS2_Status;

typedef struct {
    uint16_t id;
    uint16_t pwm;
    uint16_t time_ms;
} ZLIS2_ServoCommand;

/* Single instance, serialized foreground calls only; HAL tick must run.
 * UART must already be initialized at 115200 8N1, TX enabled, no flow control.
 * Init sends nothing and never initializes/reconfigures the peripheral.
 * Failed Init clears the old binding; the handle must outlive the driver.
 * OK means HAL finished TX, not that the controller acknowledged execution.
 * UART errors may leave bytes on the wire: there is no automatic retry.
 */
ZLIS2_Status ZLIS2_Init(UART_HandleTypeDef *huart);
ZLIS2_Status ZLIS2_SetServo(uint16_t id, uint16_t pwm, uint16_t time_ms);
ZLIS2_Status ZLIS2_SetServos(const ZLIS2_ServoCommand *commands, size_t count);
ZLIS2_Status ZLIS2_RunAction(uint16_t action);
/* repeat == 0 means infinite looping for action ranges and recorded playback. */
ZLIS2_Status ZLIS2_RunActionRange(uint16_t start_action, uint16_t end_action, uint16_t repeat);
ZLIS2_Status ZLIS2_StopAll(void);
ZLIS2_Status ZLIS2_StopServo(uint16_t id);
ZLIS2_Status ZLIS2_SetServoBias(uint16_t id, int16_t bias);
ZLIS2_Status ZLIS2_RunCombinedAction(uint16_t group, uint16_t repeat);
ZLIS2_Status ZLIS2_RecordPose(void);
ZLIS2_Status ZLIS2_RunRecorded(uint16_t repeat);
ZLIS2_Status ZLIS2_ClearRecorded(void);
ZLIS2_Status ZLIS2_SetRecordPeriod(uint16_t period_ms);
/* Caller handles any reboot wait; Reset only transmits the command. */
ZLIS2_Status ZLIS2_Reset(void);
/* Debug/extension only. Caller guarantees valid ZL-IS2 syntax and readable,
 * NUL-terminated storage. No bytes are added/changed; NUL is not transmitted.
 * Maximum length is ZLIS2_MAX_TX_LENGTH bytes, excluding NUL.
 */
ZLIS2_Status ZLIS2_SendRaw(const char *command);

#ifdef __cplusplus
}
#endif
#endif
