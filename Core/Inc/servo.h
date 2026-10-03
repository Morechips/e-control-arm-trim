#ifndef SERVO_H
#define SERVO_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SERVO_BAUD_RATE 115200U
#define SERVO_TX_TIMEOUT_MS 40U
/* Capacity choice, not a protocol limit on IDs or action numbers. */
#define SERVO_MAX_COMMANDS 24U
#define SERVO_COMMAND_LENGTH 15U
#define SERVO_DEFAULT_TIME_MS 2000U
#define SERVO_MAX_TX_LENGTH (2U + SERVO_MAX_COMMANDS * SERVO_COMMAND_LENGTH)
/* Fixed-pose groups never exceed one command per controller channel. */
#define SERVO_MAX_POSE_COMMANDS 4U
#define SERVO_POSE_FRAME_SIZE (2U + SERVO_MAX_POSE_COMMANDS * SERVO_COMMAND_LENGTH)

typedef enum {
    SERVO_OK = 0,
    SERVO_ERROR,
    SERVO_INVALID_PARAM,
    SERVO_NOT_INITIALIZED,
    SERVO_BUFFER_OVERFLOW,
    SERVO_UART_ERROR,
    SERVO_BUSY
} ServoStatus_t;

typedef struct {
    uint16_t id;
    uint16_t pwm;
    uint16_t time_ms;
} ServoCommand_t;

typedef enum {
    SERVO_TRANSFER_NONE = 0, SERVO_TRANSFER_PENDING,
    SERVO_TRANSFER_COMPLETE, SERVO_TRANSFER_FAILED
} ServoTransferState_t;

typedef struct {
    ServoTransferState_t state;
    ServoStatus_t result;
    bool started;
    uint32_t started_tick, completed_tick;
} ServoTransferStatus_t;

/* Optional foreground motion lease. The ordinary latest-pending API is BUSY
 * while held. Owned transfers are tracked through TC, never replaced or retried.
 * Cancel drops waiting bytes and forgets their tag, but preserves an active frame;
 * exactly one new stop may wait behind it. TC is transport completion only. */
ServoStatus_t Servo_Acquire(const void *owner);
ServoStatus_t Servo_Release(const void *owner);
ServoStatus_t Servo_SetValuesOwned(const void *owner, const uint16_t *pwm,
                                  size_t count, uint16_t time_ms);
ServoStatus_t Servo_SetCommandsOwned(const void *owner, const ServoCommand_t *commands,
                                    size_t count);
ServoStatus_t Servo_StopServoOwned(const void *owner, uint16_t id);
ServoStatus_t Servo_CancelPendingOwned(const void *owner);
ServoTransferStatus_t Servo_GetTransferStatus(const void *owner);
/* Service owned transport deadlines in foreground; unowned profile is unchanged. */
void Servo_Process(void);

typedef enum {
    ServoCode_NONE = -1,
    TakeBall_BEGIN = 0,
    TakeBall_Before = TakeBall_BEGIN, TakeBall_Mid, TakeBall_Gap, TakeBall_END,
    BarrelDown_BEGIN = TakeBall_END,
    BarrelDown_Up = BarrelDown_BEGIN, BarrelDown_Down, BarrelDown_END,
    TakeHostage_Begin = BarrelDown_END,
    TakeHostage_Up = TakeHostage_Begin, TakeHostage_Catch, TakeHostage_Gap,
    TakeHostage_Leave, TakeHostage_END,
    Servo_REFERENCE = TakeHostage_END, Servo_RST, Servo_AIM, TakeHostage_PreGrab,
    ServoCode_MAX
} ServoCode;
#define SERVO_PER_CODE_COUNT 4U
extern const ServoCommand_t codes[ServoCode_MAX][SERVO_PER_CODE_COUNT];
extern const uint8_t servo_code_counts[ServoCode_MAX];
extern const uint16_t g_servo_gap_time_ms;
extern const uint16_t g_servo_legacy_reset_guard_ms, g_servo_legacy_aim_guard_ms;
const char *Servo_GetName(ServoCode code);
bool GetFullCommand(ServoCode code, char *buffer, size_t capacity, size_t *length);
ServoStatus_t Servo_SendPreset(ServoCode code);
ServoStatus_t Servo_SendGap(uint16_t pwm);

/* Serialized foreground submissions; USART3 TX completion dispatch is required.
 * Init only binds an already configured 115200 8N1 UART. Rebind while sending
 * returns BUSY and preserves ownership. OK means submitted, including replacement
 * of the one waiting frame; it does not mean mechanical completion or an ACK.
 * HAL refusal before submission returns BUSY/ UART_ERROR and is not retried.
 */
uint8_t Servo_IsIdle(void);
void Servo_TxCallback(UART_HandleTypeDef *uart);
ServoStatus_t Servo_Init(UART_HandleTypeDef *huart);
/* Pure protocol formatter, independent of binding and queue state. */
ServoStatus_t Servo_FormatCommands(const ServoCommand_t *commands, size_t count,
                                char *buffer, size_t capacity, size_t *length);
ServoStatus_t Servo_SetChannel(uint16_t id, uint16_t pwm, uint16_t time_ms);
ServoStatus_t Servo_SetCommands(const ServoCommand_t *commands, size_t count);
/* IDs 000..003, one shared time; every command carries T. Driver-owned buffers
 * keep both stack and TX lifetime bounded, including historical 24-servo frames. */
ServoStatus_t Servo_SetValues(const uint16_t *pwm, size_t count, uint16_t time_ms);
ServoStatus_t Servo_RunAction(uint16_t action);
/* repeat == 0 means infinite looping for action ranges and recorded playback. */
ServoStatus_t Servo_RunActionRange(uint16_t start_action, uint16_t end_action, uint16_t repeat);
ServoStatus_t Servo_StopAll(void);
ServoStatus_t Servo_StopServo(uint16_t id);
ServoStatus_t Servo_SetBias(uint16_t id, int16_t bias);
ServoStatus_t Servo_RunCombinedAction(uint16_t group, uint16_t repeat);
ServoStatus_t Servo_RecordPose(void);
ServoStatus_t Servo_RunRecorded(uint16_t repeat);
ServoStatus_t Servo_ClearRecorded(void);
ServoStatus_t Servo_SetRecordPeriod(uint16_t period_ms);
/* Caller handles any reboot wait; Reset only transmits the command. */
ServoStatus_t Servo_Reset(void);
/* Debug/extension only. Caller guarantees valid ZL-IS2 syntax and readable,
 * NUL-terminated storage. No bytes are added/changed; NUL is not transmitted.
 * Maximum length is SERVO_MAX_TX_LENGTH bytes, excluding NUL.
 */
ServoStatus_t Servo_SendRaw(const char *command);

#ifdef __cplusplus
}
#endif
#endif
