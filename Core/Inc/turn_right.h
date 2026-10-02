#ifndef TURN_RIGHT_H
#define TURN_RIGHT_H

#include "mecanum.h"
#include <stdbool.h>

typedef enum
{
    TURN_RIGHT_IDLE = 0, TURN_RIGHT_TURNING, TURN_RIGHT_STOPPING,
    TURN_RIGHT_RESETTING, TURN_RIGHT_DONE, TURN_RIGHT_CANCELLED, TURN_RIGHT_FAULT,
    TURN_RIGHT_CORRECTING
} TurnRightState_t;

typedef enum
{
    TURN_RIGHT_NO_ERROR = 0, TURN_RIGHT_IMU_ERROR, TURN_RIGHT_MOTOR_ERROR,
    TURN_RIGHT_TIMEOUT, TURN_RIGHT_RESET_ERROR,
    TURN_RIGHT_WRONG_WAY, TURN_RIGHT_NO_PROGRESS, TURN_RIGHT_TOLERANCE_ERROR
} TurnRightError_t;

typedef struct
{
    TurnRightState_t state;
    TurnRightError_t error;
    float turned_degrees;
} TurnRightStatus_t;

/* Foreground-only turn operation, used by Bluetooth and route/mission controls.
 * Positive RPM requests are capped at 20 RPM for right 90-degree turns,
 * 20 RPM for left 90-degree turns, and 30 RPM for right 180-degree turns
 * across Bluetooth, route and mission callers.
 * Requires initialized/enabled motors, fresh IMU and exclusive motion ownership.
 * HAL_OK means accepted, HAL_BUSY means active/transport busy; no auto-enable.
 * Caller must run JY61_Process, TurnRight_Process(allowed), then Motor_Process
 * continuously. Other drive/heading-reference commands must not run concurrently.
 * Pass false (or Cancel) for any brake, disable, link-loss or safety cancellation.
 * Progress is signed relative yaw, unwrapped between samples, not absolute yaw.
 * Each sample interval must contain less than 180 degrees of physical rotation.
 * Slows to at most 20 RPM for right 180-degree turns and 20 RPM for left
 * 90-degree turns near the target (never above the initial speed),
 * leads the stop using fresh gyro rate, then checks settled relative angle.
 * Up to three low-speed corrections may run before completion; wrong-way
 * motion or lack of progress requests an immediate motor stop.
 * Completion requires stop queue drained, stable gyro, hardware zero command,
 * then fresh near-zero yaw; DONE does not mean a mechanically exact target angle.
 */
HAL_StatusTypeDef right90(int16_t rpm);
HAL_StatusTypeDef right180(int16_t rpm);
HAL_StatusTypeDef left90(int16_t rpm);
void TurnRight_Process(bool motion_allowed);
void TurnRight_Cancel(void);
const TurnRightStatus_t *TurnRight_GetStatus(void);

#endif
