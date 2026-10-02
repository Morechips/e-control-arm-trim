#ifndef SERVO_POSE_H
#define SERVO_POSE_H

#include "arm_control.h"
#include <stdint.h>

#define SERVO_POSE_COUNT 8U

/* Logical poses, independent of the legacy Bluetooth SERVO_MODE numbers. */
typedef enum {
    SERVO_MODE_NONE = 0,
    SERVO_MODE_BALL_PREPARE = 1,
    SERVO_MODE_BALL_REACH = 2,
    SERVO_MODE_BALL_GRIP = 3,
    SERVO_MODE_ABOVE_BUCKET = 4,
    SERVO_MODE_BALL_RELEASE = 5,
    SERVO_MODE_HOSTAGE_REACH = 6,
    SERVO_MODE_HOSTAGE_GRIP = 7,
    SERVO_MODE_HOSTAGE_LIFT = 8
} ServoMode_t;

typedef enum {
    SERVO_POSE_OK = 0,
    SERVO_POSE_INVALID,
    SERVO_POSE_LIMIT
} ServoPoseResult_t;

/* Numeric presets use IDs 000..002; original bool presets use IDs 000..003. */
ServoPoseResult_t ServoPose_BuildStep(uint8_t pose_index, ArmStep_t *step);
ServoPoseResult_t ServoPose_BuildMode(ServoMode_t mode, ArmStep_t *step);
ServoPoseResult_t ServoPose_BuildOriginalMode(ServoMode_t mode, ArmStep_t *step);
const char *ServoPose_Name(ServoMode_t mode);

#endif
