#ifndef ARM_CONTROL_H
#define ARM_CONTROL_H

#include "arm_config.h"
#include "servo.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum
{
    ARM_JOINT_SHOULDER = 0,
    ARM_JOINT_ELBOW,
    ARM_JOINT_WRIST,
    ARM_JOINT_GRIPPER,
    ARM_JOINT_AUX_1
} ArmJoint_t;

typedef struct
{
    bool enabled;
    bool calibrated;
    uint16_t servo_id;
    uint16_t min_position;
    uint16_t max_position;
} ArmJointConfig_t;

typedef struct
{
    ArmJointConfig_t joints[ARM_JOINT_COUNT];
} ArmConfig_t;

typedef struct
{
    /* Bit n selects joint n. Nonselected positions are ignored. */
    uint8_t joint_mask;
    uint16_t position[ARM_JOINT_COUNT];
    /* 1..9999 ms. Zero is rejected to avoid an unspecified immediate move. */
    uint16_t move_ms;
    /* Additional wait after the requested move duration; not feedback. */
    uint32_t hold_ms;
} ArmStep_t;

typedef enum
{
    ARM_UNCONFIGURED = 0,
    ARM_IDLE,
    ARM_RUNNING,
    ARM_COMPLETE_ESTIMATED,
    ARM_STOPPING,
    ARM_CANCELLED,
    ARM_FAULT
} ArmState_t;

typedef enum
{
    ARM_ERROR_NONE = 0,
    ARM_ERROR_TRANSPORT,
    ARM_ERROR_SERVICE_TIMEOUT,
    ARM_ERROR_STOP_TRANSPORT
} ArmError_t;

typedef enum
{
    ARM_OK = 0,
    ARM_INVALID_ARGUMENT,
    ARM_NOT_CONFIGURED,
    ARM_BUSY,
    ARM_INHIBITED,
    ARM_FAULT_LATCHED
} ArmResult_t;

typedef struct
{
    ArmState_t state;
    ArmError_t error;
    ServoStatus_t transport_result;
    bool motion_allowed;
    bool stop_delivery_failed;
    size_t step_index;
    size_t step_count;
} ArmStatus_t;

/* Single foreground owner; not callable from an ISR or concurrently.
 * Servo must already be bound by Board_Init. Init is silent and idempotent.
 * The default config has NO enabled or calibrated joints, IDs unassigned.
 * Configure validates and copies config, emits no bytes, and revokes permission.
 * Never use raw Servo motion/action-group APIs concurrently with this module. */
void Arm_Init(void);
void Arm_DefaultConfig(ArmConfig_t *config);
/* Loads this robot's bench-confirmed servo IDs and numeric P limits. This
 * only prepares configuration data; it never authorizes or starts motion. */
void Arm_ProjectConfig(ArmConfig_t *config);
ArmResult_t Arm_Configure(const ArmConfig_t *config);
ArmResult_t Arm_SetMotionAllowed(bool allowed);

/* Requests copy the complete sequence before accepting it; no UART TX here.
 * Require valid configuration and explicit motion permission. Positions are
 * controller P units, NOT degrees/mm. No safe poses or grasp forces are assumed.
 * A successful request means accepted, NOT moved or gripped. */
ArmResult_t Arm_MoveJoint(ArmJoint_t joint, uint16_t position,
                          uint16_t move_ms, uint32_t hold_ms);
ArmResult_t Arm_StartSequence(const ArmStep_t *steps, size_t count);
/* Explicit phone bool presets use the supplied original P values, including
 * positions outside this robot's calibrated ranges. Controller bounds and
 * the normal arm authorization, busy and fault gates still apply. */
ArmResult_t Arm_StartOriginalPreset(const ArmStep_t *step);
/* Bluetooth RST pose: sends all four configured IDs immediately without T.
 * Requires motion permission and an idle, fault-free arm. Completion is only
 * estimated by a software guard because the controller supplies no feedback. */
ArmResult_t Arm_StartResetPose(void);
/* Bluetooth AIM pose: send four configured IDs in one untimed command.
 * Uses normal motion permission, idle/fault gates and calibrated limits. */
ArmResult_t Arm_StartAimPose(void);
/* Sends $RST! only while the arm is idle; does not clear a latched fault. */
ArmResult_t Arm_ResetController(void);
/* PE4 manual override: validate and transmit one step immediately on USART3.
 * Replaces any pending/running arm step or stop sequence in the foreground.
 * The physical press is an explicit retry after an arm transport fault. */
ArmResult_t Arm_SendImmediate(const ArmStep_t *step);

/* Call frequently. Waits are nonblocking, but each Process can perform ONE
 * asynchronous Servo submission.
 * Completion is timing-based only: no position/force/grasp feedback is parsed.
 * Late service while RUNNING triggers a stop request when Process resumes.
 * This software deadline cannot stop hardware while the MCU is stalled. */
void Arm_Process(void);

/* Revokes permission and cancels unsent motion immediately. Process then sends
 * one addressed stop per enabled joint per call, never broadcast or torque-off.
 * CANCELLED means stop transmissions completed, not verified mechanical stop.
 * Failures latch FAULT; no automatic motion retry or automatic fault clear. */
ArmResult_t Arm_Stop(void);
/* Explicit acknowledgement only, NOT proof hardware recovered. Allowed after
 * all stop attempts finish; leaves motion permission false. */
ArmResult_t Arm_ClearFault(void);
ArmStatus_t Arm_GetStatus(void);

#endif

