#ifndef ARM_TRIM_SERVICE_H
#define ARM_TRIM_SERVICE_H

#include "arm_trim.h"
#include "servo.h"

typedef enum {
    ARM_TRIM_PROFILE_BALL = 0,
    ARM_TRIM_PROFILE_HOSTAGE,
    ARM_TRIM_PROFILE_BUCKET,
    ARM_TRIM_PROFILE_COUNT,
    ARM_TRIM_PROFILE_NONE = ARM_TRIM_PROFILE_COUNT
} ArmTrimServiceProfile_t;

typedef enum {
    ARM_TRIM_SERVICE_IDLE = 0,
    ARM_TRIM_SERVICE_PROFILE,
    ARM_TRIM_SERVICE_REFERENCE,
    ARM_TRIM_SERVICE_MOTION,
    ARM_TRIM_SERVICE_GRIP,
    ARM_TRIM_SERVICE_STOPPING,
    ARM_TRIM_SERVICE_FAULT,
    ARM_TRIM_SERVICE_FIXED
} ArmTrimServiceState_t;

/* This adapter's references and waits are project policy, separate from the
 * reusable geometry/planner. Init copies the configuration. No GPIO or input
 * transport is read here; the caller owns chassis and control-link interlocks. */
typedef struct {
    ArmTrimConfig_t core;
    uint16_t references[ARM_TRIM_PROFILE_COUNT][3];
    uint16_t profile_move_ms, profile_guard_ms;
    uint16_t grip_move_ms, grip_guard_ms;
    /* Zero/zero preserves the original 500..2500 protocol range. */
    uint16_t grip_min_pwm, grip_max_pwm;
} ArmTrimServiceConfig_t;

typedef struct {
    ArmTrimServiceState_t state;
    ArmTrimResult_t last_request;
    ServoStatus_t transport_result;
    ArmTrimServiceProfile_t profile;
    bool owns_motion, busy, stop_failed, pose_pending;
    ArmTrimStatus_t core;
    ServoTransferStatus_t transfer;
} ArmTrimServiceStatus_t;

/* One arm/Servo lease. The clock and its user storage must remain alive.
 * Reinitialization is refused while the service owns Servo. */
ArmTrimResult_t ArmTrimService_Init(const ArmTrimServiceConfig_t *config,
                                 uint32_t (*now)(void *user), void *user);
ArmTrimResult_t ArmTrimService_ReadyProfile(ArmTrimServiceProfile_t profile,
                                         bool synchronize);
/* All fixed poses send only 000..002. The three reference poses use the
 * installation configuration. Other poses use the existing Servo table,
 * enforcing joint limits before any send. A jog can decelerate into one
 * queued pose; other busy motions reject new poses without replay. */
ArmTrimResult_t ArmTrimService_RunPreset(ServoCode preset);
/* Begin sends nothing. parked_stable is the caller's explicit assertion that
 * actual 000..002 positions are stable and the chassis is parked. */
ArmTrimResult_t ArmTrimService_Begin(const uint16_t positions[3], bool parked_stable);
ArmTrimResult_t ArmTrimService_MoveRelativeX(float dx_mm);
ArmTrimResult_t ArmTrimService_StartJog(int direction);
ArmTrimResult_t ArmTrimService_ReleaseJog(void);
ArmTrimResult_t ArmTrimService_Cancel(void);
/* End requests cancellation when busy; the lease is released only after all
 * tracked stops and any remaining UART transfer finish. */
ArmTrimResult_t ArmTrimService_End(void);
ArmTrimResult_t ArmTrimService_ClearFault(void);
/* Only channel 003, also available without a trim reference. Normal completion
 * keeps any existing planner origin; cancellation invalidates it. */
ArmTrimResult_t ArmTrimService_Grip(uint16_t pwm);
void ArmTrimService_Process(void);
ArmTrimServiceStatus_t ArmTrimService_GetStatus(void);
bool ArmTrimService_IsBusy(void);
bool ArmTrimService_OwnsMotion(void);

#endif
