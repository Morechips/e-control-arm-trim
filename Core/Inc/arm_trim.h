#ifndef ARM_TRIM_H
#define ARM_TRIM_H

#include "arm_kinematics.h"
#include "arm_collision.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ARM_TRIM_MAX_SEGMENTS 512U

typedef enum {
    ARM_TRIM_OK = 0, ARM_TRIM_INVALID, ARM_TRIM_BUSY, ARM_TRIM_REFERENCE_REQUIRED,
    ARM_TRIM_OUT_OF_RANGE, ARM_TRIM_PATH_INVALID, ARM_TRIM_TRANSPORT,
    ARM_TRIM_SERVICE_TIMEOUT, ARM_TRIM_FAULT_LATCHED, ARM_TRIM_LINK_TIMEOUT
} ArmTrimResult_t;

typedef enum {
    ARM_TRIM_IDLE = 0, ARM_TRIM_READY, ARM_TRIM_MOVING, ARM_TRIM_SETTLING,
    ARM_TRIM_COMPLETE_ESTIMATED, ARM_TRIM_STOPPING, ARM_TRIM_CANCELLED, ARM_TRIM_FAULT
} ArmTrimState_t;

/* Foreground callbacks. send addresses exactly three synchronized joints;
 * stop addresses one joint index 0..2. Without the optional async extension,
 * success means transmission finished synchronously, never mechanical arrival.
 * now must include time spent inside send/stop. No allocation or HAL in core. */
typedef struct {
    bool (*send)(void *user, const uint16_t positions[3], uint16_t move_ms);
    bool (*stop)(void *user, unsigned joint_index);
    uint32_t (*now)(void *user);
    void (*lateral_report)(void *user, float error_mm);
    void *user;
} ArmTrimIO_t;

typedef enum {
    ARM_TRIM_TX_PENDING = 0, ARM_TRIM_TX_COMPLETE, ARM_TRIM_TX_FAILED
} ArmTrimTxState_t;

/* Optional asynchronous extension, using the original IO's user pointer.
 * A successful send/stop accepts one transfer. COMPLETE reports its actual
 * hardware start and transmit-complete ticks; PENDING timestamps are ignored.
 * cancel_pending removes queued motion without aborting an active wire frame.
 * The canceled transfer must subsequently poll COMPLETE or FAILED. */
typedef struct {
    ArmTrimTxState_t (*poll)(void *user, uint32_t *started, uint32_t *completed);
    void (*cancel_pending)(void *user);
} ArmTrimAsyncIO_t;

typedef struct {
    ArmKinematicsGeometry_t geometry;
    ArmServoCalibration_t calibration[3];
    /* Disabled by DefaultConfig. Caller supplies the local obstacle and
     * physical envelopes when migrating; all preflight edges use this model. */
    ArmCollisionModel_t collision;
    float search_mm, enabled_min_mm, enabled_max_mm;
    float max_segment_mm, speed_mm_s, acceleration_mm_s2;
    float interpolation_position_tolerance_mm, interpolation_angle_tolerance_rad;
    float min_elbow_sine;
    uint16_t max_joint_delta;
    uint16_t update_period_ms, dispatch_lateness_ms;
    uint32_t settle_ms, service_timeout_ms;
} ArmTrimConfig_t;

typedef struct {
    ArmTrimState_t state;
    ArmTrimResult_t error;
    bool reference_valid, stop_failed;
    float model_min_mm, model_max_mm, enabled_min_mm, enabled_max_mm;
    float offset_mm, target_offset_mm;
    ArmPose2D_t origin;
    uint16_t estimated_position[3], target_position[3];
    size_t segment_index, segment_count;
    bool jogging;
    int8_t jog_direction;
} ArmTrimStatus_t;

typedef struct {
    uint16_t position[3], move_ms;
    float offset_mm, end_speed_mm_s;
} ArmTrimSegment_t;

/* Caller-owned context. Do not edit it or reinitialize it during a session.
 * Config and IO are copied; callback user storage must remain alive. */
typedef struct {
    ArmTrimConfig_t config;
    ArmTrimIO_t io;
    ArmTrimAsyncIO_t async_io;
    ArmTrimStatus_t status;
    ArmTrimSegment_t segments[ARM_TRIM_MAX_SEGMENTS];
    ArmElbowBranch_t branch;
    uint32_t segment_tick, service_tick, dispatch_tick, tx_tick;
    unsigned stop_index;
    bool segment_sent, configured, jog_release_requested;
    bool tx_pending, stop_waiting;
} ArmTrim_t;

/* Defaults cover trajectory only; caller supplies geometry and calibration. */
void ArmTrim_DefaultConfig(ArmTrimConfig_t *config);
ArmTrimResult_t ArmTrim_Init(ArmTrim_t *trim, const ArmTrimConfig_t *config,
                            const ArmTrimIO_t *io);
/* Set while idle. NULL restores the original synchronous callback contract. */
ArmTrimResult_t ArmTrim_SetAsyncIO(ArmTrim_t *trim, const ArmTrimAsyncIO_t *async);
/* Caller asserts actual arm is at positions, stable, and chassis parked.
 * This only establishes an estimate; it sends no bytes. A failed replacement
 * invalidates the old reference, except when rejected as busy/fault-latched. */
ArmTrimResult_t ArmTrim_Synchronize(ArmTrim_t *trim, const uint16_t positions[3]);
ArmTrimResult_t ArmTrim_MoveRelativeX(ArmTrim_t *trim, float dx_mm);
/* Held-button motion: preflight to the enabled boundary on a timed path.
 * Release is a normal, decelerated finish that preserves the estimate;
 * Cancel is the hard-stop path and invalidates the reference. */
ArmTrimResult_t ArmTrim_StartJog(ArmTrim_t *trim, int direction);
ArmTrimResult_t ArmTrim_ReleaseJog(ArmTrim_t *trim);
void ArmTrim_Process(ArmTrim_t *trim);
ArmTrimResult_t ArmTrim_Cancel(ArmTrim_t *trim);
ArmTrimResult_t ArmTrim_Exit(ArmTrim_t *trim);
ArmTrimResult_t ArmTrim_ClearFault(ArmTrim_t *trim);
ArmTrimStatus_t ArmTrim_GetStatus(const ArmTrim_t *trim);
void ArmTrim_ReportLateral(ArmTrim_t *trim, float error_mm);

#endif
