#ifndef REMOTE_HEADING_H
#define REMOTE_HEADING_H
#include "heading_control.h"

typedef enum {
    REMOTE_FRONT = 0, REMOTE_RIGHT, REMOTE_BACK, REMOTE_LEFT
} RemoteDirection_t;
typedef enum {
    REMOTE_WAIT_REFERENCE = 0, REMOTE_ALIGNED, REMOTE_ALIGNING,
    REMOTE_DRIFTING, REMOTE_TURNING, REMOTE_NO_IMU, REMOTE_SUSPENDED
} RemoteHeadingPhase_t;
typedef struct {
    RemoteDirection_t direction;
    RemoteHeadingPhase_t phase;
    bool reference_valid, imu_valid;
    float reference_yaw, target, actual, error;
    int16_t correction_rpm;
} RemoteHeadingStatus_t;

/* Foreground-only remote owner. Reset only on a new remote session or when
 * returning from vision; Suspend preserves the reference and target. */
void RemoteHeading_Reset(void);
void RemoteHeading_Suspend(void);
/* Translating selects mild, nonblocking correction. Sustained severe drift
 * returns REMOTE_ALIGNING so the owner stops translation and aligns. */
void RemoteHeading_Update(bool translating);
void RemoteHeading_BeginTurn(void);
void RemoteHeading_CompleteTurn(int8_t clockwise_quarters);
const RemoteHeadingStatus_t *RemoteHeading_GetStatus(void);
void RemoteHeading_Log(void);
#endif
