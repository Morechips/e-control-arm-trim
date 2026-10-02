#ifndef HEADING_CONTROL_H
#define HEADING_CONTROL_H
#include "jy61.h"
typedef enum {
    HEADING_WAIT_REFERENCE, HEADING_HOLD, HEADING_CORRECTING,
    HEADING_ANGLE_ADJUST, HEADING_FAULT
} HeadingState;
typedef enum { HEADING_NO_REFERENCE, HEADING_IMU_LOST,
               HEADING_INHIBITED, HEADING_NO_FAULT } HeadingFault;
typedef struct {
    float kp, ki, kd;
} HeadingPIDParameters;
typedef struct {
    HeadingState state;
    HeadingFault fault;
    bool heading_hold, reference_valid, angle_adjusting;
    float yaw_zero, target, actual, yaw_error, omega_correction;
    int16_t omega_final;
} HeadingStatus;
void Heading_Init(void);
/* End the current manual motion segment. The next permitted motion captures
 * the current IMU yaw as its own heading reference. */
void Heading_ClearReference(void);
/* Foreground only. Returns a bounded wheel-equivalent RPM yaw correction.
 * Invalid/stale IMU data always returns zero. */
int16_t Heading_Update(bool motion_allowed);
const HeadingStatus *Heading_GetStatus(void);
/* Foreground only. One RAM parameter set, reset to defaults by Heading_Init. */
const HeadingPIDParameters *Heading_GetPID(void);
float Heading_GetIntegralOutput(void);
bool Heading_SetPID(float kp, float ki, float kd);
/* Set the current raw yaw as the software 0-degree reference. */
bool Heading_RequestReference(void);
/* Absolute and relative target-yaw commands use the same PID as heading hold. */
bool Heading_SetTarget(float target_degrees);
bool Heading_AdjustTarget(float delta_degrees);
/* applied_omega is the capped value actually passed to mecanum_drive. */
void Heading_Log(int16_t vx, int16_t vy, int16_t applied_omega);
#endif
