#ifndef MECANUM_H
#define MECANUM_H
#include "motor_driver.h"
typedef struct { int16_t m1, m2, m3, m4; } MecanumWheels_t;
/* X-layout mecanum mix in logical wheel-equivalent RPM. +forward is toward
 * the nose, +right is right translation and +heading is clockwise yaw.
 * Output order is M1 RF, M2 LF, M3 LR, M4 RR. */
void Mecanum_Calculate(int16_t forward, int16_t right,
                       int16_t heading_correction, MecanumWheels_t *out);
/* Foreground only, after the caller's enable/brake/link safety gate.
 * HAL_OK means queued; it does not confirm actual motor movement. */
HAL_StatusTypeDef mecanum_drive(int16_t forward, int16_t right,
                                int16_t heading_correction);

/* Optional direction wrappers; not called by the production control loop.
 * Speed uses the existing int16_t RPM unit: nonnegative magnitude, clamped
 * by the existing MOTOR_MAX_RPM limit. Negative input returns HAL_ERROR.
 * Same foreground/enable/safety requirements as mecanum_drive; no auto-enable,
 * heading hold, duration, or retry. Keep calling Motor_Process to dispatch.
 * brake cancels queued motion via Motor_EStopAll, allowing an in-flight frame
 * to finish before stop frames. It does not latch the car-control brake lock.
 */
HAL_StatusTypeDef up(int16_t rpm);
HAL_StatusTypeDef down(int16_t rpm);
HAL_StatusTypeDef left(int16_t rpm);
HAL_StatusTypeDef right(int16_t rpm);
HAL_StatusTypeDef brake(void);
#endif
