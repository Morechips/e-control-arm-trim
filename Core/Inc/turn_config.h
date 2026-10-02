#ifndef TURN_CONFIG_H
#define TURN_CONFIG_H

/* On the installed JY61, a clockwise right turn decreases raw yaw. This
 * observation is independent of the sign used by heading-hold PID output. */
#define TURN_RIGHT_YAW_SIGN (-1.0f)
#define TURN_RIGHT_TARGET_DEG 90.0f
#define TURN_RIGHT_TIMEOUT_MS 10000U
#define TURN_RIGHT_90_RPM 20
#define TURN_LEFT_90_RPM 20
#define TURN_RIGHT_180_RPM 30
#define TURN_APPROACH_DEG 30.0f
#define TURN_TOLERANCE_DEG 5.0f
#define TURN_APPROACH_RPM 30
#define TURN_RIGHT_180_APPROACH_RPM 20
#define TURN_STOP_LEAD_MS 80U
#define TURN_STOP_LEAD_MAX_DEG 15.0f
#define TURN_RATE_FRESH_MS 20U
#define TURN_CORRECTION_MIN_PROGRESS_DEG 2.0f
#define TURN_MAX_CORRECTIONS 3U
#define TURN_WRONG_WAY_DEG 10.0f
#define TURN_PROGRESS_DEG 3.0f
#define TURN_PROGRESS_TIMEOUT_MS 1000U
#define TURN_STOP_TIMEOUT_MS 1000U
#define TURN_STOP_STABLE_MS 100U

#endif
