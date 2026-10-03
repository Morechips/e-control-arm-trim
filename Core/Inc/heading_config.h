#ifndef HEADING_CONFIG_H
#define HEADING_CONFIG_H
#define HEADING_START_DEG 5.0f
#define HEADING_STOP_DEG 2.0f
#define REMOTE_HEADING_START_DEG 1.0f
#define REMOTE_HEADING_STOP_DEG 0.5f
#define REMOTE_HEADING_MAX_RPM 10
/* Driving correction stays mixed with translation unless large drift persists. */
#define REMOTE_MOVING_START_DEG 3.0f
#define REMOTE_MOVING_STOP_DEG 1.5f
#define REMOTE_MOVING_MAX_RPM 4
#define REMOTE_MOVING_BRAKE_DEG 10.0f
#define REMOTE_MOVING_BRAKE_CONFIRM_MS 300U
#define REMOTE_MOVING_BRAKE_MIN_SAMPLES 3U
#define HEADING_GZ_STABLE 2.0f
/* The installed sensor sends gyro and yaw at about 10 Hz. Allow modest
 * scheduling jitter without treating a normal inter-frame gap as a fault. */
#define JY61_TIMEOUT_MS 150U
#define JY61_RESET_TIMEOUT_MS 300U
#define JY61_RESET_CONFIRM_DEG 2.0f
#define JY61_RESET_TX_TIMEOUT_MS 10U
#define HEADING_KP 0.5f
#define HEADING_KI 0.0f
#define HEADING_KD 0.05f
#define MAX_YAW_CORRECTION_RPM 100.0f
#define ANGLE_ADJUST_STEP_DEG 5.0f
/* Positive mecanum omega lowers raw yaw on this car; heading error uses raw yaw. */
#define HEADING_CORRECTION_SIGN (-1.0f)
#define HEADING_LOG_PERIOD_MS 200U
#ifndef CAR_HEADING_TEST_MODE
#define CAR_HEADING_TEST_MODE 0U
#endif
#endif
