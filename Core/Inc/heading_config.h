#ifndef HEADING_CONFIG_H
#define HEADING_CONFIG_H
#define HEADING_START_DEG 5.0f
#define HEADING_STOP_DEG 2.0f
#define HEADING_GZ_STABLE 2.0f
/* The installed sensor sends gyro and yaw at about 10 Hz. Allow modest
 * scheduling jitter without treating a normal inter-frame gap as a fault. */
#define JY61_TIMEOUT_MS 150U
#define JY61_RESET_TIMEOUT_MS 300U
#define JY61_RESET_CONFIRM_DEG 2.0f
#define JY61_RESET_TX_TIMEOUT_MS 5U
#define HEADING_KP 0.5f
#define HEADING_KI 0.0f
#define HEADING_KD 0.05f
#define MAX_YAW_CORRECTION_RPM 100.0f
#define MANUAL_HEADING_LIMIT_PERCENT 20
#define ANGLE_ADJUST_STEP_DEG 5.0f
/* Positive mecanum omega lowers raw yaw on this car; heading error uses raw yaw. */
#define HEADING_CORRECTION_SIGN (-1.0f)
#define HEADING_LOG_PERIOD_MS 200U
#define HEADING_TEST_TRANSLATION_RPM 30
#ifndef CAR_HEADING_TEST_MODE
#define CAR_HEADING_TEST_MODE 0U
#endif
#endif
