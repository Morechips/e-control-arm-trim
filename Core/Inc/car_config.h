#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H
#define MOTOR_MAX_RPM 500
/* Mission and route speed modes; independent of the PD10 diagnostic speed. */
#define SPEED_SLOW_RPM 100
#define SPEED_MEDIUM_RPM 133
#define SPEED_FULL_RPM 167
#define SPEED_NORMAL_RPM SPEED_FULL_RPM
#define SPEED_PRECISE_RPM SPEED_SLOW_RPM
#define MAXICAM_ALIGN_CONFIRM_FRAMES 3U
#define MAXICAM_TARGET_LOST_FRAMES 5U
#define MOTOR_ACC 10U
#define JOY_RANGE 1000 /* Wire validation; JOY fields are inactive. */
/* Bluetooth four-direction button translation. */
#define BLUETOOTH_TEST_MOVE_RPM 35
/* No measured motor stiction threshold: zero preserves existing low speeds. */
#define MOTOR_DEADZONE_RPM 0
#ifndef CAR_HEADING_TEST_MODE
#define CAR_HEADING_TEST_MODE 0U
#endif
#ifndef CAR_MECANUM_TEST_MODE
#define CAR_MECANUM_TEST_MODE 0U
#endif
/* Bench-only physical inputs (PE0 vision, PC1 shot, PE4 servo AIM, PB8 laser,
 * PD10/PD11/PD14/PD15). The production image keeps only the single start key
 * (start_button.h); set this to 1 to compile the test inputs back in. */
#ifndef CAR_TEST_INPUTS_ENABLE
#define CAR_TEST_INPUTS_ENABLE 0U
#endif
#define CAR_MECANUM_TEST_RPM 30
#define CAR_MECANUM_TEST_DURATION_MS 1000U
#define CAR_CONTROL_PERIOD_MS 20U
#define BT_FRAME_GAP_TIMEOUT_MS 100U
#ifndef BT_FAILSAFE_TIMEOUT_MS
#define BT_FAILSAFE_TIMEOUT_MS 500U
#endif
#define MOTOR_TX_TIMEOUT_MS 20U
#define MOTOR_FRAME_GAP_MS 2U
#define CAR_UART_BRIDGE_TEST 0U
/* USART2 belongs to JY61; USART1 carries 5 Hz heading diagnostics. */
#define CAR_USART2_U1_BRIDGE 0U
#ifndef CAR_PD10_STANDALONE_TEST
#define CAR_PD10_STANDALONE_TEST 0U
#endif
#define CAR_PD10_FORWARD_RPM 100
#define CAR_BOOT_AUTO_ENABLE 1U
#if CAR_MECANUM_TEST_MODE && (CAR_PD10_STANDALONE_TEST || CAR_UART_BRIDGE_TEST)
#error Mecanum_test_requires_Bluetooth_safety_control
#endif
#if (CAR_PD10_STANDALONE_TEST || CAR_MECANUM_TEST_MODE || CAR_HEADING_TEST_MODE) && !CAR_TEST_INPUTS_ENABLE
#error Car_test_modes_require_CAR_TEST_INPUTS_ENABLE
#endif
#endif
