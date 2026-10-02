#ifndef SERVO_POSE_CONFIG_H
#define SERVO_POSE_CONFIG_H

/* Default travel time: the supplied pose list specifies PWM but no duration. */
#define SERVO_POSE_MOVE_MS 1000U

/* ID 003 PWM values for the eight Bluetooth bool presets. */
#define SERVO_POSE_TB_B_GRIPPER_PWM 2192U
#define SERVO_POSE_TB_M_GRIPPER_PWM 2192U
#define SERVO_POSE_TB_G_GRIPPER_PWM 500U
#define SERVO_POSE_BD_U_GRIPPER_PWM 500U
#define SERVO_POSE_BD_D_GRIPPER_PWM 1800U
#define SERVO_POSE_TH_C_GRIPPER_PWM 1800U
#define SERVO_POSE_TH_G_GRIPPER_PWM 500U
#define SERVO_POSE_TH_U_GRIPPER_PWM 500U

#endif
