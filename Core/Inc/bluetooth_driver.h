#ifndef BLUETOOTH_DRIVER_H
#define BLUETOOTH_DRIVER_H
#include "main.h"
#include "car_config.h"
#define BT_CONTROL_FRAME_SIZE 21U
#define BT_CONTROL_FRAME_EXT_SIZE 23U
#define BT_CONTROL_FRAME_SHOT_SIZE 25U
#define BT_CONTROL_FRAME_LEFT_SIZE 27U
#define BT_CONTROL_FRAME_RESERVED_SIZE 29U
#define BT_CONTROL_FRAME_SERVO_BOOL_SIZE 31U
#define BT_ARM_COMBINED_FRAME_SIZE 35U
#define BT_CONTROL_FRAME_PHONE20_SIZE 41U
#define BT_MAX_FRAME_SIZE BT_CONTROL_FRAME_PHONE20_SIZE
#define BT_TRIM_FRAME_SIZE 7U
#define BT_TRIM_LEGACY_FRAME_SIZE 11U
#define BT_GAP_FRAME_MARKER (-8)
/* SHOT and AIM are the tenth and eleventh packed bools. Neither is a servo
 * pose button; SHOT shares the shooting request with the older Shot short. */
#define BT_CONTROL_BOOL_SHOT_BIT (1U << 9)
#define BT_CONTROL_BOOL_AIM_BIT (1U << 10)
#define BT_CONTROL_BOOL_JOG_BIT (1U << 12)
#define BT_CONTROL_BOOL_CLOSE_BIT (1U << 13)
#define BT_CONTROL_BOOL_OPEN_BIT (1U << 14)
#define BT_CONTROL_BOOL_ARM_STOP_BIT (1U << 15)
/* Independent reference button: A5 31 00 31 5A on press, A5 30 00 30 5A
 * on release. These do not refresh chassis control or Bluetooth keepalive.
 * With ARM_TRIM_ENABLE=1, five-byte PID/reference commands wait for the next
 * A5 header or an inter-byte gap greater than 100 ms to resolve trim prefixes. */
#define BT_SERVO_ONE_PRESS_CMD 0x31U
#define BT_SERVO_ONE_RELEASE_CMD 0x30U
/* Legacy phone .pro wire order: JOY_X, JOY_Y, FORWARD, BACKWARD, STOP,
 * STRAFE_LEFT, STRAFE_RIGHT, RIGHT_90, RIGHT_180, optionally Cam_T. Values
 * are signed little-endian short. The 23-byte packet places Cam_T at bytes
 * 19-20, checksum at 21, tail at 22. The 25-byte packet appends Shot at
 * bytes 21-22 (0=remote, 1=shooting), checksum at 23, tail at 24.
 * Absent fields decode as zero.
 * The 27-byte A5 frame requires Cam_T and Shot, then adds LEFT_90 at bytes 23-24,
 * checksum at 25 and tail at 26. The 29-byte frame reserves bytes 25-26;
 * they are ignored. Checksum is at 27 and tail at 28.
 * brake/disable are internal fields absent from the APP packet. */
/* The phone's bool profile packs TB_B..TH_U, RST, SHOT, AIM and TH_L into two
 * bytes before the 13 shorts. Its final short is reserved and ignored.
 * Checksum is byte 29 and tail is byte 30. */
/* The 35-byte frame keeps all 13 existing shorts and appends ARM_CMD,
 * ARM_X and ARM_Y at bytes 27..32; checksum is at 33, tail at 34. */
/* The updated phone .pro also has a 35-byte bool profile: the 31-byte bool
 * layout plus one little-endian int GAP at bytes 29..32. The reserved short
 * must be -8 to distinguish it from the arm frame. */
/* (20).pro: two bool bytes, then 16 shorts: JOY_Y, forward, backward,
 * stop, strafe_left, strafe_right, right_90, right_180, Cam_T, Shot,
 * LEFT_90, servo_mode, joy_x, armcmd, armx, army. GAP is at 35..38,
 * checksum at 39, tail at 40. With ARM_TRIM_ENABLE=1, bools 12..15 are
 * WT/CLOSE/OPEN/ARM_STOP; short11 is ARM_STATUS and short14 is ydnum direction.
 * short13 is PID_CMD (0..11), short15=1 is REFERENCE. Existing legacy
 * armcmd/army values still parse but have no action outside these commands. */
typedef enum {
    BT_DIRECTION_NONE = 0,
    BT_DIRECTION_FORWARD = 10,
    BT_DIRECTION_BACKWARD = 11,
    BT_DIRECTION_LEFT = 12,
    BT_DIRECTION_RIGHT = 13
} BtDirection_t;
typedef enum {
    BT_ARM_REMOTE_ENTER = 20,
    BT_ARM_AUTHORIZE = 21,
    BT_ARM_STOP = 22,
    BT_ARM_EXIT = 23,
    BT_ARM_GRIP_OPEN = 24,
    BT_ARM_GRIP_BALL = 25
} BtArmButton_t;
typedef enum {
    BT_SERVO_BUTTON_TB_B = 1U << 0,
    BT_SERVO_BUTTON_TB_M = 1U << 1,
    BT_SERVO_BUTTON_TB_G = 1U << 2,
    BT_SERVO_BUTTON_BD_U = 1U << 3,
    BT_SERVO_BUTTON_BD_D = 1U << 4,
    BT_SERVO_BUTTON_TH_C = 1U << 5,
    BT_SERVO_BUTTON_TH_G = 1U << 6,
    BT_SERVO_BUTTON_TH_U = 1U << 7,
    BT_SERVO_BUTTON_RST  = 1U << 8,
    BT_SERVO_BUTTON_TH_L = 1U << 11
} BtServoButton_t;
typedef struct {
    /* Legacy wire axes remain validated signed shorts; no runtime action. */
    int16_t joy_x;
    int16_t joy_y;
    int16_t forward;
    int16_t backward;
    int16_t stop;
    int16_t strafe_left;
    int16_t strafe_right;
    int16_t right_90;
    int16_t right_180;
    int16_t Cam_T;
    int16_t Shot;
    int16_t left_90;
    uint16_t servo_buttons;
    uint8_t aim;
    uint16_t gap_pwm;
    uint8_t trim_buttons;
    int16_t trim_direction;
    uint8_t pid_command, reference_pressed;
    int16_t brake;
    int16_t disable;
} BluetoothControlFrame;
typedef struct {
    int16_t direction;
    int16_t brake;
    int16_t disable;
    int16_t x;
    int16_t y;
    int16_t arm_x;
    int16_t arm_y;
    uint32_t last_rx_tick;
    uint8_t valid;
} BtArmControl_t;
typedef struct {
    BluetoothControlFrame frame;
    uint32_t last_rx_tick;
    uint8_t valid;
} BtControl_t;
void Bluetooth_Init(void);
void Bluetooth_ProvisionName(void);
void Bluetooth_Process(void);
void Bluetooth_DispatchServoActions(uint8_t physical_aim_press);
uint32_t Bluetooth_GetAimSequence(void);
const BtControl_t *Bluetooth_GetControl(void);
uint8_t Bluetooth_IsConnected(void);
uint32_t Bluetooth_GetLastRxTick(void);
uint32_t Bluetooth_GetSequence(void);
const BtArmControl_t *Bluetooth_GetArmControl(void);
uint8_t Bluetooth_ArmIsConnected(void);
uint32_t Bluetooth_GetArmSequence(void);
uint32_t Bluetooth_GetServoOneSequence(void);
/* Independent trim frames never renew the chassis input lease. */
uint32_t Bluetooth_GetTrimSequence(void);
uint32_t Bluetooth_GetTrimPressCount(void);
void Bluetooth_SetExtended(uint8_t enabled);
uint8_t Bluetooth_IsExtended(void);
typedef void (*BluetoothTextHandler_t)(const char *line, uint32_t arrival_tick);
void Bluetooth_SetTextHandler(BluetoothTextHandler_t handler);
extern volatile uint32_t bluetooth_rx_recoveries;
void Bluetooth_RxCallback(UART_HandleTypeDef *uart);
void Bluetooth_ErrorCallback(UART_HandleTypeDef *uart);
#endif
