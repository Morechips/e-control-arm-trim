#ifndef BLUETOOTH_DRIVER_H
#define BLUETOOTH_DRIVER_H
#include "main.h"
#include "car_config.h"
#define BT_CONTROL_FRAME_SIZE 21U
/* Test page: A5 | cmd,dx_mm,close_p,open_p (four LE int16) | sum | 5A.
 * It never refreshes chassis/joystick keepalive. */
#define BT_TRIM_TEST_FRAME_SIZE 11U
/* Simple page: eight packed action bools, an unused byte, dx int16.
 * Leave the byte at the App default zero. Legacy v3 marker 0x54 also works.
 * Keep the byte: removing it makes the shorter frame ambiguous. */
#define BT_TRIM_SIMPLE_FRAME_SIZE 7U
#define BT_TRIM_SIMPLE_MARKER 0U
#define BT_TRIM_SIMPLE_LEGACY_MARKER 0x54U
typedef enum {
    BT_TRIM_BALL = 1U << 0,
    BT_TRIM_HOSTAGE = 1U << 1,
    BT_TRIM_BUCKET = 1U << 2,
    BT_TRIM_MOVE = 1U << 3,
    BT_TRIM_CLOSE = 1U << 4,
    BT_TRIM_OPEN = 1U << 5,
    BT_TRIM_STATUS = 1U << 6,
    BT_TRIM_STOP = 1U << 7
} BtTrimSimpleButton_t;
#define BT_CONTROL_FRAME_EXT_SIZE 23U
#define BT_CONTROL_FRAME_SHOT_SIZE 25U
#define BT_CONTROL_FRAME_LEFT_SIZE 27U
#define BT_SERVO_MODE_MAX 10
#define BT_CONTROL_FRAME_SERVO_MODE_SIZE 29U
#define BT_CONTROL_FRAME_SERVO_BOOL_SIZE 31U
/* Compatibility profiles kept for the existing arm phone project. */
#define BT_ARM_DUAL_FRAME_SIZE 17U
#define BT_ARM_COMBINED_FRAME_EXT_SIZE 29U
#define BT_ARM_COMBINED_FRAME_SHOT_SIZE 31U
#define BT_ARM_COMBINED_FRAME_LEFT_SIZE 33U
/* Teammate 13-short + three arm shorts, and bool + GAP, are both 35 bytes. */
#define BT_ARM_COMBINED_FRAME_SIZE 35U
/* Two 41-byte phone profiles are accepted.  The locally extended (7).pro
 * keeps JOY_X first and appends ARM_CMD/X/Y after its 13 teammate shorts.
 * The teammate (20).pro moves JOY_X after SERVO_MODE.  Both then append the
 * existing GAP int, checksum and tail. */
#define BT_ARM_BOOL_GAP_FRAME_SIZE 41U
#define BT_CONTROL_FRAME_PHONE20_SIZE 41U
#define BT_SERVO_GAP_MARKER (-8)
/* Independent mode-1 button: A5 31 00 31 5A on press, A5 30 00 30 5A
 * on release. These do not refresh chassis control or Bluetooth keepalive. */
#define BT_SERVO_ONE_PRESS_CMD 0x31U
#define BT_SERVO_ONE_RELEASE_CMD 0x30U
/* Active phone .pro wire order: JOY_X, JOY_Y, FORWARD, BACKWARD, STOP,
 * STRAFE_LEFT, STRAFE_RIGHT, RIGHT_90, RIGHT_180, optionally Cam_T. Values
 * are signed little-endian short. The 23-byte packet places Cam_T at bytes
 * 19-20, checksum at 21, tail at 22. The 25-byte packet appends Shot at
 * bytes 21-22, checksum at 23, tail at 24. Absent fields decode as zero.
 * The 27-byte A5 frame requires Cam_T and Shot, then adds LEFT_90 at bytes 23-24,
 * checksum at 25 and tail at 26. The 29-byte frame appends SERVO_MODE at
 * bytes 25-26, checksum at 27 and tail at 28. Modes 0..10 are valid.
 * brake/disable are internal fields absent from the APP packet. */
/* The phone's bool profile packs TB_B..TH_U and RST into two bytes before
 * the 13 shorts. Its final servo_mode short is ignored; bool buttons own this
 * profile. Checksum is byte 29 and tail is byte 30. */
/* The 35-byte frame keeps all 13 existing shorts and appends ARM_CMD,
 * ARM_X and ARM_Y at bytes 27..32; checksum is at 33, tail at 34. */
/* The updated phone .pro also has a 35-byte bool profile: the 31-byte bool
 * layout plus one little-endian int GAP at bytes 29..32. Its saved final
 * servo_mode short is -8, which disambiguates it from the arm frame. */
typedef enum {
    BT_DIRECTION_NONE = 0,
    BT_DIRECTION_FORWARD = 10,
    BT_DIRECTION_BACKWARD = 11,
    BT_DIRECTION_LEFT = 12,
    BT_DIRECTION_RIGHT = 13,
    BT_DIRECTION_ARM_PRESET_NEXT = 14
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
    BT_CONTROL_BUTTON_SHOT = 1U << 9,
    BT_CONTROL_BUTTON_AIM  = 1U << 10,
    BT_SERVO_BUTTON_TH_L = 1U << 11
} BtServoButton_t;
typedef struct {
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
    int16_t servo_mode;
    uint16_t servo_buttons;
    uint8_t aim;
    uint16_t gap_pwm;
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
void Bluetooth_Process(void);
const BtControl_t *Bluetooth_GetControl(void);
uint8_t Bluetooth_IsConnected(void);
uint32_t Bluetooth_GetLastRxTick(void);
uint32_t Bluetooth_GetSequence(void);
const BtArmControl_t *Bluetooth_GetArmControl(void);
uint8_t Bluetooth_ArmIsConnected(void);
uint32_t Bluetooth_GetArmSequence(void);
uint32_t Bluetooth_GetServoOneSequence(void);
/* Runtime protocol diagnostics. Zero means no matching frame since boot. */
uint8_t Bluetooth_GetLastFrameLength(void);
uint8_t Bluetooth_GetLastArmFrameLength(void);
uint8_t Bluetooth_GetLastTestFrameLength(void);
uint32_t Bluetooth_GetTestSequence(void);
uint32_t Bluetooth_GetInvalidFrameCount(void);
uint32_t Bluetooth_GetRxByteCount(void);
void Bluetooth_SetExtended(uint8_t enabled);
uint8_t Bluetooth_IsExtended(void);
typedef void (*BluetoothTextHandler_t)(const char *line, uint32_t arrival_tick);
void Bluetooth_SetTextHandler(BluetoothTextHandler_t handler);
extern volatile uint32_t bluetooth_rx_recoveries;
void Bluetooth_RxCallback(UART_HandleTypeDef *uart);
void Bluetooth_ErrorCallback(UART_HandleTypeDef *uart);
#endif
