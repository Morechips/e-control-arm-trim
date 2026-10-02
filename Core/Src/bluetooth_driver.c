#include "bluetooth_driver.h"
#include "serial_io.h"
#include "pid_tuner.h"
#include <stdio.h>
#include <string.h>

#define BT_HEADER 0xA5U
#define BT_TAIL 0x5AU
#define BT_PID_FRAME_SIZE 5U
#define BT_PAYLOAD_SIZE 18U
#define BT_EXT_PAYLOAD_SIZE 20U
#define BT_SHOT_PAYLOAD_SIZE 22U
#define BT_LEFT_PAYLOAD_SIZE 24U
#define BT_SERVO_MODE_PAYLOAD_SIZE 26U
#define BT_TRACE_PERIOD_MS 500U

static SerialRx rx;
static BtControl_t control;
static BtArmControl_t arm_control;
static uint8_t frame[BT_ARM_BOOL_GAP_FRAME_SIZE];
static uint8_t used;
static uint8_t raw_bytes[BT_ARM_BOOL_GAP_FRAME_SIZE], raw_used;
static uint8_t last_traced_bytes[BT_ARM_BOOL_GAP_FRAME_SIZE], last_traced_len, have_trace;
static uint32_t sequence, error_log_tick, last_byte_tick, trace_tick;
static uint32_t arm_sequence, text_start_tick;
static uint32_t servo_one_sequence;
static uint32_t invalid_frame_count;
static volatile uint32_t rx_byte_count;
static uint32_t rx_diagnostic_tick;
static uint8_t servo_one_pressed;
static BluetoothTextHandler_t text_handler;
static char text_line[80];
static uint8_t text_used, text_bad, arm_mode;
static uint8_t last_valid_frame_length, last_arm_frame_length;
static int16_t previous_arm_button;
static int16_t previous_test_command;
static uint8_t last_test_frame_length;
static uint32_t test_sequence;
static uint8_t previous_simple_buttons;
/* Retain these RAM diagnostics for SWD reads even without a debug UART. */
static volatile uint8_t last_simple_action;
static volatile uint32_t simple_press_count;
volatile uint32_t bluetooth_rx_recoveries;

static uint8_t TraceRaw(const uint8_t *bytes, uint8_t length);

static uint8_t SimpleByteValid(uint8_t value)
{
    return (uint8_t)(value == BT_TRIM_SIMPLE_MARKER ||
                     value == BT_TRIM_SIMPLE_LEGACY_MARKER);
}

static void TraceReception(void)
{
    char line[80];
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - rx_diagnostic_tick) < 1000U || !Debug_CanLog(2U)) return;
    rx_diagnostic_tick = now;
    (void)snprintf(line, sizeof(line), "[BT RX] bytes=%lu test=%lu invalid=%lu\r\n",
        (unsigned long)rx_byte_count, (unsigned long)test_sequence,
        (unsigned long)invalid_frame_count);
    Debug_Log(line);
    (void)snprintf(line, sizeof(line), "[BT RX] base=%lu test_len=%u recover=%lu\r\n",
        (unsigned long)sequence, (unsigned)last_test_frame_length,
        (unsigned long)bluetooth_rx_recoveries);
    Debug_Log(line);
}

static uint8_t ShortFrameComplete(void)
{
    uint8_t next;
    /* A malformed zero-byte trim frame can have a valid 5-byte PID prefix.
     * Wait for the next header or the existing 100ms inter-byte deadline;
     * never execute that prefix before its possible continuation arrives. */
    if (used > BT_PID_FRAME_SIZE) return (uint8_t)(frame[BT_PID_FRAME_SIZE] == BT_HEADER);
    if (Serial_Peek(&rx, &next)) return (uint8_t)(next == BT_HEADER);
    return (uint8_t)((uint32_t)(HAL_GetTick() - last_byte_tick) > BT_FRAME_GAP_TIMEOUT_MS);
}

static int16_t bt_read_short(const uint8_t *p)
{
    uint16_t value = (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
    return (int16_t)((value <= INT16_MAX) ? (int32_t)value : (int32_t)value - 65536);
}

static uint32_t bt_read_uint32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint8_t SwitchValid(int16_t value)
{
    return (uint8_t)(value == 0 || value == 1);
}

static uint8_t ArmDirectionValid(int16_t value)
{
    return (uint8_t)(value == 0 || value == 1 ||
        (value >= 10 && value <= 14) || (value >= 20 && value <= 25));
}

static uint8_t AxisValid(int16_t value)
{
    return (uint8_t)(value >= -JOY_RANGE && value <= JOY_RANGE);
}

static uint8_t TestGripperValid(int16_t value)
{
    return (uint8_t)(value == 0 || (value >= 500 && value <= 2500));
}

static uint8_t DecodeTrimTest(const uint8_t *p)
{
    static const char *const fixed_commands[] = {
        "", "@BENCH PREP BALL", "@BENCH PREP HOSTAGE", "@BENCH PREP BUCKET",
        "@ARM TRIM BEGIN BALL", "@ARM TRIM BEGIN HOSTAGE", "@ARM TRIM BEGIN BUCKET"
    };
    int16_t command = bt_read_short(&p[1]);
    int16_t dx = bt_read_short(&p[3]);
    int16_t close_p = bt_read_short(&p[5]);
    int16_t open_p = bt_read_short(&p[7]);
    uint8_t i, sum = 0U;
    char text[48];
    for (i = 1U; i <= 8U; ++i) sum = (uint8_t)(sum + p[i]);
    if (p[10] != BT_TAIL || p[9] != sum || command < 0 || command > 13 ||
        dx < -150 || dx > 150 || !TestGripperValid(close_p) || !TestGripperValid(open_p))
        return 0U;
    used = 0U;
    raw_used = 0U;
    if ((uint32_t)(HAL_GetTick() - last_byte_tick) > BT_FAILSAFE_TIMEOUT_MS) return 1U;
    (void)TraceRaw(p, BT_TRIM_TEST_FRAME_SIZE);
    last_test_frame_length = BT_TRIM_TEST_FRAME_SIZE;
    ++test_sequence;
    if (command == previous_test_command) return 1U;
    previous_test_command = command;
    if (command == 0 || text_handler == NULL) return 1U;
    if (command <= 6) text_handler(fixed_commands[command], last_byte_tick);
    else if (command == 7) {
        (void)snprintf(text, sizeof(text), "@ARM TRIM DX %d", (int)dx);
        text_handler(text, last_byte_tick);
    } else if (command == 8) text_handler("@ARM TRIM END", last_byte_tick);
    else if (command == 9 || command == 10) {
        (void)snprintf(text, sizeof(text), "@BENCH GRIP %d", (int)(command == 9 ? close_p : open_p));
        text_handler(text, last_byte_tick);
    } else if (command == 11) text_handler("@ARM TRIM STATUS", last_byte_tick);
    else if (command == 12) text_handler("@ARM STOP", last_byte_tick);
    else text_handler("@ARM TRIM CLEAR", last_byte_tick);
    return 1U;
}

static uint8_t DecodeTrimSimple(const uint8_t *p)
{
    uint8_t buttons = p[1], pressed, previous = previous_simple_buttons, sum = 0U, i;
    int16_t dx = bt_read_short(&p[3]);
    char text[40];
    for (i = 1U; i <= 4U; ++i) sum = (uint8_t)(sum + p[i]);
    if (!SimpleByteValid(p[2]) || p[5] != sum || p[6] != BT_TAIL ||
        ((dx < -150 || dx > 150) && !(buttons & BT_TRIM_STOP))) return 0U;
    used = raw_used = 0U;
    if ((uint32_t)(HAL_GetTick() - last_byte_tick) > BT_FAILSAFE_TIMEOUT_MS) return 1U;
    (void)TraceRaw(p, BT_TRIM_SIMPLE_FRAME_SIZE);
    last_test_frame_length = BT_TRIM_SIMPLE_FRAME_SIZE;
    ++test_sequence;
    pressed = (uint8_t)(buttons & (uint8_t)~previous_simple_buttons);
    previous_simple_buttons = buttons;
    if (text_handler == NULL) return 1U;
    if (pressed != 0U) {
        last_simple_action = pressed;
        ++simple_press_count;
    }
    /* STOP dominates conflicts, even when the numeric input is malformed.
     * Holding STOP suppresses every other action until it is released. */
    if (buttons & BT_TRIM_STOP) {
        if (pressed & BT_TRIM_STOP) text_handler("@ARM STOP", last_byte_tick);
        return 1U;
    }
    /* A held trim button is a dead-man jog, rather than another finite DX.
     * Every valid held frame refreshes its watchdog.  Releases are delivered
     * before other action edges, and KEEP never starts a rejected/old press. */
    if ((previous & BT_TRIM_MOVE) && buttons != BT_TRIM_MOVE)
        text_handler("@ARM TRIM RELEASE", last_byte_tick);
    if ((buttons & (uint8_t)(buttons - 1U)) != 0U) {
        if (pressed != 0U) text_handler("@BENCH BUTTON_CONFLICT", last_byte_tick);
        return 1U;
    }
    if (buttons == BT_TRIM_MOVE) {
        (void)snprintf(text, sizeof(text), "@ARM TRIM %s %d",
                       (pressed & BT_TRIM_MOVE) ? "JOG" : "KEEP",
                       dx > 0 ? 1 : dx < 0 ? -1 : 0);
        text_handler(text, last_byte_tick);
        return 1U;
    }
    if (pressed == 0U) return 1U;
    switch (pressed) {
    case BT_TRIM_BALL: text_handler("@BENCH READY BALL", last_byte_tick); break;
    case BT_TRIM_HOSTAGE: text_handler("@BENCH READY HOSTAGE", last_byte_tick); break;
    case BT_TRIM_BUCKET: text_handler("@BENCH READY BUCKET", last_byte_tick); break;
    case BT_TRIM_CLOSE: text_handler("@BENCH CLOSE", last_byte_tick); break;
    case BT_TRIM_OPEN: text_handler("@BENCH OPEN", last_byte_tick); break;
    case BT_TRIM_STATUS: text_handler("@ARM TRIM STATUS", last_byte_tick); break;
    default: break;
    }
    return 1U;
}

static uint8_t ArmPayloadValid(const uint8_t *p)
{
    return (uint8_t)(ArmDirectionValid(bt_read_short(&p[1])) &&
                     SwitchValid(bt_read_short(&p[3])) &&
                     SwitchValid(bt_read_short(&p[5])) &&
                     AxisValid(bt_read_short(&p[7])) &&
                     AxisValid(bt_read_short(&p[9])) &&
                     AxisValid(bt_read_short(&p[11])) &&
                     AxisValid(bt_read_short(&p[13])));
}

static uint8_t IsArmCommand(int16_t value)
{
    return (uint8_t)(value >= 20 && value <= 25);
}

static void ArmButton(int16_t value, uint32_t tick)
{
    static const char *const commands[] = {
        "@ARM REMOTE_ENTER", "@ARM ARM", "@ARM STOP", "@ARM EXIT",
        "@ARM GRIP 1500", "@ARM GRIP 900"
    };
    if (value == previous_arm_button || text_handler == NULL ||
        (uint32_t)(HAL_GetTick() - tick) > BT_FAILSAFE_TIMEOUT_MS) return;
    previous_arm_button = value;
    text_handler(commands[(unsigned)(value - 20)], tick);
}

static uint8_t TextByte(uint8_t byte, uint32_t tick)
{
    if (text_used && (uint32_t)(tick - last_byte_tick) > BT_FRAME_GAP_TIMEOUT_MS)
        text_used = text_bad = 0U;
    if (byte == BT_HEADER) {
        text_used = text_bad = 0U;
        return 0U;
    }
    if (!text_used) {
        if (byte != '@' || text_handler == NULL) return 0U;
        text_start_tick = tick;
        text_line[text_used++] = '@';
    } else if (byte == '\n') {
        if (text_used > 1U && text_line[text_used - 1U] == '\r') --text_used;
        text_line[text_used] = '\0';
        if (!text_bad &&
            (uint32_t)(HAL_GetTick() - text_start_tick) <= BT_FAILSAFE_TIMEOUT_MS)
            text_handler(text_line, tick);
        text_used = text_bad = 0U;
        last_byte_tick = tick;
        return 2U;
    } else {
        if (byte < 32U && byte != '\r') text_bad = 1U;
        if (byte > 126U || (text_used && text_line[text_used - 1U] == '\r'))
            text_bad = 1U;
        if (text_used < sizeof(text_line) - 1U) text_line[text_used++] = (char)byte;
        else text_bad = 1U;
    }
    last_byte_tick = tick;
    return 1U;
}

static uint8_t PayloadValid(const uint8_t *p, uint8_t length)
{
    uint8_t i;
    for (i = 0U; i < 7U; ++i)
        if (!SwitchValid(bt_read_short(&p[5U + 2U * i]))) return 0U;
    return (uint8_t)((length < BT_CONTROL_FRAME_EXT_SIZE ||
                      SwitchValid(bt_read_short(&p[19]))) &&
                     (length < BT_CONTROL_FRAME_SHOT_SIZE ||
                      SwitchValid(bt_read_short(&p[21]))) &&
                     (length < BT_CONTROL_FRAME_LEFT_SIZE ||
                      SwitchValid(bt_read_short(&p[23]))) &&
                     (length < BT_CONTROL_FRAME_SERVO_MODE_SIZE ||
                      (bt_read_short(&p[25]) >= 0 &&
                       bt_read_short(&p[25]) <= BT_SERVO_MODE_MAX)) &&
                     bt_read_short(&p[1]) >= -JOY_RANGE &&
                     bt_read_short(&p[1]) <= JOY_RANGE &&
                     bt_read_short(&p[3]) >= -JOY_RANGE &&
                     bt_read_short(&p[3]) <= JOY_RANGE);
}

static void DecodeControl(BluetoothControlFrame *c, const uint8_t *p,
                          uint8_t length)
{
    c->joy_x = bt_read_short(&p[1]);
    c->joy_y = bt_read_short(&p[3]);
    c->forward = bt_read_short(&p[5]);
    c->backward = bt_read_short(&p[7]);
    c->stop = bt_read_short(&p[9]);
    c->strafe_left = bt_read_short(&p[11]);
    c->strafe_right = bt_read_short(&p[13]);
    c->right_90 = bt_read_short(&p[15]);
    c->right_180 = bt_read_short(&p[17]);
    c->Cam_T = length >= BT_CONTROL_FRAME_EXT_SIZE ? bt_read_short(&p[19]) : 0;
    c->Shot = length >= BT_CONTROL_FRAME_SHOT_SIZE ? bt_read_short(&p[21]) : 0;
    c->left_90 = length >= BT_CONTROL_FRAME_LEFT_SIZE ? bt_read_short(&p[23]) : 0;
    c->servo_mode = length >= BT_CONTROL_FRAME_SERVO_MODE_SIZE ?
                    bt_read_short(&p[25]) : 0;
    c->servo_buttons = 0U;
    c->aim = 0U;
    c->gap_pwm = 0U;
    c->brake = 0;
    c->disable = 0;
}

static uint8_t DecodeCombinedControl(const uint8_t *p, uint8_t length,
                                     uint8_t base_length,
                                     uint8_t arm_offset)
{
    BluetoothControlFrame *c = &control.frame;
    int16_t direction;
    uint8_t i, sum = 0U;
    uint8_t payload_size = (uint8_t)(length - 3U);
    uint8_t was_connected;
    for (i = 1U; i <= payload_size; ++i) sum = (uint8_t)(sum + p[i]);
    if (p[length - 1U] != BT_TAIL || p[length - 2U] != sum ||
        !PayloadValid(p, base_length) ||
        !ArmDirectionValid(bt_read_short(&p[arm_offset])) ||
        !AxisValid(bt_read_short(&p[arm_offset + 2U])) ||
        !AxisValid(bt_read_short(&p[arm_offset + 4U]))) return 0U;
    direction = bt_read_short(&p[arm_offset]);
    was_connected = Bluetooth_IsConnected();
    (void)TraceRaw(p, length);
    raw_used = 0U;
    DecodeControl(c, p, base_length);
    control.last_rx_tick = last_byte_tick;
    control.valid = 1U;
    ++sequence;
    arm_control.direction = IsArmCommand(direction) ? 0 : direction;
    arm_control.brake = c->stop;
    arm_control.disable = 0;
    arm_control.x = c->joy_x;
    arm_control.y = c->joy_y;
    arm_control.arm_x = bt_read_short(&p[arm_offset + 2U]);
    arm_control.arm_y = bt_read_short(&p[arm_offset + 4U]);
    arm_control.last_rx_tick = last_byte_tick;
    arm_control.valid = 1U;
    ++arm_sequence;
    last_valid_frame_length = length;
    last_arm_frame_length = length;
    used = 0U;
    if (!IsArmCommand(direction)) previous_arm_button = 0;
    else ArmButton(direction, last_byte_tick);
    if (!was_connected) Debug_Log("[BT] valid frame\r\n");
    return 1U;
}

static void DecodePackedButtons(BluetoothControlFrame *c, uint16_t buttons)
{
    c->servo_buttons = (uint16_t)(buttons &
        (0x01FFU | BT_SERVO_BUTTON_TH_L));
    if ((buttons & BT_CONTROL_BUTTON_SHOT) != 0U) c->Shot = 1;
    c->aim = (uint8_t)((buttons & BT_CONTROL_BUTTON_AIM) != 0U);
}

/* Teammate (20).pro wire order, taken from origin/main@3021961:
 * bool[2], JOY_Y,F,B,STOP,L,R,R90,R180,Cam_T,Shot,L90,SERVO_MODE,
 * JOY_X,ARM_CMD,ARM_X,ARM_Y,GAP.  The arm values are routed through the
 * local arm controller instead of the teammate's direct-servo sender. */
static uint8_t DecodePhone20(const uint8_t *p)
{
    BluetoothControlFrame *c = &control.frame;
    uint16_t buttons;
    uint32_t gap_pwm;
    int16_t direction;
    uint8_t i, sum = 0U;
    uint8_t was_connected;

    for (i = 1U; i <= 38U; ++i) sum = (uint8_t)(sum + p[i]);
    buttons = (uint16_t)(p[1] | ((uint16_t)p[2] << 8));
    direction = bt_read_short(&p[29]);
    gap_pwm = bt_read_uint32(&p[35]);
    if (p[40] != BT_TAIL || p[39] != sum ||
        (p[2] & 0xF0U) != 0U ||
        !AxisValid(bt_read_short(&p[3])) ||
        !AxisValid(bt_read_short(&p[27])) ||
        !ArmDirectionValid(direction) ||
        !AxisValid(bt_read_short(&p[31])) ||
        !AxisValid(bt_read_short(&p[33])) ||
        (gap_pwm != 0U && (gap_pwm < 500U || gap_pwm > 2500U))) return 0U;
    for (i = 1U; i <= 11U; ++i)
        if (!SwitchValid(bt_read_short(&p[3U + 2U * i]))) return 0U;

    was_connected = Bluetooth_IsConnected();
    (void)TraceRaw(p, BT_CONTROL_FRAME_PHONE20_SIZE);
    raw_used = 0U;
    memset(c, 0, sizeof(*c));
    c->joy_y = bt_read_short(&p[3]);
    c->forward = bt_read_short(&p[5]);
    c->backward = bt_read_short(&p[7]);
    c->stop = bt_read_short(&p[9]);
    c->strafe_left = bt_read_short(&p[11]);
    c->strafe_right = bt_read_short(&p[13]);
    c->right_90 = bt_read_short(&p[15]);
    c->right_180 = bt_read_short(&p[17]);
    c->Cam_T = bt_read_short(&p[19]);
    c->Shot = bt_read_short(&p[21]);
    c->left_90 = bt_read_short(&p[23]);
    c->servo_mode = bt_read_short(&p[25]);
    c->joy_x = bt_read_short(&p[27]);
    DecodePackedButtons(c, buttons);
    c->gap_pwm = (uint16_t)gap_pwm;
    control.last_rx_tick = last_byte_tick;
    control.valid = 1U;
    ++sequence;

    arm_control.direction = IsArmCommand(direction) ? 0 : direction;
    arm_control.brake = c->stop;
    arm_control.disable = 0;
    arm_control.x = c->joy_x;
    arm_control.y = c->joy_y;
    arm_control.arm_x = bt_read_short(&p[31]);
    arm_control.arm_y = bt_read_short(&p[33]);
    arm_control.last_rx_tick = last_byte_tick;
    arm_control.valid = 1U;
    ++arm_sequence;
    last_valid_frame_length = BT_CONTROL_FRAME_PHONE20_SIZE;
    last_arm_frame_length = BT_CONTROL_FRAME_PHONE20_SIZE;
    used = 0U;
    if (!IsArmCommand(direction)) previous_arm_button = 0;
    else ArmButton(direction, last_byte_tick);
    if (!was_connected) Debug_Log("[BT] valid frame\r\n");
    return 1U;
}

/* Exact incremental layout of 调试工程_H_20260908-203710(7).pro after
 * appending three shorts.  ValuePack groups values by type, so GAP remains
 * after all 16 shorts:
 * A5 | bool[2] | base short[13] | ARM_CMD/X/Y | GAP int32 | sum | 5A. */
static uint8_t DecodeBoolArmGap41(const uint8_t *p)
{
    BluetoothControlFrame *c = &control.frame;
    uint16_t buttons;
    uint32_t gap_pwm;
    int16_t direction;
    uint8_t i, sum = 0U;
    uint8_t was_connected;
    for (i = 1U; i <= 38U; ++i) sum = (uint8_t)(sum + p[i]);
    buttons = (uint16_t)(p[1] | ((uint16_t)p[2] << 8));
    direction = bt_read_short(&p[29]);
    gap_pwm = bt_read_uint32(&p[35]);
    if (p[40] != BT_TAIL || p[39] != sum ||
        bt_read_short(&p[27]) != BT_SERVO_GAP_MARKER ||
        (p[2] & 0xF0U) != 0U ||
        !PayloadValid(&p[2], BT_CONTROL_FRAME_LEFT_SIZE) ||
        !ArmDirectionValid(direction) ||
        !AxisValid(bt_read_short(&p[31])) ||
        !AxisValid(bt_read_short(&p[33])) ||
        (gap_pwm != 0U && (gap_pwm < 500U || gap_pwm > 2500U))) return 0U;
    was_connected = Bluetooth_IsConnected();
    (void)TraceRaw(p, BT_ARM_BOOL_GAP_FRAME_SIZE);
    raw_used = 0U;
    DecodeControl(c, &p[2], BT_CONTROL_FRAME_LEFT_SIZE);
    DecodePackedButtons(c, buttons);
    c->gap_pwm = (uint16_t)gap_pwm;
    control.last_rx_tick = last_byte_tick;
    control.valid = 1U;
    ++sequence;
    arm_control.direction = IsArmCommand(direction) ? 0 : direction;
    arm_control.brake = c->stop;
    arm_control.disable = 0;
    arm_control.x = c->joy_x;
    arm_control.y = c->joy_y;
    arm_control.arm_x = bt_read_short(&p[31]);
    arm_control.arm_y = bt_read_short(&p[33]);
    arm_control.last_rx_tick = last_byte_tick;
    arm_control.valid = 1U;
    ++arm_sequence;
    last_valid_frame_length = BT_ARM_BOOL_GAP_FRAME_SIZE;
    last_arm_frame_length = BT_ARM_BOOL_GAP_FRAME_SIZE;
    used = 0U;
    if (!IsArmCommand(direction)) previous_arm_button = 0;
    else ArmButton(direction, last_byte_tick);
    if (!was_connected) Debug_Log("[BT] valid frame\r\n");
    return 1U;
}

/* The historic 9-short chassis + 3-short arm packet and the current
 * 12-short chassis packet are both 27 bytes. In an arm session, decode the
 * final three shorts as the legacy arm overlay while preserving any valid
 * current Cam_T/Shot/LEFT_90 fields for the chassis. */
static uint8_t DecodeArmMode27(const uint8_t *p)
{
    uint8_t current_extension;
    if (!arm_mode || p[BT_CONTROL_FRAME_LEFT_SIZE - 1U] != BT_TAIL)
        return 0U;
    current_extension = PayloadValid(p, BT_CONTROL_FRAME_LEFT_SIZE);
    if (!DecodeCombinedControl(p, BT_CONTROL_FRAME_LEFT_SIZE,
                               BT_CONTROL_FRAME_SIZE, 19U))
        return 0U;
    if (current_extension) {
        control.frame.Cam_T = bt_read_short(&p[19]);
        control.frame.Shot = bt_read_short(&p[21]);
        control.frame.left_90 = bt_read_short(&p[23]);
    }
    return 1U;
}

static uint8_t TraceRaw(const uint8_t *bytes, uint8_t length)
{
    char line[80];
    uint8_t i;
    uint8_t offset = 0U;
    int written;
    uint8_t changed = (uint8_t)(!have_trace || length != last_traced_len ||
        memcmp(bytes, last_traced_bytes, length) != 0);
    if ((!changed && (uint32_t)(HAL_GetTick() - trace_tick) < BT_TRACE_PERIOD_MS) ||
        !Debug_CanLog(2U)) return 0U;
    trace_tick = HAL_GetTick();
    memcpy(last_traced_bytes, bytes, length);
    last_traced_len = length;
    have_trace = 1U;
    written = snprintf(line, sizeof(line), "BT RAW len=%u:", (unsigned)length);
    for (i = 0U; i < length; ++i) {
        if ((size_t)written + 5U >= sizeof(line)) {
            line[written++] = '\r'; line[written++] = '\n'; line[written] = '\0';
            Debug_Log(line);
            written = snprintf(line, sizeof(line), "BT RAW +%u:", (unsigned)offset);
        }
        written += snprintf(&line[written], sizeof(line) - (size_t)written,
                            " %02X", (unsigned)bytes[i]);
        offset = (uint8_t)(i + 1U);
    }
    (void)snprintf(&line[written], sizeof(line) - (size_t)written, "\r\n");
    Debug_Log(line);
    return 1U;
}

static void TraceVariables(const BluetoothControlFrame *c)
{
    char line[80];
    if (!Debug_CanLog(2U)) return;
    (void)snprintf(line, sizeof(line),
        "BT FRAME OK F=%u B=%u L=%u R=%u STOP=%u R90=%u R180=%u L90=%u C=%u SH=%u\r\n",
        (unsigned)(c->forward != 0), (unsigned)(c->backward != 0),
        (unsigned)(c->strafe_left != 0), (unsigned)(c->strafe_right != 0),
        (unsigned)(c->stop != 0), (unsigned)(c->right_90 != 0),
        (unsigned)(c->right_180 != 0), (unsigned)(c->left_90 != 0),
        (unsigned)(c->Cam_T != 0),
        (unsigned)(c->Shot != 0));
    Debug_Log(line);
    (void)snprintf(line, sizeof(line),
        "BT JOY X=%d Y=%d\r\n", c->joy_x, c->joy_y);
    Debug_Log(line);
}

static void Resync(void)
{
    uint8_t i;
    for (i = 1U; i < used && frame[i] != BT_HEADER; ++i) {}
    used = (uint8_t)(used - i);
    memmove(frame, frame + i, used);
}

void Bluetooth_Init(void)
{
    memset(&control, 0, sizeof(control));
    memset(&arm_control, 0, sizeof(arm_control));
    used = 0U;
    raw_used = 0U;
    last_traced_len = have_trace = 0U;
    sequence = 0U;
    arm_sequence = 0U;
    servo_one_sequence = 0U;
    invalid_frame_count = 0U;
    rx_byte_count = 0U;
    rx_diagnostic_tick = HAL_GetTick();
    servo_one_pressed = 0U;
    text_handler = NULL;
    text_used = text_bad = arm_mode = 0U;
    last_valid_frame_length = last_arm_frame_length = 0U;
    previous_arm_button = 0;
    previous_test_command = 0;
    previous_simple_buttons = 0U;
    last_simple_action = 0U;
    simple_press_count = 0U;
    last_test_frame_length = 0U;
    test_sequence = 0U;
    bluetooth_rx_recoveries = 0U;
    error_log_tick = HAL_GetTick() - 1000U;
    trace_tick = HAL_GetTick() - BT_TRACE_PERIOD_MS;
    Serial_Init(&rx, &huart6);
}

void Bluetooth_RxCallback(UART_HandleTypeDef *uart)
{
    if (uart != NULL && uart == rx.uart) ++rx_byte_count;
    Serial_RxCallback(&rx, uart);
}
void Bluetooth_ErrorCallback(UART_HandleTypeDef *uart) { Serial_ErrorCallback(&rx, uart); }

void Bluetooth_Process(void)
{
    uint8_t byte, i, sum;
    uint32_t tick;
    TraceReception();
    if (Serial_Recover(&rx)) {
        used = 0U;
        control.valid = 0U;
        arm_control.valid = 0U;
        text_used = text_bad = 0U;
        previous_arm_button = 0;
        previous_test_command = 0;
        previous_simple_buttons = 0U;
        ++bluetooth_rx_recoveries;
        if (text_handler != NULL) text_handler(NULL, HAL_GetTick());
    }
    for (;;) {
        /* A bad 5-byte checksum followed by a fresh header must not swallow
         * the following packet. Retain STOP's out-of-range trim continuation. */
        if (used == BT_PID_FRAME_SIZE && frame[4] == BT_TAIL &&
            SimpleByteValid(frame[2]) && !(frame[1] & BT_TRIM_STOP) &&
            frame[3] != (uint8_t)(frame[1] + frame[2]) && ShortFrameComplete())
            goto invalid_frame;
        /* PID tuning retains its original, separate one-short ValuePack. */
        if (used >= BT_PID_FRAME_SIZE && frame[0] == BT_HEADER &&
            frame[4] == BT_TAIL &&
            frame[3] == (uint8_t)(frame[1] + frame[2]) && ShortFrameComplete()) {
            uint8_t cmd = frame[1], high = frame[2];
            TraceRaw(frame, BT_PID_FRAME_SIZE);
            raw_used = 0U;
            used = (uint8_t)(used - BT_PID_FRAME_SIZE);
            memmove(frame, frame + BT_PID_FRAME_SIZE, used);
            if (high == 0U &&
                (uint32_t)(HAL_GetTick() - last_byte_tick) <= BT_FAILSAFE_TIMEOUT_MS) {
                if (cmd >= 1U && cmd <= 11U)
                    PID_Tuner_HandleCommand(cmd);
                else if (cmd == BT_SERVO_ONE_RELEASE_CMD)
                    servo_one_pressed = 0U;
                else if (cmd == BT_SERVO_ONE_PRESS_CMD && !servo_one_pressed) {
                    servo_one_pressed = 1U;
                    ++servo_one_sequence;
                }
            }
            return;
        }
        /* Offset 6 cannot be 5A in any longer valid supported profile:
         * it is a bounded axis/switch high byte. Check length, sum and byte. */
        if (used == BT_TRIM_SIMPLE_FRAME_SIZE && frame[6] == BT_TAIL) {
            if (DecodeTrimSimple(frame)) return;
            if ((uint32_t)(HAL_GetTick() - error_log_tick) >= 1000U) {
                char reason[80];
                (void)snprintf(reason, sizeof(reason),
                    "[BT TRIM] reject byte=%u sum=%u/%u dx=%d\r\n",
                    (unsigned)frame[2], (unsigned)frame[5],
                    (unsigned)(uint8_t)(frame[1]+frame[2]+frame[3]+frame[4]),
                    (int)bt_read_short(&frame[3]));
                Debug_Log(reason);
            }
            goto invalid_frame;
        }
        /* The first four control bytes are JOY_X/Y. An out-of-range prefix
         * cannot become a valid APP frame; release it before a following PID
         * command or control packet arrives. */
        if (used >= 5U) {
            uint8_t normal_prefix = (uint8_t)(
                AxisValid(bt_read_short(&frame[1])) &&
                AxisValid(bt_read_short(&frame[3])));
            uint8_t bool_prefix = (uint8_t)(
                (frame[2] & 0xF0U) == 0U &&
                AxisValid(bt_read_short(&frame[3])));
            if (!normal_prefix && !bool_prefix &&
                !(used < BT_TRIM_SIMPLE_FRAME_SIZE && SimpleByteValid(frame[2])))
                goto invalid_frame;
        }
        /* Longer valid profiles cannot have a 5A high byte at offset 10:
         * it belongs to a switch or an axis limited to +/-1000. */
        if (used == BT_TRIM_TEST_FRAME_SIZE && frame[10] == BT_TAIL) {
            if (DecodeTrimTest(frame)) return;
            goto invalid_frame;
        }
        if (used == BT_ARM_DUAL_FRAME_SIZE) {
            sum = 0U;
            for (i = 1U; i <= 14U; ++i) sum = (uint8_t)(sum + frame[i]);
            if (frame[16] == BT_TAIL && frame[15] == sum &&
                ArmPayloadValid(frame)) {
                int16_t direction = bt_read_short(&frame[1]);
                BluetoothControlFrame *c = &control.frame;
                uint8_t was_connected = Bluetooth_IsConnected();
                (void)TraceRaw(frame, BT_ARM_DUAL_FRAME_SIZE);
                raw_used = 0U;
                memset(c, 0, sizeof(*c));
                c->joy_x = bt_read_short(&frame[7]);
                c->joy_y = bt_read_short(&frame[9]);
                c->stop = bt_read_short(&frame[3]);
                c->brake = c->stop;
                c->disable = bt_read_short(&frame[5]);
                control.last_rx_tick = last_byte_tick;
                control.valid = 1U;
                ++sequence;
                arm_control.direction = IsArmCommand(direction) ? 0 : direction;
                arm_control.brake = c->brake;
                arm_control.disable = c->disable;
                arm_control.x = c->joy_x;
                arm_control.y = c->joy_y;
                arm_control.arm_x = bt_read_short(&frame[11]);
                arm_control.arm_y = bt_read_short(&frame[13]);
                arm_control.last_rx_tick = last_byte_tick;
                arm_control.valid = 1U;
                ++arm_sequence;
                last_valid_frame_length = BT_ARM_DUAL_FRAME_SIZE;
                last_arm_frame_length = BT_ARM_DUAL_FRAME_SIZE;
                used = 0U;
                if (!IsArmCommand(direction)) previous_arm_button = 0;
                else ArmButton(direction, last_byte_tick);
                if (!was_connected) Debug_Log("[BT] valid frame\r\n");
                return;
            }
            /* Byte 16 is a normal high byte in longer phone profiles. */
            if (frame[16] == BT_TAIL) goto invalid_frame;
        }
        if (used == BT_CONTROL_FRAME_LEFT_SIZE && DecodeArmMode27(frame))
            return;
        if (used == BT_ARM_COMBINED_FRAME_EXT_SIZE && arm_mode &&
            DecodeCombinedControl(frame, BT_ARM_COMBINED_FRAME_EXT_SIZE,
                                  BT_CONTROL_FRAME_EXT_SIZE, 21U)) return;
        if (used == BT_ARM_COMBINED_FRAME_SHOT_SIZE && arm_mode &&
            DecodeCombinedControl(frame, BT_ARM_COMBINED_FRAME_SHOT_SIZE,
                                  BT_CONTROL_FRAME_SHOT_SIZE, 23U)) return;
        if (used == BT_ARM_COMBINED_FRAME_LEFT_SIZE &&
            DecodeCombinedControl(frame, BT_ARM_COMBINED_FRAME_LEFT_SIZE,
                                  BT_CONTROL_FRAME_LEFT_SIZE, 25U)) return;
        if (used == BT_ARM_COMBINED_FRAME_SIZE) {
            int16_t direction = bt_read_short(&frame[27]);
            uint32_t gap_pwm = bt_read_uint32(&frame[29]);
            sum = 0U;
            for (i = 1U; i <= 32U; ++i) sum = (uint8_t)(sum + frame[i]);
            if (frame[34] == BT_TAIL && frame[33] == sum &&
                direction == BT_SERVO_GAP_MARKER &&
                (frame[2] & 0xF0U) == 0U &&
                PayloadValid(&frame[2], BT_CONTROL_FRAME_LEFT_SIZE) &&
                (gap_pwm == 0U || (gap_pwm >= 500U && gap_pwm <= 2500U))) {
                BluetoothControlFrame *c = &control.frame;
                uint8_t was_connected = Bluetooth_IsConnected();
                uint8_t traced = TraceRaw(frame, used);
                raw_used = 0U;
                DecodeControl(c, &frame[2], BT_CONTROL_FRAME_LEFT_SIZE);
                DecodePackedButtons(c,
                    (uint16_t)(frame[1] | ((uint16_t)frame[2] << 8)));
                c->gap_pwm = (uint16_t)gap_pwm;
                control.last_rx_tick = last_byte_tick;
                control.valid = 1U;
                ++sequence;
                last_valid_frame_length = BT_ARM_COMBINED_FRAME_SIZE;
                used = 0U;
                if (traced) TraceVariables(c);
                if (!was_connected) Debug_Log("[BT] valid frame\r\n");
                return;
            }
            if (DecodeCombinedControl(frame, BT_ARM_COMBINED_FRAME_SIZE,
                                      BT_CONTROL_FRAME_SERVO_MODE_SIZE,
                                      27U)) return;
            /* It may be the valid 35-byte prefix of the extended (7).pro
             * packet. A complete 35-byte packet has its tail here. */
            if (frame[34] == BT_TAIL) goto invalid_frame;
        }
        if (used == BT_CONTROL_FRAME_SERVO_BOOL_SIZE) {
            sum = 0U;
            for (i = 1U; i <= 28U; ++i) sum = (uint8_t)(sum + frame[i]);
            if (frame[30] == BT_TAIL && frame[29] == sum &&
                (frame[2] & 0xF0U) == 0U &&
                PayloadValid(&frame[2], BT_CONTROL_FRAME_LEFT_SIZE)) {
                BluetoothControlFrame *c = &control.frame;
                uint8_t was_connected = Bluetooth_IsConnected();
                uint8_t traced = TraceRaw(frame, used);
                raw_used = 0U;
                DecodeControl(c, &frame[2], BT_CONTROL_FRAME_LEFT_SIZE);
                DecodePackedButtons(c,
                    (uint16_t)(frame[1] | ((uint16_t)frame[2] << 8)));
                control.last_rx_tick = last_byte_tick;
                control.valid = 1U;
                ++sequence;
                last_valid_frame_length = BT_CONTROL_FRAME_SERVO_BOOL_SIZE;
                used = 0U;
                if (traced) TraceVariables(c);
                if (!was_connected) Debug_Log("[BT] valid frame\r\n");
                return;
            }
            /* A 35-byte arm frame also passes through length 31. */
        }
        if (used == BT_ARM_BOOL_GAP_FRAME_SIZE) {
            if (DecodeBoolArmGap41(frame)) return;
            if (DecodePhone20(frame)) return;
            goto invalid_frame;
        }
        if (used == BT_CONTROL_FRAME_SIZE || used == BT_CONTROL_FRAME_EXT_SIZE ||
            used == BT_CONTROL_FRAME_SHOT_SIZE ||
            used == BT_CONTROL_FRAME_LEFT_SIZE ||
            used == BT_CONTROL_FRAME_SERVO_MODE_SIZE) {
            uint8_t payload_size = used == BT_CONTROL_FRAME_SERVO_MODE_SIZE ?
                BT_SERVO_MODE_PAYLOAD_SIZE : used == BT_CONTROL_FRAME_LEFT_SIZE ?
                BT_LEFT_PAYLOAD_SIZE : used == BT_CONTROL_FRAME_SHOT_SIZE ?
                BT_SHOT_PAYLOAD_SIZE :
                (used == BT_CONTROL_FRAME_EXT_SIZE ? BT_EXT_PAYLOAD_SIZE : BT_PAYLOAD_SIZE);
            sum = 0U;
            for (i = 1U; i <= payload_size; ++i) sum = (uint8_t)(sum + frame[i]);
            if (frame[payload_size + 2U] == BT_TAIL &&
                frame[payload_size + 1U] == sum && PayloadValid(frame, used)) {
                uint8_t was_connected = Bluetooth_IsConnected();
                uint8_t traced = TraceRaw(frame, used);
                BluetoothControlFrame *c = &control.frame;
                raw_used = 0U;
                DecodeControl(c, frame, used);
                control.last_rx_tick = last_byte_tick;
                control.valid = 1U;
                ++sequence;
                last_valid_frame_length = used;
                used = 0U;
                if (traced) TraceVariables(c);
                if (!was_connected) Debug_Log("[BT] valid frame\r\n");
                return; /* Deliver safety pulses within bursts individually. */
            }
            if (used == BT_CONTROL_FRAME_SERVO_MODE_SIZE &&
                frame[BT_CONTROL_FRAME_SERVO_MODE_SIZE - 1U] == BT_TAIL)
                goto invalid_frame;
            /* A longer frame can have data where the shorter tail would be. */
        }
        if (!Serial_Pop(&rx, &byte, &tick)) {
            if (raw_used && (uint32_t)(HAL_GetTick() - last_byte_tick) >= 20U) {
                (void)TraceRaw(raw_bytes, raw_used);
                raw_used = 0U;
            }
            return;
        }
        if (used && (uint32_t)(tick - last_byte_tick) > BT_FRAME_GAP_TIMEOUT_MS)
            used = 0U;
        if (used == 0U) {
            uint8_t text_result = TextByte(byte, tick);
            if (text_result == 2U) return;
            if (text_result) continue;
        }
        if (raw_used == sizeof(raw_bytes)) raw_used = 0U;
        raw_bytes[raw_used++] = byte;
        last_byte_tick = tick;
        if (used == 0U && byte != BT_HEADER) continue;
        frame[used++] = byte;
        continue;
invalid_frame:
        ++invalid_frame_count;
        (void)TraceRaw(frame, used);
        if ((uint32_t)(HAL_GetTick() - error_log_tick) >= 1000U) {
            Debug_Log("[BT] invalid control frame\r\n");
            error_log_tick = HAL_GetTick();
        }
        Resync();
    }
}

const BtControl_t *Bluetooth_GetControl(void) { return &control; }
uint8_t Bluetooth_IsConnected(void)
{
    return (uint8_t)(control.valid && !rx.broken &&
        (uint32_t)(HAL_GetTick() - control.last_rx_tick) <= BT_FAILSAFE_TIMEOUT_MS);
}
uint32_t Bluetooth_GetLastRxTick(void) { return control.last_rx_tick; }
uint32_t Bluetooth_GetSequence(void) { return sequence; }
const BtArmControl_t *Bluetooth_GetArmControl(void) { return &arm_control; }
uint8_t Bluetooth_ArmIsConnected(void)
{
    return (uint8_t)(arm_control.valid && !rx.broken &&
        (uint32_t)(HAL_GetTick() - arm_control.last_rx_tick) <= BT_FAILSAFE_TIMEOUT_MS);
}
uint32_t Bluetooth_GetArmSequence(void) { return arm_sequence; }
uint32_t Bluetooth_GetServoOneSequence(void) { return servo_one_sequence; }
uint8_t Bluetooth_GetLastFrameLength(void) { return last_valid_frame_length; }
uint8_t Bluetooth_GetLastArmFrameLength(void) { return last_arm_frame_length; }
uint8_t Bluetooth_GetLastTestFrameLength(void) { return last_test_frame_length; }
uint32_t Bluetooth_GetTestSequence(void) { return test_sequence; }
uint32_t Bluetooth_GetInvalidFrameCount(void) { return invalid_frame_count; }
uint32_t Bluetooth_GetRxByteCount(void) { return rx_byte_count; }
void Bluetooth_SetExtended(uint8_t enabled) { arm_mode = enabled != 0U; }
uint8_t Bluetooth_IsExtended(void) { return arm_mode; }
void Bluetooth_SetTextHandler(BluetoothTextHandler_t handler) { text_handler = handler; }
