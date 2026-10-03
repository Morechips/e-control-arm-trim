#include "car_control.h"
#include "servo.h"
#include "board_inputs.h"
#include "uart_driver.h"
#include "bluetooth_driver.h"
#include "serial_io.h"
#include "pid_tuner.h"
#include "arm_trim_input.h"
#include <stdio.h>
#include <string.h>

#define BT_HEADER 0xA5U
#define BT_TAIL 0x5AU
#define BT_PID_FRAME_SIZE 5U
#define BT_PAYLOAD_SIZE 18U
#define BT_EXT_PAYLOAD_SIZE 20U
#define BT_SHOT_PAYLOAD_SIZE 22U
#define BT_LEFT_PAYLOAD_SIZE 24U
#define BT_RESERVED_PAYLOAD_SIZE 26U
#define BT_TRACE_PERIOD_MS 500U

typedef struct {
    uint16_t button;
    ServoCode pose;
} ButtonPose;

static const ButtonPose button_poses[] = {
    {BT_SERVO_BUTTON_TB_B, TakeBall_Before},
    {BT_SERVO_BUTTON_TB_M, TakeBall_Mid},
    {BT_SERVO_BUTTON_TB_G, TakeBall_Gap},
    {BT_SERVO_BUTTON_BD_U, BarrelDown_Up},
    {BT_SERVO_BUTTON_BD_D, BarrelDown_Down},
    {BT_SERVO_BUTTON_TH_C, TakeHostage_Catch},
    {BT_SERVO_BUTTON_TH_G, TakeHostage_Gap},
    {BT_SERVO_BUTTON_TH_U, TakeHostage_Up},
    {BT_SERVO_BUTTON_TH_L, TakeHostage_Leave}
};

static uint32_t servo_last_sequence, servo_last_servo_one_sequence, servo_last_rx_tick;
static uint16_t servo_previous_buttons, servo_previous_gap_pwm;
static uint8_t servo_previous_aim, servo_gap_initialized;
static uint32_t servo_aim_sequence, servo_transmit_count;
static unsigned servo_last_transmit_result;
static UartRx_t rx;
static BtControl_t control;
static BtArmControl_t arm_control;
static uint8_t frame[BT_MAX_FRAME_SIZE];
static uint8_t used;
static uint8_t raw_bytes[BT_MAX_FRAME_SIZE], raw_used;
static uint8_t last_traced_bytes[BT_MAX_FRAME_SIZE], last_traced_len, have_trace;
static uint32_t sequence, error_log_tick, last_byte_tick, trace_tick;
static uint32_t arm_sequence, text_start_tick;
static uint32_t servo_one_sequence;
static uint8_t servo_one_pressed;
static BluetoothTextHandler_t text_handler;
static char text_line[80];
static uint8_t text_used, text_bad, arm_mode;
static int16_t previous_arm_button;
volatile uint32_t bluetooth_rx_recoveries;
static uint32_t rx_diagnostic_tick;
static uint8_t first_frame_logged;
#if ARM_TRIM_ENABLE
static volatile uint32_t test_sequence, simple_press_count, invalid_frame_count;
static volatile uint8_t last_test_frame_length, last_simple_action;
static uint8_t previous_trim_buttons;
static int16_t previous_test_command;
#endif

static void InputError(UART_HandleTypeDef *uart)
{
    (void)uart; Car_Control_InvalidateRemoteInput();
#if ARM_TRIM_ENABLE
    ArmTrimInput_Invalidate();
#endif
}
static void InputReady(UART_HandleTypeDef *uart)
{
    (void)uart;
    if (rx.broken) {
        Car_Control_InvalidateRemoteInput();
#if ARM_TRIM_ENABLE
        ArmTrimInput_Invalidate();
#endif
    }
}
static void PublishControl(void)
{
    const BluetoothControlFrame *c = &control.frame;
    CarRemoteInput_t input = {
        .command = {
            .joy_x = c->joy_x, .joy_y = c->joy_y,
            .forward = c->forward, .backward = c->backward, .stop = c->stop,
            .strafe_left = c->strafe_left, .strafe_right = c->strafe_right,
            .right_90 = c->right_90, .right_180 = c->right_180, .left_90 = c->left_90,
            .vision_follow = c->Cam_T, .shot = c->Shot, .brake = c->brake, .disable = c->disable
        },
        .sequence = sequence, .received_tick = control.last_rx_tick, .valid = control.valid
    };
#if ARM_TRIM_ENABLE
    if (c->stop || c->brake || c->disable) ArmTrimInput_Invalidate();
#endif
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    if (rx.broken) Car_Control_InvalidateRemoteInput();
    else Car_Control_SubmitRemoteInput(&input);
    __set_PRIMASK(mask);
}
static void TraceReception(void)
{
    char line[80];
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - rx_diagnostic_tick) < 1000U || !Debug_CanLog(1U)) return;
    rx_diagnostic_tick = now;
    (void)snprintf(line, sizeof(line),
        "[BT RX] bytes=%lu seq=%lu online=%u recover=%lu\r\n",
        (unsigned long)UART_RxBytes(&huart6), (unsigned long)sequence,
        (unsigned)Bluetooth_IsConnected(), (unsigned long)bluetooth_rx_recoveries);
    Debug_Log(line);
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
        (value >= 10 && value <= 13) || (value >= 20 && value <= 25));
}

static uint8_t AxisValid(int16_t value)
{
    return (uint8_t)(value >= -JOY_RANGE && value <= JOY_RANGE);
}

static uint8_t IsArmCommand(int16_t value)
{
    return (uint8_t)(value >= 20 && value <= 25);
}

/* At byte 35, only keep waiting if the full numeric prefix can belong to
 * (20).pro. Its army high byte cannot be 0x5A, so a valid legacy 35-byte
 * packet remains unambiguous and is delivered without delay. */
static uint8_t Phone20PrefixValid(const uint8_t *p)
{
    unsigned i;
    if ((p[2] & 0xF0U) != 0U || !AxisValid(bt_read_short(&p[3])) ||
        !AxisValid(bt_read_short(&p[27])) ||
        !ArmDirectionValid(bt_read_short(&p[29])) ||
        !AxisValid(bt_read_short(&p[31])) ||
        !AxisValid(bt_read_short(&p[33]))) return 0U;
    for (i = 1U; i <= 11U; ++i)
        if (!SwitchValid(bt_read_short(&p[3U + 2U * i]))) return 0U;
    return 1U;
}

static void DecodePhone20(BluetoothControlFrame *c, const uint8_t *p)
{
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
    c->Shot = (int16_t)(bt_read_short(&p[21]) || (p[2] & 0x02U));
    c->left_90 = bt_read_short(&p[23]);
    /* servo_mode and the three arm fields have no runtime action. */
    c->joy_x = bt_read_short(&p[27]);
    c->aim = (uint8_t)((p[2] & 0x04U) != 0U);
    c->servo_buttons = (uint16_t)((p[1] | ((uint16_t)p[2] << 8)) &
        ~(BT_CONTROL_BOOL_SHOT_BIT | BT_CONTROL_BOOL_AIM_BIT));
    c->gap_pwm = (uint16_t)bt_read_uint32(&p[35]);
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
    c->servo_buttons = 0U;
    c->aim = 0U;
    c->gap_pwm = 0U;
    c->brake = 0;
    c->disable = 0;
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
        "BT JOY X=%d Y=%d AIM=%u BTN=%03X GAP=%u\r\n",
        c->joy_x, c->joy_y, (unsigned)c->aim,
        (unsigned)c->servo_buttons, (unsigned)c->gap_pwm);
    Debug_Log(line);
}

static void Resync(void)
{
    uint8_t i;
    for (i = 1U; i < used && frame[i] != BT_HEADER; ++i) {}
    used = (uint8_t)(used - i);
    memmove(frame, frame + i, used);
}

#if ARM_TRIM_ENABLE
static uint8_t ShortFrameComplete(void)
{
    uint16_t tail;
    /* An out-of-range trim short can contain an entire valid PID frame.
     * Its next byte must be a fresh header, or the inter-byte lease must
     * expire, before the five-byte prefix is a complete command. */
    if (used > BT_PID_FRAME_SIZE)
        return (uint8_t)(frame[BT_PID_FRAME_SIZE] == BT_HEADER);
    if (rx.broken) return 0U;
    tail = rx.tail;
    if (tail != rx.head) {
        /* Foreground alone advances tail. IRQ publishes head only after
         * writing this slot; the ring bytes stay owned by this consumer. */
        __DMB();
        return (uint8_t)(rx.data[tail] == BT_HEADER);
    }
    return (uint8_t)((uint32_t)(HAL_GetTick() - last_byte_tick) > BT_FRAME_GAP_TIMEOUT_MS);
}

static uint8_t TrimGripperValid(int16_t value)
{
    return (uint8_t)(value == 0 || (value >= 500 && value <= 2500));
}

static uint8_t DecodeLegacyTrim(void)
{
    static const char *const fixed[] = {
        "", "@BENCH PREP BALL", "@BENCH PREP HOSTAGE", "@BENCH PREP BUCKET",
        "@ARM TRIM BEGIN BALL", "@ARM TRIM BEGIN HOSTAGE", "@ARM TRIM BEGIN BUCKET"
    };
    int16_t command = bt_read_short(&frame[1]);
    int16_t dx = bt_read_short(&frame[3]);
    int16_t close = bt_read_short(&frame[5]);
    int16_t open = bt_read_short(&frame[7]);
    uint8_t checksum = 0U;
    char line[40];
    unsigned i;
    for (i = 1U; i <= 8U; ++i) checksum = (uint8_t)(checksum + frame[i]);
    if (command < 0 || command > 13 || dx < -150 || dx > 150 ||
        !TrimGripperValid(close) || !TrimGripperValid(open) || frame[9] != checksum)
        return 0U;
    (void)TraceRaw(frame, used);
    used = raw_used = 0U;
    last_test_frame_length = BT_TRIM_LEGACY_FRAME_SIZE;
    ++test_sequence;
    if (command == previous_test_command) return 1U;
    /* Consume the edge before freshness filtering: a delayed held command
     * cannot become a fresh operation merely by sending its next repeat. */
    previous_test_command = command;
    if (command == 0 || (uint32_t)(HAL_GetTick() - last_byte_tick) > BT_FAILSAFE_TIMEOUT_MS)
        return 1U;
    if (command <= 6) ArmTrimInput_HandleLine(fixed[command], last_byte_tick);
    else {
        if (command == 7) (void)snprintf(line, sizeof(line), "@ARM TRIM DX %d", (int)dx);
        else if (command == 8) strcpy(line, "@ARM TRIM END");
        else if (command == 9 || command == 10)
            (void)snprintf(line, sizeof(line), "@BENCH GRIP %d", command == 9 ? (int)close : (int)open);
        else if (command == 11) strcpy(line, "@ARM TRIM STATUS");
        else if (command == 12) strcpy(line, "@ARM TRIM STOP");
        else strcpy(line, "@ARM TRIM CLEAR");
        ArmTrimInput_HandleLine(line, last_byte_tick);
    }
    return 1U;
}
#endif

void Bluetooth_ProvisionName(void)
{
#if BT_AT_NAME_ENABLE
    /* Factory one-shot: the module stores the name, so this must not run on
     * every boot. Reception stays armed throughout (no polling, no reply
     * parsing). Send once without waiting for module readiness or its reply. */
    static const uint8_t command[] = BT_AT_NAME_COMMAND;
    (void)UART_SEND(command, sizeof(command) - 1U, &huart6, UART_TX_BLOCKING, 1000U);
#endif
}

void Bluetooth_Init(void)
{
    memset(&control, 0, sizeof(control));
    memset(&arm_control, 0, sizeof(arm_control));
    used = 0U;
    raw_used = 0U;
    last_traced_len = have_trace = 0U;
    sequence = 0U;
    servo_last_sequence = servo_last_servo_one_sequence = servo_last_rx_tick = 0U;
    servo_previous_buttons = servo_previous_gap_pwm = 0U;
    servo_previous_aim = servo_gap_initialized = 0U;
    servo_aim_sequence = servo_transmit_count = 0U;
    servo_last_transmit_result = 255U;
    arm_sequence = 0U;
    servo_one_sequence = 0U;
    servo_one_pressed = 0U;
    text_handler = NULL;
    text_used = text_bad = arm_mode = 0U;
    previous_arm_button = 0;
    bluetooth_rx_recoveries = 0U;
    first_frame_logged = 0U;
#if ARM_TRIM_ENABLE
    test_sequence = simple_press_count = invalid_frame_count = 0U;
    last_test_frame_length = last_simple_action = previous_trim_buttons = 0U;
    previous_test_command = 0;
#endif
    rx_diagnostic_tick = HAL_GetTick();
    error_log_tick = HAL_GetTick() - 1000U;
    trace_tick = HAL_GetTick() - BT_TRACE_PERIOD_MS;
    (void)UART_BindRx(&rx, &huart6, InputReady, InputError);
    Bluetooth_ProvisionName();
}

void Bluetooth_RxCallback(UART_HandleTypeDef *uart)
{
    UART_RxCallback(uart);
}
void Bluetooth_ErrorCallback(UART_HandleTypeDef *uart)
{
    UART_ErrorCallback(uart);
}

static void LogFirstFrame(void)
{
    if (!first_frame_logged && control.valid) {
        char first[48];
        first_frame_logged = 1U;
        /* Bench instrument for "how long until the remote works". */
        (void)snprintf(first, sizeof(first), "[BT] first frame t=%lu\r\n",
                       (unsigned long)HAL_GetTick());
        Debug_Log(first);
    }
}

void Bluetooth_Process(void)
{
    uint8_t byte, i, sum;
    uint32_t tick;
    size_t received;
    TraceReception();
    if (UART_Recover(&huart6)) {
        used = 0U;
        control.valid = 0U;
        Car_Control_InvalidateRemoteInput();
#if ARM_TRIM_ENABLE
        ArmTrimInput_Invalidate();
        previous_trim_buttons = 0U;
        previous_test_command = 0;
#endif
        arm_control.valid = 0U;
        text_used = text_bad = 0U;
        previous_arm_button = 0;
        ++bluetooth_rx_recoveries;
        if (text_handler != NULL) text_handler(NULL, HAL_GetTick());
    }
    for (;;) {
#if ARM_TRIM_ENABLE
        /* The independent page is bool/byte/short, not chassis input. Its
         * numeric range belongs to the consumer so STOP survives a bad ydnum. */
        if (used == BT_TRIM_FRAME_SIZE && frame[0] == BT_HEADER &&
            (frame[2] == 0U || frame[2] == 84U) && frame[6] == BT_TAIL) {
            sum = (uint8_t)(frame[1] + frame[2] + frame[3] + frame[4]);
            if (frame[5] != sum) goto invalid_frame;
            last_test_frame_length = BT_TRIM_FRAME_SIZE;
            last_simple_action = (uint8_t)(frame[1] & (uint8_t)~previous_trim_buttons);
            if (last_simple_action != 0U) ++simple_press_count;
            previous_trim_buttons = frame[1];
            ++test_sequence;
            (void)TraceRaw(frame, used);
            raw_used = used = 0U;
            ArmTrimInput_Submit(frame[1], bt_read_short(&frame[3]), last_byte_tick);
            return;
        }
        /* At byte 10 a valid longer layout has a bounded switch high byte;
         * it cannot be 5A. The old four-short page remains separate from car. */
        if (used == BT_TRIM_LEGACY_FRAME_SIZE && frame[0] == BT_HEADER &&
            frame[10] == BT_TAIL) {
            if (DecodeLegacyTrim()) return;
            goto invalid_frame;
        }
        /* Drop a broken short at its boundary before reading the next
         * header, while still retaining a possible seven-byte STOP. */
        if (used == BT_PID_FRAME_SIZE && frame[4] == BT_TAIL &&
            (frame[2] == 0U || frame[2] == 84U) &&
            !(frame[1] & ARM_TRIM_BUTTON_STOP) &&
            frame[3] != (uint8_t)(frame[1] + frame[2]) && ShortFrameComplete())
            goto invalid_frame;
#endif
        /* PID tuning retains its original, separate one-short ValuePack. */
        if (used >= BT_PID_FRAME_SIZE && frame[0] == BT_HEADER &&
            frame[4] == BT_TAIL &&
            frame[3] == (uint8_t)(frame[1] + frame[2])
#if ARM_TRIM_ENABLE
            /* Unknown shorts can be a trim prefix with an invalid direction.
             * Consume only established five-byte commands when trim is enabled. */
            && frame[2] == 0U && ((frame[1] >= 1U && frame[1] <= 11U) ||
                frame[1] == BT_SERVO_ONE_RELEASE_CMD || frame[1] == BT_SERVO_ONE_PRESS_CMD)
            && ShortFrameComplete()
#endif
            ) {
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
        /* A legacy frame starts with two joystick shorts. A bool frame starts
         * with two button bytes, then its first joystick short. Reject a
         * prefix only when neither layout can still be valid. */
        if (used >= 5U &&
#if ARM_TRIM_ENABLE
            !(used <= 6U && (frame[2] == 0U || frame[2] == 84U)) &&
            !(used <= 10U && frame[2] == 0U && frame[1] <= 13U &&
              bt_read_short(&frame[3]) >= -150 && bt_read_short(&frame[3]) <= 150) &&
#endif
            (bt_read_short(&frame[1]) < -JOY_RANGE ||
             bt_read_short(&frame[1]) > JOY_RANGE ||
             bt_read_short(&frame[3]) < -JOY_RANGE ||
             bt_read_short(&frame[3]) > JOY_RANGE) &&
            ((frame[2] & 0xF0U) != 0U ||
             bt_read_short(&frame[3]) < -JOY_RANGE ||
             bt_read_short(&frame[3]) > JOY_RANGE)) goto invalid_frame;
        if (used == BT_CONTROL_FRAME_PHONE20_SIZE) {
            uint32_t gap_pwm = bt_read_uint32(&frame[35]);
            sum = 0U;
            for (i = 1U; i <= 38U; ++i) sum = (uint8_t)(sum + frame[i]);
            if (frame[40] == BT_TAIL && frame[39] == sum &&
                Phone20PrefixValid(frame) &&
                (gap_pwm == 0U || (gap_pwm >= 500U && gap_pwm <= 2500U))) {
                uint8_t was_connected = Bluetooth_IsConnected();
                uint8_t traced = TraceRaw(frame, used);
                raw_used = 0U;
                DecodePhone20(&control.frame, frame);
                control.last_rx_tick = last_byte_tick;
                control.valid = 1U;
                LogFirstFrame();
                ++sequence;
                PublishControl();
                used = 0U;
                if (traced) TraceVariables(&control.frame);
                if (!was_connected) Debug_Log("[BT] valid frame\r\n");
                return;
            }
            goto invalid_frame;
        }
        if (used == BT_ARM_COMBINED_FRAME_SIZE) {
            int16_t direction = bt_read_short(&frame[27]);
            uint32_t gap_pwm = bt_read_uint32(&frame[29]);
            sum = 0U;
            for (i = 1U; i <= 32U; ++i) sum = (uint8_t)(sum + frame[i]);
            if (frame[34] == BT_TAIL && frame[33] == sum &&
                direction == BT_GAP_FRAME_MARKER &&
                (frame[2] & 0xF0U) == 0U &&
                PayloadValid(&frame[2], BT_CONTROL_FRAME_LEFT_SIZE) &&
                (gap_pwm == 0U || (gap_pwm >= 500U && gap_pwm <= 2500U))) {
                BluetoothControlFrame *c = &control.frame;
                uint8_t was_connected = Bluetooth_IsConnected();
                uint8_t traced = TraceRaw(frame, used);
                raw_used = 0U;
                DecodeControl(c, &frame[2], BT_CONTROL_FRAME_LEFT_SIZE);
                c->Shot = (int16_t)(c->Shot || (frame[2] & 0x02U));
                c->aim = (uint8_t)((frame[2] & 0x04U) != 0U);
                c->servo_buttons = (uint16_t)((frame[1] | ((uint16_t)frame[2] << 8)) &
                                               ~(BT_CONTROL_BOOL_SHOT_BIT | BT_CONTROL_BOOL_AIM_BIT));
                c->gap_pwm = (uint16_t)gap_pwm;
                control.last_rx_tick = last_byte_tick;
                control.valid = 1U;
                LogFirstFrame();
                ++sequence;
                PublishControl();
                used = 0U;
                if (traced) TraceVariables(c);
                if (!was_connected) Debug_Log("[BT] valid frame\r\n");
                return;
            }
            if (frame[34] == BT_TAIL && frame[33] == sum &&
                PayloadValid(frame, BT_CONTROL_FRAME_RESERVED_SIZE) &&
                ArmDirectionValid(direction) &&
                AxisValid(bt_read_short(&frame[29])) &&
                AxisValid(bt_read_short(&frame[31]))) {
                BluetoothControlFrame *c = &control.frame;
                uint8_t was_connected = Bluetooth_IsConnected();
                (void)TraceRaw(frame, used);
                raw_used = 0U;
                DecodeControl(c, frame, BT_CONTROL_FRAME_RESERVED_SIZE);
                control.last_rx_tick = last_byte_tick;
                control.valid = 1U;
                LogFirstFrame();
                ++sequence;
                PublishControl();
                arm_control.direction = IsArmCommand(direction) ? 0 : direction;
                arm_control.brake = c->stop;
                arm_control.disable = 0;
                arm_control.x = c->joy_x;
                arm_control.y = c->joy_y;
                arm_control.arm_x = bt_read_short(&frame[29]);
                arm_control.arm_y = bt_read_short(&frame[31]);
                arm_control.last_rx_tick = last_byte_tick;
                arm_control.valid = 1U;
                ++arm_sequence;
                used = 0U;
                if (!IsArmCommand(direction)) previous_arm_button = 0;
                else ArmButton(direction, last_byte_tick);
                if (!was_connected) Debug_Log("[BT] valid frame\r\n");
                return;
            }
            if (!Phone20PrefixValid(frame)) goto invalid_frame;
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
                c->Shot = (int16_t)(c->Shot || (frame[2] & 0x02U));
                c->aim = (uint8_t)((frame[2] & 0x04U) != 0U);
                c->servo_buttons = (uint16_t)((frame[1] | ((uint16_t)frame[2] << 8)) &
                                               ~(BT_CONTROL_BOOL_SHOT_BIT | BT_CONTROL_BOOL_AIM_BIT));
                control.last_rx_tick = last_byte_tick;
                control.valid = 1U;
                LogFirstFrame();
                ++sequence;
                PublishControl();
                used = 0U;
                if (traced) TraceVariables(c);
                if (!was_connected) Debug_Log("[BT] valid frame\r\n");
                return;
            }
            if (frame[30] == BT_TAIL) goto invalid_frame;
            /* A 35-byte arm frame also passes through length 31. */
        }
        if (used == BT_CONTROL_FRAME_SIZE || used == BT_CONTROL_FRAME_EXT_SIZE ||
            used == BT_CONTROL_FRAME_SHOT_SIZE ||
            used == BT_CONTROL_FRAME_LEFT_SIZE ||
            used == BT_CONTROL_FRAME_RESERVED_SIZE) {
            uint8_t payload_size = used == BT_CONTROL_FRAME_RESERVED_SIZE ?
                BT_RESERVED_PAYLOAD_SIZE : used == BT_CONTROL_FRAME_LEFT_SIZE ?
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
                LogFirstFrame();
                ++sequence;
                PublishControl();
                used = 0U;
                if (traced) TraceVariables(c);
                if (!was_connected) Debug_Log("[BT] valid frame\r\n");
                return; /* Deliver safety pulses within bursts individually. */
            }
            if (used == BT_CONTROL_FRAME_RESERVED_SIZE &&
                frame[BT_CONTROL_FRAME_RESERVED_SIZE - 1U] == BT_TAIL)
                goto invalid_frame;
            /* A longer frame can have data where the shorter tail would be. */
        }
        if ((UART_RECV(&byte, 1U, &huart6, &received, &tick) != HAL_OK || received == 0U)) {
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
        if (raw_used == sizeof(raw_bytes)) {
            /* Preserve the prefix of unsupported/oversized packets in logs.
             * Previously it was discarded, leaving misleading tail-only RAW. */
            (void)TraceRaw(raw_bytes, raw_used);
            raw_used = 0U;
        }
        raw_bytes[raw_used++] = byte;
        last_byte_tick = tick;
        if (used == 0U && byte != BT_HEADER) continue;
        frame[used++] = byte;
        continue;
invalid_frame:
#if ARM_TRIM_ENABLE
        ++invalid_frame_count;
#endif
        if ((uint32_t)(HAL_GetTick() - error_log_tick) >= 1000U) {
            Debug_Log("[BT] invalid control frame\r\n");
            (void)TraceRaw(frame, used);
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
uint32_t Bluetooth_GetTrimSequence(void)
{
#if ARM_TRIM_ENABLE
    return test_sequence;
#else
    return 0U;
#endif
}
uint32_t Bluetooth_GetTrimPressCount(void)
{
#if ARM_TRIM_ENABLE
    return simple_press_count;
#else
    return 0U;
#endif
}
void Bluetooth_SetExtended(uint8_t enabled) { arm_mode = enabled != 0U; }
uint8_t Bluetooth_IsExtended(void) { return arm_mode; }
void Bluetooth_SetTextHandler(BluetoothTextHandler_t handler) { text_handler = handler; }

void Bluetooth_DispatchServoActions(uint8_t physical_aim_press)
{
    const BluetoothControlFrame *c = &Bluetooth_GetControl()->frame;
    uint32_t sequence = Bluetooth_GetSequence();
    uint32_t one_sequence = Bluetooth_GetServoOneSequence();
    uint8_t connected = Bluetooth_IsConnected();
    uint8_t physical = physical_aim_press;
    uint8_t reference = (uint8_t)(connected && one_sequence != servo_last_servo_one_sequence);
    uint8_t aim = 0U, gap = 0U;
    uint16_t buttons = 0U;
    unsigned requests;
    unsigned i;
    ServoCode pose = ServoCode_NONE;
    const char *name = NULL;
    ServoStatus_t result;
    char message[80];

    requests = physical + reference;
    servo_last_servo_one_sequence = one_sequence;
    if (!connected) {
        servo_previous_buttons = servo_previous_gap_pwm = 0U;
        servo_previous_aim = servo_gap_initialized = 0U;
        servo_last_sequence = sequence;
    } else if (sequence != servo_last_sequence) {
        uint32_t rx_tick = Bluetooth_GetLastRxTick();
        /* Also detect a reconnect when a fresh frame arrived before this
         * module had a chance to observe the link timeout. */
        if (servo_gap_initialized && (uint32_t)(rx_tick - servo_last_rx_tick) > BT_FAILSAFE_TIMEOUT_MS) {
            servo_previous_buttons = 0U;
            servo_previous_aim = servo_gap_initialized = 0U;
        }
        servo_last_sequence = sequence;
        servo_last_rx_tick = rx_tick;
        buttons = (uint16_t)(c->servo_buttons & (uint16_t)~servo_previous_buttons);
        aim = (uint8_t)(c->aim && !servo_previous_aim);
        gap = (uint8_t)(servo_gap_initialized && c->gap_pwm != servo_previous_gap_pwm &&
                       c->gap_pwm >= 500U && c->gap_pwm <= 2500U);
        servo_previous_buttons = c->servo_buttons;
        servo_previous_aim = c->aim;
        servo_previous_gap_pwm = c->gap_pwm;
        servo_gap_initialized = 1U;
        if (aim) ++servo_aim_sequence;
    }

    requests += aim + gap + ((buttons & BT_SERVO_BUTTON_RST) != 0U);
    for (i = 0U; i < sizeof(button_poses) / sizeof(button_poses[0]); ++i)
        if (buttons & button_poses[i].button) ++requests;

    /* All edges above are consumed before selection: no deferred actions. */
    if (physical) { pose = Servo_AIM; name = "PE4"; }
    else if (aim) { pose = Servo_AIM; name = "AIM"; }    else if (buttons & BT_SERVO_BUTTON_RST) { pose = Servo_RST; name = "RST"; }
    else {
        for (i = 0U; i < sizeof(button_poses) / sizeof(button_poses[0]); ++i) {
            if (buttons & button_poses[i].button) {
                pose = button_poses[i].pose;
                name = Servo_GetName(pose);
                break;
            }
        }
        if (pose == ServoCode_NONE && reference) {
            pose = Servo_REFERENCE;
            name = "REFERENCE";
        }
    }
    if (pose == ServoCode_NONE && !gap) {
        BoardInputs_TraceServo(servo_transmit_count, servo_last_transmit_result);
        return;
    }
    if (pose != ServoCode_NONE) name = Servo_GetName(pose);
    if (pose == ServoCode_NONE) {
        name = "GAP";
        result = Servo_SendGap(c->gap_pwm);
    } else result = Servo_SendPreset(pose);
    ++servo_transmit_count;
    servo_last_transmit_result = (unsigned)result;
    (void)snprintf(message, sizeof(message),
        "[SERVO] %s TX=%u dropped=%u buttons=%03X gap=%u\r\n",
        name, (unsigned)result, (requests ? requests - 1U : 0U) + (result == SERVO_BUSY ? 1U : 0U),
        (unsigned)buttons, (unsigned)c->gap_pwm);
    Debug_Log(message);
    BoardInputs_TraceServo(servo_transmit_count, servo_last_transmit_result);
}

uint32_t Bluetooth_GetAimSequence(void) { return servo_aim_sequence; }
