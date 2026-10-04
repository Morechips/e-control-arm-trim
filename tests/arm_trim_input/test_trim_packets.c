#include "arm_trim_input.h"
#include "bluetooth_driver.h"
#include "car_control.h"
#include "pid_tuner.h"
#include "serial_io.h"
#include "servo.h"
#include "board_inputs.h"
#include "../phone20_packet.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart6;
static uint32_t now;
static unsigned checks, trim_count, trim_invalidations, car_count, car_invalidations, pid_count;
static uint8_t trim_buttons, pid_command;
static int16_t trim_direction;
static uint32_t trim_arrival;
static unsigned line_count;
static unsigned combined_count;
static ServoCode combined_preset;
static unsigned combined_requests;
static char trim_line[80];
#define CHECK(c) do { ++checks; assert(c); } while (0)

uint32_t HAL_GetTick(void) { return now; }
void Debug_Log(const char *line) { (void)line; }
uint8_t Debug_CanLog(uint8_t count) { (void)count; return 0U; }
void PID_Tuner_HandleCommand(uint8_t command) { pid_command = command; ++pid_count; }
void Car_Control_SubmitRemoteInput(const CarRemoteInput_t *input)
{ CHECK(input->valid); ++car_count; }
void Car_Control_InvalidateRemoteInput(void) { ++car_invalidations; }
void ArmTrimInput_Submit(uint8_t buttons, int16_t direction, uint32_t arrival_tick)
{ trim_buttons = buttons; trim_direction = direction; trim_arrival = arrival_tick; ++trim_count; }
void ArmTrimInput_SubmitCombined(uint8_t buttons, int16_t direction,
                                 ServoCode preset, uint16_t gap_pwm,
                                 unsigned requests, uint32_t arrival_tick)
{
    (void)gap_pwm;
    trim_buttons = buttons; trim_direction = direction; trim_arrival = arrival_tick;
    combined_preset = preset; combined_requests = requests; ++combined_count;
}
void ArmTrimInput_Invalidate(void) { ++trim_invalidations; }
void ArmTrimInput_HandleLine(const char *line, uint32_t arrival_tick)
{ CHECK(strlen(line) < sizeof(trim_line)); strcpy(trim_line, line); trim_arrival = arrival_tick; ++line_count; }
const char *Servo_GetName(ServoCode code) { (void)code; return "TEST"; }
ServoStatus_t Servo_SendPreset(ServoCode code) { (void)code; assert(0); return SERVO_ERROR; }
ServoStatus_t Servo_SendGap(uint16_t pwm) { (void)pwm; assert(0); return SERVO_ERROR; }
void BoardInputs_TraceServo(uint32_t count, unsigned result) { (void)count; (void)result; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *byte, uint16_t count)
{ CHECK(count == 1U); uart->rx = byte; uart->RxState = 1U; return HAL_OK; }

static void Feed(const uint8_t *data, size_t count)
{
    while (count-- != 0U) {
        *huart6.rx = *data++;
        huart6.RxState = HAL_UART_STATE_READY;
        Bluetooth_RxCallback(&huart6);
    }
}
static void Reset(void)
{
    now += 2000U;
    Bluetooth_Init();
    trim_count = trim_invalidations = car_count = car_invalidations = pid_count = line_count = 0U;
    combined_count = 0U;
}
static void Packet(uint8_t p[7], uint8_t buttons, uint8_t marker, int16_t direction)
{
    p[0] = 0xA5U; p[1] = buttons; p[2] = marker;
    p[3] = (uint8_t)direction; p[4] = (uint8_t)((uint16_t)direction >> 8);
    p[5] = (uint8_t)(p[1] + p[2] + p[3] + p[4]); p[6] = 0x5AU;
}

static void Legacy(uint8_t p[11], int16_t command, int16_t dx, int16_t close, int16_t open)
{
    int16_t values[] = {command, dx, close, open};
    p[0] = 0xA5U; p[9] = 0U; p[10] = 0x5AU;
    for (unsigned i = 0U; i < 4U; ++i) {
        p[1U + 2U * i] = (uint8_t)values[i];
        p[2U + 2U * i] = (uint8_t)((uint16_t)values[i] >> 8);
        p[9] = (uint8_t)(p[9] + p[1U + 2U * i] + p[2U + 2U * i]);
    }
}

int main(void)
{
    const int16_t values[] = {-150, -30, -1, 0, 1, 30, 150};
    uint8_t p[7];
    /* Every bool combination, both supported byte markers, and every split
     * position must remain distinct from five-byte commands and car frames. */
    for (unsigned marker = 0U; marker < 2U; ++marker)
        for (unsigned buttons = 0U; buttons <= 255U; ++buttons)
            for (unsigned value = 0U; value < sizeof(values) / sizeof(values[0]); ++value)
                for (unsigned split = 1U; split < 7U; ++split) {
                    Reset(); Packet(p, (uint8_t)buttons, marker ? 84U : 0U, values[value]);
                    Feed(p, split); Bluetooth_Process(); CHECK(trim_count == 0U);
                    Feed(p + split, 7U - split); Bluetooth_Process();
                    CHECK(trim_count == 1U && trim_buttons == buttons && trim_direction == values[value]);
                    CHECK(trim_arrival == now && Bluetooth_GetTrimSequence() == 1U);
                    CHECK(Bluetooth_GetSequence() == 0U && !Bluetooth_IsConnected() && car_count == 0U && pid_count == 0U);
                }
    /* Numeric validation is downstream: malformed/out-of-range STOP still
     * reaches the consumer, including a 0x5A high byte that resembles PID. */
    {
        const int16_t invalid[] = {INT16_MIN, -151, 151, 1000, 0x5A80, INT16_MAX};
        for (unsigned i = 0U; i < sizeof(invalid) / sizeof(invalid[0]); ++i)
            for (unsigned split = 1U; split < 7U; ++split) {
                Reset(); Packet(p, ARM_TRIM_BUTTON_STOP, 0U, invalid[i]);
                Feed(p, split); Bluetooth_Process(); Feed(p + split, 7U - split); Bluetooth_Process();
                CHECK(trim_count == 1U && trim_direction == invalid[i] && pid_count == 0U);
            }
    }
    Reset(); Packet(p, ARM_TRIM_BUTTON_JOG, 84U, 30);
    Feed(p, 7U); Bluetooth_Process(); Feed(p, 7U); Bluetooth_Process();
    CHECK(Bluetooth_GetTrimSequence() == 2U && Bluetooth_GetTrimPressCount() == 1U);
    Packet(p, 0U, 0U, -1); Feed(p, 7U); Bluetooth_Process();
    Packet(p, ARM_TRIM_BUTTON_JOG, 0U, -1); Feed(p, 7U); Bluetooth_Process();
    CHECK(Bluetooth_GetTrimPressCount() == 2U);
    {
        uint8_t burst[14]; Packet(burst, ARM_TRIM_BUTTON_JOG, 0U, 1);
        Packet(burst + 7U, ARM_TRIM_BUTTON_STOP, 0U, 1);
        Reset(); Feed(burst, sizeof(burst)); Bluetooth_Process(); CHECK(trim_count == 1U);
        Bluetooth_Process(); CHECK(trim_count == 2U && trim_buttons == ARM_TRIM_BUTTON_STOP);
    }
    /* Bad wire checksum/tail/marker is never sent as a trim request and a
     * following valid packet recovers without publishing chassis motion. */
    for (unsigned bad = 0U; bad < 3U; ++bad) {
        Reset(); Packet(p, ARM_TRIM_BUTTON_JOG, 0U, 1);
        if (bad == 0U) ++p[5]; else if (bad == 1U) p[6] = 0U; else p[2] = 85U;
        Feed(p, 7U); Bluetooth_Process(); CHECK(trim_count == 0U);
        now += BT_FRAME_GAP_TIMEOUT_MS + 1U;
        Packet(p, ARM_TRIM_BUTTON_STOP, 0U, -1); Feed(p, 7U); Bluetooth_Process();
        CHECK(trim_count == 1U && car_count == 0U);
    }
    /* Out-of-range trim payloads may have an entire valid PID/reference
     * prefix. Continuation wins for every split and for one complete burst. */
    {
        const uint8_t ambiguous[] = {1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U, 10U, 11U, 0x30U, 0x31U};
        for (unsigned i = 0U; i < sizeof(ambiguous); ++i)
            for (unsigned split = 0U; split < 7U; ++split) {
                int16_t direction = (int16_t)(0x5A00U | ambiguous[i]);
                Reset(); Packet(p, ambiguous[i], 0U, direction);
                if (split != 0U) {
                    Feed(p, split); Bluetooth_Process();
                    CHECK(trim_count == 0U && pid_count == 0U && Bluetooth_GetServoOneSequence() == 0U);
                }
                Feed(p + split, sizeof(p) - split); Bluetooth_Process();
                CHECK(trim_count == 1U && trim_buttons == ambiguous[i] && trim_direction == direction);
                CHECK(pid_count == 0U && car_count == 0U && Bluetooth_GetServoOneSequence() == 0U);
            }
    }
    /* Feature1 five-byte commands wait for a definite boundary. The exact
     * 100 ms gap is still a possible continuation, with wrap-safe timing. */
    for (unsigned command = 1U; command <= 11U; ++command) {
        uint8_t short_frame[5] = {0xA5U, (uint8_t)command, 0U, (uint8_t)command, 0x5AU};
        Reset(); Feed(short_frame, sizeof(short_frame)); Bluetooth_Process();
        CHECK(pid_count == 0U && trim_count == 0U);
        now += BT_FRAME_GAP_TIMEOUT_MS; Bluetooth_Process(); CHECK(pid_count == 0U);
        ++now; Bluetooth_Process();
        CHECK(pid_count == 1U && pid_command == command && trim_count == 0U && car_count == 0U);
        uint8_t burst[12]; memcpy(burst, short_frame, sizeof(short_frame));
        Packet(burst + sizeof(short_frame), ARM_TRIM_BUTTON_JOG, 0U, 1);
        Reset(); Feed(burst, sizeof(burst)); Bluetooth_Process();
        CHECK(pid_count == 1U && pid_command == command && trim_count == 0U);
        Bluetooth_Process(); CHECK(trim_count == 1U && trim_buttons == ARM_TRIM_BUTTON_JOG && pid_count == 1U);
        /* A bad short checksum never consumes the header after it. */
        for (unsigned split = 0U; split < 2U; ++split) {
            Reset(); ++burst[3];
            if (split != 0U) { Feed(burst, 5U); Bluetooth_Process(); CHECK(pid_count == 0U); }
            Feed(burst + (split ? 5U : 0U), sizeof(burst) - (split ? 5U : 0U)); Bluetooth_Process();
            CHECK(trim_count == 1U && trim_buttons == ARM_TRIM_BUTTON_JOG && pid_count == 0U && car_count == 0U);
            --burst[3];
        }
    }
    {
        uint8_t short_frame[5] = {0xA5U, 1U, 0U, 1U, 0x5AU};
        Reset(); now = UINT32_MAX - 50U; Feed(short_frame, sizeof(short_frame)); Bluetooth_Process();
        now += 100U; Bluetooth_Process(); CHECK(pid_count == 0U);
        ++now; Bluetooth_Process(); CHECK(pid_count == 1U);
        Reset(); Feed(short_frame, sizeof(short_frame)); now += 501U; Bluetooth_Process(); CHECK(pid_count == 0U);
    }
    {
        uint8_t press[] = {0xA5U, 0x31U, 0U, 0x31U, 0x5AU};
        uint8_t release[] = {0xA5U, 0x30U, 0U, 0x30U, 0x5AU};
        Reset(); Feed(press, sizeof(press)); Bluetooth_Process();
        CHECK(Bluetooth_GetServoOneSequence() == 0U);
        now += BT_FRAME_GAP_TIMEOUT_MS + 1U; Bluetooth_Process();
        CHECK(Bluetooth_GetServoOneSequence() == 1U && car_count == 0U && trim_count == 0U);
        Feed(release, sizeof(release)); Bluetooth_Process(); Feed(press, sizeof(press)); Bluetooth_Process();
        CHECK(Bluetooth_GetServoOneSequence() == 1U);
        now += BT_FRAME_GAP_TIMEOUT_MS + 1U; Bluetooth_Process();
        CHECK(Bluetooth_GetServoOneSequence() == 2U);
    }
    /* The old independent four-short page uses command edges, no car lease,
     * and queues only the replacement trim text interface. */
    {
        const char *expected[] = {"", "@BENCH PREP BALL", "@BENCH PREP HOSTAGE", "@BENCH PREP BUCKET",
            "@ARM TRIM BEGIN BALL", "@ARM TRIM BEGIN HOSTAGE", "@ARM TRIM BEGIN BUCKET", "@ARM TRIM DX -30",
            "@ARM TRIM END", "@BENCH GRIP 500", "@BENCH GRIP 1800", "@ARM TRIM STATUS", "@ARM TRIM STOP", "@ARM TRIM CLEAR"};
        uint8_t legacy[11];
        for (unsigned command = 0U; command <= 13U; ++command)
            for (unsigned split = 1U; split < sizeof(legacy); ++split) {
                Reset(); Legacy(legacy, (int16_t)command, -30, 500, 1800);
                Feed(legacy, split); Bluetooth_Process(); CHECK(line_count == 0U);
                Feed(legacy + split, sizeof(legacy) - split); Bluetooth_Process();
                CHECK(line_count == (command != 0U) && trim_count == 0U && car_count == 0U && pid_count == 0U);
                if (command != 0U) CHECK(strcmp(trim_line, expected[command]) == 0 && trim_arrival == now);
                Feed(legacy, sizeof(legacy)); Bluetooth_Process(); CHECK(line_count == (command != 0U));
            }
        Reset(); Legacy(legacy, 7, 30, 0, 0); Feed(legacy, sizeof(legacy)); now += 501U; Bluetooth_Process();
        CHECK(line_count == 0U && Bluetooth_GetTrimSequence() == 1U);
        Feed(legacy, sizeof(legacy)); Bluetooth_Process(); CHECK(line_count == 0U);
        Legacy(legacy, 0, 0, 0, 0); Feed(legacy, sizeof(legacy)); Bluetooth_Process();
        Legacy(legacy, 7, 30, 0, 0); Feed(legacy, sizeof(legacy)); Bluetooth_Process();
        CHECK(line_count == 1U && strcmp(trim_line, "@ARM TRIM DX 30") == 0 && car_count == 0U);
        for (unsigned bad = 0U; bad < 4U; ++bad) {
            Reset(); Legacy(legacy, bad == 0U ? 14 : 9, bad == 1U ? 151 : 0,
                bad == 2U ? 499 : 500, 1800);
            if (bad == 3U) ++legacy[9];
            Feed(legacy, sizeof(legacy)); Bluetooth_Process(); CHECK(line_count == 0U && trim_count == 0U && car_count == 0U);
        }
    }
    /* Interleaved real phone20 traffic still publishes only its own lease. */
    {
        uint8_t phone[41]; int16_t shorts[16] = {0};
        Reset(); PackPhone20(phone, shorts, BT_SERVO_BUTTON_TB_B, 500U);
        for (unsigned split = 1U; split < sizeof(phone); ++split) {
            Feed(phone, split); Bluetooth_Process(); Feed(phone + split, sizeof(phone) - split); Bluetooth_Process();
        }
        CHECK(car_count == 40U && Bluetooth_GetSequence() == 40U && trim_count == 0U);
        uint32_t car_tick = Bluetooth_GetLastRxTick(); now += 400U;
        Packet(p, ARM_TRIM_BUTTON_JOG, 0U, 30); Feed(p, 7U); Bluetooth_Process();
        CHECK(car_count == 40U && Bluetooth_GetLastRxTick() == car_tick);
        now += 101U; CHECK(!Bluetooth_IsConnected());
    }
    /* All sixteen bools fit the same two bytes. Every split must keep the
     * unified frame distinct from short/legacy packets, including bit15. */
    {
        uint8_t phone[41]; int16_t shorts[16] = {0};
        shorts[11] = 1; shorts[13] = 5; shorts[14] = -30;
        for (unsigned bit = 0U; bit < 16U; ++bit)
            for (unsigned split = 1U; split < sizeof(phone); ++split) {
                Reset(); PackPhone20(phone, shorts, (uint16_t)(1U << bit), 0U);
                Feed(phone, split); Bluetooth_Process(); CHECK(car_count == 0U);
                Feed(phone + split, sizeof(phone) - split); Bluetooth_Process();
                CHECK(car_count == 1U && trim_count == 0U && Bluetooth_GetSequence() == 1U);
                Bluetooth_DispatchServoActions(0U);
                CHECK(combined_count == 1U && trim_direction == -30 && pid_count == 1U);
                CHECK((trim_buttons & ARM_TRIM_BUTTON_STATUS) != 0U);
                if (bit == 12U) CHECK((trim_buttons & ARM_TRIM_BUTTON_JOG) != 0U);
                if (bit == 13U) CHECK((trim_buttons & ARM_TRIM_BUTTON_CLOSE) != 0U);
                if (bit == 14U) CHECK((trim_buttons & ARM_TRIM_BUTTON_OPEN) != 0U);
                if (bit == 15U) CHECK((trim_buttons & ARM_TRIM_BUTTON_STOP) != 0U);
                Feed(phone, sizeof(phone)); Bluetooth_Process(); Bluetooth_DispatchServoActions(0U);
                CHECK(combined_requests == 0U && pid_count == 1U);
            }
        Reset(); memset(shorts, 0, sizeof(shorts)); shorts[15] = 1;
        PackPhone20(phone, shorts, 0U, 0U);
        Feed(phone, sizeof(phone)); Bluetooth_Process(); Bluetooth_DispatchServoActions(0U);
        CHECK(combined_preset == Servo_REFERENCE && combined_requests == 1U);
    }
    /* Switching to any shorter chassis schema clears unified-only fields.
     * Old neutral packets cannot renew held WT or repeat PID/reference. */
    {
        const unsigned lengths[] = {21U, 23U, 25U, 27U, 29U, 31U, 35U};
        uint8_t phone[41], legacy[35]; int16_t shorts[16] = {0};
        shorts[11] = 1; shorts[13] = 9; shorts[14] = -1; shorts[15] = 1;
        for (unsigned i = 0U; i < sizeof(lengths) / sizeof(lengths[0]); ++i) {
            Reset(); PackPhone20(phone, shorts, BT_CONTROL_BOOL_JOG_BIT, 0U);
            Feed(phone, sizeof(phone)); Bluetooth_Process(); Bluetooth_DispatchServoActions(0U);
            CHECK(trim_buttons != 0U && trim_direction == -1 && pid_count == 1U);
            memset(legacy, 0, sizeof(legacy));
            legacy[0] = 0xA5U; legacy[lengths[i] - 1U] = 0x5AU;
            Feed(legacy, lengths[i]); Bluetooth_Process(); Bluetooth_DispatchServoActions(0U);
            CHECK(car_count == 2U && combined_count == 2U);
            CHECK(trim_buttons == 0U && trim_direction == 0 && pid_count == 1U);
            CHECK(combined_preset == ServoCode_NONE && combined_requests == 0U);
            const BluetoothControlFrame *decoded = &Bluetooth_GetControl()->frame;
            CHECK(decoded->pid_command == 0U && decoded->reference_pressed == 0U);
        }
    }
    /* RX errors and overflow signal an ISR-safe invalidation immediately. */
    {
        uint8_t phone[41]; int16_t shorts[16] = {0};
        Reset(); shorts[3] = 1; PackPhone20(phone, shorts, 0U, 0U);
        Feed(phone, sizeof(phone)); Bluetooth_Process();
        CHECK(car_count == 1U && trim_invalidations == 1U && trim_count == 0U);
        Packet(p, 0U, 0U, 0); Feed(p, 7U); Bluetooth_Process();
        CHECK(trim_count == 1U && trim_invalidations == 1U && car_count == 1U);
    }
    Reset(); Bluetooth_ErrorCallback(&huart6);
    CHECK(trim_invalidations == 1U && car_invalidations == 1U && trim_count == 0U);
    Bluetooth_Process(); CHECK(trim_invalidations >= 2U && bluetooth_rx_recoveries == 1U);
    Reset(); for (unsigned i = 0U; i < 300U; ++i) Feed((const uint8_t[]){0x01U}, 1U);
    CHECK(trim_invalidations != 0U && car_invalidations != 0U && trim_count == 0U);
    printf("PASS trim packet parser: %u checks; splits, bools, signs, STOP, legacy frames, lease and ISR invalidation\n", checks);
    return 0;
}
