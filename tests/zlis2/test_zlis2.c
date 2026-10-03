#include "servo.h"
#include "servo.h"
#include "servo.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static int peripheral;
static UART_HandleTypeDef uart = {
    &peripheral,
    {115200U, UART_WORDLENGTH_8B, UART_STOPBITS_1, UART_PARITY_NONE,
     UART_MODE_TX_RX, UART_HWCONTROL_NONE},
    HAL_UART_STATE_READY, HAL_UART_STATE_READY, NULL
};
static HAL_StatusTypeDef tx_result = HAL_OK;
static unsigned int calls, checks;
static size_t sent_length;
static char sent[1024];
static char history[512][SERVO_MAX_TX_LENGTH + 1U];
static uint32_t tick;
static unsigned aborts;
static HAL_StatusTypeDef abort_result = HAL_OK;

uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *handle)
{
    ++aborts;
    if (abort_result == HAL_OK) handle->gState = HAL_UART_STATE_READY;
    return abort_result;
}

static const uint8_t *inflight;
static uint16_t inflight_length;
static uint8_t complete_immediately = 1U;

HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *handle, const uint8_t *data,
                                       uint16_t length)
{
    assert(handle == &uart);
    assert(data != NULL && length > 0U && length < sizeof(sent));
    memcpy(sent, data, length);
    sent[length] = '\0';
    sent_length = length;
    assert(calls < 512U && length <= SERVO_MAX_TX_LENGTH);
    memcpy(history[calls], data, length); history[calls][length] = '\0';
    ++calls;
    if (tx_result == HAL_OK) {
        inflight = data;
        inflight_length = length;
        handle->gState = HAL_UART_STATE_BUSY_TX;
        if (complete_immediately) {
            handle->gState = HAL_UART_STATE_READY;
            Servo_TxCallback(handle);
        }
    }
    return tx_result;
}

#define EXPECT_TX(expression, expected) do { \
    unsigned int before = calls; \
    assert((expression) == SERVO_OK); \
    assert(calls == before + 1U); \
    assert(sent_length == strlen(expected)); \
    assert(memcmp(sent, expected, sent_length) == 0); \
    ++checks; \
} while (0)
#define EXPECT_NO_TX(expression, status) do { \
    unsigned int before = calls; \
    assert((expression) == (status)); \
    assert(calls == before); \
    ++checks; \
} while (0)

static void check_uninitialized(void)
{
    ServoCommand_t c = {0U, 1500U, 1000U};
    static const uint16_t values[1] = {1500U};
    EXPECT_NO_TX(Servo_SetChannel(0, 1500, 1000), SERVO_NOT_INITIALIZED);
    EXPECT_NO_TX(Servo_SetCommands(&c, 1), SERVO_NOT_INITIALIZED);
    EXPECT_NO_TX(Servo_SetValues(values, 1U, 2000U), SERVO_NOT_INITIALIZED);
    EXPECT_NO_TX(Servo_RunAction(0), SERVO_NOT_INITIALIZED);
    EXPECT_NO_TX(Servo_RunActionRange(0, 1, 0), SERVO_NOT_INITIALIZED);
    EXPECT_NO_TX(Servo_StopAll(), SERVO_NOT_INITIALIZED);
    EXPECT_NO_TX(Servo_StopServo(0), SERVO_NOT_INITIALIZED);
    EXPECT_NO_TX(Servo_SetBias(0, 0), SERVO_NOT_INITIALIZED);
    EXPECT_NO_TX(Servo_RunCombinedAction(0, 0), SERVO_NOT_INITIALIZED);
    EXPECT_NO_TX(Servo_RecordPose(), SERVO_NOT_INITIALIZED);
    EXPECT_NO_TX(Servo_RunRecorded(0), SERVO_NOT_INITIALIZED);
    EXPECT_NO_TX(Servo_ClearRecorded(), SERVO_NOT_INITIALIZED);
    EXPECT_NO_TX(Servo_SetRecordPeriod(0), SERVO_NOT_INITIALIZED);
    EXPECT_NO_TX(Servo_Reset(), SERVO_NOT_INITIALIZED);
    EXPECT_NO_TX(Servo_SendRaw("$DST!"), SERVO_NOT_INITIALIZED);
}

static void complete_at(uint32_t at)
{
    tick = at;
    uart.gState = HAL_UART_STATE_READY;
    Servo_TxCallback(&uart);
}

static void check_owned(void)
{
    static int owner, competitor;
    static const uint16_t positions[3] = {1356U, 1850U, 698U};
    const ServoCommand_t grip = {3U, 500U, 1500U};
    ServoTransferStatus_t status;
    unsigned before, stops, before_aborts;

    complete_immediately = 0U;
    EXPECT_TX(Servo_StopAll(), "$DST!");
    EXPECT_NO_TX(Servo_Acquire(&owner), SERVO_BUSY);
    complete_at(99U);
    EXPECT_NO_TX(Servo_Acquire(NULL), SERVO_INVALID_PARAM);
    EXPECT_NO_TX(Servo_Acquire(&owner), SERVO_OK);
    EXPECT_NO_TX(Servo_Acquire(&competitor), SERVO_BUSY);
    assert(Servo_GetTransferStatus(&owner).state == SERVO_TRANSFER_NONE);
    assert(Servo_GetTransferStatus(&competitor).state == SERVO_TRANSFER_FAILED);
    tick = 100U;
    EXPECT_TX(Servo_SetValuesOwned(&owner, positions, 3U, 50U),
        "{#000P1356T0050!#001P1850T0050!#002P0698T0050!}");
    status = Servo_GetTransferStatus(&owner);
    assert(status.state == SERVO_TRANSFER_PENDING && status.started && status.started_tick == 100U);
    EXPECT_NO_TX(Servo_SendPreset(Servo_AIM), SERVO_BUSY);
    EXPECT_NO_TX(Servo_SendGap(500U), SERVO_BUSY);
    EXPECT_NO_TX(Servo_SendRaw("$RST!"), SERVO_BUSY);
    EXPECT_NO_TX(Servo_StopAll(), SERVO_BUSY);
    EXPECT_NO_TX(Servo_Init(&uart), SERVO_BUSY);
    EXPECT_NO_TX(Servo_SetValuesOwned(&owner, positions, 3U, 50U), SERVO_BUSY);
    EXPECT_NO_TX(Servo_StopServoOwned(&owner, 0U), SERVO_BUSY);
    EXPECT_NO_TX(Servo_Release(&owner), SERVO_BUSY);
    EXPECT_NO_TX(Servo_CancelPendingOwned(&competitor), SERVO_BUSY);
    complete_at(105U);
    status = Servo_GetTransferStatus(&owner);
    assert(status.state == SERVO_TRANSFER_COMPLETE && status.result == SERVO_OK && status.completed_tick == 105U);
    complete_immediately = 1U; tick = 110U;
    EXPECT_TX(Servo_SetCommandsOwned(&owner, &grip, 1U), "{#003P0500T1500!}");
    status = Servo_GetTransferStatus(&owner);
    assert(status.state == SERVO_TRANSFER_COMPLETE && status.started && status.started_tick == 110U && status.completed_tick == 110U);
    EXPECT_NO_TX(Servo_SetValuesOwned(&owner, NULL, 3U, 50U), SERVO_INVALID_PARAM);
    EXPECT_NO_TX(Servo_SetValuesOwned(&owner, positions, 5U, 50U), SERVO_BUFFER_OVERFLOW);
    EXPECT_NO_TX(Servo_SetCommandsOwned(&owner, NULL, 1U), SERVO_INVALID_PARAM);
    EXPECT_NO_TX(Servo_StopServoOwned(&owner, 255U), SERVO_INVALID_PARAM);

    /* Cancel preserves the active frame and stops are completion-serialized.
     * A pending stop can itself be cancelled without its bytes reaching UART. */
    complete_immediately = 0U; tick = 120U;
    EXPECT_TX(Servo_SetValuesOwned(&owner, positions, 3U, 50U),
        "{#000P1356T0050!#001P1850T0050!#002P0698T0050!}");
    EXPECT_NO_TX(Servo_CancelPendingOwned(&owner), SERVO_OK);
    assert(Servo_GetTransferStatus(&owner).state == SERVO_TRANSFER_NONE && !Servo_IsIdle());
    assert(memcmp(inflight, sent, inflight_length) == 0);
    EXPECT_NO_TX(Servo_SetValuesOwned(&owner, positions, 3U, 50U), SERVO_BUSY);
    EXPECT_NO_TX(Servo_StopServoOwned(&owner, 0U), SERVO_OK);
    assert(!Servo_GetTransferStatus(&owner).started);
    EXPECT_NO_TX(Servo_StopServoOwned(&owner, 1U), SERVO_BUSY);
    EXPECT_NO_TX(Servo_CancelPendingOwned(&owner), SERVO_OK);
    EXPECT_NO_TX(Servo_StopServoOwned(&owner, 0U), SERVO_OK);
    stops = calls;
    complete_at(125U);
    assert(strcmp(sent, "$DST:0!") == 0);
    status = Servo_GetTransferStatus(&owner);
    assert(status.state == SERVO_TRANSFER_PENDING && status.started_tick == 125U);
    complete_at(126U);
    assert(Servo_GetTransferStatus(&owner).state == SERVO_TRANSFER_COMPLETE);
    EXPECT_TX(Servo_StopServoOwned(&owner, 1U), "$DST:1!");
    EXPECT_NO_TX(Servo_StopServoOwned(&owner, 2U), SERVO_BUSY);
    complete_at(127U);
    EXPECT_TX(Servo_StopServoOwned(&owner, 2U), "$DST:2!");
    complete_at(128U);
    assert(calls == stops + 3U && strcmp(history[stops], "$DST:0!") == 0 &&
        strcmp(history[stops + 1U], "$DST:1!") == 0 && strcmp(history[stops + 2U], "$DST:2!") == 0);

    /* The caller sees a queued dispatch failure, even though the UART is idle. */
    tick = 140U;
    EXPECT_TX(Servo_SetValuesOwned(&owner, positions, 3U, 50U),
        "{#000P1356T0050!#001P1850T0050!#002P0698T0050!}");
    EXPECT_NO_TX(Servo_CancelPendingOwned(&owner), SERVO_OK);
    EXPECT_NO_TX(Servo_StopServoOwned(&owner, 0U), SERVO_OK);
    tx_result = HAL_ERROR; complete_at(142U);
    status = Servo_GetTransferStatus(&owner);
    assert(Servo_IsIdle() && status.state == SERVO_TRANSFER_FAILED && !status.started &&
        status.result == SERVO_UART_ERROR && status.completed_tick == 142U);
    before = calls; Servo_Process(); assert(calls == before);
    tx_result = HAL_OK;
    for (unsigned failure = HAL_ERROR; failure <= HAL_TIMEOUT; ++failure) {
        tx_result = (HAL_StatusTypeDef)failure;
        assert(Servo_StopServoOwned(&owner, 0U) == (failure == HAL_BUSY ? SERVO_BUSY : SERVO_UART_ERROR));
        status = Servo_GetTransferStatus(&owner);
        assert(status.state == SERVO_TRANSFER_FAILED && !status.started);
        before = calls; Servo_Process(); assert(calls == before); ++checks;
    }
    tx_result = HAL_OK;

    tick = UINT32_MAX - 10U; before_aborts = aborts;
    EXPECT_TX(Servo_StopServoOwned(&owner, 0U), "$DST:0!");
    tick += SERVO_TX_TIMEOUT_MS - 1U; Servo_Process();
    assert(Servo_GetTransferStatus(&owner).state == SERVO_TRANSFER_PENDING && aborts == before_aborts);
    ++tick; Servo_Process();
    status = Servo_GetTransferStatus(&owner);
    assert(status.state == SERVO_TRANSFER_FAILED && status.result == SERVO_UART_ERROR &&
        status.started && status.completed_tick == tick && aborts == before_aborts + 1U && Servo_IsIdle());
    before = calls; Servo_Process(); assert(calls == before);

    /* Abort refusal protects current bytes and retains the lease until TC. */
    tick = 200U;
    EXPECT_TX(Servo_StopServoOwned(&owner, 1U), "$DST:1!");
    abort_result = HAL_ERROR; tick += SERVO_TX_TIMEOUT_MS; Servo_Process();
    assert(Servo_GetTransferStatus(&owner).state == SERVO_TRANSFER_FAILED && !Servo_IsIdle());
    EXPECT_NO_TX(Servo_Release(&owner), SERVO_BUSY);
    abort_result = HAL_OK; complete_at(241U);
    assert(Servo_GetTransferStatus(&owner).state == SERVO_TRANSFER_FAILED);
    EXPECT_NO_TX(Servo_Release(&owner), SERVO_OK);

    /* Unowned traffic still has no new deadline or ownership restrictions. */
    EXPECT_TX(Servo_StopAll(), "$DST!");
    before_aborts = aborts; tick += 1000U; Servo_Process();
    assert(!Servo_IsIdle() && aborts == before_aborts);
    complete_at(tick);
    complete_immediately = 1U;
    EXPECT_TX(Servo_SendGap(500U), "{#003P0500T2000!}");
}

int main(void)
{
    ServoCommand_t pair[] = {{0, 1500, 1000}, {1, 900, 1000}};
    ServoCommand_t full[24];
    char expected[363], raw[364];
    UART_HandleTypeDef bad;
    size_t i;
    check_uninitialized();
    EXPECT_NO_TX(Servo_Init(NULL), SERVO_INVALID_PARAM);
    EXPECT_NO_TX(Servo_Init(&uart), SERVO_OK);

    EXPECT_TX(Servo_SetChannel(0, 500, 2000), "#000P0500T2000!");
    EXPECT_TX(Servo_SetChannel(4, 1500, 1000), "#004P1500T1000!");
    EXPECT_TX(Servo_SetChannel(254, 2500, 9999), "#254P2500T9999!");
    EXPECT_TX(Servo_SetChannel(0, 500, 0), "#000P0500T0000!");
    EXPECT_TX(Servo_SetCommands(pair, 2), "{#000P1500T1000!#001P0900T1000!}");
    EXPECT_TX(Servo_SetCommands(pair, 1), "{#000P1500T1000!}");
    EXPECT_TX(Servo_RunAction(0), "$DGS:0!");
    EXPECT_TX(Servo_RunAction(65535), "$DGS:65535!");
    EXPECT_TX(Servo_RunActionRange(0, 10, 1), "$DGT:0-10,1!");
    EXPECT_TX(Servo_RunActionRange(0, 10, 0), "$DGT:0-10,0!");
    EXPECT_TX(Servo_RunActionRange(0, 4, 1), "$DGT:0-4,1!");
    EXPECT_TX(Servo_RunActionRange(65535, 65535, 65535), "$DGT:65535-65535,65535!");
    EXPECT_TX(Servo_StopAll(), "$DST!");
    EXPECT_TX(Servo_StopServo(5), "$DST:5!");
    EXPECT_TX(Servo_StopServo(254), "$DST:254!");
    EXPECT_TX(Servo_SetBias(5, 10), "#005PSCK+010!");
    EXPECT_TX(Servo_SetBias(5, -10), "#005PSCK-010!");
    EXPECT_TX(Servo_SetBias(0, 0), "#000PSCK+000!");
    EXPECT_TX(Servo_SetBias(254, 500), "#254PSCK+500!");
    EXPECT_TX(Servo_SetBias(254, -500), "#254PSCK-500!");
    EXPECT_TX(Servo_RunCombinedAction(1, 5), "$DKT:1,5!");
    EXPECT_TX(Servo_RunCombinedAction(65535, 65535), "$DKT:65535,65535!");
    EXPECT_TX(Servo_RecordPose(), "$DJ_RECORD!");
    EXPECT_TX(Servo_RunRecorded(1), "$DJ_RECORD_DO:1!");
    EXPECT_TX(Servo_RunRecorded(0), "$DJ_RECORD_DO:0!");
    EXPECT_TX(Servo_RunRecorded(65535), "$DJ_RECORD_DO:65535!");
    EXPECT_TX(Servo_ClearRecorded(), "$DJ_RECORD_CLEAR!");
    EXPECT_TX(Servo_SetRecordPeriod(1000), "$DJ_RECORD_TIME:1000!");
    EXPECT_TX(Servo_SetRecordPeriod(0), "$DJ_RECORD_TIME:0!");
    EXPECT_TX(Servo_SetRecordPeriod(65535), "$DJ_RECORD_TIME:65535!");
    EXPECT_TX(Servo_Reset(), "$RST!");
    EXPECT_TX(Servo_SendRaw("$DST!\r\n"), "$DST!\r\n");

    EXPECT_TX(Servo_SendPreset(Servo_AIM),
              "{#000P1058T2000!#001P0821T2000!#002P0554T2000!#003P1499T2000!}");
    EXPECT_TX(Servo_SendPreset(TakeHostage_Catch),
              "{#000P1667T2000!#001P1882T2000!#002P0651T2000!#003P1483T2000!}");
    EXPECT_TX(Servo_SendPreset(TakeHostage_PreGrab),
              "{#000P1684T2000!#001P2136T2000!#002P0785T2000!#003P1483T2000!}");
    EXPECT_TX(Servo_SendPreset(TakeHostage_Gap),
              "{#000P1667T2000!#001P1882T2000!#002P0651T2000!#003P0500T2000!}");
    EXPECT_TX(Servo_SendPreset(Servo_REFERENCE),
              "{#000P1532T2000!#001P2219T2000!#002P1202T2000!}");
    EXPECT_TX(Servo_SendGap(500U), "{#003P0500T2000!}");
    EXPECT_NO_TX(Servo_SendPreset(ServoCode_NONE), SERVO_INVALID_PARAM);
    EXPECT_NO_TX(Servo_SendPreset((ServoCode)-1), SERVO_INVALID_PARAM);
    EXPECT_NO_TX(Servo_SendPreset((ServoCode)99), SERVO_INVALID_PARAM);
    EXPECT_NO_TX(Servo_SendGap(499U), SERVO_INVALID_PARAM);
    EXPECT_NO_TX(Servo_SendGap(2501U), SERVO_INVALID_PARAM);
    pair[0].time_ms = 65535U; /* Out of range for the T field. */
    EXPECT_NO_TX(Servo_SetCommands(pair, 2), SERVO_INVALID_PARAM);
    pair[0].time_ms = 1000U;

    /* Fixed-pose entry point: channels numbered from first_id, one shared time. */
    {
        static const uint16_t four[4] = {1058U, 821U, 554U, 1499U};
        static const uint16_t three[4] = {1532U, 2219U, 1202U, 0U};
        static const uint16_t gap[1] = {500U};
        EXPECT_TX(Servo_SetValues(four, 4U, 2000U),
                  "{#000P1058T2000!#001P0821T2000!#002P0554T2000!#003P1499T2000!}");
        EXPECT_TX(Servo_SetValues(three, 3U, 1500U),
                  "{#000P1532T1500!#001P2219T1500!#002P1202T1500!}");
        EXPECT_TX(Servo_SetValues(four, 1U, 0U), "{#000P1058T0000!}");
        EXPECT_TX(Servo_SetValues(gap, 1U, 2000U), "{#000P0500T2000!}");
        EXPECT_NO_TX(Servo_SetValues(NULL, 1U, 2000U), SERVO_INVALID_PARAM);
        EXPECT_NO_TX(Servo_SetValues(four, 0U, 2000U), SERVO_INVALID_PARAM);
        EXPECT_NO_TX(Servo_SetValues(four, 5U, 2000U), SERVO_BUFFER_OVERFLOW);
        EXPECT_NO_TX(Servo_SetValues(four, SIZE_MAX, 2000U), SERVO_BUFFER_OVERFLOW);
        {
            static const uint16_t low[2] = {1500U, 499U};
            static const uint16_t high[2] = {2501U, 1500U};
            EXPECT_NO_TX(Servo_SetValues(low, 2U, 2000U), SERVO_INVALID_PARAM);
            EXPECT_NO_TX(Servo_SetValues(high, 2U, 2000U), SERVO_INVALID_PARAM);
            EXPECT_NO_TX(Servo_SetValues(low, 2U, 10000U), SERVO_INVALID_PARAM);
        }
        /* All enum rows are checked through formatting and actual queued TX. */
        assert(TakeBall_Before == 0 && TakeBall_Mid == 1 && TakeBall_Gap == 2);
        assert(BarrelDown_Up == 3 && BarrelDown_Down == 4);
        assert(TakeHostage_Up == 5 && TakeHostage_Catch == 6 && TakeHostage_Gap == 7 && TakeHostage_Leave == 8);
        assert(Servo_REFERENCE == 9 && Servo_RST == 10 && Servo_AIM == 11 && TakeHostage_PreGrab == 12 && ServoCode_MAX == 13);
        for (i = 0U; i < (size_t)ServoCode_MAX; ++i) {
            char pose_frame[63];
            size_t length;
            assert(Servo_GetName((ServoCode)i)[0] != '\0');
            assert(servo_code_counts[i] >= 1U && servo_code_counts[i] <= 4U);
            for (size_t ch = 0U; ch < servo_code_counts[i]; ++ch) {
                assert(codes[i][ch].id == ch && codes[i][ch].pwm >= 500U && codes[i][ch].pwm <= 2500U);
                assert(codes[i][ch].time_ms == 2000U);
            }
            assert(GetFullCommand((ServoCode)i, pose_frame, sizeof(pose_frame), &length));
            assert(length == 2U + 15U * servo_code_counts[i]);
            EXPECT_TX(Servo_SendPreset((ServoCode)i), pose_frame);
        }
        {
            char frame[63]; size_t length = 99U;
            assert(!GetFullCommand(ServoCode_NONE, frame, sizeof(frame), &length) && length == 0U);
            assert(!GetFullCommand(ServoCode_MAX, frame, sizeof(frame), &length));
            assert(!GetFullCommand(Servo_AIM, frame, 62U, &length));
            assert(GetFullCommand(Servo_REFERENCE, frame, 48U, &length) && length == 47U);
            assert(!GetFullCommand(Servo_REFERENCE, frame, 47U, &length));
            assert(!GetFullCommand(Servo_AIM, NULL, 63U, &length));
            assert(!GetFullCommand(Servo_AIM, frame, sizeof(frame), NULL));
        }
        assert(strcmp(Servo_GetName(ServoCode_NONE), "NONE") == 0);
        assert(strcmp(Servo_GetName(ServoCode_MAX), "NONE") == 0);
    }

    EXPECT_NO_TX(Servo_SetChannel(255, 1500, 1000), SERVO_INVALID_PARAM);
    EXPECT_NO_TX(Servo_SetChannel(0, 499, 1000), SERVO_INVALID_PARAM);
    EXPECT_NO_TX(Servo_SetChannel(0, 2501, 1000), SERVO_INVALID_PARAM);
    EXPECT_NO_TX(Servo_SetChannel(0, 1500, 10000), SERVO_INVALID_PARAM);
    EXPECT_NO_TX(Servo_StopServo(255), SERVO_INVALID_PARAM);
    EXPECT_NO_TX(Servo_SetBias(255, 0), SERVO_INVALID_PARAM);
    EXPECT_NO_TX(Servo_SetBias(5, 501), SERVO_INVALID_PARAM);
    EXPECT_NO_TX(Servo_SetBias(5, -501), SERVO_INVALID_PARAM);
    EXPECT_NO_TX(Servo_SetBias(5, INT16_MIN), SERVO_INVALID_PARAM);
    EXPECT_NO_TX(Servo_RunActionRange(10, 0, 1), SERVO_INVALID_PARAM);
    EXPECT_NO_TX(Servo_SetCommands(NULL, 1), SERVO_INVALID_PARAM);
    EXPECT_NO_TX(Servo_SetCommands(pair, 0), SERVO_INVALID_PARAM);
    EXPECT_NO_TX(Servo_SetCommands(pair, 25), SERVO_BUFFER_OVERFLOW);
    EXPECT_NO_TX(Servo_SetCommands(pair, SIZE_MAX), SERVO_BUFFER_OVERFLOW);
    pair[1].id = 255;
    EXPECT_NO_TX(Servo_SetCommands(pair, 2), SERVO_INVALID_PARAM);
    pair[1].id = 1; pair[1].pwm = 499;
    EXPECT_NO_TX(Servo_SetCommands(pair, 2), SERVO_INVALID_PARAM);
    pair[1].pwm = 2501;
    EXPECT_NO_TX(Servo_SetCommands(pair, 2), SERVO_INVALID_PARAM);
    pair[1].pwm = 1500; pair[1].time_ms = 10000;
    EXPECT_NO_TX(Servo_SetCommands(pair, 2), SERVO_INVALID_PARAM);
    EXPECT_NO_TX(Servo_SendRaw(NULL), SERVO_INVALID_PARAM);
    EXPECT_NO_TX(Servo_SendRaw(""), SERVO_INVALID_PARAM);

    expected[0] = '{';
    for (i = 0; i < 24; ++i) {
        full[i].id = 254; full[i].pwm = 2500; full[i].time_ms = 9999;
        memcpy(expected + 1 + i * 15, "#254P2500T9999!", 15);
    }
    expected[361] = '}'; expected[362] = '\0';
    EXPECT_TX(Servo_SetCommands(full, 24), expected);
    full[23].id = 255;
    EXPECT_NO_TX(Servo_SetCommands(full, 24), SERVO_INVALID_PARAM);
    memcpy(raw, expected, sizeof(expected));
    EXPECT_TX(Servo_SendRaw(raw), expected);
    raw[362] = '!'; raw[363] = '\0';
    EXPECT_NO_TX(Servo_SendRaw(raw), SERVO_BUFFER_OVERFLOW);
    memset(raw, 'X', sizeof(raw));
    EXPECT_NO_TX(Servo_SendRaw(raw), SERVO_BUFFER_OVERFLOW);

    for (i = HAL_ERROR; i <= HAL_TIMEOUT; ++i) {
        unsigned int before = calls;
        tx_result = (HAL_StatusTypeDef)i;
        assert(Servo_StopAll() == (i == HAL_BUSY ? SERVO_BUSY : SERVO_UART_ERROR));
        assert(calls == before + 1U); /* Never retry a partial/failed command. */
        ++checks;
    }
    tx_result = HAL_OK;
    EXPECT_TX(Servo_StopAll(), "$DST!");

    /* Ownership, delayed completion, latest waiting frame, wrong UART and
     * a HAL refusal: pending frames are copied, never caller-owned. */
    complete_immediately = 0U;
    EXPECT_TX(Servo_StopAll(), "$DST!");
    assert(!Servo_IsIdle());
    EXPECT_NO_TX(Servo_Init(&uart), SERVO_BUSY);
    {
        UART_HandleTypeDef other = uart;
        char waiting[] = "$DGS:1!";
        unsigned before = calls;
        assert(Servo_SendRaw(waiting) == SERVO_OK);
        memset(waiting, 'X', strlen(waiting));
        assert(Servo_RunAction(2U) == SERVO_OK); /* Replace the waiting action. */
        assert(calls == before && memcmp(inflight, "$DST!", inflight_length) == 0);
        Servo_TxCallback(&other);
        assert(calls == before && !Servo_IsIdle());
        uart.gState = HAL_UART_STATE_READY; Servo_TxCallback(&uart);
        assert(calls == before + 1U && strcmp(sent, "$DGS:2!") == 0);
        assert(!Servo_IsIdle());
        uart.gState = HAL_UART_STATE_READY; Servo_TxCallback(&uart);
        assert(Servo_IsIdle());
        /* Copy semantics for a single waiting frame, without replacement. */
        EXPECT_TX(Servo_StopAll(), "$DST!");
        memcpy(waiting, "$DGS:1!", sizeof(waiting));
        assert(Servo_SendRaw(waiting) == SERVO_OK);
        memset(waiting, 'X', strlen(waiting));
        uart.gState = HAL_UART_STATE_READY; Servo_TxCallback(&uart);
        assert(strcmp(sent, "$DGS:1!") == 0);
        uart.gState = HAL_UART_STATE_READY; Servo_TxCallback(&uart);
        assert(Servo_IsIdle());
        tx_result = HAL_BUSY;
        before = calls;
        assert(Servo_StopAll() == SERVO_BUSY && calls == before + 1U && Servo_IsIdle());
        tx_result = HAL_OK;
        EXPECT_TX(Servo_StopAll(), "$DST!");
        assert(Servo_RunAction(3U) == SERVO_OK);
        tx_result = HAL_ERROR;
        uart.gState = HAL_UART_STATE_READY; Servo_TxCallback(&uart);
        assert(Servo_IsIdle()); /* Failed queued dispatch never locks TX. */
        tx_result = HAL_OK;
    }
    complete_immediately = 1U;
    /* A failed rebind must not accidentally send via the previous UART. */
    EXPECT_NO_TX(Servo_Init(NULL), SERVO_INVALID_PARAM);
    check_uninitialized();
    bad = uart; bad.Instance = NULL;
    EXPECT_NO_TX(Servo_Init(&bad), SERVO_INVALID_PARAM);
    bad = uart; bad.gState = HAL_UART_STATE_RESET;
    EXPECT_NO_TX(Servo_Init(&bad), SERVO_UART_ERROR);
    bad = uart; bad.Init.BaudRate = 9600;
    EXPECT_NO_TX(Servo_Init(&bad), SERVO_INVALID_PARAM);
    bad = uart; bad.Init.WordLength = UART_WORDLENGTH_9B;
    EXPECT_NO_TX(Servo_Init(&bad), SERVO_INVALID_PARAM);
    bad = uart; bad.Init.StopBits = UART_STOPBITS_2;
    EXPECT_NO_TX(Servo_Init(&bad), SERVO_INVALID_PARAM);
    bad = uart; bad.Init.Parity = UART_PARITY_EVEN;
    EXPECT_NO_TX(Servo_Init(&bad), SERVO_INVALID_PARAM);
    bad = uart; bad.Init.Mode = UART_MODE_RX;
    EXPECT_NO_TX(Servo_Init(&bad), SERVO_INVALID_PARAM);
    bad = uart; bad.Init.HwFlowCtl = UART_HWCONTROL_RTS;
    EXPECT_NO_TX(Servo_Init(&bad), SERVO_INVALID_PARAM);
    EXPECT_NO_TX(Servo_Init(&uart), SERVO_OK);
    EXPECT_TX(Servo_StopAll(), "$DST!");
    check_owned();
    printf("SERVO protocol PASS: %u checks; exact bytes, limits, no-TX failures, HAL errors, silent init.\n", checks);
    return 0;
}
