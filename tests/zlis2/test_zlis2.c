#include "zlis2_driver.h"
#include "servo_pose.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static int peripheral;
static UART_HandleTypeDef uart = {
    &peripheral,
    {115200U, UART_WORDLENGTH_8B, UART_STOPBITS_1, UART_PARITY_NONE,
     UART_MODE_TX_RX, UART_HWCONTROL_NONE},
    HAL_UART_STATE_READY
};
static HAL_StatusTypeDef tx_result = HAL_OK;
static unsigned int calls, checks;
static size_t sent_length;
static char sent[1024];

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *handle, const uint8_t *data,
                                    uint16_t length, uint32_t timeout)
{
    assert(handle == &uart);
    assert(data != NULL && length > 0U && length < sizeof(sent));
    assert(timeout == 40U);
    /* Time budget covers 8N1 wire time for every permitted packet. */
    assert((uint32_t)length * 10000U < timeout * 115200U);
    memcpy(sent, data, length);
    sent[length] = '\0';
    sent_length = length;
    ++calls;
    return tx_result;
}

#define EXPECT_TX(expression, expected) do { \
    unsigned int before = calls; \
    assert((expression) == ZLIS2_OK); \
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
    ZLIS2_ServoCommand c = {0U, 1500U, 1000U};
    EXPECT_NO_TX(ZLIS2_SetServo(0, 1500, 1000), ZLIS2_NOT_INITIALIZED);
    EXPECT_NO_TX(ZLIS2_SetServos(&c, 1), ZLIS2_NOT_INITIALIZED);
    EXPECT_NO_TX(ZLIS2_RunAction(0), ZLIS2_NOT_INITIALIZED);
    EXPECT_NO_TX(ZLIS2_RunActionRange(0, 1, 0), ZLIS2_NOT_INITIALIZED);
    EXPECT_NO_TX(ZLIS2_StopAll(), ZLIS2_NOT_INITIALIZED);
    EXPECT_NO_TX(ZLIS2_StopServo(0), ZLIS2_NOT_INITIALIZED);
    EXPECT_NO_TX(ZLIS2_SetServoBias(0, 0), ZLIS2_NOT_INITIALIZED);
    EXPECT_NO_TX(ZLIS2_RunCombinedAction(0, 0), ZLIS2_NOT_INITIALIZED);
    EXPECT_NO_TX(ZLIS2_RecordPose(), ZLIS2_NOT_INITIALIZED);
    EXPECT_NO_TX(ZLIS2_RunRecorded(0), ZLIS2_NOT_INITIALIZED);
    EXPECT_NO_TX(ZLIS2_ClearRecorded(), ZLIS2_NOT_INITIALIZED);
    EXPECT_NO_TX(ZLIS2_SetRecordPeriod(0), ZLIS2_NOT_INITIALIZED);
    EXPECT_NO_TX(ZLIS2_Reset(), ZLIS2_NOT_INITIALIZED);
    EXPECT_NO_TX(ZLIS2_SendRaw("$DST!"), ZLIS2_NOT_INITIALIZED);
}

int main(void)
{
    ZLIS2_ServoCommand pair[] = {{0, 1500, 1000}, {1, 900, 1000}};
    ZLIS2_ServoCommand full[24];
    char expected[363], raw[364];
    UART_HandleTypeDef bad;
    ArmStep_t step;
    size_t i;
    check_uninitialized();
    EXPECT_NO_TX(ZLIS2_Init(NULL), ZLIS2_INVALID_PARAM);
    EXPECT_NO_TX(ZLIS2_Init(&uart), ZLIS2_OK);

    EXPECT_TX(ZLIS2_SetServo(0, 500, 2000), "#000P0500T2000!");
    EXPECT_TX(ZLIS2_SetServo(4, 1500, 1000), "#004P1500T1000!");
    EXPECT_TX(ZLIS2_SetServo(254, 2500, 9999), "#254P2500T9999!");
    EXPECT_TX(ZLIS2_SetServo(0, 500, 0), "#000P0500T0000!");
    EXPECT_TX(ZLIS2_SetServos(pair, 2), "{#000P1500T1000!#001P0900T1000!}");
    EXPECT_TX(ZLIS2_SetServos(pair, 1), "{#000P1500T1000!}");
    EXPECT_TX(ZLIS2_RunAction(0), "$DGS:0!");
    EXPECT_TX(ZLIS2_RunAction(65535), "$DGS:65535!");
    EXPECT_TX(ZLIS2_RunActionRange(0, 10, 1), "$DGT:0-10,1!");
    EXPECT_TX(ZLIS2_RunActionRange(0, 10, 0), "$DGT:0-10,0!");
    EXPECT_TX(ZLIS2_RunActionRange(0, 4, 1), "$DGT:0-4,1!");
    EXPECT_TX(ZLIS2_RunActionRange(65535, 65535, 65535), "$DGT:65535-65535,65535!");
    EXPECT_TX(ZLIS2_StopAll(), "$DST!");
    EXPECT_TX(ZLIS2_StopServo(5), "$DST:5!");
    EXPECT_TX(ZLIS2_StopServo(254), "$DST:254!");
    EXPECT_TX(ZLIS2_SetServoBias(5, 10), "#005PSCK+010!");
    EXPECT_TX(ZLIS2_SetServoBias(5, -10), "#005PSCK-010!");
    EXPECT_TX(ZLIS2_SetServoBias(0, 0), "#000PSCK+000!");
    EXPECT_TX(ZLIS2_SetServoBias(254, 500), "#254PSCK+500!");
    EXPECT_TX(ZLIS2_SetServoBias(254, -500), "#254PSCK-500!");
    EXPECT_TX(ZLIS2_RunCombinedAction(1, 5), "$DKT:1,5!");
    EXPECT_TX(ZLIS2_RunCombinedAction(65535, 65535), "$DKT:65535,65535!");
    EXPECT_TX(ZLIS2_RecordPose(), "$DJ_RECORD!");
    EXPECT_TX(ZLIS2_RunRecorded(1), "$DJ_RECORD_DO:1!");
    EXPECT_TX(ZLIS2_RunRecorded(0), "$DJ_RECORD_DO:0!");
    EXPECT_TX(ZLIS2_RunRecorded(65535), "$DJ_RECORD_DO:65535!");
    EXPECT_TX(ZLIS2_ClearRecorded(), "$DJ_RECORD_CLEAR!");
    EXPECT_TX(ZLIS2_SetRecordPeriod(1000), "$DJ_RECORD_TIME:1000!");
    EXPECT_TX(ZLIS2_SetRecordPeriod(0), "$DJ_RECORD_TIME:0!");
    EXPECT_TX(ZLIS2_SetRecordPeriod(65535), "$DJ_RECORD_TIME:65535!");
    EXPECT_TX(ZLIS2_Reset(), "$RST!");
    EXPECT_TX(ZLIS2_SendRaw("$DST!\r\n"), "$DST!\r\n");

    assert(SERVO_MODE_NONE == 0 &&
           SERVO_MODE_BALL_PREPARE == 1 &&
           SERVO_MODE_BALL_REACH == 2 &&
           SERVO_MODE_BALL_GRIP == 3 &&
           SERVO_MODE_ABOVE_BUCKET == 4 &&
           SERVO_MODE_BALL_RELEASE == 5 &&
           SERVO_MODE_HOSTAGE_REACH == 6 &&
           SERVO_MODE_HOSTAGE_GRIP == 7 &&
           SERVO_MODE_HOSTAGE_LIFT == 8);
    ++checks;
    assert(ServoPose_BuildMode(SERVO_MODE_NONE, &step) == SERVO_POSE_INVALID);
    assert(ServoPose_BuildMode((ServoMode_t)9, &step) == SERVO_POSE_INVALID);
    checks += 2;

    for (i = 0U; i < SERVO_POSE_COUNT; ++i) {
        unsigned before = calls;
        ServoPoseResult_t result = ServoPose_BuildStep((uint8_t)i, &step);
        assert(result == SERVO_POSE_OK);
        assert(ServoPose_BuildMode((ServoMode_t)(i + 1U), &step) == result);
        if (result == SERVO_POSE_OK)
            assert(step.joint_mask == 0x07U && step.position[3] == 0U);
        assert(calls == before); /* Preset lookup never bypasses Arm_Control. */
        ++checks;
    }
    assert(ServoPose_BuildStep(0U, &step) == SERVO_POSE_OK &&
           step.position[0] == 1532U && step.position[1] == 2219U &&
           step.position[2] == 1202U && step.position[3] == 0U);
    assert(ServoPose_BuildStep(2U, &step) == SERVO_POSE_OK);
    assert(step.joint_mask == 0x07U && step.move_ms == 1000U &&
           step.position[0] == 1421U && step.position[3] == 0U);
    assert(ServoPose_BuildStep(6U, &step) == SERVO_POSE_OK &&
           step.position[0] == 1499U && step.position[3] == 0U);
    assert(ServoPose_BuildStep(8U, &step) == SERVO_POSE_INVALID);
    assert(ServoPose_BuildStep(0U, NULL) == SERVO_POSE_INVALID);
    {
        static const uint16_t original[8][3] = {
            {1532U, 1972U, 573U},
            {1421U, 2071U, 812U},
            {1421U, 2071U, 812U},
            {1717U, 2297U, 884U},
            {1717U, 2297U, 884U},
            {1499U, 1855U, 528U},
            {1499U, 1855U, 528U},
            {1800U, 1855U, 528U}
        };
        static const uint16_t original_gripper[8] = {
            2192U, 2192U, 500U, 500U, 1800U, 1800U, 500U, 500U
        };
        unsigned joint;
        for (i = 0U; i < SERVO_POSE_COUNT; ++i) {
            unsigned before = calls;
            assert(ServoPose_BuildOriginalMode((ServoMode_t)(i + 1U), &step) ==
                   SERVO_POSE_OK);
            assert(step.joint_mask == 0x0FU && step.move_ms == 1000U &&
                   step.position[3] == original_gripper[i]);
            for (joint = 0U; joint < 3U; ++joint)
                assert(step.position[joint] == original[i][joint]);
            assert(calls == before);
            ++checks;
        }
        assert(ServoPose_BuildOriginalMode(SERVO_MODE_NONE, &step) ==
               SERVO_POSE_INVALID);
        assert(ServoPose_BuildOriginalMode(SERVO_MODE_BALL_PREPARE, NULL) ==
               SERVO_POSE_INVALID);
        checks += 2;
    }

    EXPECT_NO_TX(ZLIS2_SetServo(255, 1500, 1000), ZLIS2_INVALID_PARAM);
    EXPECT_NO_TX(ZLIS2_SetServo(0, 499, 1000), ZLIS2_INVALID_PARAM);
    EXPECT_NO_TX(ZLIS2_SetServo(0, 2501, 1000), ZLIS2_INVALID_PARAM);
    EXPECT_NO_TX(ZLIS2_SetServo(0, 1500, 10000), ZLIS2_INVALID_PARAM);
    EXPECT_NO_TX(ZLIS2_StopServo(255), ZLIS2_INVALID_PARAM);
    EXPECT_NO_TX(ZLIS2_SetServoBias(255, 0), ZLIS2_INVALID_PARAM);
    EXPECT_NO_TX(ZLIS2_SetServoBias(5, 501), ZLIS2_INVALID_PARAM);
    EXPECT_NO_TX(ZLIS2_SetServoBias(5, -501), ZLIS2_INVALID_PARAM);
    EXPECT_NO_TX(ZLIS2_SetServoBias(5, INT16_MIN), ZLIS2_INVALID_PARAM);
    EXPECT_NO_TX(ZLIS2_RunActionRange(10, 0, 1), ZLIS2_INVALID_PARAM);
    EXPECT_NO_TX(ZLIS2_SetServos(NULL, 1), ZLIS2_INVALID_PARAM);
    EXPECT_NO_TX(ZLIS2_SetServos(pair, 0), ZLIS2_INVALID_PARAM);
    EXPECT_NO_TX(ZLIS2_SetServos(pair, 25), ZLIS2_BUFFER_OVERFLOW);
    EXPECT_NO_TX(ZLIS2_SetServos(pair, SIZE_MAX), ZLIS2_BUFFER_OVERFLOW);
    pair[1].id = 255;
    EXPECT_NO_TX(ZLIS2_SetServos(pair, 2), ZLIS2_INVALID_PARAM);
    pair[1].id = 1; pair[1].pwm = 499;
    EXPECT_NO_TX(ZLIS2_SetServos(pair, 2), ZLIS2_INVALID_PARAM);
    pair[1].pwm = 2501;
    EXPECT_NO_TX(ZLIS2_SetServos(pair, 2), ZLIS2_INVALID_PARAM);
    pair[1].pwm = 1500; pair[1].time_ms = 10000;
    EXPECT_NO_TX(ZLIS2_SetServos(pair, 2), ZLIS2_INVALID_PARAM);
    EXPECT_NO_TX(ZLIS2_SendRaw(NULL), ZLIS2_INVALID_PARAM);
    EXPECT_NO_TX(ZLIS2_SendRaw(""), ZLIS2_INVALID_PARAM);

    expected[0] = '{';
    for (i = 0; i < 24; ++i) {
        full[i].id = 254; full[i].pwm = 2500; full[i].time_ms = 9999;
        memcpy(expected + 1 + i * 15, "#254P2500T9999!", 15);
    }
    expected[361] = '}'; expected[362] = '\0';
    EXPECT_TX(ZLIS2_SetServos(full, 24), expected);
    full[23].id = 255;
    EXPECT_NO_TX(ZLIS2_SetServos(full, 24), ZLIS2_INVALID_PARAM);
    memcpy(raw, expected, sizeof(expected));
    EXPECT_TX(ZLIS2_SendRaw(raw), expected);
    raw[362] = '!'; raw[363] = '\0';
    EXPECT_NO_TX(ZLIS2_SendRaw(raw), ZLIS2_BUFFER_OVERFLOW);
    memset(raw, 'X', sizeof(raw));
    EXPECT_NO_TX(ZLIS2_SendRaw(raw), ZLIS2_BUFFER_OVERFLOW);

    for (i = HAL_ERROR; i <= HAL_TIMEOUT; ++i) {
        unsigned int before = calls;
        tx_result = (HAL_StatusTypeDef)i;
        assert(ZLIS2_StopAll() == ZLIS2_UART_ERROR);
        assert(calls == before + 1U); /* Never retry a partial/failed command. */
        ++checks;
    }
    tx_result = HAL_OK;
    EXPECT_TX(ZLIS2_StopAll(), "$DST!");

    /* A failed rebind must not accidentally send via the previous UART. */
    EXPECT_NO_TX(ZLIS2_Init(NULL), ZLIS2_INVALID_PARAM);
    check_uninitialized();
    bad = uart; bad.Instance = NULL;
    EXPECT_NO_TX(ZLIS2_Init(&bad), ZLIS2_INVALID_PARAM);
    bad = uart; bad.gState = HAL_UART_STATE_RESET;
    EXPECT_NO_TX(ZLIS2_Init(&bad), ZLIS2_UART_ERROR);
    bad = uart; bad.Init.BaudRate = 9600;
    EXPECT_NO_TX(ZLIS2_Init(&bad), ZLIS2_INVALID_PARAM);
    bad = uart; bad.Init.WordLength = UART_WORDLENGTH_9B;
    EXPECT_NO_TX(ZLIS2_Init(&bad), ZLIS2_INVALID_PARAM);
    bad = uart; bad.Init.StopBits = UART_STOPBITS_2;
    EXPECT_NO_TX(ZLIS2_Init(&bad), ZLIS2_INVALID_PARAM);
    bad = uart; bad.Init.Parity = UART_PARITY_EVEN;
    EXPECT_NO_TX(ZLIS2_Init(&bad), ZLIS2_INVALID_PARAM);
    bad = uart; bad.Init.Mode = UART_MODE_RX;
    EXPECT_NO_TX(ZLIS2_Init(&bad), ZLIS2_INVALID_PARAM);
    bad = uart; bad.Init.HwFlowCtl = UART_HWCONTROL_RTS;
    EXPECT_NO_TX(ZLIS2_Init(&bad), ZLIS2_INVALID_PARAM);
    EXPECT_NO_TX(ZLIS2_Init(&uart), ZLIS2_OK);
    EXPECT_TX(ZLIS2_StopAll(), "$DST!");
    printf("ZLIS2 PASS: %u checks; exact bytes, limits, no-TX failures, HAL errors, silent init.\n", checks);
    return 0;
}
