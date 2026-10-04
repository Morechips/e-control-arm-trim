#include "arm_trim_service.h"
#include "arm_trim_project.h"
#include "arm_trim_project_config.h"
#include "uart_driver.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(c) do { ++checks; if (!(c)) { fprintf(stderr, "FAIL %u: %s\n", (unsigned)__LINE__, #c); exit(1); } } while (0)
static int peripheral;
static UART_HandleTypeDef uart = {
    &peripheral,
    {115200U, UART_WORDLENGTH_8B, UART_STOPBITS_1, UART_PARITY_NONE,
     UART_MODE_TX_RX, UART_HWCONTROL_NONE},
    HAL_UART_STATE_READY, HAL_UART_STATE_READY, NULL
};
static unsigned checks, calls, aborts;
static uint32_t tick, started;
static char history[1024][SERVO_MAX_TX_LENGTH + 1U];
static const uint8_t *inflight;
static uint16_t inflight_length;
static bool active;
static HAL_StatusTypeDef next_tx = HAL_OK, abort_result = HAL_OK;

uint32_t HAL_GetTick(void) { return tick; }
static uint32_t Now(void *user) { CHECK(user == &tick); return tick; }
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *handle, const uint8_t *data,
                                      uint16_t length)
{
    HAL_StatusTypeDef result = next_tx;
    next_tx = HAL_OK;
    CHECK(handle == &uart && data != NULL && length > 0U && length <= SERVO_MAX_TX_LENGTH);
    CHECK(!active && calls < 1024U);
    memcpy(history[calls], data, length);
    history[calls][length] = '\0';
    ++calls;
    if (result == HAL_OK) {
        active = true;
        inflight = data;
        inflight_length = length;
        started = tick;
        handle->gState = HAL_UART_STATE_BUSY_TX;
    }
    return result;
}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *handle)
{
    CHECK(handle == &uart);
    ++aborts;
    if (abort_result == HAL_OK) { active = false; handle->gState = HAL_UART_STATE_READY; }
    return abort_result;
}
static void CompleteAt(uint32_t at)
{
    CHECK(active);
    CHECK(strlen(history[calls - 1U]) == inflight_length);
    CHECK(memcmp(history[calls - 1U], inflight, inflight_length) == 0);
    tick = at;
    active = false;
    uart.gState = HAL_UART_STATE_READY;
    /* Exercise the shared driver routing used by production HAL callbacks. */
    UART_TxCallback(&uart);
}
static ArmTrimServiceConfig_t Config(void)
{
    ArmTrimServiceConfig_t config;
    static const uint16_t refs[3][3] = {
        {ARM_TRIM_BALL_P0, ARM_TRIM_BALL_P1, ARM_TRIM_BALL_P2},
        {ARM_TRIM_HOSTAGE_P0, ARM_TRIM_HOSTAGE_P1, ARM_TRIM_HOSTAGE_P2},
        {ARM_TRIM_BUCKET_P0, ARM_TRIM_BUCKET_P1, ARM_TRIM_BUCKET_P2}
    };
    memset(&config, 0, sizeof(config));
    ArmTrimProject_DefaultConfig(&config.core);
    memcpy(config.references, refs, sizeof(refs));
    config.profile_move_ms = 2000U;
    config.profile_guard_ms = 300U;
    config.grip_move_ms = 1500U;
    config.grip_guard_ms = 300U;
    return config;
}
static void Init(void)
{
    ArmTrimServiceConfig_t config = Config();
    CHECK(!ArmTrimService_OwnsMotion() && !active);
    tick = calls = aborts = 0U;
    next_tx = abort_result = HAL_OK;
    CHECK(Servo_Init(&uart) == SERVO_OK);
    CHECK(ArmTrimService_Init(&config, Now, &tick) == ARM_TRIM_OK);
}
static void Step(uint32_t ms)
{
    tick += ms;
    ArmTrimService_Process();
}
static void Finish(void)
{
    unsigned i;
    for (i = 0U; i < 10000U && ArmTrimService_IsBusy(); ++i) {
        tick += 5U;
        if (active && (uint32_t)(tick - started) >= 3U) CompleteAt(tick);
        ArmTrimService_Process();
    }
    CHECK(!ArmTrimService_IsBusy());
}
static void BeginBall(void)
{
    const uint16_t positions[3] = {ARM_TRIM_BALL_P0, ARM_TRIM_BALL_P1, ARM_TRIM_BALL_P2};
    CHECK(ArmTrimService_Begin(positions, true) == ARM_TRIM_OK);
    CHECK(calls == 0U);
}
static void TestCompletedWaitAndGrip(void)
{
    ArmTrimStatus_t before;
    unsigned previous;
    Init();
    CHECK(ArmTrimService_ReadyProfile(ARM_TRIM_PROFILE_BALL, true) == ARM_TRIM_OK);
    CHECK(strcmp(history[0], "{#000P1356T2000!#001P1850T2000!#002P0698T2000!}") == 0);
    CHECK(Servo_SendPreset(Servo_AIM) == SERVO_BUSY && calls == 1U);
    Step(29U);
    CHECK(ArmTrimService_IsBusy() && !ArmTrimService_GetStatus().core.reference_valid);
    CompleteAt(30U);
    ArmTrimService_Process();
    Step(2299U);
    CHECK(!ArmTrimService_GetStatus().core.reference_valid);
    Step(1U);
    CHECK(ArmTrimService_GetStatus().state == ARM_TRIM_SERVICE_REFERENCE);
    CHECK(ArmTrimService_GetStatus().core.reference_valid && ArmTrimService_OwnsMotion());
    CHECK(Servo_SendGap(500U) == SERVO_BUSY && calls == 1U);
    CHECK(ArmTrimService_MoveRelativeX(2.0f) == ARM_TRIM_OK);
    Finish();
    before = ArmTrimService_GetStatus().core;
    CHECK(fabsf(before.offset_mm - 2.0f) < 0.0001f);
    previous = calls;
    CHECK(ArmTrimService_Grip(500U) == ARM_TRIM_OK);
    CHECK(calls == previous + 1U && strcmp(history[previous], "{#003P0500T1500!}") == 0);
    CompleteAt(tick + 25U);
    ArmTrimService_Process();
    Step(1799U);
    CHECK(ArmTrimService_IsBusy());
    Step(1U);
    CHECK(!ArmTrimService_IsBusy());
    CHECK(ArmTrimService_GetStatus().core.reference_valid);
    CHECK(ArmTrimService_GetStatus().core.offset_mm == before.offset_mm);
    CHECK(memcmp(ArmTrimService_GetStatus().core.estimated_position, before.estimated_position,
                 sizeof(before.estimated_position)) == 0);
    CHECK(ArmTrimService_End() == ARM_TRIM_OK && !ArmTrimService_OwnsMotion());
    CHECK(Servo_SendPreset(Servo_AIM) == SERVO_OK);
    CompleteAt(tick + 5U);
    CHECK(Servo_IsIdle());
}
static void TestCancelLiveFrame(void)
{
    Init();
    CHECK(ArmTrimService_ReadyProfile(ARM_TRIM_PROFILE_BALL, true) == ARM_TRIM_OK);
    CHECK(ArmTrimService_Cancel() == ARM_TRIM_OK);
    ArmTrimService_Process();
    CHECK(calls == 1U && active && ArmTrimService_OwnsMotion());
    CHECK(ArmTrimService_GetStatus().transfer.state == SERVO_TRANSFER_PENDING);
    CHECK(!ArmTrimService_GetStatus().transfer.started);
    CHECK(ArmTrimService_End() == ARM_TRIM_OK);
    CHECK(Servo_SendPreset(Servo_RST) == SERVO_BUSY);
    CompleteAt(5U);
    CHECK(calls == 2U && strcmp(history[1], "$DST:0!") == 0);
    CHECK(ArmTrimService_GetStatus().transfer.started_tick == 5U);
    ArmTrimService_Process();
    CHECK(calls == 2U && ArmTrimService_OwnsMotion());
    CompleteAt(10U);
    ArmTrimService_Process();
    CHECK(calls == 3U && strcmp(history[2], "$DST:1!") == 0);
    CompleteAt(15U);
    ArmTrimService_Process();
    CHECK(calls == 4U && strcmp(history[3], "$DST:2!") == 0);
    CHECK(ArmTrimService_OwnsMotion() && !ArmTrimService_GetStatus().core.reference_valid);
    CompleteAt(20U);
    ArmTrimService_Process();
    CHECK(!ArmTrimService_OwnsMotion() && !ArmTrimService_IsBusy());
    CHECK(ArmTrimService_GetStatus().state == ARM_TRIM_SERVICE_IDLE);
    CHECK(aborts == 0U);
}
static void TestCoreCancelAndGripCancel(void)
{
    Init();
    BeginBall();
    CHECK(ArmTrimService_StartJog(1) == ARM_TRIM_OK);
    ArmTrimService_Process();
    CHECK(calls == 1U && active);
    CHECK(ArmTrimService_Cancel() == ARM_TRIM_OK);
    Finish();
    CHECK(ArmTrimService_GetStatus().state == ARM_TRIM_SERVICE_IDLE);
    CHECK(calls == 4U && strcmp(history[1], "$DST:0!") == 0 &&
          strcmp(history[2], "$DST:1!") == 0 && strcmp(history[3], "$DST:2!") == 0);
    Init();
    BeginBall();
    CHECK(ArmTrimService_Grip(1800U) == ARM_TRIM_OK);
    CHECK(ArmTrimService_End() == ARM_TRIM_OK);
    ArmTrimService_Process();
    CHECK(ArmTrimService_OwnsMotion() && calls == 1U);
    Finish();
    CHECK(calls == 2U && strcmp(history[1], "$DST:3!") == 0);
    CHECK(!ArmTrimService_OwnsMotion() && !ArmTrimService_GetStatus().core.reference_valid);
}
static void TestTimeoutAndRefusedAbort(void)
{
    unsigned after;
    Init();
    CHECK(ArmTrimService_ReadyProfile(ARM_TRIM_PROFILE_BALL, true) == ARM_TRIM_OK);
    Step(39U);
    CHECK(aborts == 0U && active);
    Step(1U);
    CHECK(aborts == 1U && !active);
    CHECK(ArmTrimService_GetStatus().state == ARM_TRIM_SERVICE_STOPPING);
    CHECK(!ArmTrimService_GetStatus().core.reference_valid && ArmTrimService_OwnsMotion());
    CHECK(ArmTrimService_End() == ARM_TRIM_FAULT_LATCHED);
    Finish();
    CHECK(calls == 4U && ArmTrimService_GetStatus().state == ARM_TRIM_SERVICE_FAULT);
    after = calls;
    Step(1000U);
    CHECK(calls == after);
    CHECK(ArmTrimService_ReadyProfile(ARM_TRIM_PROFILE_BALL, true) == ARM_TRIM_FAULT_LATCHED);
    CHECK(ArmTrimService_ClearFault() == ARM_TRIM_OK);
    Init();
    BeginBall();
    CHECK(ArmTrimService_Grip(500U) == ARM_TRIM_OK);
    abort_result = HAL_ERROR;
    Step(40U);
    CHECK(active && aborts == 1U && ArmTrimService_OwnsMotion());
    CHECK(ArmTrimService_GetStatus().state == ARM_TRIM_SERVICE_STOPPING);
    CHECK(ArmTrimService_End() == ARM_TRIM_FAULT_LATCHED);
    ArmTrimService_Process();
    CHECK(active && ArmTrimService_OwnsMotion());
    CHECK(Servo_SendGap(1800U) == SERVO_BUSY);
    /* A late physical completion finally makes the protected buffer safe.
     * No failed grip is repeated, and END can then finish releasing the lease. */
    CompleteAt(45U);
    abort_result = HAL_OK;
    Finish();
    CHECK(!ArmTrimService_OwnsMotion() && ArmTrimService_GetStatus().state == ARM_TRIM_SERVICE_FAULT);
    CHECK(calls <= 2U);
    CHECK(ArmTrimService_ClearFault() == ARM_TRIM_OK);
}
static void TestLateIRQCompletion(void)
{
    Init();
    CHECK(ArmTrimService_ReadyProfile(ARM_TRIM_PROFILE_BALL, true) == ARM_TRIM_OK);
    CompleteAt(40U); /* ISR fires at the inclusive deadline before foreground. */
    CHECK(ArmTrimService_GetStatus().transfer.state == SERVO_TRANSFER_FAILED);
    CHECK(ArmTrimService_GetStatus().transfer.result == SERVO_UART_ERROR);
    ArmTrimService_Process();
    CHECK(ArmTrimService_GetStatus().state == ARM_TRIM_SERVICE_STOPPING);
    CHECK(!ArmTrimService_GetStatus().core.reference_valid);
    Finish();
    CHECK(ArmTrimService_GetStatus().state == ARM_TRIM_SERVICE_FAULT && calls == 4U);
    CHECK(ArmTrimService_ClearFault() == ARM_TRIM_OK);
    Init(); BeginBall();
    CHECK(ArmTrimService_Grip(500U) == ARM_TRIM_OK);
    CompleteAt(41U);
    CHECK(ArmTrimService_GetStatus().transfer.state == SERVO_TRANSFER_FAILED);
    ArmTrimService_Process(); Finish();
    CHECK(calls == 2U && strcmp(history[1], "$DST:3!") == 0);
    CHECK(!ArmTrimService_GetStatus().core.reference_valid);
    CHECK(ArmTrimService_ClearFault() == ARM_TRIM_OK);
    Init();
    CHECK(ArmTrimService_ReadyProfile(ARM_TRIM_PROFILE_BALL, true) == ARM_TRIM_OK);
    CHECK(ArmTrimService_Cancel() == ARM_TRIM_OK);
    ArmTrimService_Process();
    CompleteAt(5U); /* Queued stop 000 starts now. */
    CompleteAt(45U); /* Late stop TC also fails rather than being skipped. */
    CHECK(ArmTrimService_GetStatus().transfer.state == SERVO_TRANSFER_FAILED);
    ArmTrimService_Process();
    CHECK(ArmTrimService_GetStatus().stop_failed && ArmTrimService_OwnsMotion());
    Finish();
    CHECK(ArmTrimService_GetStatus().state == ARM_TRIM_SERVICE_FAULT && calls == 4U);
    CHECK(ArmTrimService_ClearFault() == ARM_TRIM_OK);
    Init(); tick = UINT32_MAX - 20U;
    CHECK(ArmTrimService_ReadyProfile(ARM_TRIM_PROFILE_BALL, true) == ARM_TRIM_OK);
    CompleteAt(tick + 40U);
    CHECK(ArmTrimService_GetStatus().transfer.state == SERVO_TRANSFER_FAILED);
    ArmTrimService_Process(); Finish();
    CHECK(ArmTrimService_GetStatus().state == ARM_TRIM_SERVICE_FAULT);
    CHECK(ArmTrimService_ClearFault() == ARM_TRIM_OK);
    Init();
    CHECK(ArmTrimService_ReadyProfile(ARM_TRIM_PROFILE_BALL, false) == ARM_TRIM_OK);
    CompleteAt(39U); /* Just below the deadline is still accepted. */
    CHECK(ArmTrimService_GetStatus().transfer.state == SERVO_TRANSFER_COMPLETE);
    Finish();
    CHECK(!ArmTrimService_OwnsMotion() && ArmTrimService_GetStatus().state == ARM_TRIM_SERVICE_IDLE);
}
static void TestEveryFixedPoseIsPlanar(void)
{
    for (unsigned preset = 0U; preset < (unsigned)ServoCode_MAX; ++preset) {
        Init();
        CHECK(ArmTrimService_RunPreset((ServoCode)preset) == ARM_TRIM_OK);
        CHECK(calls == 1U && strstr(history[0], "#000P") != NULL);
        CHECK(strstr(history[0], "#001P") != NULL && strstr(history[0], "#002P") != NULL);
        CHECK(strstr(history[0], "#003") == NULL);
        if (preset == (unsigned)TakeHostage_Catch)
            CHECK(strstr(history[0], "#001P2136") != NULL);
        if (preset == (unsigned)Servo_AIM)
            CHECK(strcmp(history[0], "{#000P1058T2000!#001P0821T2000!#002P0554T2000!}") == 0);
        Finish();
        CHECK(ArmTrimService_GetStatus().state != ARM_TRIM_SERVICE_FAULT);
        CHECK(ArmTrimService_End() == ARM_TRIM_OK); Finish();
    }
}

int main(void)
{
    TestCompletedWaitAndGrip();
    TestCancelLiveFrame();
    TestCoreCancelAndGripCancel();
    TestTimeoutAndRefusedAbort();
    TestLateIRQCompletion();
    TestEveryFixedPoseIsPlanar();
    printf("Arm trim actual Servo/UART pipeline: %u checks passed\n", checks);
    return 0;
}
