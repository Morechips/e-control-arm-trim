#include "arm_trim_service.h"
#include "arm_trim_project.h"
#include "arm_trim_project_config.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(c) do { ++checks; if (!(c)) { fprintf(stderr, "FAIL %u: %s\n", (unsigned)__LINE__, #c); exit(1); } } while (0)
/* Deliberate owned-transport fake: keeps a canceled active wire frame alive,
 * tracks the single stop queued behind it, and publishes real start/TC ticks.
 * Protocol formatting and HAL behavior are exercised by tests/zlis2. */
typedef struct {
    bool stop;
    size_t count;
    ServoCommand_t commands[3];
} Record_t;
static Record_t records[2048];
static unsigned checks, record_count, active_record, queued_record;
static const void *lease;
static ServoTransferStatus_t transfer;
static uint32_t tick, active_started;
static bool active, queued, active_tracked, automatic;
static ServoStatus_t next_submit;

static uint32_t Now(void *user) { CHECK(user == &tick); return tick; }
ServoStatus_t Servo_Acquire(const void *owner)
{
    if (owner == NULL) return SERVO_INVALID_PARAM;
    if (lease != NULL || active || queued) return SERVO_BUSY;
    lease = owner;
    memset(&transfer, 0, sizeof(transfer));
    return SERVO_OK;
}
ServoStatus_t Servo_Release(const void *owner)
{
    CHECK(owner == lease);
    if (active || queued) return SERVO_BUSY;
    lease = NULL;
    return SERVO_OK;
}
static ServoStatus_t Submit(const void *owner, const ServoCommand_t *commands,
                           size_t count, bool stop)
{
    unsigned index;
    if (owner == NULL || owner != lease) return SERVO_BUSY;
    if (transfer.state == SERVO_TRANSFER_PENDING || queued || (active && !stop))
        return SERVO_BUSY;
    if (next_submit != SERVO_OK) {
        ServoStatus_t result = next_submit;
        next_submit = SERVO_OK;
        return result;
    }
    CHECK(record_count < 2048U && count > 0U && count <= 3U);
    index = record_count++;
    records[index].stop = stop;
    records[index].count = count;
    memcpy(records[index].commands, commands, count * sizeof(*commands));
    memset(&transfer, 0, sizeof(transfer));
    transfer.state = SERVO_TRANSFER_PENDING;
    transfer.result = SERVO_OK;
    if (active) { queued = true; queued_record = index; }
    else {
        active = active_tracked = true;
        active_record = index;
        active_started = tick;
        transfer.started = true;
        transfer.started_tick = tick;
    }
    return SERVO_OK;
}
ServoStatus_t Servo_SetValuesOwned(const void *owner, const uint16_t *p, size_t count,
                                  uint16_t ms)
{
    ServoCommand_t commands[3];
    size_t i;
    CHECK(count == 3U);
    for (i = 0U; i < count; ++i) commands[i] = (ServoCommand_t){(uint16_t)i, p[i], ms};
    return Submit(owner, commands, count, false);
}
ServoStatus_t Servo_SetCommandsOwned(const void *owner, const ServoCommand_t *commands,
                                    size_t count)
{
    CHECK(count == 1U && commands[0].id == 3U);
    return Submit(owner, commands, count, false);
}
ServoStatus_t Servo_StopServoOwned(const void *owner, uint16_t id)
{
    ServoCommand_t command = {id, 0U, 0U};
    CHECK(id < 4U);
    return Submit(owner, &command, 1U, true);
}
ServoStatus_t Servo_CancelPendingOwned(const void *owner)
{
    CHECK(owner == lease);
    queued = active_tracked = false;
    memset(&transfer, 0, sizeof(transfer));
    return SERVO_OK;
}
ServoTransferStatus_t Servo_GetTransferStatus(const void *owner)
{
    CHECK(owner == lease);
    return transfer;
}
static void Complete(bool success)
{
    CHECK(active);
    if (active_tracked) {
        transfer.state = success ? SERVO_TRANSFER_COMPLETE : SERVO_TRANSFER_FAILED;
        transfer.result = success ? SERVO_OK : SERVO_UART_ERROR;
        transfer.completed_tick = tick;
    }
    active = active_tracked = false;
    if (queued) {
        queued = false;
        active = active_tracked = true;
        active_record = queued_record;
        active_started = tick;
        transfer.started = true;
        transfer.started_tick = tick;
    }
}
void Servo_Process(void)
{
    if (automatic && active && (uint32_t)(tick - active_started) >= 3U) Complete(true);
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
    CHECK(!ArmTrimService_OwnsMotion());
    CHECK(!active && !queued && lease == NULL);
    tick = 0U;
    record_count = 0U;
    next_submit = SERVO_OK;
    automatic = false;
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
    automatic = true;
    for (i = 0U; i < 10000U && ArmTrimService_IsBusy(); ++i) Step(5U);
    CHECK(!ArmTrimService_IsBusy());
}
static void BeginBall(void)
{
    const uint16_t ball[3] = {ARM_TRIM_BALL_P0, ARM_TRIM_BALL_P1, ARM_TRIM_BALL_P2};
    CHECK(ArmTrimService_Begin(ball, true) == ARM_TRIM_OK);
    CHECK(record_count == 0U);
}
static void TestProfileAndGrip(void)
{
    ArmTrimServiceStatus_t status;
    ArmTrimStatus_t before;
    unsigned i;
    Init();
    CHECK(ArmTrimService_MoveRelativeX(1.0f) == ARM_TRIM_REFERENCE_REQUIRED);
    CHECK(ArmTrimService_Grip(500U) == ARM_TRIM_REFERENCE_REQUIRED);
    CHECK(ArmTrimService_ReadyProfile(ARM_TRIM_PROFILE_BALL, true) == ARM_TRIM_OK);
    CHECK(record_count == 1U && records[0].count == 3U);
    for (i = 0U; i < 3U; ++i) CHECK(records[0].commands[i].id == i);
    CHECK(!ArmTrimService_GetStatus().core.reference_valid);
    CHECK(ArmTrimService_Begin(Config().references[0], true) == ARM_TRIM_BUSY);
    Step(100U);
    CHECK(ArmTrimService_IsBusy() && ArmTrimService_OwnsMotion());
    Complete(true);
    ArmTrimService_Process();
    Step(2299U);
    CHECK(!ArmTrimService_GetStatus().core.reference_valid);
    Step(1U);
    status = ArmTrimService_GetStatus();
    CHECK(status.state == ARM_TRIM_SERVICE_REFERENCE && status.core.reference_valid);
    CHECK(!status.busy && status.owns_motion);
    CHECK(status.core.estimated_position[0] == ARM_TRIM_BALL_P0);
    CHECK(ArmTrimService_MoveRelativeX(2.0f) == ARM_TRIM_OK);
    Finish();
    before = ArmTrimService_GetStatus().core;
    CHECK(fabsf(before.offset_mm - 2.0f) < 0.0001f);
    CHECK(ArmTrimService_Grip(500U) == ARM_TRIM_OK);
    CHECK(records[record_count - 1U].count == 1U);
    CHECK(records[record_count - 1U].commands[0].id == 3U);
    CHECK(records[record_count - 1U].commands[0].time_ms == 1500U);
    Finish();
    status = ArmTrimService_GetStatus();
    CHECK(status.core.reference_valid && status.owns_motion);
    CHECK(status.core.offset_mm == before.offset_mm);
    CHECK(memcmp(status.core.estimated_position, before.estimated_position,
                 sizeof(before.estimated_position)) == 0);
    CHECK(ArmTrimService_ReadyProfile(ARM_TRIM_PROFILE_BUCKET, true) == ARM_TRIM_OK);
    CHECK(!ArmTrimService_GetStatus().core.reference_valid);
    Finish();
    CHECK(ArmTrimService_GetStatus().core.estimated_position[0] == ARM_TRIM_BUCKET_P0);
    CHECK(ArmTrimService_End() == ARM_TRIM_OK);
    CHECK(!ArmTrimService_OwnsMotion() && !ArmTrimService_GetStatus().core.reference_valid);
}
static void TestInvalidBegin(void)
{
    uint16_t invalid[3] = {100U, 100U, 100U};
    Init();
    CHECK(ArmTrimService_Begin(invalid, false) == ARM_TRIM_INVALID);
    CHECK(!ArmTrimService_OwnsMotion());
    BeginBall();
    CHECK(ArmTrimService_Begin(invalid, true) == ARM_TRIM_OUT_OF_RANGE);
    CHECK(!ArmTrimService_GetStatus().core.reference_valid && !ArmTrimService_OwnsMotion());
    CHECK(ArmTrimService_MoveRelativeX(1.0f) == ARM_TRIM_REFERENCE_REQUIRED);
}
static void TestCancelMasks(void)
{
    unsigned i;
    Init();
    CHECK(ArmTrimService_ReadyProfile(ARM_TRIM_PROFILE_BALL, true) == ARM_TRIM_OK);
    CHECK(ArmTrimService_Cancel() == ARM_TRIM_OK);
    ArmTrimService_Process();
    CHECK(active && queued && record_count == 2U);
    CHECK(!transfer.started && records[1].commands[0].id == 0U);
    CHECK(ArmTrimService_End() == ARM_TRIM_OK);
    CHECK(ArmTrimService_OwnsMotion());
    Step(3U);
    Complete(true); /* Original fixed frame finishes; first stop now starts. */
    CHECK(transfer.started && transfer.state == SERVO_TRANSFER_PENDING);
    ArmTrimService_Process();
    CHECK(record_count == 2U && ArmTrimService_OwnsMotion());
    Finish();
    CHECK(record_count == 4U && !ArmTrimService_OwnsMotion());
    for (i = 0U; i < 3U; ++i)
        CHECK(records[i + 1U].stop && records[i + 1U].commands[0].id == i);
    CHECK(!ArmTrimService_GetStatus().core.reference_valid);
    Init();
    BeginBall();
    CHECK(ArmTrimService_Grip(1800U) == ARM_TRIM_OK);
    CHECK(ArmTrimService_Cancel() == ARM_TRIM_OK);
    Finish();
    CHECK(record_count == 2U && records[1].stop && records[1].commands[0].id == 3U);
    CHECK(!ArmTrimService_GetStatus().core.reference_valid && !ArmTrimService_OwnsMotion());
}
static void TestFailureNoRetry(void)
{
    unsigned i;
    Init();
    CHECK(ArmTrimService_ReadyProfile(ARM_TRIM_PROFILE_BALL, true) == ARM_TRIM_OK);
    Step(3U);
    Complete(false);
    ArmTrimService_Process();
    CHECK(ArmTrimService_GetStatus().state == ARM_TRIM_SERVICE_STOPPING);
    CHECK(ArmTrimService_End() == ARM_TRIM_FAULT_LATCHED);
    CHECK(ArmTrimService_OwnsMotion());
    CHECK(ArmTrimService_ClearFault() == ARM_TRIM_BUSY);
    Finish();
    CHECK(ArmTrimService_GetStatus().state == ARM_TRIM_SERVICE_FAULT);
    CHECK(record_count == 4U);
    for (i = 1U; i < record_count; ++i) CHECK(records[i].stop);
    CHECK(ArmTrimService_ReadyProfile(ARM_TRIM_PROFILE_BALL, true) == ARM_TRIM_FAULT_LATCHED);
    CHECK(record_count == 4U);
    CHECK(ArmTrimService_ClearFault() == ARM_TRIM_OK);
    CHECK(ArmTrimService_MoveRelativeX(1.0f) == ARM_TRIM_REFERENCE_REQUIRED);
    CHECK(ArmTrimService_ReadyProfile(ARM_TRIM_PROFILE_BALL, false) == ARM_TRIM_OK);
    Finish();
    CHECK(!ArmTrimService_OwnsMotion() && !ArmTrimService_GetStatus().core.reference_valid);
}
static void TestCoreCancelAndClockWrap(void)
{
    Init();
    BeginBall();
    CHECK(ArmTrimService_StartJog(1) == ARM_TRIM_OK);
    ArmTrimService_Process();
    CHECK(active && record_count == 1U);
    CHECK(ArmTrimService_Cancel() == ARM_TRIM_OK);
    Finish();
    CHECK(ArmTrimService_GetStatus().state == ARM_TRIM_SERVICE_IDLE);
    CHECK(record_count == 4U && !ArmTrimService_OwnsMotion());
    Init();
    tick = UINT32_MAX - 1000U;
    CHECK(ArmTrimService_ReadyProfile(ARM_TRIM_PROFILE_BALL, true) == ARM_TRIM_OK);
    Finish();
    CHECK(ArmTrimService_GetStatus().core.reference_valid);
    CHECK(ArmTrimService_End() == ARM_TRIM_OK);
}
static void TestMotionFailureAndStopFailure(void)
{
    unsigned i;
    Init();
    BeginBall();
    CHECK(ArmTrimService_MoveRelativeX(2.0f) == ARM_TRIM_OK);
    next_submit = SERVO_UART_ERROR;
    ArmTrimService_Process();
    CHECK(!ArmTrimService_GetStatus().core.reference_valid);
    CHECK(ArmTrimService_OwnsMotion());
    Finish();
    CHECK(ArmTrimService_GetStatus().state == ARM_TRIM_SERVICE_FAULT);
    CHECK(record_count == 3U);
    for (i = 0U; i < record_count; ++i) CHECK(records[i].stop);
    CHECK(ArmTrimService_MoveRelativeX(2.0f) == ARM_TRIM_FAULT_LATCHED);
    CHECK(ArmTrimService_ClearFault() == ARM_TRIM_OK);
    Init();
    BeginBall();
    CHECK(ArmTrimService_Grip(500U) == ARM_TRIM_OK);
    CHECK(ArmTrimService_Cancel() == ARM_TRIM_OK);
    next_submit = SERVO_UART_ERROR;
    ArmTrimService_Process();
    CHECK(ArmTrimService_GetStatus().stop_failed);
    /* Failed stop submission must not release the original active frame. */
    ArmTrimService_Process();
    CHECK(ArmTrimService_OwnsMotion());
    Step(3U);
    Complete(true);
    ArmTrimService_Process();
    CHECK(!ArmTrimService_OwnsMotion());
    CHECK(ArmTrimService_GetStatus().state == ARM_TRIM_SERVICE_FAULT);
    CHECK(record_count == 1U); /* No automatic repetition of the failed stop. */
    CHECK(ArmTrimService_ClearFault() == ARM_TRIM_OK);
}
static void TestCompletedDeadlineValidation(void)
{
    Init();
    CHECK(ArmTrimService_ReadyProfile(ARM_TRIM_PROFILE_BALL, true) == ARM_TRIM_OK);
    tick = 251U; Complete(true); /* No foreground service before the late TC. */
    ArmTrimService_Process();
    CHECK(ArmTrimService_GetStatus().last_request == ARM_TRIM_SERVICE_TIMEOUT);
    CHECK(!ArmTrimService_GetStatus().core.reference_valid);
    Finish();
    CHECK(ArmTrimService_GetStatus().state == ARM_TRIM_SERVICE_FAULT);
    CHECK(ArmTrimService_ClearFault() == ARM_TRIM_OK);
    Init(); BeginBall();
    CHECK(ArmTrimService_Grip(500U) == ARM_TRIM_OK);
    tick = 251U; Complete(true);
    ArmTrimService_Process();
    CHECK(ArmTrimService_GetStatus().last_request == ARM_TRIM_SERVICE_TIMEOUT);
    Finish();
    CHECK(record_count == 2U && records[1].commands[0].id == 3U);
    CHECK(!ArmTrimService_GetStatus().core.reference_valid);
    CHECK(ArmTrimService_ClearFault() == ARM_TRIM_OK);
    Init();
    CHECK(ArmTrimService_ReadyProfile(ARM_TRIM_PROFILE_BALL, true) == ARM_TRIM_OK);
    CHECK(ArmTrimService_Cancel() == ARM_TRIM_OK);
    ArmTrimService_Process(); /* Stop 000 accepted at 0, waits behind fixed TX. */
    tick = 3U; Complete(true); /* Stop 000 starts at 3. */
    tick = 251U; Complete(true); /* Start->TC=248, accept->TC=251: reject. */
    ArmTrimService_Process();
    CHECK(ArmTrimService_GetStatus().stop_failed);
    CHECK(ArmTrimService_GetStatus().last_request == ARM_TRIM_SERVICE_TIMEOUT);
    Finish();
    CHECK(record_count == 4U && ArmTrimService_GetStatus().state == ARM_TRIM_SERVICE_FAULT);
    CHECK(ArmTrimService_ClearFault() == ARM_TRIM_OK);
    Init();
    CHECK(ArmTrimService_ReadyProfile(ARM_TRIM_PROFILE_BALL, true) == ARM_TRIM_OK);
    tick = 3U; Complete(true);
    transfer.completed_tick = 4U; /* Corrupt/future completion timestamp. */
    ArmTrimService_Process();
    CHECK(ArmTrimService_GetStatus().last_request == ARM_TRIM_SERVICE_TIMEOUT);
    Finish();
    CHECK(ArmTrimService_ClearFault() == ARM_TRIM_OK);
}
static void TestConfiguredGripperLimits(void)
{
    ArmTrimServiceConfig_t config;
    unsigned before;
    Init();
    config = Config();
    config.grip_min_pwm = 700U;
    config.grip_max_pwm = 1900U;
    CHECK(ArmTrimService_Init(&config, Now, &tick) == ARM_TRIM_OK);
    BeginBall();
    before = record_count;
    CHECK(ArmTrimService_Grip(500U) == ARM_TRIM_INVALID);
    CHECK(ArmTrimService_Grip(2000U) == ARM_TRIM_INVALID);
    CHECK(record_count == before);
    CHECK(ArmTrimService_Grip(700U) == ARM_TRIM_OK);
    Finish();
    CHECK(ArmTrimService_End() == ARM_TRIM_OK);
    Finish();
    CHECK(!ArmTrimService_OwnsMotion());
    config.grip_min_pwm = 1900U;
    config.grip_max_pwm = 700U;
    CHECK(ArmTrimService_Init(&config, Now, &tick) == ARM_TRIM_INVALID);
    config.grip_min_pwm = 0U;
    config.grip_max_pwm = 1900U;
    CHECK(ArmTrimService_Init(&config, Now, &tick) == ARM_TRIM_INVALID);
}
int main(void)
{
    TestProfileAndGrip();
    TestInvalidBegin();
    TestCancelMasks();
    TestFailureNoRetry();
    TestCoreCancelAndClockWrap();
    TestMotionFailureAndStopFailure();
    TestCompletedDeadlineValidation();
    TestConfiguredGripperLimits();
    printf("Arm trim service tests: %u checks passed\n", checks);
    return 0;
}
