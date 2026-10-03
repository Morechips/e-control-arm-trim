#include "arm_trim_project.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(c) do { ++checks; if (!(c)) { fprintf(stderr, "FAIL %u: %s\n", (unsigned)__LINE__, #c); exit(1); } } while (0)

static unsigned checks;
static struct {
    ArmTrim_t trim;
    uint32_t now, started, completed, next_delay, duration;
    uint32_t send_tick[1024], stop_tick[3];
    uint16_t move_ms[1024];
    unsigned sends, stops, cancellations;
    bool active, failed, hanging, canceled;
} fixture;

static bool Transfer(void)
{
    if (fixture.active) return false;
    fixture.started = fixture.now + fixture.next_delay;
    fixture.completed = fixture.started + fixture.duration;
    fixture.next_delay = 0U;
    fixture.active = true;
    fixture.canceled = false;
    return true;
}

static bool Send(void *user, const uint16_t p[3], uint16_t move_ms)
{
    CHECK(user == &fixture && p != NULL && fixture.sends < 1024U);
    if (!Transfer()) return false;
    fixture.send_tick[fixture.sends] = fixture.now;
    fixture.move_ms[fixture.sends++] = move_ms;
    return true;
}

static bool Stop(void *user, unsigned joint)
{
    CHECK(user == &fixture && joint == fixture.stops && joint < 3U);
    fixture.stop_tick[fixture.stops++] = fixture.now;
    return Transfer();
}

static uint32_t Now(void *user)
{
    CHECK(user == &fixture);
    return fixture.now;
}

static ArmTrimTxState_t Poll(void *user, uint32_t *started, uint32_t *completed)
{
    CHECK(user == &fixture);
    if (fixture.canceled) return ARM_TRIM_TX_FAILED;
    if (fixture.hanging || (int32_t)(fixture.now - fixture.completed) < 0)
        return ARM_TRIM_TX_PENDING;
    fixture.active = false;
    if (fixture.failed) return ARM_TRIM_TX_FAILED;
    *started = fixture.started;
    *completed = fixture.completed;
    return ARM_TRIM_TX_COMPLETE;
}

static void CancelPending(void *user)
{
    CHECK(user == &fixture);
    ++fixture.cancellations;
    if (fixture.active && (int32_t)(fixture.now - fixture.started) < 0) {
        fixture.canceled = true;
        fixture.active = false;
    }
}

static void Init(void)
{
    const uint16_t ball[] = {1356U, 1850U, 698U};
    ArmTrimConfig_t config;
    const ArmTrimIO_t io = {Send, Stop, Now, NULL, &fixture};
    const ArmTrimAsyncIO_t async = {Poll, CancelPending};
    memset(&fixture, 0, sizeof(fixture));
    fixture.duration = 4U;
    ArmTrimProject_DefaultConfig(&config);
    CHECK(ArmTrim_Init(&fixture.trim, &config, &io) == ARM_TRIM_OK);
    CHECK(ArmTrim_SetAsyncIO(&fixture.trim, &async) == ARM_TRIM_OK);
    CHECK(ArmTrim_Synchronize(&fixture.trim, ball) == ARM_TRIM_OK);
}

static void ProcessAt(uint32_t now)
{
    fixture.now = now;
    ArmTrim_Process(&fixture.trim);
}

static void Finish(void)
{
    unsigned i;
    for (i = 0U; i < 20000U; ++i) {
        ArmTrim_Process(&fixture.trim);
        if (fixture.trim.status.state == ARM_TRIM_COMPLETE_ESTIMATED ||
            fixture.trim.status.state == ARM_TRIM_CANCELLED ||
            fixture.trim.status.state == ARM_TRIM_FAULT) return;
        fixture.now += 5U;
    }
    CHECK(false);
}

static void TestActualStartCadence(void)
{
    Init();
    fixture.next_delay = 100U;
    CHECK(ArmTrim_StartJog(&fixture.trim, -1) == ARM_TRIM_OK);
    CHECK(ArmTrim_SetAsyncIO(&fixture.trim, NULL) == ARM_TRIM_BUSY);
    ProcessAt(0U);
    CHECK(fixture.sends == 1U);
    ProcessAt(50U);
    CHECK(fixture.sends == 1U && fixture.trim.status.offset_mm == 0.0f);
    ProcessAt(104U);
    CHECK(fixture.trim.dispatch_tick == 100U && fixture.trim.segment_tick == 104U);
    ProcessAt(149U);
    CHECK(fixture.sends == 1U);
    ProcessAt(150U);
    CHECK(fixture.sends == 2U && fixture.send_tick[1] == 150U);
    ProcessAt(154U);
    ProcessAt(200U);
    CHECK(fixture.sends == 3U && fixture.send_tick[2] == 200U);
    CHECK(ArmTrim_Cancel(&fixture.trim) == ARM_TRIM_OK);
    Finish();
    CHECK(fixture.trim.status.state == ARM_TRIM_CANCELLED && fixture.stops == 3U);
}

static void TestFinalTimerAndWrap(void)
{
    uint32_t complete, motion_end;
    Init();
    fixture.now = UINT32_MAX - 100U;
    fixture.next_delay = 20U;
    CHECK(ArmTrim_MoveRelativeX(&fixture.trim, 1.0f) == ARM_TRIM_OK);
    ArmTrim_Process(&fixture.trim);
    complete = fixture.completed;
    motion_end = complete + fixture.move_ms[0];
    ProcessAt(complete);
    while ((uint32_t)(motion_end - fixture.now) > 100U) ProcessAt(fixture.now + 100U);
    ProcessAt(motion_end - 1U);
    CHECK(fixture.trim.status.state == ARM_TRIM_MOVING && fixture.trim.status.offset_mm == 0.0f);
    ProcessAt(motion_end);
    CHECK(fixture.trim.status.state == ARM_TRIM_SETTLING && fixture.trim.status.offset_mm == 1.0f);
    ProcessAt(motion_end + 200U);
    ProcessAt(motion_end + 300U);
    CHECK(fixture.trim.status.state == ARM_TRIM_COMPLETE_ESTIMATED && fixture.trim.status.reference_valid);
}

static void TestReleaseWaitsForTc(void)
{
    Init();
    fixture.duration = 30U;
    CHECK(ArmTrim_StartJog(&fixture.trim, -1) == ARM_TRIM_OK);
    ProcessAt(0U);
    CHECK(ArmTrim_ReleaseJog(&fixture.trim) == ARM_TRIM_OK);
    ProcessAt(30U);
    ProcessAt(79U);
    CHECK(fixture.sends == 1U && fixture.trim.status.offset_mm == 0.0f);
    ProcessAt(80U);
    CHECK(fixture.sends == 2U); /* braking tail starts only after TC + T */
    Finish();
    CHECK(fixture.trim.status.state == ARM_TRIM_COMPLETE_ESTIMATED && fixture.trim.status.reference_valid);
}

static void TestSequentialStopsAndCanceledQueue(void)
{
    Init();
    fixture.next_delay = 100U;
    CHECK(ArmTrim_StartJog(&fixture.trim, -1) == ARM_TRIM_OK);
    ProcessAt(0U);
    CHECK(ArmTrim_Cancel(&fixture.trim) == ARM_TRIM_OK && fixture.cancellations == 1U);
    ProcessAt(1U); /* consumes canceled pending move */
    CHECK(fixture.stops == 0U);
    ProcessAt(2U);
    CHECK(fixture.stops == 1U && fixture.trim.stop_index == 0U);
    ProcessAt(5U);
    CHECK(fixture.stops == 1U && fixture.trim.stop_index == 0U);
    ProcessAt(6U);
    CHECK(fixture.trim.stop_index == 1U);
    ProcessAt(7U);
    CHECK(fixture.stops == 2U);
    Finish();
    CHECK(fixture.stops == 3U && fixture.trim.status.state == ARM_TRIM_CANCELLED);
    CHECK(!fixture.trim.status.reference_valid && !fixture.trim.status.stop_failed);
}

static void TestFailureAndBoundedTimeout(void)
{
    Init();
    CHECK(ArmTrim_MoveRelativeX(&fixture.trim, 1.0f) == ARM_TRIM_OK);
    ProcessAt(0U);
    fixture.failed = true;
    ProcessAt(4U);
    CHECK(fixture.trim.status.state == ARM_TRIM_STOPPING && fixture.trim.status.error == ARM_TRIM_TRANSPORT);
    fixture.failed = false;
    Finish();
    CHECK(fixture.trim.status.state == ARM_TRIM_FAULT && fixture.stops == 3U);
    CHECK(!fixture.trim.status.reference_valid);

    Init();
    fixture.hanging = true;
    CHECK(ArmTrim_MoveRelativeX(&fixture.trim, 1.0f) == ARM_TRIM_OK);
    ProcessAt(0U);
    ProcessAt(100U);
    ProcessAt(200U);
    ProcessAt(251U);
    CHECK(fixture.trim.status.state == ARM_TRIM_STOPPING && fixture.trim.status.error == ARM_TRIM_SERVICE_TIMEOUT);
    Finish();
    CHECK(fixture.trim.status.state == ARM_TRIM_FAULT && fixture.stops == 3U);

    Init();
    CHECK(ArmTrim_Cancel(&fixture.trim) == ARM_TRIM_OK);
    ProcessAt(0U);
    fixture.hanging = true;
    ProcessAt(100U);
    ProcessAt(200U);
    ProcessAt(251U);
    CHECK(fixture.trim.stop_index == 1U && fixture.trim.status.stop_failed);
    Finish();
    CHECK(fixture.trim.status.state == ARM_TRIM_FAULT && fixture.stops == 3U);
    CHECK(fixture.trim.status.error == ARM_TRIM_SERVICE_TIMEOUT);
}

static void TestLateCompletionDoesNotBurst(void)
{
    Init();
    fixture.duration = 75U;
    CHECK(ArmTrim_StartJog(&fixture.trim, -1) == ARM_TRIM_OK);
    ProcessAt(0U);
    ProcessAt(75U);
    CHECK(fixture.sends == 1U && fixture.trim.status.state == ARM_TRIM_STOPPING);
    CHECK(fixture.trim.status.error == ARM_TRIM_SERVICE_TIMEOUT);
    fixture.duration = 4U;
    Finish();
    CHECK(fixture.trim.status.state == ARM_TRIM_FAULT && fixture.sends == 1U);
}

static void TestStopTcFailure(void)
{
    Init();
    CHECK(ArmTrim_Cancel(&fixture.trim) == ARM_TRIM_OK);
    ProcessAt(0U);
    fixture.failed = true;
    ProcessAt(3U);
    CHECK(fixture.trim.stop_index == 0U && !fixture.trim.status.stop_failed);
    ProcessAt(4U);
    CHECK(fixture.trim.stop_index == 1U && fixture.trim.status.stop_failed);
    CHECK(fixture.trim.status.error == ARM_TRIM_TRANSPORT);
    fixture.failed = false;
    Finish();
    CHECK(fixture.trim.status.state == ARM_TRIM_FAULT && fixture.stops == 3U);
    CHECK(!fixture.trim.status.reference_valid);
}

int main(void)
{
    TestActualStartCadence();
    TestFinalTimerAndWrap();
    TestReleaseWaitsForTc();
    TestSequentialStopsAndCanceledQueue();
    TestFailureAndBoundedTimeout();
    TestLateCompletionDoesNotBurst();
    TestStopTcFailure();
    printf("arm trim async: %u checks passed\n", checks);
    return 0;
}
