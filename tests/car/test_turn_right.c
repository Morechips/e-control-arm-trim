#include "turn_right.h"
#include "turn_config.h"
#include "heading_control.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

extern unsigned mock_reset_count;
extern HAL_StatusTypeDef mock_reset_result;
extern uint8_t mock_reset_bytes[5];
static uint32_t now;
static unsigned drives, stops;
static int16_t last_omega;
static uint8_t motor_idle = 1U, motor_fault;
static HAL_StatusTypeDef drive_result = HAL_OK;
uint32_t HAL_GetTick(void) { return now; }
void Debug_Log(const char *text) { (void)text; }
uint8_t Debug_CanLog(uint8_t n) { (void)n; return 1U; }
uint8_t Motor_IsIdle(void) { return motor_idle; }
uint8_t Motor_HasFault(void) { return motor_fault; }
HAL_StatusTypeDef mecanum_drive(int16_t forward, int16_t lateral, int16_t omega)
{
    assert(forward == 0 && lateral == 0 && omega != 0);
    last_omega = omega;
    ++drives;
    return drive_result;
}
HAL_StatusTypeDef brake(void) { ++stops; return HAL_OK; }

static void Sample(float yaw, float gz)
{
    uint8_t frames[22] = {0};
    unsigned i, j;
    int16_t values[] = {(int16_t)(gz * 32768.0f / 2000.0f), (int16_t)(yaw * 32768.0f / 180.0f)};
    for (i = 0U; i < 2U; ++i)
    {
        uint8_t *f = frames + i * 11U;
        f[0] = 0x55; f[1] = (uint8_t)(0x52U + i);
        f[6] = (uint8_t)values[i]; f[7] = (uint8_t)((uint16_t)values[i] >> 8);
        for (j = 0U; j < 10U; ++j) f[10] = (uint8_t)(f[10] + f[j]);
    }
    JY61_Parse(frames, sizeof(frames), now);
}
static void SampleYaw(float yaw)
{
    uint8_t frame[11] = {0x55U, 0x53U};
    int16_t raw = (int16_t)(yaw * 32768.0f / 180.0f);
    unsigned i;
    frame[6] = (uint8_t)raw;
    frame[7] = (uint8_t)((uint16_t)raw >> 8);
    for (i = 0U; i < 10U; ++i) frame[10] = (uint8_t)(frame[10] + frame[i]);
    JY61_Parse(frame, sizeof(frame), now);
}
static void Reset(uint32_t tick)
{
    TurnRight_Cancel();
    now = tick; drives = stops = mock_reset_count = 0U; last_omega = 0;
    motor_idle = 1U; motor_fault = 0U; drive_result = mock_reset_result = HAL_OK;
    JY61_Init(); Heading_Init();
}
static void Expect(TurnRightState_t state)
{
    if (TurnRight_GetStatus()->state != state)
        fprintf(stderr, "Expected state %d, got %d at %lu ms, angle %.2f\n",
                (int)state, (int)TurnRight_GetStatus()->state,
                (unsigned long)now, (double)TurnRight_GetStatus()->turned_degrees);
    assert(TurnRight_GetStatus()->state == state);
}
static void Feed(unsigned count, float yaw, float gz)
{
    while (count--) { now += 10U; Sample(yaw, gz); TurnRight_Process(true); }
}
static void Reach90(void)
{
    Sample(0, 0); assert(right90(CAR_PD10_FORWARD_RPM) == HAL_OK);
    Feed(1, -59, -20); Expect(TURN_RIGHT_TURNING);
    Feed(1, -61, -20); Expect(TURN_RIGHT_TURNING);
    assert(drives == 1U && last_omega == TURN_RIGHT_90_RPM);
    Feed(1, -84, -20); Expect(TURN_RIGHT_TURNING);
    Feed(1, -86, -20); Expect(TURN_RIGHT_STOPPING);
    assert(stops == 1U && mock_reset_count == 0U);
}
static void Test180DegreeReuse(void)
{
    Reset(0); Sample(0, 0);
    assert(right180(CAR_PD10_FORWARD_RPM) == HAL_OK);
    assert(last_omega == 30);
    Feed(1, -149, -20); Expect(TURN_RIGHT_TURNING);
    Feed(1, -151, -20); Expect(TURN_RIGHT_TURNING);
    assert(last_omega == 20);
    Feed(1, -174, -20); Expect(TURN_RIGHT_TURNING);
    Feed(1, -176, -20); Expect(TURN_RIGHT_STOPPING);
    assert(fabsf(TurnRight_GetStatus()->turned_degrees - 176.0f) < 0.02f);
    assert(stops == 1U && mock_reset_count == 0U);
    Feed(11, -180, 0); Expect(TURN_RIGHT_RESETTING);
}
static void TestLeft90DegreeReuse(void)
{
    Reset(0); Sample(0, 0);
    assert(left90(30) == HAL_OK && last_omega == -TURN_LEFT_90_RPM);
    Feed(1, 84, 20); Expect(TURN_RIGHT_TURNING);
    Feed(1, 86, 20); Expect(TURN_RIGHT_STOPPING);
    assert(stops == 1U && mock_reset_count == 0U);
    Feed(11, 91, 0); Expect(TURN_RIGHT_RESETTING);
    Feed(1, 0, 0); Expect(TURN_RIGHT_DONE);
    assert(mock_reset_count == 1U);

    Reset(0); Sample(170, 0);
    assert(left90(30) == HAL_OK && last_omega == -TURN_LEFT_90_RPM);
    Feed(1, -179, 20); Expect(TURN_RIGHT_TURNING);
    Feed(1, -104, 20); Expect(TURN_RIGHT_STOPPING);
    assert(fabsf(TurnRight_GetStatus()->turned_degrees - 86.0f) < 0.02f);
}
static void Settle(void)
{
    Feed(11, -91, 0); Expect(TURN_RIGHT_RESETTING); assert(mock_reset_count == 1U);
}

static void TestCompletion(void)
{
    const uint8_t zero[] = {0xFF, 0xAA, 0x01, 0x04, 0x00};
    Reset(0); Reach90();
    motor_idle = 0U; Feed(12, -91, 0); assert(mock_reset_count == 0U);
    motor_idle = 1U; Settle();
    assert(memcmp(mock_reset_bytes, zero, sizeof(zero)) == 0 && !JY61_IsValid());
    TurnRight_Process(true); Expect(TURN_RIGHT_RESETTING);
    Sample(0, 0); TurnRight_Process(true); Expect(TURN_RIGHT_RESETTING); /* Same tick is not proof. */
    Feed(1, 20, 0); Expect(TURN_RIGHT_RESETTING);
    assert(Heading_SetPID(2, 1, 0.5f));
    Feed(1, 0, 0); Expect(TURN_RIGHT_DONE);
    assert(Heading_GetStatus()->actual == 0 && Heading_GetStatus()->target == 0);
    assert(Heading_GetPID()->kp == 2);
    TurnRight_Process(true); assert(mock_reset_count == 1U && drives == 1U);
    assert(right90(CAR_PD10_FORWARD_RPM) == HAL_OK);
    TurnRight_Cancel(); Expect(TURN_RIGHT_CANCELLED);
}
static void TestProgress(void)
{
    Reset(0); Sample(-170, 0);
    assert(right90(30) == HAL_OK && right90(30) == HAL_BUSY);
    Feed(1, 179, -20); Expect(TURN_RIGHT_TURNING);
    assert(fabsf(TurnRight_GetStatus()->turned_degrees - 11) < 0.02f);
    TurnRight_Process(true);
    assert(fabsf(TurnRight_GetStatus()->turned_degrees - 11) < 0.02f);
    Feed(1, 104, -20); Expect(TURN_RIGHT_STOPPING);
    Reset(0); Sample(100, 0); assert(right90(30) == HAL_OK);
    TurnRight_Process(true); Expect(TURN_RIGHT_TURNING); /* Absolute yaw >= 90 is not completion. */
    Feed(1, 109, 20); Expect(TURN_RIGHT_TURNING);
    assert(TurnRight_GetStatus()->turned_degrees < 0 && stops == 0U);
    Feed(1, 111, 20); Expect(TURN_RIGHT_FAULT);
    assert(TurnRight_GetStatus()->error == TURN_RIGHT_WRONG_WAY && stops == 1U);

    Reset(0); Sample(-91.53f, 0); assert(right90(100) == HAL_OK);
    Feed(1, 171.87f, -510.86f); Expect(TURN_RIGHT_STOPPING);
    assert(TurnRight_GetStatus()->turned_degrees > 90.0f && stops == 1U);
}
static void TestFaults(void)
{
    Reset(0); assert(right90(30) == HAL_ERROR && drives == 0U);
    Sample(0, 0); assert(right90(0) == HAL_ERROR && right90(-1) == HAL_ERROR);
    motor_idle = 0U; assert(right90(30) == HAL_BUSY); motor_idle = 1U;
    motor_fault = 1U; assert(right90(30) == HAL_ERROR); motor_fault = 0U;
    drive_result = HAL_BUSY; assert(right90(30) == HAL_BUSY); drive_result = HAL_OK;
    assert(right90(30) == HAL_OK);
    now += JY61_TIMEOUT_MS + 1U; TurnRight_Process(true); Expect(TURN_RIGHT_FAULT);
    assert(TurnRight_GetStatus()->error == TURN_RIGHT_IMU_ERROR && stops == 1U);
    TurnRight_Process(true); assert(stops == 1U && mock_reset_count == 0U);

    Reset(0); Sample(0, 0); assert(right90(30) == HAL_OK);
    motor_fault = 1U; TurnRight_Process(true); Expect(TURN_RIGHT_FAULT);
    assert(TurnRight_GetStatus()->error == TURN_RIGHT_MOTOR_ERROR && stops == 1U);
    Reset(0); Sample(0, 0); assert(right90(30) == HAL_OK);
    TurnRight_Process(false); Expect(TURN_RIGHT_CANCELLED);
    assert(stops == 1U && mock_reset_count == 0U);

    Reset(UINT32_MAX - 40U); Sample(0, 0); assert(right90(30) == HAL_OK);
    Feed(TURN_PROGRESS_TIMEOUT_MS / 10U, 0, 0); Expect(TURN_RIGHT_FAULT);
    assert(TurnRight_GetStatus()->error == TURN_RIGHT_NO_PROGRESS && stops == 1U);

    Reset(0); Sample(0, 0); assert(right90(30) == HAL_OK);
    for (unsigned i = 1U; i <= TURN_RIGHT_TIMEOUT_MS / 10U; ++i)
        Feed(1, -(float)i * 0.05f, -5);
    Expect(TURN_RIGHT_FAULT);
    assert(TurnRight_GetStatus()->error == TURN_RIGHT_TIMEOUT);
    Reset(0); Reach90(); Feed(TURN_STOP_TIMEOUT_MS / 10U, -91, -20);
    Expect(TURN_RIGHT_FAULT); assert(mock_reset_count == 0U);
    Reset(0); Reach90(); mock_reset_result = HAL_ERROR; Feed(11, -91, 0);
    Expect(TURN_RIGHT_FAULT);
    assert(TurnRight_GetStatus()->error == TURN_RIGHT_RESET_ERROR && mock_reset_count == 1U);
    Reset(0); Reach90(); Settle();
    now += JY61_RESET_TIMEOUT_MS + 1U; TurnRight_Process(true); Expect(TURN_RIGHT_FAULT);
    assert(TurnRight_GetStatus()->error == TURN_RIGHT_RESET_ERROR && mock_reset_count == 1U);
}
static void TestStableSamples(void)
{
    unsigned i;
    Reset(0); Reach90(); Feed(1, -91, 0);
    for (i = 0U; i < 10U; ++i) { now += 10U; TurnRight_Process(true); }
    assert(mock_reset_count == 0U); /* Re-reading one sample cannot prove stillness. */
    Sample(-91, 10); Sample(-91, 0); TurnRight_Process(true);
    Feed(9, -91, 0); assert(mock_reset_count == 0U); /* Motion earlier in a batch resets stability. */
    Feed(1, -91, 0); Expect(TURN_RIGHT_RESETTING);
    TurnRight_Process(false); Expect(TURN_RIGHT_CANCELLED);
    Feed(1, 0, 0); Expect(TURN_RIGHT_CANCELLED);
}

static void TestSettledCorrections(void)
{
    Reset(0); Reach90();
    Feed(11, -120, 0); Expect(TURN_RIGHT_CORRECTING);
    assert(last_omega == -TURN_RIGHT_90_RPM && drives == 2U && mock_reset_count == 0U);
    Feed(1, -94, 20); Expect(TURN_RIGHT_STOPPING);
    Feed(11, -94, 0); Expect(TURN_RIGHT_RESETTING);
    assert(stops == 2U && mock_reset_count == 1U);
    assert(fabsf(TurnRight_GetStatus()->turned_degrees - 94.0f) < 0.02f);
    Feed(1, 0, 0); Expect(TURN_RIGHT_DONE);

    Reset(0); Reach90();
    Feed(11, -80, 0); Expect(TURN_RIGHT_CORRECTING);
    assert(last_omega == TURN_RIGHT_90_RPM);
    Feed(1, -86, -20); Expect(TURN_RIGHT_STOPPING);
    Feed(11, -88, 0); Expect(TURN_RIGHT_RESETTING);
    assert(fabsf(TurnRight_GetStatus()->turned_degrees - 88.0f) < 0.02f);

    Reset(0); Reach90();
    Feed(11, -120, 0); Expect(TURN_RIGHT_CORRECTING);
    Feed(TURN_PROGRESS_TIMEOUT_MS / 10U, -120, 0); Expect(TURN_RIGHT_FAULT);
    assert(TurnRight_GetStatus()->error == TURN_RIGHT_NO_PROGRESS);

    Reset(0); Reach90();
    Feed(11, -120, 0); Expect(TURN_RIGHT_CORRECTING);
    Feed(1, -132, -20); Expect(TURN_RIGHT_FAULT);
    assert(TurnRight_GetStatus()->error == TURN_RIGHT_WRONG_WAY);

    Reset(0); Reach90();
    Feed(11, -120, 0); Expect(TURN_RIGHT_CORRECTING);
    TurnRight_Cancel(); Expect(TURN_RIGHT_CANCELLED);
    Feed(1, -94, 20); Expect(TURN_RIGHT_CANCELLED);
    assert(stops == 2U && mock_reset_count == 0U);
}

static void TestCorrectionLimit(void)
{
    Reset(0); Reach90();
    Feed(11, -120, 0); Expect(TURN_RIGHT_CORRECTING);
    Feed(1, -94, 20); Expect(TURN_RIGHT_STOPPING);
    Feed(11, -80, 0); Expect(TURN_RIGHT_CORRECTING);
    Feed(1, -86, -20); Expect(TURN_RIGHT_STOPPING);
    Feed(11, -120, 0); Expect(TURN_RIGHT_CORRECTING);
    Feed(1, -94, 20); Expect(TURN_RIGHT_STOPPING);
    Feed(11, -80, 0); Expect(TURN_RIGHT_FAULT);
    assert(TurnRight_GetStatus()->error == TURN_RIGHT_TOLERANCE_ERROR);
    assert(mock_reset_count == 0U && stops == 5U && drives == 4U);
    TurnRight_Process(true); assert(stops == 5U && drives == 4U);
}
static void TestSpeedCapsAndPredictedStop(void)
{
    Reset(0); Sample(0, 0);
    assert(right90(167) == HAL_OK && last_omega == TURN_RIGHT_90_RPM);
    Feed(1, -61, -100); Expect(TURN_RIGHT_TURNING);
    assert(last_omega == TURN_RIGHT_90_RPM && drives == 1U);
    Feed(1, -82, -100); Expect(TURN_RIGHT_TURNING);
    Feed(1, -83, -100); Expect(TURN_RIGHT_STOPPING);
    assert(stops == 1U); /* 80 ms at 100 deg/s leads the 5-degree threshold. */

    Reset(0); Sample(0, 0);
    assert(right90(60) == HAL_OK);
    Feed(1, -74, -500); Expect(TURN_RIGHT_TURNING);
    Feed(1, -76, -500); Expect(TURN_RIGHT_STOPPING); /* Lead is capped at 15 degrees. */

    Reset(0); Sample(0, 0);
    assert(right180(167) == HAL_OK && last_omega == TURN_RIGHT_180_RPM);
    Feed(1, -150, -150); Expect(TURN_RIGHT_TURNING);
    Feed(1, -169, -150); Expect(TURN_RIGHT_STOPPING);

    Reset(0); Sample(0, 0);
    assert(left90(167) == HAL_OK && last_omega == -TURN_LEFT_90_RPM);

    Reset(0); Sample(0, 0);
    assert(right90(60) == HAL_OK);
    Feed(1, -80, -100); Expect(TURN_RIGHT_TURNING);
    now += TURN_RATE_FRESH_MS + 10U;
    SampleYaw(-84); TurnRight_Process(true); Expect(TURN_RIGHT_TURNING);
    now += 10U;
    SampleYaw(-86); TurnRight_Process(true); Expect(TURN_RIGHT_STOPPING);

    Reset(0); Reach90(); Feed(11, -120, 0); Expect(TURN_RIGHT_CORRECTING);
    Feed(1, -100, 100); Expect(TURN_RIGHT_CORRECTING);
    Feed(1, -98, 100); Expect(TURN_RIGHT_STOPPING);

    Reset(0); Reach90(); Feed(11, -100, 0); Expect(TURN_RIGHT_CORRECTING);
    Feed(1, -97, 100); Expect(TURN_RIGHT_STOPPING);

    Reset(0); Reach90(); Feed(11, -96, 0); Expect(TURN_RIGHT_CORRECTING);
    Feed(1, -96, 100); Expect(TURN_RIGHT_CORRECTING);
    Feed(1, -95, 100); Expect(TURN_RIGHT_STOPPING);
}
int main(void)
{
    TestCompletion(); TestProgress(); TestFaults(); TestStableSamples();
    Test180DegreeReuse(); TestLeft90DegreeReuse();
    TestSettledCorrections(); TestCorrectionLimit();
    TestSpeedCapsAndPredictedStop();
    puts("PASS right90: relative/wrapped/signed yaw / stop before zero / fresh zero confirmation");
    puts("PASS right90: busy/retrigger/cancel / stale IMU/motor fault / timeouts/tick wrap / stable samples");
    puts("PASS right180: reuses the same JY61 relative-yaw stop/reset/confirmation workflow");
    puts("PASS left90: reverse drive / relative yaw / wrapped yaw / stop and reset confirmation");
    puts("PASS correction: settled overshoot/undershoot / reverse low-speed motion / no progress / three-attempt limit");
    puts("PASS turn speed/prediction: 90/180 caps / early stop / stale gyro / bounded correction lead");
    return 0;
}
