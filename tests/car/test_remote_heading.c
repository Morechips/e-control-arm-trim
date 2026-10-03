#include "remote_heading.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static uint32_t now;
static bool moving;
uint32_t HAL_GetTick(void) { return now; }
void Debug_Log(const char *s) { (void)s; }
uint8_t Debug_CanLog(uint8_t n) { (void)n; return 1U; }

static void Sample(float yaw, float gz)
{
    uint8_t bytes[22] = {0};
    int16_t raw[2];
    unsigned i, j;
    while (yaw > 180.0f) yaw -= 360.0f;
    while (yaw < -180.0f) yaw += 360.0f;
    raw[0] = (int16_t)(gz * 32768.0f / 2000.0f);
    raw[1] = (int16_t)(yaw * 32768.0f / 180.0f);
    for (i = 0U; i < 2U; ++i) {
        uint8_t *p = bytes + 11U * i;
        p[0] = 0x55U; p[1] = (uint8_t)(0x52U + i);
        p[6] = (uint8_t)raw[i]; p[7] = (uint8_t)((uint16_t)raw[i] >> 8);
        for (j = 0U; j < 10U; ++j) p[10] = (uint8_t)(p[10] + p[j]);
    }
    JY61_Parse(bytes, sizeof(bytes), now);
    RemoteHeading_Update(moving);
}

static void Reset(uint32_t tick)
{
    now = tick; moving = false; JY61_Init(); Heading_Init(); RemoteHeading_Reset();
}

static void TestHysteresisAndSampling(void)
{
    float integral;
    unsigned i;
    Reset(UINT32_MAX - 200U); RemoteHeading_Update(moving);
    assert(RemoteHeading_GetStatus()->phase == REMOTE_NO_IMU);
    Sample(20, 0); assert(RemoteHeading_GetStatus()->phase == REMOTE_ALIGNED);
    now += 50U; Sample(20.8f, 0); assert(RemoteHeading_GetStatus()->phase == REMOTE_ALIGNED);
    now += 50U; Sample(21.02f, 0);
    assert(RemoteHeading_GetStatus()->phase == REMOTE_ALIGNING && RemoteHeading_GetStatus()->correction_rpm == 1);
    now += 50U; Sample(20.8f, 0); assert(RemoteHeading_GetStatus()->phase == REMOTE_ALIGNING);
    now += 50U; Sample(20.49f, 0); assert(RemoteHeading_GetStatus()->phase == REMOTE_ALIGNED);
    now += 50U; Sample(18.98f, 0); assert(RemoteHeading_GetStatus()->correction_rpm == -1);
    assert(Heading_SetPID(1, 2, 0));
    now += 50U; Sample(25, 0);
    integral = Heading_GetIntegralOutput();
    assert(integral < 0);
    for (i = 0U; i < 50U; ++i) { ++now; RemoteHeading_Update(moving); }
    assert(Heading_GetIntegralOutput() == integral);
    now += 10U; Sample(120, 0); assert(RemoteHeading_GetStatus()->correction_rpm == 10);
    now += 10U; Sample(-80, 0); assert(RemoteHeading_GetStatus()->correction_rpm == -10);
    now += JY61_TIMEOUT_MS + 1U; RemoteHeading_Update(moving);
    assert(RemoteHeading_GetStatus()->phase == REMOTE_NO_IMU && RemoteHeading_GetStatus()->correction_rpm == 0);
    Sample(20, 0); assert(RemoteHeading_GetStatus()->phase == REMOTE_ALIGNED);
    assert(fabsf(RemoteHeading_GetStatus()->reference_yaw - 20) < 0.01f);
    puts("PASS remote heading: 1/0.5 hysteresis, signed minimum RPM, 10 RPM cap, fresh-sample PID, wrap and stale recovery");
}

static void TestTargetsAndReference(void)
{
    unsigned i;
    Reset(0U); Sample(179, 0);
    now += 10U; Sample(-179, 0);
    assert(fabsf(RemoteHeading_GetStatus()->error + 2) < 0.02f);
    assert(RemoteHeading_GetStatus()->correction_rpm > 0);
    RemoteHeading_Suspend();
    assert(RemoteHeading_GetStatus()->phase == REMOTE_SUSPENDED && RemoteHeading_GetStatus()->reference_valid);
    now += 10U; Sample(179, 0); assert(RemoteHeading_GetStatus()->phase == REMOTE_ALIGNED);
    for (i = 1U; i <= 4U; ++i) {
        RemoteHeading_BeginTurn(); now += 10U; Sample(179 - 90 * (float)i, 0);
        assert(RemoteHeading_GetStatus()->phase == REMOTE_TURNING && RemoteHeading_GetStatus()->correction_rpm == 0);
        RemoteHeading_CompleteTurn(1); RemoteHeading_Update(moving);
        assert(RemoteHeading_GetStatus()->direction == (RemoteDirection_t)(i % 4U));
        assert(RemoteHeading_GetStatus()->phase == REMOTE_ALIGNED);
        assert(fabsf(RemoteHeading_GetStatus()->reference_yaw - 179) < 0.01f);
    }
    RemoteHeading_BeginTurn(); RemoteHeading_CompleteTurn(-1); now += 10U; Sample(-91, 0);
    assert(RemoteHeading_GetStatus()->direction == REMOTE_LEFT && RemoteHeading_GetStatus()->phase == REMOTE_ALIGNED);
    RemoteHeading_BeginTurn(); RemoteHeading_CompleteTurn(2); now += 10U; Sample(89, 0);
    assert(RemoteHeading_GetStatus()->direction == REMOTE_RIGHT && RemoteHeading_GetStatus()->phase == REMOTE_ALIGNED);
    RemoteHeading_Reset(); Sample(40, 0);
    assert(RemoteHeading_GetStatus()->direction == REMOTE_FRONT && fabsf(RemoteHeading_GetStatus()->reference_yaw - 40) < 0.01f);
    puts("PASS remote targets: FRONT/RIGHT/BACK/LEFT cycle, left/180 composition, stable reference, suspend and new session");
}

static void TestMovingCorrection(void)
{
    unsigned i;
    Reset(0U); Sample(0,0);
    moving = true; RemoteHeading_Update(moving);
    now += 100U; Sample(2.0f,0);
    assert(RemoteHeading_GetStatus()->phase == REMOTE_ALIGNED &&
           RemoteHeading_GetStatus()->correction_rpm == 0);
    now += 100U; Sample(3.1f,0);
    assert(RemoteHeading_GetStatus()->phase == REMOTE_DRIFTING &&
           RemoteHeading_GetStatus()->correction_rpm == 1);
    now += 100U; Sample(30,0);
    assert(RemoteHeading_GetStatus()->phase == REMOTE_DRIFTING &&
           RemoteHeading_GetStatus()->correction_rpm == 4);
    /* A single severe frame cannot brake, and repeat foreground calls do not
     * count as fresh IMU samples. */
    for (i = 0U; i < 30U; ++i) RemoteHeading_Update(moving);
    assert(RemoteHeading_GetStatus()->phase == REMOTE_DRIFTING);
    now += 100U; Sample(2,0);
    assert(RemoteHeading_GetStatus()->phase == REMOTE_DRIFTING);
    now += 100U; Sample(1.4f,0);
    assert(RemoteHeading_GetStatus()->phase == REMOTE_ALIGNED);
    for (i = 0U; i < 4U; ++i) {
        now += 100U; Sample(11.0f,0);
        assert(RemoteHeading_GetStatus()->phase ==
               (i == 3U ? REMOTE_ALIGNING : REMOTE_DRIFTING));
    }
    moving = false; RemoteHeading_Update(moving);
    assert(RemoteHeading_GetStatus()->phase == REMOTE_ALIGNING &&
           RemoteHeading_GetStatus()->correction_rpm <= REMOTE_HEADING_MAX_RPM);
    now += 100U; Sample(0.4f,0);
    assert(RemoteHeading_GetStatus()->phase == REMOTE_ALIGNED);
    now += 100U; Sample(1.2f,0);
    assert(RemoteHeading_GetStatus()->phase == REMOTE_ALIGNING);
    puts("PASS moving heading: 3/1.5 degree hysteresis, 4 RPM mix, isolated spike ignored, 300 ms sustained drift brake, static 1/0.5 restored");
}

int main(void)
{
    TestHysteresisAndSampling(); TestTargetsAndReference(); TestMovingCorrection();
    return 0;
}
