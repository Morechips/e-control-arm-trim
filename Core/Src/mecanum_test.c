#include "mecanum_test.h"
#include "serial_io.h"
#include <stdio.h>
volatile uint32_t mecanum_test_action, mecanum_test_request;
volatile uint8_t mecanum_test_active;
static uint32_t seen_request, started;

void Mecanum_Test_Vector(MecanumTestAction_t action, int16_t *vx, int16_t *vy, int16_t *omega)
{
    /* Explicit test inputs, independently checked against the wheel table. */
    static const int8_t vectors[MEC_ACTION_COUNT][3] = {
        {0,0,0}, {1,0,0}, {-1,0,0}, {0,-1,0}, {0,1,0},
        {1,-1,0}, {1,1,0}, {-1,-1,0}, {-1,1,0}
    };
    *vx = *vy = *omega = 0;
    if ((unsigned)action >= MEC_ACTION_COUNT) return;
    /* Half components on diagonals keep every wheel <= test RPM (30). */
    {
        int16_t scale = (vectors[action][0] && vectors[action][1]) ?
                        CAR_MECANUM_TEST_RPM / 2 : CAR_MECANUM_TEST_RPM;
        *vx = (int16_t)(vectors[action][0] * scale);
        *vy = (int16_t)(vectors[action][1] * scale);
        *omega = (int16_t)(vectors[action][2] * scale);
    }
}
static void LogAction(MecanumTestAction_t action, const MecanumWheels_t *w)
{
    static const char *const names[] = {"STOP", "FORWARD", "BACKWARD", "LEFT", "RIGHT",
        "FORWARD_LEFT", "FORWARD_RIGHT", "BACKWARD_LEFT", "BACKWARD_RIGHT"};
    int16_t logical[4] = {w->m1, w->m2, w->m3, w->m4};
    char text[80];
    unsigned i;
    (void)snprintf(text, sizeof(text), "[TEST] %s QUEUED (not feedback)\r\n", names[action]);
    Debug_Log(text);
    for (i = 0U; i < 4U; i++) {
        (void)snprintf(text, sizeof(text), "M%u logical=%d physical=%d RPM\r\n", i + 1U,
                       (int)Car_Motor_Limit(logical[i]), (int)Car_Motor_Physical((uint8_t)(i + 1U), logical[i]));
        Debug_Log(text);
    }
}
static void Stop(void)
{
    const MecanumWheels_t zero = {0,0,0,0};
    (void)Motor_EStopAll();
    mecanum_test_active = 0U;
    LogAction(MEC_STOP, &zero);
}
void Mecanum_Test_Init(void)
{
    mecanum_test_action = MEC_STOP;
    mecanum_test_request = seen_request = started = 0U;
    mecanum_test_active = 0U;
}
void Mecanum_Test_Process(uint8_t allowed)
{
    uint32_t request = mecanum_test_request;
    if (!CAR_MECANUM_TEST_MODE) return;
    if (!allowed) {
        /* Car control already queues stop/disable on a safety transition.
         * Do not cancel its DISABLE frames by enqueueing another stop. */
        mecanum_test_active = 0U;
        seen_request = request;
        return;
    }
    if (mecanum_test_active && (uint32_t)(HAL_GetTick() - started) >= CAR_MECANUM_TEST_DURATION_MS) Stop();
    if (request != seen_request) {
        int16_t vx, vy, omega;
        MecanumWheels_t wheels;
        uint32_t action = mecanum_test_action;
        seen_request = request;
        if (action >= MEC_ACTION_COUNT) { Stop(); Debug_Log("[TEST] invalid action\r\n"); return; }
        if (action == MEC_STOP) { Stop(); return; }
        if (mecanum_test_active || !Motor_IsIdle()) { Debug_Log("[TEST] BUSY; request again after STOP\r\n"); return; }
        Mecanum_Test_Vector((MecanumTestAction_t)action, &vx, &vy, &omega);
        Mecanum_Calculate(vx, vy, omega, &wheels);
        if (mecanum_drive(vx, vy, omega) == HAL_OK) {
            started = HAL_GetTick(); mecanum_test_active = 1U;
            LogAction((MecanumTestAction_t)action, &wheels);
        } else Debug_Log("[TEST] enqueue failed\r\n");
    }
}
