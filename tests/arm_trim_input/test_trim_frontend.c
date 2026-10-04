#include "arm_trim_input.h"
#include "arm_trim_input_config.h"
#include "arm_trim_service.h"
#include "arm_trim_project.h"
#include "bluetooth_driver.h"
#include "car_control.h"
#include "motor_driver.h"
#include "pid_tuner.h"
#include "serial_io.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t now;
static CarState_t car_state;
static uint8_t motor_idle, motor_fault;
static ArmTrimServiceStatus_t service;
static unsigned checks, starts, accepted_jogs, releases, cancels, readies, grips, moves, begins, ends, clears, processes;
static int last_direction;
static uint16_t last_grip;
static float last_move;
static bool last_synchronize;
static ArmTrimServiceProfile_t last_profile;
static ServoCode last_preset;
static unsigned presets;
static ArmTrimResult_t start_result;
static BluetoothTextHandler_t handler;
static char replies[4096];
#define CHECK(c) do { ++checks; assert(c); } while (0)

uint32_t HAL_GetTick(void) { return now; }
CarState_t Car_Control_GetState(void) { return car_state; }
uint8_t Motor_IsIdle(void) { return motor_idle; }
uint8_t Motor_HasFault(void) { return motor_fault; }
void Bluetooth_SetTextHandler(BluetoothTextHandler_t callback) { handler = callback; }
void Debug_Log(const char *line) { (void)line; }
bool PID_Tuner_QueueReply(const char *line)
{ CHECK(strlen(replies) + strlen(line) < sizeof(replies)); strcat(replies, line); return true; }
void ArmTrimProject_DefaultConfig(ArmTrimConfig_t *config) { memset(config, 0, sizeof(*config)); }
ArmTrimResult_t ArmTrimService_Init(const ArmTrimServiceConfig_t *config,
                                 uint32_t (*clock)(void *), void *user)
{ CHECK(config->profile_move_ms != 0U && clock(user) == now); return ARM_TRIM_OK; }
ArmTrimResult_t ArmTrimService_ReadyProfile(ArmTrimServiceProfile_t profile, bool synchronize)
{ ++readies; last_profile = profile; last_synchronize = synchronize; return ARM_TRIM_OK; }
ArmTrimResult_t ArmTrimService_RunPreset(ServoCode preset)
{ ++presets; last_preset = preset; return ARM_TRIM_OK; }
ArmTrimResult_t ArmTrimService_Begin(const uint16_t positions[3], bool parked_stable)
{ CHECK(positions != NULL && parked_stable); ++begins; return ARM_TRIM_OK; }
ArmTrimResult_t ArmTrimService_MoveRelativeX(float distance)
{ ++moves; last_move = distance; return ARM_TRIM_OK; }
ArmTrimResult_t ArmTrimService_StartJog(int direction)
{
    ++starts; last_direction = direction;
    if (direction != 1 && direction != -1) return ARM_TRIM_INVALID;
    if (start_result == ARM_TRIM_OK) {
        ++accepted_jogs; service.owns_motion = service.busy = service.core.jogging = true;
        service.state = ARM_TRIM_SERVICE_MOTION;
    }
    return start_result;
}
ArmTrimResult_t ArmTrimService_ReleaseJog(void)
{ ++releases; service.core.jogging = false; return ARM_TRIM_OK; }
ArmTrimResult_t ArmTrimService_Cancel(void)
{ ++cancels; service.core.jogging = false; service.state = ARM_TRIM_SERVICE_STOPPING; return ARM_TRIM_OK; }
ArmTrimResult_t ArmTrimService_End(void) { ++ends; return ARM_TRIM_OK; }
ArmTrimResult_t ArmTrimService_ClearFault(void) { ++clears; return ARM_TRIM_OK; }
ArmTrimResult_t ArmTrimService_Grip(uint16_t pwm) { ++grips; last_grip = pwm; return ARM_TRIM_OK; }
void ArmTrimService_Process(void) { ++processes; }
ArmTrimServiceStatus_t ArmTrimService_GetStatus(void) { return service; }
bool ArmTrimService_IsBusy(void) { return service.busy; }
bool ArmTrimService_OwnsMotion(void) { return service.owns_motion; }

static void Reset(void)
{
    memset(&service, 0, sizeof(service)); replies[0] = '\0';
    starts = accepted_jogs = releases = cancels = readies = grips = moves = begins = ends = clears = processes = 0U;
    presets = 0U;
    car_state = CAR_READY; motor_idle = 1U; motor_fault = 0U; start_result = ARM_TRIM_OK;
    now = 1000U; ArmTrimInput_Init(); CHECK(handler == ArmTrimInput_HandleLine);
}
static void Packet(uint8_t buttons, int16_t direction)
{ ArmTrimInput_Submit(buttons, direction, now); ArmTrimInput_Process(); }
static void Line(const char *line)
{ ArmTrimInput_HandleLine(line, now); ArmTrimInput_Process(); }

int main(void)
{
    Reset(); ArmTrimInput_Submit(ARM_TRIM_BUTTON_BALL, 0, now);
    CHECK(readies == 0U); ArmTrimInput_Process();
    CHECK(readies == 1U && last_profile == ARM_TRIM_PROFILE_BALL && last_synchronize);
    Packet(ARM_TRIM_BUTTON_BALL, 0); CHECK(readies == 1U);
    Packet(0U, 0); Packet(ARM_TRIM_BUTTON_CLOSE, 0); CHECK(grips == 1U && last_grip == 500U);
    Packet(0U, 0); Packet(ARM_TRIM_BUTTON_OPEN, 0); CHECK(grips == 2U && last_grip == 1800U);

    Reset(); Packet(ARM_TRIM_BUTTON_JOG, 30); CHECK(accepted_jogs == 1U && last_direction == 1);
    Packet(ARM_TRIM_BUTTON_JOG, 2); CHECK(starts == 1U);
    Packet(ARM_TRIM_BUTTON_JOG, -1); CHECK(releases == 1U && starts == 1U);
    Packet(ARM_TRIM_BUTTON_JOG, -1); CHECK(starts == 1U);
    Packet(0U, -1); Packet(ARM_TRIM_BUTTON_JOG, -30); CHECK(accepted_jogs == 2U && last_direction == -1);
    Packet(0U, -30); CHECK(releases == 2U);

    Reset(); Packet(ARM_TRIM_BUTTON_BALL | ARM_TRIM_BUTTON_JOG, 1);
    CHECK(starts == 0U && readies == 0U && strstr(replies, "ONE_ACTION") != NULL);
    Packet(ARM_TRIM_BUTTON_JOG, 1); CHECK(starts == 0U);
    Packet(0U, 1); Packet(ARM_TRIM_BUTTON_JOG, 1); CHECK(accepted_jogs == 1U);
    Packet(ARM_TRIM_BUTTON_CLOSE | ARM_TRIM_BUTTON_JOG, 1); CHECK(releases == 1U && grips == 0U);
    Packet(ARM_TRIM_BUTTON_JOG, 1); CHECK(starts == 1U);

    Reset(); Packet(ARM_TRIM_BUTTON_JOG, 151); CHECK(starts == 0U && strstr(replies, "YDNUM_RANGE") != NULL);
    Packet(ARM_TRIM_BUTTON_JOG, 1); CHECK(starts == 0U);
    Packet(0U, 1); Packet(ARM_TRIM_BUTTON_JOG, 0); CHECK(accepted_jogs == 0U);
    Packet(ARM_TRIM_BUTTON_JOG, 1); CHECK(accepted_jogs == 0U);
    Packet(0U, 1); Packet(ARM_TRIM_BUTTON_JOG, -1); CHECK(accepted_jogs == 1U);
    Packet(ARM_TRIM_BUTTON_STOP | ARM_TRIM_BUTTON_BALL | ARM_TRIM_BUTTON_JOG, INT16_MIN);
    CHECK(cancels == 1U && readies == 0U && strstr(replies, "STOP REQUESTED") != NULL);

    Reset(); car_state = CAR_RUNNING; Packet(ARM_TRIM_BUTTON_JOG, 1); CHECK(starts == 0U);
    car_state = CAR_READY; Packet(ARM_TRIM_BUTTON_JOG, 1); CHECK(starts == 0U);
    Packet(0U, 1); motor_idle = 0U; Packet(ARM_TRIM_BUTTON_BALL, 0); CHECK(readies == 0U);
    motor_idle = 1U; Packet(0U, 0); motor_fault = 1U; Packet(ARM_TRIM_BUTTON_BUCKET, 0); CHECK(readies == 0U);
    motor_fault = 0U; Packet(0U, 0); Packet(ARM_TRIM_BUTTON_JOG, 1); CHECK(accepted_jogs == 1U);
    car_state = CAR_RUNNING; ArmTrimInput_Process(); CHECK(cancels == 1U);

    Reset(); now = UINT32_MAX - 100U; Packet(ARM_TRIM_BUTTON_JOG, 1);
    now += ARM_TRIM_INPUT_LEASE_MS; ArmTrimInput_Process(); CHECK(cancels == 0U);
    ++now; ArmTrimInput_Process(); CHECK(cancels == 1U);
    Packet(ARM_TRIM_BUTTON_JOG, 1); CHECK(accepted_jogs == 1U);
    Packet(0U, 1); Packet(ARM_TRIM_BUTTON_JOG, 1); CHECK(accepted_jogs == 2U);

    Reset(); Packet(ARM_TRIM_BUTTON_JOG, 1);
    now += 400U; Packet(ARM_TRIM_BUTTON_JOG, 1);
    now += ARM_TRIM_INPUT_LEASE_MS; ArmTrimInput_Process(); CHECK(cancels == 0U);
    ++now; ArmTrimInput_Process(); CHECK(cancels == 1U);
    Reset(); Packet(ARM_TRIM_BUTTON_JOG, 1); ArmTrimInput_Invalidate(); CHECK(cancels == 0U);
    ArmTrimInput_Process(); CHECK(cancels == 1U && strstr(replies, "LINK_INVALIDATED") != NULL);

    /* Rejected requests that arrive in the cancellation loop consume their
     * held levels. Restoring the link or parking cannot create a new press. */
    Reset(); service.owns_motion = service.busy = true;
    ArmTrimInput_Invalidate(); ArmTrimInput_Submit(ARM_TRIM_BUTTON_BALL, 0, now);
    CHECK(cancels == 0U && readies == 0U); ArmTrimInput_Process();
    CHECK(cancels == 1U && readies == 0U);
    Packet(ARM_TRIM_BUTTON_BALL, 0); CHECK(readies == 0U);
    Packet(0U, 0); Packet(ARM_TRIM_BUTTON_BALL, 0); CHECK(readies == 1U);
    Reset(); service.owns_motion = service.busy = true; car_state = CAR_RUNNING;
    ArmTrimInput_Submit(ARM_TRIM_BUTTON_JOG, 1, now); ArmTrimInput_Process();
    CHECK(cancels == 1U && starts == 0U);
    car_state = CAR_READY; Packet(ARM_TRIM_BUTTON_JOG, 1); CHECK(starts == 0U);
    Packet(0U, 1); Packet(ARM_TRIM_BUTTON_JOG, 1); CHECK(accepted_jogs == 1U);
    for (unsigned broken = 0U; broken < 2U; ++broken) {
        Reset(); service.owns_motion = service.busy = true;
        if (broken != 0U) ArmTrimInput_Invalidate(); else car_state = CAR_RUNNING;
        ArmTrimInput_HandleLine("@ARM TRIM JOG 1", now); ArmTrimInput_Process();
        CHECK(cancels == 1U && starts == 0U);
        car_state = CAR_READY; Line("@ARM TRIM JOG 1"); CHECK(starts == 0U);
        Line("@ARM TRIM RELEASE"); Line("@ARM TRIM JOG 1"); CHECK(accepted_jogs == 1U);
    }

    Reset(); start_result = ARM_TRIM_BUSY; Packet(ARM_TRIM_BUTTON_JOG, 1);
    CHECK(starts == 1U && accepted_jogs == 0U);
    start_result = ARM_TRIM_OK; Packet(ARM_TRIM_BUTTON_JOG, 1); CHECK(starts == 1U);
    Packet(0U, 1); Packet(ARM_TRIM_BUTTON_JOG, 1); CHECK(accepted_jogs == 1U);

    Reset(); Line("@BENCH PREP BUCKET"); CHECK(readies == 1U && !last_synchronize);
    Line("@BENCH READY HOSTAGE"); CHECK(readies == 2U && last_synchronize && last_profile == ARM_TRIM_PROFILE_HOSTAGE);
    Line("@ARM TRIM BEGIN BALL"); CHECK(begins == 1U);
    Line("@ARM TRIM X -2.5"); CHECK(moves == 1U && last_move == -2.5f);
    Line("@ARM TRIM JOG 1"); CHECK(accepted_jogs == 1U);
    Line("@ARM TRIM KEEP 1"); CHECK(starts == 1U);
    Line("@ARM TRIM RELEASE"); CHECK(releases == 1U);
    Line("@ARM TRIM END"); CHECK(ends == 1U);
    Line("@ARM TRIM CLEAR"); CHECK(clears == 1U);
    Line("@BENCH GRIP 500"); CHECK(grips == 1U && last_grip == 500U);
    {
        unsigned operations = starts + readies + moves + grips + begins + ends + clears;
        const char *bad[] = {"@ARM TRIMMER JOG 1", "@ARM TRIM X nan", "@ARM TRIM X inf", "@ARM TRIM X 2junk",
            "@ARM TRIM JOG 2", "@ARM TRIM BEGIN UNKNOWN", "@BENCH GRIP 499", "@BENCH READY BALL junk", "@ARM TRIM"};
        for (unsigned i = 0U; i < sizeof(bad) / sizeof(bad[0]); ++i) Line(bad[i]);
        CHECK(starts + readies + moves + grips + begins + ends + clears == operations);
    }
    Reset(); service.owns_motion = true; Line("@ARM TRIM STOP"); CHECK(cancels == 1U);
    Reset(); ArmTrimInput_HandleLine("@BENCH READY BALL", now); now += ARM_TRIM_INPUT_LEASE_MS + 1U;
    ArmTrimInput_Process(); CHECK(readies == 0U);

    /* A delayed held press is consumed as rejected; a fresh repeat cannot
     * become a new press until the operator releases it. */
    Reset(); ArmTrimInput_Submit(ARM_TRIM_BUTTON_JOG, 1, now);
    now += ARM_TRIM_INPUT_LEASE_MS + 1U; ArmTrimInput_Process(); CHECK(starts == 0U);
    Packet(ARM_TRIM_BUTTON_JOG, 1); CHECK(starts == 0U);
    Packet(0U, 1); Packet(ARM_TRIM_BUTTON_JOG, 1); CHECK(accepted_jogs == 1U);
    Reset(); ArmTrimInput_HandleLine("@ARM TRIM JOG 1", now);
    now += ARM_TRIM_INPUT_LEASE_MS + 1U; ArmTrimInput_Process(); CHECK(starts == 0U);
    Line("@ARM TRIM JOG 1"); CHECK(starts == 0U);
    Line("@ARM TRIM RELEASE"); Line("@ARM TRIM JOG 1"); CHECK(accepted_jogs == 1U);
    Reset(); ArmTrimInput_SubmitCombined(0U, -1, Servo_RST, 0U, 1U, now);
    ArmTrimInput_Process(); CHECK(presets == 1U && last_preset == Servo_RST);
    Reset(); Packet(ARM_TRIM_BUTTON_JOG, -1);
    ArmTrimInput_SubmitCombined(ARM_TRIM_BUTTON_JOG, -1, Servo_RST, 0U, 1U, now);
    ArmTrimInput_Process(); CHECK(presets == 1U);
    Packet(ARM_TRIM_BUTTON_JOG, -1); CHECK(starts == 1U);
    Reset(); ArmTrimInput_SubmitCombined(ARM_TRIM_BUTTON_CLOSE, 0, Servo_RST, 0U, 1U, now);
    ArmTrimInput_Process(); CHECK(presets == 0U && grips == 0U);
    Reset(); ArmTrimInput_SubmitCombined(ARM_TRIM_BUTTON_STOP, 0, Servo_RST, 0U, 1U, now);
    service.owns_motion = true; ArmTrimInput_Process(); CHECK(presets == 0U && cancels == 1U);
    Reset(); ArmTrimInput_SubmitCombined(0U, 0, ServoCode_NONE, 700U, 1U, now);
    ArmTrimInput_Process(); CHECK(grips == 1U && last_grip == 700U);
    Line("@BENCH POSE RST"); CHECK(presets == 1U);
    printf("PASS trim frontend: %u checks; late-loop dispatch, held presses, conflicts, stop, lease, parking and text\n", checks);
    return 0;
}
