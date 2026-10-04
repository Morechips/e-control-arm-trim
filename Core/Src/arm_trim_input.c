#include "arm_trim_input.h"
#include "arm_trim_input_config.h"
#include "arm_trim_service.h"
#include "arm_trim_project.h"
#include "arm_trim_project_config.h"
#include "bluetooth_driver.h"
#include "car_control.h"
#include "motor_driver.h"
#include "pid_tuner.h"
#include "serial_io.h"
#include <errno.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static bool configured, packet_pending, line_pending, jog_down, jog_active;
static volatile bool invalidated;
static uint8_t packet_buttons, previous_buttons;
static int16_t packet_direction;
static ServoCode packet_preset;
static uint16_t packet_gap;
static unsigned packet_requests;
static int jog_direction;
static uint32_t packet_tick, line_tick, jog_tick;
static char pending_line[80];
static ArmTrimServiceState_t reported_state;
static const char *const profile_names[] = {"BALL", "HOSTAGE", "BUCKET", "NONE"};
static const uint16_t references[ARM_TRIM_PROFILE_COUNT][3] = {
    {ARM_TRIM_BALL_P0, ARM_TRIM_BALL_P1, ARM_TRIM_BALL_P2},
    {ARM_TRIM_HOSTAGE_P0, ARM_TRIM_HOSTAGE_P1, ARM_TRIM_HOSTAGE_P2},
    {ARM_TRIM_BUCKET_P0, ARM_TRIM_BUCKET_P1, ARM_TRIM_BUCKET_P2}
};

static uint32_t Now(void *user) { (void)user; return HAL_GetTick(); }

static void Reply(const char *format, ...)
{
    char line[PID_TX_LINE_SIZE];
    va_list args;
    int length;
    va_start(args, format);
    length = vsnprintf(line, sizeof(line), format, args);
    va_end(args);
    if (length > 0 && (size_t)length < sizeof(line)) {
        (void)PID_Tuner_QueueReply(line);
        Debug_Log(line);
    }
}

static bool Parked(void)
{
    CarState_t state = Car_Control_GetState();
    return (state == CAR_READY || state == CAR_OFF || state == CAR_WAIT_CENTER ||
            state == CAR_BRAKE_LOCK || state == CAR_LINK_LOST) &&
           Motor_IsIdle() && !Motor_HasFault();
}

static bool CanStart(void)
{
    if (!configured || CAR_PD10_STANDALONE_TEST || CAR_MECANUM_TEST_MODE ||
        CAR_HEADING_TEST_MODE || !Parked()) {
        Reply("TRIM ERR PARKED_IDLE_REQUIRED\r\n");
        return false;
    }
    return true;
}

static void Cancel(void)
{
    jog_active = false;
    if (ArmTrimService_OwnsMotion()) (void)ArmTrimService_Cancel();
}

static void Release(void)
{
    if (jog_active) (void)ArmTrimService_ReleaseJog();
    jog_active = jog_down = false;
}

static void Jog(int direction, uint32_t arrival, bool press)
{
    if (!press) {
        if (jog_down && jog_active) {
            jog_tick = arrival;
            if (direction != jog_direction) {
                (void)ArmTrimService_ReleaseJog();
                jog_active = false;
                Reply("TRIM ERR DIRECTION_CHANGED_RELEASE_THEN_PRESS\r\n");
            }
        }
        return;
    }
    if (jog_down) return;
    jog_down = true;
    if (CanStart()) {
        ArmTrimResult_t result = ArmTrimService_StartJog(direction);
        if (result == ARM_TRIM_OK) {
            jog_active = true;
            jog_direction = direction;
            jog_tick = arrival;
        }
        Reply("TRIM JOG RESULT=%u DIR=%d\r\n", (unsigned)result, direction);
    }
}

static ArmTrimServiceProfile_t Profile(const char *name)
{
    unsigned i;
    for (i = 0U; i < ARM_TRIM_PROFILE_COUNT; ++i)
        if (strcmp(name, profile_names[i]) == 0) return (ArmTrimServiceProfile_t)i;
    return ARM_TRIM_PROFILE_NONE;
}

static void Show(void)
{
    ArmTrimServiceStatus_t s = ArmTrimService_GetStatus();
    unsigned profile = (unsigned)s.profile;
    if (profile > ARM_TRIM_PROFILE_COUNT) profile = ARM_TRIM_PROFILE_COUNT;
    Reply("TRIM v4.7 PROFILE=%s OWNER=%u STATE=%u REF=%u ERR=%u REQUEST_ERR=%u STOP_FAIL=%u PENDING=%u\r\n",
          profile_names[profile], (unsigned)s.owns_motion, (unsigned)s.state,
          (unsigned)s.core.reference_valid, (unsigned)s.core.error,
          (unsigned)s.last_request, (unsigned)s.stop_failed, (unsigned)s.pose_pending);
    Reply("TRIM MODEL_MM=%d,%d ENABLED_MM=%d,%d OFFSET_X10=%d TARGET_X10=%d\r\n",
          (int)s.core.model_min_mm, (int)s.core.model_max_mm,
          (int)s.core.enabled_min_mm, (int)s.core.enabled_max_mm,
          (int)(s.core.offset_mm * 10.0f), (int)(s.core.target_offset_mm * 10.0f));
    Reply("TRIM EST_P=%u,%u,%u SEG=%u/%u TX=%u RESULT=%u ESTIMATE_ONLY\r\n",
          s.core.estimated_position[0], s.core.estimated_position[1],
          s.core.estimated_position[2], (unsigned)s.core.segment_index,
          (unsigned)s.core.segment_count, (unsigned)s.transfer.state,
          (unsigned)s.transport_result);
}

static void Ready(ArmTrimServiceProfile_t profile, bool synchronize)
{
    if (profile == ARM_TRIM_PROFILE_NONE) { Reply("TRIM ERR PROFILE\r\n"); return; }
    if (CanStart()) {
        ArmTrimResult_t result = ArmTrimService_ReadyProfile(profile, synchronize);
        if (result == ARM_TRIM_OK) jog_active = false;
        Reply("TRIM READY RESULT=%u PROFILE=%s ONLY_000_001_002\r\n",
              (unsigned)result, profile_names[profile]);
    }
}

static void Preset(ServoCode preset)
{
    if (CanStart()) {
        ArmTrimResult_t result = ArmTrimService_RunPreset(preset);
        if (result == ARM_TRIM_OK) jog_active = false;
        Reply("TRIM POSE RESULT=%u CODE=%u ONLY_000_001_002\r\n",
              (unsigned)result, (unsigned)preset);
    }
}

static void Grip(uint16_t pwm)
{
    if (CanStart())
        Reply("TRIM GRIP RESULT=%u P=%u ONLY_003\r\n",
              (unsigned)ArmTrimService_Grip(pwm), pwm);
}

void ArmTrimInput_Init(void)
{
    ArmTrimServiceConfig_t config;
    memset(&config, 0, sizeof(config));
    ArmTrimProject_DefaultConfig(&config.core);
    memcpy(config.references, references, sizeof(references));
    config.profile_move_ms = ARM_TRIM_PROFILE_MOVE_MS;
    config.profile_guard_ms = ARM_TRIM_PROFILE_GUARD_MS;
    config.grip_move_ms = ARM_TRIM_GRIP_MOVE_MS;
    config.grip_guard_ms = ARM_TRIM_GRIP_GUARD_MS;
    config.grip_min_pwm = ARM_TRIM_GRIPPER_MIN_P;
    config.grip_max_pwm = ARM_TRIM_GRIPPER_MAX_P;
    configured = ArmTrimService_Init(&config, Now, NULL) == ARM_TRIM_OK;
    packet_pending = line_pending = invalidated = jog_down = jog_active = false;
    previous_buttons = 0U;
    packet_preset = ServoCode_NONE;
    packet_gap = 0U;
    packet_requests = 0U;
    reported_state = ARM_TRIM_SERVICE_IDLE;
    Bluetooth_SetTextHandler(ArmTrimInput_HandleLine);
    Debug_Log("[TRIM] v4.7 unified pose/grip/trim service; no startup arm motion\r\n");
}

void ArmTrimInput_Submit(uint8_t buttons, int16_t direction, uint32_t arrival_tick)
{
    ArmTrimInput_SubmitCombined(buttons, direction, ServoCode_NONE, 0U, 0U, arrival_tick);
}

void ArmTrimInput_SubmitCombined(uint8_t buttons, int16_t direction,
                                 ServoCode preset, uint16_t gap_pwm,
                                 unsigned requests, uint32_t arrival_tick)
{
    packet_buttons = buttons;
    packet_direction = direction;
    packet_preset = preset;
    packet_gap = gap_pwm;
    packet_requests = requests;
    packet_tick = arrival_tick;
    packet_pending = true;
}

void ArmTrimInput_Invalidate(void) { invalidated = true; }

void ArmTrimInput_HandleLine(const char *line, uint32_t arrival_tick)
{
    if (line == NULL) { ArmTrimInput_Invalidate(); return; }
    if ((strncmp(line, "@ARM TRIM", 9U) != 0 ||
         (line[9] != '\0' && line[9] != ' ')) && strncmp(line, "@BENCH ", 7U) != 0)
        return;
    if (strlen(line) >= sizeof(pending_line)) return;
    strcpy(pending_line, line);
    line_tick = arrival_tick;
    line_pending = true;
}

static void ProcessPacket(void)
{
    uint8_t buttons = packet_buttons, rising = (uint8_t)(buttons & (uint8_t)~previous_buttons);
    uint8_t action = (uint8_t)(buttons & (uint8_t)~ARM_TRIM_BUTTON_STATUS);
    uint8_t fixed = (uint8_t)(action & (ARM_TRIM_BUTTON_BALL | ARM_TRIM_BUTTON_HOSTAGE |
                                       ARM_TRIM_BUTTON_BUCKET));
    bool switch_pose = jog_active && (fixed != 0U ||
        (packet_requests == 1U && packet_preset != ServoCode_NONE));
    unsigned requests = packet_requests;
    int direction = packet_direction > 0 ? 1 : packet_direction < 0 ? -1 : 0;
    previous_buttons = buttons;
    if ((buttons & ARM_TRIM_BUTTON_STOP) != 0U) {
        Cancel();
        Reply("TRIM STOP REQUESTED\r\n");
        return;
    }
    if ((buttons & ARM_TRIM_BUTTON_JOG) == 0U) Release();
    if ((rising & ARM_TRIM_BUTTON_STATUS) != 0U) Show();
    if (switch_pose) action = (uint8_t)(action & (uint8_t)~ARM_TRIM_BUTTON_JOG);
    for (uint8_t bits = action; bits != 0U; bits = (uint8_t)(bits & (uint8_t)(bits - 1U)))
        ++requests;
    if (requests > 1U) {
        if (jog_active) { (void)ArmTrimService_ReleaseJog(); jog_active = false; }
        /* Consume a rejected held press; it cannot retry after the conflict. */
        if ((buttons & ARM_TRIM_BUTTON_JOG) != 0U) jog_down = true;
        if (rising != 0U || packet_requests != 0U) Reply("TRIM ERR ONE_ACTION_BUTTON_AT_A_TIME\r\n");
        return;
    }
    if ((buttons & ARM_TRIM_BUTTON_JOG) != 0U &&
        (packet_direction < -150 || packet_direction > 150)) {
        if (jog_active) { (void)ArmTrimService_ReleaseJog(); jog_active = false; }
        if ((buttons & ARM_TRIM_BUTTON_JOG) != 0U) jog_down = true;
        if (rising != 0U) Reply("TRIM ERR YDNUM_RANGE\r\n");
        return;
    }
    if (packet_requests != 0U) {
        if (packet_preset != ServoCode_NONE) Preset(packet_preset);
        else if (packet_gap != 0U) Grip(packet_gap);
    } else if (switch_pose) {
        if ((rising & ARM_TRIM_BUTTON_BALL) != 0U) Ready(ARM_TRIM_PROFILE_BALL, true);
        else if ((rising & ARM_TRIM_BUTTON_HOSTAGE) != 0U) Ready(ARM_TRIM_PROFILE_HOSTAGE, true);
        else if ((rising & ARM_TRIM_BUTTON_BUCKET) != 0U) Ready(ARM_TRIM_PROFILE_BUCKET, true);
    } else if ((buttons & ARM_TRIM_BUTTON_JOG) != 0U) {
        Jog(direction, packet_tick, (rising & ARM_TRIM_BUTTON_JOG) != 0U);
    } else if ((rising & ARM_TRIM_BUTTON_BALL) != 0U) Ready(ARM_TRIM_PROFILE_BALL, true);
    else if ((rising & ARM_TRIM_BUTTON_HOSTAGE) != 0U) Ready(ARM_TRIM_PROFILE_HOSTAGE, true);
    else if ((rising & ARM_TRIM_BUTTON_BUCKET) != 0U) Ready(ARM_TRIM_PROFILE_BUCKET, true);
    else if ((rising & ARM_TRIM_BUTTON_CLOSE) != 0U) Grip(ARM_TRIM_BENCH_CLOSE_P);
    else if ((rising & ARM_TRIM_BUTTON_OPEN) != 0U) Grip(ARM_TRIM_BENCH_OPEN_P);
}

static void ProcessLine(void)
{
    char *words[4], *copy;
    unsigned count = 0U;
    bool bench = strncmp(pending_line, "@BENCH ", 7U) == 0;
    copy = pending_line + (bench ? 7U : 9U);
    while (*copy != '\0') {
        while (*copy == ' ') ++copy;
        if (*copy == '\0') break;
        if (count == 4U) { Reply("TRIM ERR SYNTAX\r\n"); return; }
        words[count++] = copy;
        while (*copy != '\0' && *copy != ' ') ++copy;
        if (*copy != '\0') *copy++ = '\0';
    }
    if (count == 1U) {
        if (strcmp(words[0], "STATUS") == 0) Show();
        else if (strcmp(words[0], "STOP") == 0) { Cancel(); Reply("TRIM STOP REQUESTED\r\n"); }
        else if (strcmp(words[0], "RELEASE") == 0) Release();
        else if (strcmp(words[0], "END") == 0) {
            Release();
            Reply("TRIM END RESULT=%u\r\n", (unsigned)ArmTrimService_End());
        } else if (strcmp(words[0], "CLEAR") == 0)
            Reply("TRIM CLEAR RESULT=%u\r\n", (unsigned)ArmTrimService_ClearFault());
        else if (bench && strcmp(words[0], "CLOSE") == 0) Grip(ARM_TRIM_BENCH_CLOSE_P);
        else if (bench && strcmp(words[0], "OPEN") == 0) Grip(ARM_TRIM_BENCH_OPEN_P);
        else Reply("TRIM ERR SYNTAX\r\n");
    } else if (count == 2U) {
        if (bench && (strcmp(words[0], "READY") == 0 || strcmp(words[0], "PREP") == 0))
            Ready(Profile(words[1]), strcmp(words[0], "READY") == 0);
        else if (bench && strcmp(words[0], "POSE") == 0) {
            static const char *const names[] = {
                "TB_B", "TB_M", "TB_G", "BD_U", "BD_D", "TH_U", "TH_C",
                "TH_G", "TH_L", "REFERENCE", "RST", "AIM", "TH_PRE"
            };
            unsigned preset;
            for (preset = 0U; preset < (unsigned)ServoCode_MAX; ++preset)
                if (strcmp(words[1], names[preset]) == 0) break;
            if (preset == (unsigned)ServoCode_MAX) Reply("TRIM ERR POSE\r\n");
            else Preset((ServoCode)preset);
        }
        else if (!bench && strcmp(words[0], "BEGIN") == 0) {
            ArmTrimServiceProfile_t profile = Profile(words[1]);
            if (profile == ARM_TRIM_PROFILE_NONE) Reply("TRIM ERR PROFILE\r\n");
            else if (CanStart())
                Reply("TRIM BEGIN RESULT=%u NO_MOTION_OPERATOR_ASSERTED_POSE\r\n",
                      (unsigned)ArmTrimService_Begin(references[profile], true));
        } else if (!bench && (strcmp(words[0], "JOG") == 0 || strcmp(words[0], "KEEP") == 0)) {
            int direction;
            if (strcmp(words[1], "1") == 0) direction = 1;
            else if (strcmp(words[1], "-1") == 0) direction = -1;
            else if (strcmp(words[1], "0") == 0) direction = 0;
            else { Reply("TRIM ERR JOG_SIGN_ONLY\r\n"); return; }
            Jog(direction, line_tick, strcmp(words[0], "JOG") == 0);
        } else if (!bench && (strcmp(words[0], "X") == 0 || strcmp(words[0], "DX") == 0)) {
            char *end;
            float distance;
            errno = 0;
            distance = strtof(words[1], &end);
            if (end == words[1] || *end != '\0' || errno == ERANGE || !isfinite(distance))
                Reply("TRIM ERR X_NUMBER\r\n");
            else if (CanStart())
                Reply("TRIM X RESULT=%u\r\n", (unsigned)ArmTrimService_MoveRelativeX(distance));
        } else if (bench && strcmp(words[0], "GRIP") == 0) {
            char *end;
            long pwm;
            errno = 0;
            pwm = strtol(words[1], &end, 10);
            if (end == words[1] || *end != '\0' || errno == ERANGE || pwm < 500 || pwm > 2500)
                Reply("TRIM ERR GRIP_RANGE\r\n");
            else Grip((uint16_t)pwm);
        } else Reply("TRIM ERR SYNTAX\r\n");
    } else Reply("TRIM ERR SYNTAX\r\n");
}

static void RememberRejectedPress(void)
{
    if (packet_pending) {
        previous_buttons = packet_buttons;
        if ((packet_buttons & ARM_TRIM_BUTTON_JOG) != 0U) jog_down = true;
    }
    if (line_pending && strncmp(pending_line, "@ARM TRIM JOG ", 14U) == 0)
        jog_down = true;
}

void ArmTrimInput_Process(void)
{
    ArmTrimServiceStatus_t status;
    uint32_t now = HAL_GetTick();
    uint32_t mask = __get_PRIMASK();
    bool broken;
    __disable_irq();
    broken = invalidated;
    invalidated = false;
    __set_PRIMASK(mask);
    if (broken) {
        RememberRejectedPress();
        packet_pending = line_pending = false;
        Cancel();
        Reply("TRIM LINK_INVALIDATED REFERENCE_REQUIRED\r\n");
    }
    if (ArmTrimService_IsBusy() && !Parked()) {
        Cancel();
        RememberRejectedPress();
        packet_pending = line_pending = false;
    }
    if (jog_active && (uint32_t)(now - jog_tick) > ARM_TRIM_INPUT_LEASE_MS) {
        Cancel();
        Reply("TRIM JOG LINK_TIMEOUT RELEASE_THEN_PRESS\r\n");
    }
    if (packet_pending) {
        if ((uint32_t)(now - packet_tick) <= ARM_TRIM_INPUT_LEASE_MS) ProcessPacket();
        else {
            /* Remember rejected levels so a fresh repeat of the same held
             * press cannot become a new action after the stale request. */
            RememberRejectedPress();
        }
        packet_pending = false;
    }
    if (line_pending) {
        if ((uint32_t)(now - line_tick) <= ARM_TRIM_INPUT_LEASE_MS) ProcessLine();
        else RememberRejectedPress();
        line_pending = false;
    }
    ArmTrimService_Process();
    status = ArmTrimService_GetStatus();
    if (jog_active && !status.core.jogging) jog_active = false;
    if (status.state != reported_state) {
        reported_state = status.state;
        Reply("TRIM STATE=%u CORE=%u REF=%u ERR=%u ESTIMATE_ONLY\r\n",
              (unsigned)status.state, (unsigned)status.core.state,
              (unsigned)status.core.reference_valid, (unsigned)status.core.error);
    }
}
