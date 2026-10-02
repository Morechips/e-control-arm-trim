#include "arm_trim_bluetooth.h"
#include "arm_trim.h"
#include "arm_trim_project_config.h"
#include "arm_collision_config.h"
#include "arm_control.h"
#include "bluetooth_driver.h"
#include "pid_tuner.h"
#include "serial_io.h"
#include "heading_config.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <limits.h>
#include <stdarg.h>

static ArmTrim_t trim;
static bool initialized, owns_motion;
static unsigned profile;
static ArmTrimState_t reported_state;
static bool jog_button_down, jog_request_active;
static int jog_direction;
static uint32_t jog_rx_tick;
static volatile ArmTrimResult_t last_request_result;
static uint32_t chassis_sequence;
static const struct {
    const char *name;
    uint16_t position[3];
} profiles[] = {
    {"BALL", {ARM_TRIM_BALL_P0, ARM_TRIM_BALL_P1, ARM_TRIM_BALL_P2}},
    {"HOSTAGE", {ARM_TRIM_HOSTAGE_P0, ARM_TRIM_HOSTAGE_P1, ARM_TRIM_HOSTAGE_P2}},
    {"BUCKET", {ARM_TRIM_BUCKET_P0, ARM_TRIM_BUCKET_P1, ARM_TRIM_BUCKET_P2}}
};

static const char *StateName(ArmTrimState_t state)
{
    static const char *const names[] = {"IDLE", "READY", "MOVING", "SETTLING",
        "COMPLETE_ESTIMATED", "STOPPING", "CANCELLED", "FAULT"};
    return (unsigned)state < sizeof(names)/sizeof(names[0]) ? names[state] : "INVALID";
}

static void Reply(const char *format, ...)
{
    char line[PID_TX_LINE_SIZE];
    int length;
    va_list args;
    va_start(args, format);
    length = vsnprintf(line, sizeof(line), format, args);
    va_end(args);
    if (length > 0 && (size_t)length < sizeof(line)) {
        (void)PID_Tuner_QueueReply(line);
        Debug_Log(line);
    }
}

static bool Send(void *user, const uint16_t p[3], uint16_t ms)
{
    ZLIS2_ServoCommand commands[3];
    unsigned i;
    (void)user;
    if (!owns_motion || !Arm_ExternalMotionOwned()) return false;
    for (i = 0U; i < 3U; ++i) commands[i] = (ZLIS2_ServoCommand){(uint16_t)i, p[i], ms};
    return ZLIS2_SetServos(commands, 3U) == ZLIS2_OK;
}

static bool StopJoint(void *user, unsigned joint)
{
    (void)user;
    return joint < 3U && ZLIS2_StopServo((uint16_t)joint) == ZLIS2_OK;
}

static uint32_t Now(void *user) { (void)user; return HAL_GetTick(); }
static void ExternalStop(void *user) { (void)ArmTrim_Cancel((ArmTrim_t *)user); }

void ArmTrimBluetooth_Init(void)
{
    ArmTrimConfig_t c;
    const ArmTrimIO_t io = {Send, StopJoint, Now, NULL, &trim};
    if (initialized) return;
    ArmTrim_DefaultConfig(&c);
    ArmKinematics_ProjectGeometry(&c.geometry);
    ArmKinematics_ProjectCalibrations(c.calibration);
    ArmCollision_ProjectModel(&c.collision);
    c.calibration[0].min_position = ARM_TRIM_PROJECT_P0_MIN;
    c.calibration[0].max_position = ARM_TRIM_PROJECT_P0_MAX;
    c.calibration[1].min_position = ARM_TRIM_PROJECT_P1_MIN;
    c.calibration[1].max_position = ARM_TRIM_PROJECT_P1_MAX;
    c.calibration[2].min_position = ARM_TRIM_PROJECT_P2_MIN;
    c.calibration[2].max_position = ARM_TRIM_PROJECT_P2_MAX;
    c.enabled_min_mm = ARM_TRIM_PROJECT_MIN_MM;
    c.enabled_max_mm = ARM_TRIM_PROJECT_MAX_MM;
    initialized = ArmTrim_Init(&trim, &c, &io) == ARM_TRIM_OK;
    reported_state = ARM_TRIM_IDLE;
}

bool ArmTrimBluetooth_OwnsMotion(void) { return owns_motion; }
ArmTrimStatus_t ArmTrimBluetooth_GetStatus(void) { return ArmTrim_GetStatus(&trim); }
void ArmTrimBluetooth_Cancel(void)
{
    jog_button_down = jog_request_active = false;
    if (owns_motion) (void)ArmTrim_Cancel(&trim);
}

static void Show(void)
{
    ArmTrimStatus_t s = ArmTrim_GetStatus(&trim);
    /* Integer tenths avoids newlib-nano floating printf. Separate short lines. */
    Reply("TRIM PROFILE=%s OWNER=%u STATE=%s REF=%u ERR=%u REQUEST_ERR=%u STOP_FAIL=%u\r\n",
          profiles[profile].name, (unsigned)owns_motion, StateName(s.state),
          (unsigned)s.reference_valid, (unsigned)s.error, (unsigned)last_request_result, (unsigned)s.stop_failed);
    Reply("TRIM MODEL_MM=%d,%d ENABLED_MM=%d,%d OFFSET_X10=%d TARGET_X10=%d\r\n",
          (int)s.model_min_mm, (int)s.model_max_mm, (int)s.enabled_min_mm,
          (int)s.enabled_max_mm, (int)(s.offset_mm*10.0f), (int)(s.target_offset_mm*10.0f));
    Reply("TRIM LIMIT_P=%u..%u,%u..%u,%u..%u L2_X100=%u PERIOD_MS=%u JOG=%u DIR=%d ESTIMATE_ONLY\r\n",
          trim.config.calibration[0].min_position, trim.config.calibration[0].max_position,
          trim.config.calibration[1].min_position, trim.config.calibration[1].max_position,
          trim.config.calibration[2].min_position, trim.config.calibration[2].max_position,
          (unsigned)(trim.config.geometry.link_2_mm * 100.0f + 0.5f),
          trim.config.update_period_ms, (unsigned)s.jogging, (int)s.jog_direction);
    Reply("TRIM REAR_BOX_GUARD=%u ENVELOPE_ESTIMATED=%u CLEARANCE_MM=%u\r\n",
          (unsigned)trim.config.collision.enabled, (unsigned)ARM_COLLISION_ENVELOPE_ESTIMATED,
          (unsigned)trim.config.collision.clearance_mm);
    Reply("TRIM EST_P=%u,%u,%u TARGET_P=%u,%u,%u SEG=%u/%u NOT_FEEDBACK\r\n",
          s.estimated_position[0], s.estimated_position[1], s.estimated_position[2],
          s.target_position[0], s.target_position[1], s.target_position[2],
          (unsigned)s.segment_index, (unsigned)s.segment_count);
}

ArmTrimResult_t ArmTrimBluetooth_BeginProfile(const char *name, bool legacy_session)
{
    unsigned selected;
    ArmTrimResult_t result;
    if (!initialized) return ARM_TRIM_INVALID;
    if (CAR_PD10_STANDALONE_TEST || CAR_MECANUM_TEST_MODE || CAR_HEADING_TEST_MODE) {
        Reply("TRIM ERR DIAGNOSTIC_IMAGE\r\n"); return ARM_TRIM_INVALID;
    }
    if (owns_motion) { Reply("TRIM ERR END_REQUIRED\r\n"); return ARM_TRIM_BUSY; }
    if (legacy_session) { Reply("TRIM ERR LEGACY_EXIT_REQUIRED\r\n"); return ARM_TRIM_BUSY; }
    for (selected = 0U; selected < sizeof(profiles)/sizeof(profiles[0]); ++selected)
        if (name != NULL && strcmp(name, profiles[selected].name) == 0) break;
    if (selected == sizeof(profiles)/sizeof(profiles[0])) {
        Reply("TRIM ERR PROFILE\r\n"); return ARM_TRIM_INVALID;
    }
    if (Arm_AcquireExternalMotion(ExternalStop, &trim) != ARM_OK) {
        Reply("TRIM ERR LEGACY_BUSY_OR_FAULT\r\n"); return ARM_TRIM_BUSY;
    }
    result = ArmTrim_Synchronize(&trim, profiles[selected].position);
    last_request_result = result;
    profile = selected;
    if (result != ARM_TRIM_OK) {
        (void)Arm_ReleaseExternalMotion(&trim);
        Reply("TRIM BEGIN RESULT=%u NO_MOTION\r\n", (unsigned)result);
        return result;
    }
    owns_motion = true;
    jog_button_down = jog_request_active = false;
    last_request_result = ARM_TRIM_OK;
    chassis_sequence = Bluetooth_GetSequence();
    profile = selected;
    reported_state = ARM_TRIM_READY;
    Reply("TRIM BEGIN RESULT=0 PROFILE=%s NO_MOTION REFERENCE_ESTIMATED_PARKED\r\n",
          profiles[profile].name);
    return ARM_TRIM_OK;
}

ArmTrimResult_t ArmTrimBluetooth_End(void)
{
    ArmTrimResult_t result;
    if (!owns_motion) return ARM_TRIM_REFERENCE_REQUIRED;
    result = ArmTrim_Exit(&trim);
    if (result == ARM_TRIM_OK) {
        (void)Arm_ReleaseExternalMotion(&trim);
        owns_motion = false;
        jog_button_down = jog_request_active = false;
        reported_state = ARM_TRIM_IDLE;
    }
    return result;
}

bool ArmTrimBluetooth_HandleLine(const char *line, uint32_t arrival, bool legacy_session)
{
    char copy[64], *words[3];
    size_t count = 0U, i;
    ArmTrimResult_t result;
    if (line == NULL) { ArmTrimBluetooth_Cancel(); return owns_motion; }
    if (strncmp(line, "@ARM TRIM", 9U) != 0 || (line[9] != '\0' && line[9] != ' ')) return false;
    if ((uint32_t)(HAL_GetTick() - arrival) > BT_FAILSAFE_TIMEOUT_MS) return true;
    if (!initialized) { Reply("TRIM ERR NOT_CONFIGURED\r\n"); return true; }
    if (strlen(line + 9U) >= sizeof(copy)) { Reply("TRIM ERR SYNTAX\r\n"); return true; }
    strcpy(copy, line + 9U);
    for (i = 0U; copy[i] != '\0'; ) {
        while (copy[i] == ' ') ++i;
        if (copy[i] == '\0') break;
        if (count == 3U) { Reply("TRIM ERR SYNTAX\r\n"); return true; }
        words[count++] = &copy[i];
        while (copy[i] != '\0' && copy[i] != ' ') ++i;
        if (copy[i] != '\0') copy[i++] = '\0';
    }
    if (count == 1U && strcmp(words[0], "STATUS") == 0) { Show(); return true; }
    if (count == 1U && strcmp(words[0], "RELEASE") == 0) {
        if (jog_request_active) {
            (void)ArmTrim_ReleaseJog(&trim);
            Reply("TRIM JOG RELEASED DECELERATING; REFERENCE_ESTIMATED\r\n");
        }
        jog_button_down = jog_request_active = false;
        return true;
    }
    if (count == 2U && (strcmp(words[0], "JOG") == 0 || strcmp(words[0], "KEEP") == 0)) {
        bool start = strcmp(words[0], "JOG") == 0;
        int direction;
        if (strcmp(words[1], "1") == 0) direction = 1;
        else if (strcmp(words[1], "-1") == 0) direction = -1;
        else if (strcmp(words[1], "0") == 0) direction = 0;
        else { Reply("TRIM ERR JOG_SIGN_ONLY\r\n"); return true; }
        if (!start) {
            if (jog_button_down && jog_request_active) {
                jog_rx_tick = arrival;
                if (direction != jog_direction) {
                    (void)ArmTrim_ReleaseJog(&trim);
                    jog_request_active = false;
                    last_request_result = ARM_TRIM_INVALID;
                    Reply("TRIM JOG DIRECTION_CHANGED; RELEASE_THEN_PRESS_AGAIN\r\n");
                }
            }
            return true;
        }
        if (jog_button_down) return true;
        jog_button_down = true;
        if (!owns_motion) {
            last_request_result = ARM_TRIM_REFERENCE_REQUIRED;
            Reply("TRIM ERR BEGIN_REQUIRED\r\n"); return true;
        }
        last_request_result = result = ArmTrim_StartJog(&trim, direction);
        if (result == ARM_TRIM_OK) {
            jog_request_active = true;
            jog_direction = direction;
            jog_rx_tick = arrival;
        }
        Reply("TRIM JOG RESULT=%u DIR=%d %s\r\n", (unsigned)result, direction,
              result == ARM_TRIM_OK ? "HOLD_TO_MOVE; RELEASE_TO_DECELERATE" : "REJECTED_NO_MOTION; QUERY_STATUS");
        return true;
    }
    if (count == 2U && strcmp(words[0], "BEGIN") == 0) {
        (void)ArmTrimBluetooth_BeginProfile(words[1], legacy_session);
        return true;
    }
    if (!owns_motion) { Reply("TRIM ERR BEGIN_REQUIRED\r\n"); return true; }
    if (count == 1U && strcmp(words[0], "STOP") == 0) {
        ArmTrimBluetooth_Cancel(); Reply("TRIM STOP REQUESTED\r\n"); return true;
    }
    if (count == 1U && strcmp(words[0], "CLEAR") == 0) {
        result = ArmTrim_ClearFault(&trim);
        reported_state = ArmTrim_GetStatus(&trim).state;
        Reply("TRIM CLEAR RESULT=%u CHECK_HARDWARE_THEN_END_AND_BEGIN\r\n", (unsigned)result);
        return true;
    }
    if (count == 1U && strcmp(words[0], "END") == 0) {
        result = ArmTrimBluetooth_End();
        Reply("TRIM END RESULT=%u\r\n", (unsigned)result);
        return true;
    }
    if (count == 2U && strcmp(words[0], "DX") == 0) {
        char *end;
        long value;
        errno = 0;
        value = strtol(words[1], &end, 10);
        if (words[1] == end || *end != '\0' || errno == ERANGE || value < -150L || value > 150L) {
            Reply("TRIM ERR DX_INTEGER_MM\r\n"); return true;
        }
        result = ArmTrim_MoveRelativeX(&trim, (float)value);
        last_request_result = result;
        Reply("TRIM DX RESULT=%u DX=%ld %s\r\n", (unsigned)result, value,
              result == ARM_TRIM_OK ? (value == 0L ? "NO_MOTION" : "ACCEPTED_NOT_ARRIVED") : "REJECTED_NO_MOTION");
        return true;
    }
    Reply("TRIM ERR SYNTAX\r\n");
    return true;
}

void ArmTrimBluetooth_Process(void)
{
    ArmTrimStatus_t s;
    uint32_t sequence;
    if (!owns_motion) return;
    s = ArmTrim_GetStatus(&trim);
    if (jog_request_active && (s.state == ARM_TRIM_MOVING || s.state == ARM_TRIM_SETTLING) &&
        (uint32_t)(HAL_GetTick() - jog_rx_tick) > BT_FAILSAFE_TIMEOUT_MS) {
        last_request_result = ARM_TRIM_LINK_TIMEOUT;
        ArmTrimBluetooth_Cancel();
        Reply("TRIM JOG LINK_TIMEOUT STOPPED; FIXED_ACTION_REQUIRED\r\n");
    }
    sequence = Bluetooth_GetSequence();
    if (sequence != chassis_sequence) {
        const BtControl_t *control = Bluetooth_GetControl();
        chassis_sequence = sequence;
        if (control->valid && trim.status.reference_valid &&
            (control->frame.stop || control->frame.brake || control->frame.disable))
            ArmTrimBluetooth_Cancel();
    }
    ArmTrim_Process(&trim);
    s = ArmTrim_GetStatus(&trim);
    if (s.state != reported_state) {
        reported_state = s.state;
        if (s.state == ARM_TRIM_COMPLETE_ESTIMATED || s.state == ARM_TRIM_CANCELLED || s.state == ARM_TRIM_FAULT)
            Reply("TRIM STATE=%s ERR=%u OFFSET_X10=%d REF=%u STOP_FAIL=%u\r\n", StateName(s.state),
                  (unsigned)s.error, (int)(s.offset_mm * 10.0f), (unsigned)s.reference_valid,
                  (unsigned)s.stop_failed);
    }
}
