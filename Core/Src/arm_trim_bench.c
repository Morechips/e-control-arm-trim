#include "arm_trim_bench.h"
#include "arm_trim_project_config.h"
#include "arm_control.h"
#include "arm_trim_bluetooth.h"
#include "bluetooth_driver.h"
#include "car_control.h"
#include "motor_driver.h"
#include "pid_tuner.h"
#include "serial_io.h"
#include "heading_config.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>

static bool active, direct_grip, gripper_fault;
static int pending_profile = -1;
static uint32_t chassis_seq, arm_seq, gripper_tick;
_Static_assert(ARM_TRIM_BENCH_CLOSE_P >= 500U && ARM_TRIM_BENCH_CLOSE_P <= 2500U,
               "Configured close P must be a valid controller value");
_Static_assert(ARM_TRIM_BENCH_OPEN_P >= 500U && ARM_TRIM_BENCH_OPEN_P <= 2500U,
               "Configured open P must be a valid controller value");
/* A reference pose never changes the gripper, including when switching scene
 * with an object already held. Only the explicit grip commands address 003. */
static const struct { const char *name; uint16_t p[3]; } references[] = {
    {"BALL", {ARM_TRIM_BALL_P0, ARM_TRIM_BALL_P1, ARM_TRIM_BALL_P2}},
    {"HOSTAGE", {ARM_TRIM_HOSTAGE_P0, ARM_TRIM_HOSTAGE_P1, ARM_TRIM_HOSTAGE_P2}},
    {"BUCKET", {ARM_TRIM_BUCKET_P0, ARM_TRIM_BUCKET_P1, ARM_TRIM_BUCKET_P2}}
};
static void Reply(const char *text)
{
    (void)PID_Tuner_QueueReply(text);
    Debug_Log(text);
}
bool ArmTrimBench_IsActive(void) { return active; }
void ArmTrimBench_Cancel(void)
{
    pending_profile = -1; /* Cancelled PREP must never auto-synchronize. */
    if (!active) return;
    if (direct_grip) {
        gripper_fault = ZLIS2_StopServo(3U) != ZLIS2_OK;
        active = direct_grip = false;
        ArmTrimBluetooth_Cancel();
        Reply(gripper_fault ? "BENCH GRIP FAULT; CHECK_HARDWARE_THEN_CLEAR\r\n" :
                             "BENCH GRIP CANCELLED\r\n");
    } else (void)Arm_Stop();
}

static bool ChassisStationary(void)
{
    CarState_t state = Car_Control_GetState();
    /* The independent arm page deliberately sends no chassis keepalive.
     * Waiting for its first control frame, OFF and completed stop locks are
     * stationary too. Still require an idle, fault-free motor transport. */
    return (state == CAR_READY || state == CAR_OFF || state == CAR_WAIT_CENTER ||
            state == CAR_BRAKE_LOCK || state == CAR_LINK_LOST) &&
           Motor_IsIdle() && !Motor_HasFault();
}

static bool StartGripper(uint16_t p, bool legacy_session)
{
    ArmStatus_t arm = Arm_GetStatus();
    ArmTrimStatus_t trim = ArmTrimBluetooth_GetStatus();
    if (!ArmTrimBluetooth_OwnsMotion()) {
        Reply("BENCH ERR FIXED_ACTION_REQUIRED\r\n"); return true;
    }
    if (active || legacy_session || gripper_fault || arm.motion_allowed ||
        arm.state == ARM_RUNNING || arm.state == ARM_STOPPING || arm.state == ARM_FAULT ||
        (Arm_ExternalMotionOwned() && !ArmTrimBluetooth_OwnsMotion())) {
        Reply("BENCH ERR BUSY_OR_FAULT\r\n"); return true;
    }
    if (ArmTrimBluetooth_OwnsMotion() &&
        (trim.state == ARM_TRIM_MOVING || trim.state == ARM_TRIM_SETTLING ||
         trim.state == ARM_TRIM_STOPPING || trim.state == ARM_TRIM_FAULT)) {
        Reply("BENCH ERR TRIM_BUSY_OR_FAULT\r\n"); return true;
    }
    if (CAR_PD10_STANDALONE_TEST || CAR_MECANUM_TEST_MODE || CAR_HEADING_TEST_MODE ||
        !ChassisStationary()) {
        Reply("BENCH ERR PARKED_IDLE_REQUIRED\r\n"); return true;
    }
    /* The portable core owns 000..002. This outer helper exclusively addresses
     * 003 and reserves the dispatcher until its bounded motion completes. */
    if (ZLIS2_SetServo(3U, p, 1500U) != ZLIS2_OK) {
        gripper_fault = true;
        (void)ZLIS2_StopServo(3U);
        ArmTrimBluetooth_Cancel();
        Reply("BENCH GRIP FAULT; CHECK_HARDWARE_THEN_CLEAR\r\n"); return true;
    }
    active = direct_grip = true;
    pending_profile = -1;
    gripper_tick = HAL_GetTick();
    chassis_seq = Bluetooth_GetSequence();
    arm_seq = Bluetooth_GetArmSequence();
    Reply("BENCH GRIP ACCEPTED_NOT_ARRIVED; ONLY_003\r\n");
    return true;
}

bool ArmTrimBench_HandleLine(const char *line, uint32_t arrival, bool legacy_session)
{
    ArmStep_t step = {0};
    ArmResult_t result;
    bool auto_reference = false;
    int selected_profile = -1;
    char reply[100];
    if (line == NULL) { ArmTrimBench_Cancel(); return active; }
    if (strncmp(line, "@BENCH ", 7U) != 0) return false;
    if ((uint32_t)(HAL_GetTick() - arrival) > BT_FAILSAFE_TIMEOUT_MS) return true;
    if (strcmp(line, "@BENCH STOP") == 0) {
        ArmTrimBench_Cancel(); Reply("BENCH STOP REQUESTED\r\n"); return true;
    }
    if (strcmp(line, "@BENCH BUTTON_CONFLICT") == 0) {
        Reply("BENCH ERR ONE_ACTION_BUTTON_AT_A_TIME\r\n"); return true;
    }
    if (strcmp(line, "@BENCH CLOSE") == 0)
        return StartGripper(ARM_TRIM_BENCH_CLOSE_P, legacy_session);
    if (strcmp(line, "@BENCH OPEN") == 0)
        return StartGripper(ARM_TRIM_BENCH_OPEN_P, legacy_session);
    if (strcmp(line, "@BENCH CLEAR") == 0) {
        ArmTrimState_t state = ArmTrimBluetooth_GetStatus().state;
        if (active || state == ARM_TRIM_MOVING || state == ARM_TRIM_SETTLING ||
            state == ARM_TRIM_STOPPING || Arm_GetStatus().state == ARM_STOPPING) {
            Reply("BENCH ERR STOP_FIRST\r\n"); return true;
        }
        gripper_fault = false;
        Reply("BENCH GRIP FAULT_CLEARED; REFERENCE_NOT_RESTORED\r\n"); return true;
    }
    if (strncmp(line, "@BENCH READY ", 13U) == 0) {
        unsigned i;
        for (i = 0U; i < sizeof(references)/sizeof(references[0]); ++i)
            if (strcmp(line + 13U, references[i].name) == 0) break;
        if (i == sizeof(references)/sizeof(references[0])) {
            Reply("BENCH ERR PROFILE\r\n"); return true;
        }
        if (active || legacy_session || gripper_fault) {
            Reply("BENCH ERR BUSY_OR_FAULT\r\n"); return true;
        }
        if (ArmTrimBluetooth_OwnsMotion() && ArmTrimBluetooth_End() != ARM_TRIM_OK) {
            Reply("BENCH ERR TRIM_BUSY_OR_FAULT\r\n"); return true;
        }
        auto_reference = true;
        selected_profile = (int)i;
    }
    if (active || Arm_ExternalMotionOwned() || legacy_session) {
        Reply("BENCH ERR BUSY_OR_SESSION; END_TRIM_OR_EXIT_LEGACY_FIRST\r\n"); return true;
    }
    if (Arm_GetStatus().motion_allowed || Arm_GetStatus().state == ARM_RUNNING ||
        Arm_GetStatus().state == ARM_STOPPING) {
        Reply("BENCH ERR LEGACY_ARM_BUSY_OR_AUTHORIZED\r\n"); return true;
    }
    if (CAR_PD10_STANDALONE_TEST || CAR_MECANUM_TEST_MODE || CAR_HEADING_TEST_MODE) {
        Reply("BENCH ERR DIAGNOSTIC_IMAGE\r\n"); return true;
    }
    if (auto_reference) {
        step.joint_mask = 0x07U;
        memcpy(step.position, references[selected_profile].p, sizeof(references[0].p));
        step.move_ms = 2000U;
    } else if (strncmp(line, "@BENCH PREP ", 12U) == 0) {
        unsigned i;
        for (i = 0U; i < sizeof(references)/sizeof(references[0]); ++i)
            if (strcmp(line + 12U, references[i].name) == 0) break;
        if (i == sizeof(references)/sizeof(references[0])) { Reply("BENCH ERR PROFILE\r\n"); return true; }
        step.joint_mask = 0x07U;
        memcpy(step.position, references[i].p, sizeof(references[i].p));
        step.move_ms = 2000U;
    } else if (strncmp(line, "@BENCH GRIP ", 12U) == 0) {
        char *end;
        long p;
        errno = 0;
        p = strtol(line + 12U, &end, 10);
        if (end == line + 12U || *end != '\0' || errno == ERANGE || p < 500L || p > 2500L) {
            Reply("BENCH ERR GRIP_P_500_TO_2500\r\n"); return true;
        }
        step.joint_mask = 0x08U;
        step.position[3] = (uint16_t)p;
        step.move_ms = 1500U;
    } else { Reply("BENCH ERR SYNTAX\r\n"); return true; }
    step.hold_ms = 300U;
    /* Legacy GRIP accepts the controller's full gripper range. The masked
     * planar-only references still obey configured joint travel. */
    result = Arm_SetMotionAllowed(true);
    if (result == ARM_OK) {
        result = Arm_StartOriginalPreset(&step);
        if (result != ARM_OK) (void)Arm_SetMotionAllowed(false);
    }
    if (result == ARM_OK) {
        active = true;
        direct_grip = false;
        pending_profile = auto_reference ? selected_profile : -1;
        chassis_seq = Bluetooth_GetSequence();
        arm_seq = Bluetooth_GetArmSequence();
    }
    (void)snprintf(reply, sizeof(reply), "BENCH RESULT=%u MASK=%u %s USER_PARKED_REQUIRED\r\n",
                   (unsigned)result, (unsigned)step.joint_mask,
                   result == ARM_OK ? "ACCEPTED_NOT_ARRIVED" : "REJECTED");
    Reply(reply);
    return true;
}

void ArmTrimBench_Process(void)
{
    ArmStatus_t s;
    if (!active) return;
    if (chassis_seq != Bluetooth_GetSequence()) {
        const BluetoothControlFrame *c = &Bluetooth_GetControl()->frame;
        chassis_seq = Bluetooth_GetSequence();
        if (c->stop || c->brake || c->disable) ArmTrimBench_Cancel();
    }
    if (arm_seq != Bluetooth_GetArmSequence()) {
        const BtArmControl_t *c = Bluetooth_GetArmControl();
        arm_seq = Bluetooth_GetArmSequence();
        if (c->brake || c->disable) ArmTrimBench_Cancel();
    }
    if (!active) return;
    if (direct_grip) {
        if ((uint32_t)(HAL_GetTick() - gripper_tick) >= 1800U) {
            active = direct_grip = false;
            Reply("BENCH GRIP COMPLETE_ESTIMATED; PLANAR_JOINTS_UNCHANGED\r\n");
        }
        return;
    }
    s = Arm_GetStatus();
    if (s.state == ARM_COMPLETE_ESTIMATED) {
        (void)Arm_SetMotionAllowed(false);
        active = false;
        if (pending_profile >= 0) {
            int completed = pending_profile;
            pending_profile = -1;
            if (ArmTrimBluetooth_BeginProfile(references[completed].name, false) == ARM_TRIM_OK)
                Reply("BENCH READY_ESTIMATED; TRIM_AUTO_INITIALIZED\r\n");
            else Reply("BENCH ERR AUTO_REFERENCE_FAILED; TRIM_NOT_READY\r\n");
        } else Reply("BENCH COMPLETE_ESTIMATED; READY_FOR_NEXT_COMMAND\r\n");
    } else if (s.state == ARM_CANCELLED || s.state == ARM_FAULT) {
        active = false;
        pending_profile = -1;
        Reply(s.state == ARM_FAULT ? "BENCH FAULT; CHECK_HARDWARE\r\n" : "BENCH CANCELLED\r\n");
    }
}
