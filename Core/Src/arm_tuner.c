#include "arm_tuner.h"
#include "arm_config.h"
#include "arm_control.h"
#include "arm_kinematics.h"
#include "arm_trim_bluetooth.h"
#include "arm_trim_bench.h"
#include "bluetooth_driver.h"
#include "car_control.h"
#include "heading_config.h"
#include "motor_driver.h"
#include "pid_tuner.h"
#include "serial_io.h"
#include "servo_pose.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#define ARM_TUNER_PI 3.14159265358979323846f
#define ARM_TUNER_COORDINATE_LIMIT_MM 2000

static ArmConfig_t config;
static bool session;
static unsigned selected;
static uint16_t move_ms, step_size;
static uint16_t target[ARM_JOINT_COUNT];
static bool target_known[ARM_JOINT_COUNT];
static uint16_t slots[ARM_JOINT_COUNT][ARM_TUNER_SLOT_COUNT];
static bool slot_valid[ARM_JOINT_COUNT][ARM_TUNER_SLOT_COUNT];
static uint32_t heartbeat_tick, joystick_sequence;
static uint16_t cartesian_position[3];
static ArmPose2D_t cartesian_pose;
static bool cartesian_known;
static bool remote_enabled, remote_center_seen;
static bool dual_enabled, finite_setup, finite_pending;
static bool dual_start_waiting, dual_start_moving, dual_auto_arm_pending;
static bool dual_restart_pending;
static uint8_t dual_center_samples;
static uint32_t remote_step_tick, remote_rx_tick, dual_restart_tick;
static int16_t remote_arm_x, remote_arm_y;
/* The phone may repeat a held direction value.  Wrist/gripper directions are
 * discrete safety steps, so accept a value only once until it is released or
 * changed.  The two analogue joystick axes keep their existing bounded-repeat
 * behaviour. */
static int16_t remote_direction_latch;
static uint32_t remote_axis_tick;
static uint8_t next_preset;
static uint16_t remote_preset_buttons_latch;

static void Reply(const char *format, ...)
{
    char line[PID_TX_LINE_SIZE];
    int length;
    va_list args;
    va_start(args, format);
    length = vsnprintf(line, sizeof(line), format, args);
    va_end(args);
    if (length > 0 && (size_t)length < sizeof(line)) (void)PID_Tuner_QueueReply(line);
}

static bool Active(void)
{
    ArmState_t state = Arm_GetStatus().state;
    return state == ARM_RUNNING || state == ARM_STOPPING;
}

static bool WorkspaceReady(void)
{
    return session && (!dual_enabled || Bluetooth_IsExtended());
}

static void Stop(void)
{
    ArmTrimBench_Cancel();
    (void)Arm_SetMotionAllowed(false);
    if (Arm_GetStatus().state != ARM_UNCONFIGURED) (void)Arm_Stop();
    memset(target_known, 0, sizeof(target_known));
    cartesian_known = false;
    remote_center_seen = false;
    finite_pending = false;
    dual_start_waiting = false;
    dual_start_moving = false;
    dual_auto_arm_pending = false;
    dual_restart_pending = false;
    dual_restart_tick = 0U;
    dual_center_samples = 0U;
    remote_arm_x = remote_arm_y = 0;
    remote_direction_latch = BT_DIRECTION_NONE;
    remote_preset_buttons_latch = 0U;
}

static const char *StateName(ArmState_t state)
{
    static const char *const names[] = {
        "UNCONFIGURED", "IDLE", "RUNNING", "COMPLETE_ESTIMATED",
        "STOPPING", "CANCELLED", "FAULT"
    };
    return (unsigned)state < sizeof(names)/sizeof(names[0]) ? names[state] : "UNKNOWN";
}

static const char *JointName(unsigned joint)
{
    static const char *const names[ARM_JOINT_COUNT] = {
        "SHOULDER", "ELBOW", "WRIST", "GRIPPER", "AUX1"
    };
    return joint < ARM_JOINT_COUNT ? names[joint] : "UNKNOWN";
}

static void Show(void)
{
    ArmStatus_t s = Arm_GetStatus();
    const ArmJointConfig_t *j = &config.joints[selected];
    Reply("ARM MODE=%u J=%u NAME=%s STATE=%s ALLOW=%u ERR=%u TX=%u CHASSIS=DECOUPLED ID=%u MIN=%u MAX=%u TIME=%u STEP=%u LAST=%u KNOWN=%u RAM_ONLY\r\n",
          (unsigned)session, selected, JointName(selected), StateName(s.state), (unsigned)s.motion_allowed,
          (unsigned)s.error, (unsigned)s.transport_result,
          (unsigned)j->servo_id, (unsigned)j->min_position, (unsigned)j->max_position,
          (unsigned)move_ms, (unsigned)step_size, (unsigned)target[selected],
          (unsigned)target_known[selected]);
}

static void ShowTargets(void)
{
    unsigned known_mask = 0U;
    size_t index;
    for (index = 0U; index < ARM_JOINT_COUNT; ++index) {
        if (target_known[index]) known_mask |= 1U << index;
    }
    Reply("ARM TARGETS KNOWN=0x%02X P=%u,%u,%u,%u,%u "
          "COMMAND_ESTIMATES_NOT_FEEDBACK\r\n",
          known_mask, (unsigned)target[0], (unsigned)target[1],
          (unsigned)target[2], (unsigned)target[3], (unsigned)target[4]);
}

static void ShowLink(void)
{
    const BtArmControl_t *control = Bluetooth_GetArmControl();
    Reply("ARM LINK RX_LEN=%u ARM_LEN=%u SEQ=%lu ARM_SEQ=%lu INVALID=%lu "
          "RECOVER=%lu BASE=%u ARM=%u CMD=%d AX=%d AY=%d TEST_LEN=%u TEST_SEQ=%lu\r\n",
          (unsigned)Bluetooth_GetLastFrameLength(),
          (unsigned)Bluetooth_GetLastArmFrameLength(),
          (unsigned long)Bluetooth_GetSequence(),
          (unsigned long)Bluetooth_GetArmSequence(),
          (unsigned long)Bluetooth_GetInvalidFrameCount(),
          (unsigned long)bluetooth_rx_recoveries,
          (unsigned)Bluetooth_IsConnected(),
          (unsigned)Bluetooth_ArmIsConnected(),
          (int)control->direction, (int)control->arm_x,
          (int)control->arm_y,
          (unsigned)Bluetooth_GetLastTestFrameLength(),
          (unsigned long)Bluetooth_GetTestSequence());
}

static bool RemoteCentered(const BtArmControl_t *control)
{
    return (control->direction == 0 || control->direction == 1) &&
           control->x >= -ARM_TUNER_REMOTE_DEADZONE &&
           control->x <= ARM_TUNER_REMOTE_DEADZONE &&
           control->y >= -ARM_TUNER_REMOTE_DEADZONE &&
           control->y <= ARM_TUNER_REMOTE_DEADZONE &&
           (!dual_enabled ||
            (control->arm_x >= -ARM_TUNER_REMOTE_DEADZONE &&
             control->arm_x <= ARM_TUNER_REMOTE_DEADZONE &&
             control->arm_y >= -ARM_TUNER_REMOTE_DEADZONE &&
             control->arm_y <= ARM_TUNER_REMOTE_DEADZONE)) &&
           !control->brake && !control->disable;
}

static bool ChassisControlsNeutral(void)
{
    const BluetoothControlFrame *c = &Bluetooth_GetControl()->frame;
    return c->joy_x >= -JOY_DEADZONE && c->joy_x <= JOY_DEADZONE &&
           c->joy_y >= -JOY_DEADZONE && c->joy_y <= JOY_DEADZONE &&
           !c->forward && !c->backward && !c->stop && !c->strafe_left &&
           !c->strafe_right && !c->right_90 && !c->right_180 &&
           !c->left_90 && !c->Cam_T && !c->Shot && !c->aim &&
           !c->brake && !c->disable;
}

static bool CartesianConfigurationValid(void)
{
    unsigned index;
    for (index = 0U; index < 3U; ++index) {
        const ArmJointConfig_t *joint = &config.joints[index];
        if (!joint->enabled || !joint->calibrated ||
            joint->servo_id != index) return false;
    }
    return true;
}

/* Strict decimal tokens: no signs, overflow, partial conversion or trailing data. */
static bool Number(const char *text, unsigned maximum, unsigned *value)
{
    unsigned n = 0U;
    if (*text == '\0') return false;
    while (*text != '\0') {
        unsigned digit;
        if (*text < '0' || *text > '9') return false;
        digit = (unsigned)(*text++ - '0');
        if (digit > maximum || n > (maximum-digit)/10U) return false;
        n = n*10U+digit;
    }
    *value = n;
    return true;
}

static bool SignedNumber(const char *text, int minimum, int maximum,
                         int *value)
{
    bool negative = false;
    unsigned magnitude, limit;
    if (text == NULL || value == NULL || minimum > 0 || maximum < 0)
        return false;
    if (*text == '-') {
        negative = true;
        ++text;
    }
    limit = negative ? (unsigned)(-minimum) : (unsigned)maximum;
    if (!Number(text, limit, &magnitude)) return false;
    *value = negative ? -(int)magnitude : (int)magnitude;
    return *value >= minimum && *value <= maximum;
}

static void PreviewInverseKinematics(int x_mm, int ground_z_mm,
                                     int phi_deg)
{
    const ArmPose2D_t pose = {
        (float)x_mm,
        (float)ground_z_mm - ARM_BASE_AXIS_HEIGHT_MM,
        (float)phi_deg * ARM_TUNER_PI / 180.0f
    };
    uint16_t negative[3] = {0U, 0U, 0U};
    uint16_t positive[3] = {0U, 0U, 0U};
    ArmKinematicsResult_t negative_result;
    ArmKinematicsResult_t positive_result;

    negative_result = ArmKinematics_ProjectInverse(
        &pose, ARM_ELBOW_NEGATIVE, negative);
    positive_result = ArmKinematics_ProjectInverse(
        &pose, ARM_ELBOW_POSITIVE, positive);
    Reply("ARM IK DRY_RUN NO_MOTION X=%d ZG=%d PHI=%d "
          "NEG=%u PN=%u,%u,%u POS=%u PP=%u,%u,%u\r\n",
          x_mm, ground_z_mm, phi_deg,
          (unsigned)negative_result,
          (unsigned)negative[0], (unsigned)negative[1],
          (unsigned)negative[2],
          (unsigned)positive_result,
          (unsigned)positive[0], (unsigned)positive[1],
          (unsigned)positive[2]);
}

static unsigned PositionDelta(uint16_t first, uint16_t second)
{
    return first > second ? (unsigned)(first - second) :
                            (unsigned)(second - first);
}

static bool SynchronizeCartesian(unsigned p0, unsigned p1, unsigned p2)
{
    uint16_t positions[3] = {
        (uint16_t)p0, (uint16_t)p1, (uint16_t)p2
    };
    ArmPose2D_t pose;
    ArmKinematicsResult_t result;
    if (Arm_GetStatus().motion_allowed) {
        Reply("ARM ERR STOP_BEFORE_SYNC\r\n");
        return false;
    }
    result = ArmKinematics_ProjectForward(positions, &pose);
    if (result != ARM_KINEMATICS_OK) {
        Reply("ARM SYNC RESULT=%u REJECTED NO_MOTION\r\n", (unsigned)result);
        return false;
    }
    memcpy(cartesian_position, positions, sizeof(cartesian_position));
    cartesian_pose = pose;
    cartesian_known = true;
    Reply("ARM SYNC RESULT=0 P=%u,%u,%u REFERENCE_ONLY NO_MOTION\r\n",
          p0, p1, p2);
    return true;
}

static ArmResult_t StartDualReferenceMove(void)
{
    ArmStep_t step = {0};
    ArmResult_t result;
    step.joint_mask = 0x0FU;
    step.position[0] = ARM_REFERENCE_P0;
    step.position[1] = ARM_REFERENCE_P1;
    step.position[2] = ARM_REFERENCE_P2;
    step.position[3] = ARM_REFERENCE_P3;
    step.move_ms = ARM_TUNER_DUAL_REFERENCE_MS;
    result = Arm_SetMotionAllowed(true);
    if (result != ARM_OK) return result;
    result = Arm_StartSequence(&step, 1U);
    if (result != ARM_OK) (void)Arm_SetMotionAllowed(false);
    return result;
}

static void BeginDualStart(void)
{
    memset(target_known, 0, sizeof(target_known));
    cartesian_known = false;
    session = true;
    dual_enabled = false;
    remote_enabled = false;
    remote_center_seen = false;
    finite_setup = true;
    finite_pending = false;
    dual_start_waiting = true;
    dual_start_moving = false;
    dual_auto_arm_pending = false;
    dual_restart_pending = false;
    dual_restart_tick = 0U;
    dual_center_samples = 0U;
    remote_arm_x = remote_arm_y = 0;
    remote_direction_latch = BT_DIRECTION_NONE;
    next_preset = 0U;
    remote_preset_buttons_latch = 0U;
    move_ms = ARM_TUNER_DUAL_REFERENCE_MS;
    Bluetooth_SetExtended(1U);
    joystick_sequence = Bluetooth_GetArmSequence();
    Reply("ARM DUAL START; CHASSIS_UNCHANGED; REFERENCE P=%u,%u,%u "
          "G=%u T=%u\r\n",
          ARM_REFERENCE_P0, ARM_REFERENCE_P1, ARM_REFERENCE_P2,
          ARM_REFERENCE_P3, ARM_TUNER_DUAL_REFERENCE_MS);
}

static void FinishDualReferenceMove(void)
{
    size_t index;
    (void)Arm_SetMotionAllowed(false);
    target[0] = ARM_REFERENCE_P0;
    target[1] = ARM_REFERENCE_P1;
    target[2] = ARM_REFERENCE_P2;
    target[3] = ARM_REFERENCE_P3;
    for (index = 0U; index < 4U; ++index) target_known[index] = true;
    dual_start_moving = false;
    finite_setup = false;
    if (!SynchronizeCartesian(ARM_REFERENCE_P0, ARM_REFERENCE_P1,
                              ARM_REFERENCE_P2)) {
        Reply("ARM DUAL START FAILED_SYNC; NO_MOTION\r\n");
        return;
    }
    dual_enabled = true;
    remote_enabled = true;
    remote_center_seen = false;
    dual_auto_arm_pending = true;
    dual_center_samples = 0U;
    remote_arm_x = remote_arm_y = 0;
    remote_direction_latch = BT_DIRECTION_NONE;
    remote_preset_buttons_latch = 0U;
    remote_axis_tick = HAL_GetTick();
    move_ms = ARM_TUNER_REMOTE_MIN_PERIOD_MS;
    joystick_sequence = Bluetooth_GetArmSequence();
    Reply("ARM DUAL READY SHORTS=16; ENTER_CONTROL_CENTER_PACKET\r\n");
}

static void JogCartesian(int delta_x_mm, int delta_z_mm, int delta_phi_deg)
{
    ArmPose2D_t candidate_pose;
    uint16_t candidate_position[3] = {0U, 0U, 0U};
    ArmStep_t step = {0};
    ArmKinematicsResult_t kinematics_result;
    ArmResult_t arm_result;
    size_t index;
    if (!CartesianConfigurationValid()) {
        Reply("ARM ERR CART_CONFIG_IDS_0_1_2_REQUIRED\r\n");
        return;
    }
    if (!cartesian_known) {
        Reply("ARM ERR SYNC_REQUIRED\r\n");
        return;
    }
    if (!Arm_GetStatus().motion_allowed) {
        Reply("ARM ERR ARM_REQUIRED\r\n");
        return;
    }
    if (!WorkspaceReady() || move_ms == 0U) {
        Reply("ARM ERR PARK_TIME_REQUIRED\r\n");
        return;
    }
    if (delta_x_mm == 0 && delta_z_mm == 0 && delta_phi_deg == 0) {
        Reply("ARM ERR ZERO_JOG\r\n");
        return;
    }
    candidate_pose = cartesian_pose;
    candidate_pose.x_mm += (float)delta_x_mm;
    candidate_pose.z_mm += (float)delta_z_mm;
    candidate_pose.phi_rad +=
        (float)delta_phi_deg * ARM_TUNER_PI / 180.0f;
    kinematics_result = ArmKinematics_ProjectInverse(
        &candidate_pose, ARM_ELBOW_NEGATIVE, candidate_position);
    if (kinematics_result != ARM_KINEMATICS_OK) {
        Reply("ARM JOG IK=%u REJECTED NO_MOTION\r\n",
              (unsigned)kinematics_result);
        return;
    }
    for (index = 0U; index < 3U; ++index) {
        if (PositionDelta(candidate_position[index], cartesian_position[index]) >
            ARM_TUNER_MAX_JOG_P_DELTA) {
            Reply("ARM ERR JOG_JOINT_DELTA J=%u FROM=%u TO=%u NO_MOTION\r\n",
                  (unsigned)index, (unsigned)cartesian_position[index],
                  (unsigned)candidate_position[index]);
            return;
        }
        step.position[index] = candidate_position[index];
    }
    step.joint_mask = 0x07U;
    step.move_ms = move_ms;
    arm_result = Arm_StartSequence(&step, 1U);
    if (arm_result == ARM_OK) {
        memcpy(cartesian_position, candidate_position,
               sizeof(cartesian_position));
        cartesian_pose = candidate_pose;
        for (index = 0U; index < 3U; ++index) {
            target[index] = candidate_position[index];
            target_known[index] = true;
        }
    }
    Reply("ARM JOG RESULT=%u DX=%d DZ=%d DPHI=%d P=%u,%u,%u %s\r\n",
          (unsigned)arm_result, delta_x_mm, delta_z_mm, delta_phi_deg,
          (unsigned)candidate_position[0],
          (unsigned)candidate_position[1],
          (unsigned)candidate_position[2],
          arm_result == ARM_OK ? "ACCEPTED_NOT_ARRIVED" : "REJECTED");
}

static int RemoteAxisStep(int value)
{
    int magnitude = value < 0 ? -value : value;
    int step;
    if (magnitude <= ARM_TUNER_REMOTE_DEADZONE) return 0;
    step = magnitude > JOY_RANGE / 2 ? ARM_TUNER_REMOTE_STEP_MM : 1;
    return value < 0 ? -step : step;
}

static void RemoteWrist(int delta_position)
{
    ArmPose2D_t pose;
    ArmResult_t result;
    int candidate;
    if (!cartesian_known || !Arm_GetStatus().motion_allowed ||
        !WorkspaceReady() || Active()) return;
    candidate = (int)cartesian_position[2] + delta_position;
    if (candidate < (int)config.joints[ARM_JOINT_WRIST].min_position)
        candidate = (int)config.joints[ARM_JOINT_WRIST].min_position;
    if (candidate > (int)config.joints[ARM_JOINT_WRIST].max_position)
        candidate = (int)config.joints[ARM_JOINT_WRIST].max_position;
    if (candidate == (int)cartesian_position[2]) {
        Reply("ARM WRIST LIMIT P=%u NO_MOTION\r\n",
              (unsigned)cartesian_position[2]);
        return;
    }
    result = Arm_MoveJoint(ARM_JOINT_WRIST, (uint16_t)candidate, move_ms, 0U);
    if (result == ARM_OK) {
        cartesian_position[2] = (uint16_t)candidate;
        target[2] = (uint16_t)candidate;
        target_known[2] = true;
        if (ArmKinematics_ProjectForward(cartesian_position, &pose) ==
            ARM_KINEMATICS_OK) {
            cartesian_pose = pose;
        } else {
            cartesian_known = false;
        }
    }
    Reply("ARM WRIST RESULT=%u DP=%d P=%u %s\r\n",
          (unsigned)result, delta_position, (unsigned)candidate,
          result == ARM_OK ? "ACCEPTED_NOT_ARRIVED" : "REJECTED");
}

static void RemoteGrip(uint16_t position)
{
    ArmResult_t result;
    if (!Arm_GetStatus().motion_allowed || !WorkspaceReady() || Active()) return;
    result = Arm_MoveJoint(ARM_JOINT_GRIPPER, position,
                           ARM_TUNER_REMOTE_GRIP_MOVE_MS, 0U);
    if (result == ARM_OK) {
        target[ARM_JOINT_GRIPPER] = position;
        target_known[ARM_JOINT_GRIPPER] = true;
    }
    Reply("ARM REMOTE GRIP RESULT=%u P=%u %s\r\n",
          (unsigned)result, (unsigned)position,
          result == ARM_OK ? "ACCEPTED_NOT_ARRIVED" : "REJECTED");
}

static void RemoteGripStep(int delta_position)
{
    int candidate;
    if (!target_known[ARM_JOINT_GRIPPER]) {
        Reply("ARM ERR GRIP_REFERENCE_REQUIRED; RESTART_DUAL\r\n");
        return;
    }
    candidate = (int)target[ARM_JOINT_GRIPPER] + delta_position;
    if (candidate < (int)config.joints[ARM_JOINT_GRIPPER].min_position)
        candidate = (int)config.joints[ARM_JOINT_GRIPPER].min_position;
    if (candidate > (int)config.joints[ARM_JOINT_GRIPPER].max_position)
        candidate = (int)config.joints[ARM_JOINT_GRIPPER].max_position;
    if (candidate == (int)target[ARM_JOINT_GRIPPER]) {
        Reply("ARM GRIP LIMIT P=%u NO_MOTION\r\n",
              (unsigned)target[ARM_JOINT_GRIPPER]);
        return;
    }
    RemoteGrip((uint16_t)candidate);
}

static void StartPreset(unsigned index)
{
    ArmStep_t step;
    ArmPose2D_t pose;
    ArmResult_t result;
    ServoPoseResult_t pose_result;
    ServoMode_t mode;
    size_t joint;
    if (index >= SERVO_POSE_COUNT) {
        Reply("ARM ERR PRESET_RANGE 0_TO_%u\r\n", SERVO_POSE_COUNT - 1U);
        return;
    }
    if (!dual_enabled || !remote_enabled || !WorkspaceReady() ||
        !Arm_GetStatus().motion_allowed) {
        Reply("ARM ERR PRESET_DUAL_NOT_READY\r\n");
        return;
    }
    if (!Bluetooth_IsConnected() ||
        Car_Control_GetState() != CAR_READY || !Motor_IsIdle() ||
        !ChassisControlsNeutral()) {
        Reply("ARM ERR PRESET_CHASSIS_NOT_READY\r\n");
        return;
    }
    if (Active()) {
        Reply("ARM ERR BUSY\r\n");
        return;
    }
    mode = (ServoMode_t)(SERVO_MODE_BALL_PREPARE + index);
    pose_result = ServoPose_BuildOriginalMode(mode, &step);
    if (pose_result != SERVO_POSE_OK) {
        Reply("ARM ERR PRESET_DATA RESULT=%u\r\n", (unsigned)pose_result);
        return;
    }
    step.move_ms = ARM_TUNER_PRESET_MOVE_MS;
    result = Arm_StartOriginalPreset(&step);
    if (result == ARM_OK) {
        for (joint = 0U; joint < 4U; ++joint) {
            target[joint] = step.position[joint];
            target_known[joint] = true;
        }
        memcpy(cartesian_position, step.position, sizeof(cartesian_position));
        if (ArmKinematics_ProjectForward(cartesian_position, &pose) ==
            ARM_KINEMATICS_OK) {
            cartesian_pose = pose;
            cartesian_known = true;
        } else {
            cartesian_known = false;
        }
        next_preset = (uint8_t)((index + 1U) % SERVO_POSE_COUNT);
        remote_arm_x = remote_arm_y = 0;
    }
    Reply("ARM PRESET RESULT=%u INDEX=%u NAME=%s P=%u,%u,%u,%u T=%u %s\r\n",
          (unsigned)result, index, ServoPose_Name(mode),
          (unsigned)step.position[0], (unsigned)step.position[1],
          (unsigned)step.position[2], (unsigned)step.position[3],
          ARM_TUNER_PRESET_MOVE_MS,
          result == ARM_OK ? "ACCEPTED_NOT_ARRIVED" : "REJECTED");
}

static void RemoteCartesianDelta(int dx, int dz)
{
    if (dx == 0 && dz == 0) return;
    if (move_ms < ARM_TUNER_MIN_MOVE_MS ||
        move_ms > ARM_TUNER_REMOTE_MAX_MOVE_MS) return;
    if (Active() || (uint32_t)(HAL_GetTick() - remote_step_tick) <
        ARM_TUNER_REMOTE_MIN_PERIOD_MS) return;
    remote_step_tick = HAL_GetTick();
    JogCartesian(dx, dz, 0);
}

static void RemoteJog(const BtArmControl_t *control)
{
    int dx = 0, dz = 0;
    /* Reuse the existing five-short packet. In arm mode its direction short
     * selects elevation/reach; outside arm mode it retains vehicle meaning. */
    if (dual_enabled) {
        remote_arm_x = control->arm_x;
        remote_arm_y = control->arm_y;
        remote_axis_tick = control->last_rx_tick;
        switch (control->direction) {
        case BT_DIRECTION_FORWARD:
            remote_arm_x = remote_arm_y = 0;
            if (remote_direction_latch == control->direction) return;
            remote_direction_latch = control->direction;
            RemoteWrist((int)ARM_TUNER_REMOTE_WRIST_STEP_P);
            return;
        case BT_DIRECTION_BACKWARD:
            remote_arm_x = remote_arm_y = 0;
            if (remote_direction_latch == control->direction) return;
            remote_direction_latch = control->direction;
            RemoteWrist(-(int)ARM_TUNER_REMOTE_WRIST_STEP_P);
            return;
        case BT_DIRECTION_LEFT:
            remote_arm_x = remote_arm_y = 0;
            if (remote_direction_latch == control->direction) return;
            remote_direction_latch = control->direction;
            RemoteGripStep((int)ARM_TUNER_REMOTE_GRIP_STEP_P);
            return;
        case BT_DIRECTION_RIGHT:
            remote_arm_x = remote_arm_y = 0;
            if (remote_direction_latch == control->direction) return;
            remote_direction_latch = control->direction;
            RemoteGripStep(-(int)ARM_TUNER_REMOTE_GRIP_STEP_P);
            return;
        case BT_DIRECTION_ARM_PRESET_NEXT:
            remote_arm_x = remote_arm_y = 0;
            if (remote_direction_latch == control->direction) return;
            remote_direction_latch = control->direction;
            StartPreset(next_preset);
            return;
        default:
            remote_direction_latch = BT_DIRECTION_NONE;
            dx = RemoteAxisStep(control->arm_x);
            dz = RemoteAxisStep(control->arm_y);
            break;
        }
    } else switch (control->direction) {
    case BT_DIRECTION_FORWARD: dz = ARM_TUNER_REMOTE_STEP_MM; break;
    case BT_DIRECTION_BACKWARD: dz = -ARM_TUNER_REMOTE_STEP_MM; break;
    case BT_DIRECTION_LEFT: dx = -ARM_TUNER_REMOTE_STEP_MM; break;
    case BT_DIRECTION_RIGHT: dx = ARM_TUNER_REMOTE_STEP_MM; break;
    default:
        dx = RemoteAxisStep(control->x);
        dz = RemoteAxisStep(control->y);
        break;
    }
    RemoteCartesianDelta(dx, dz);
}

void ArmTuner_Process(void)
{
    ArmStatus_t s = Arm_GetStatus();
    uint32_t seq = Bluetooth_GetArmSequence();
    uint32_t bounded_motion_timeout = ARM_TUNER_HEARTBEAT_MS;
    bool fresh = seq != joystick_sequence;
    bool brake = fresh &&
                 (Bluetooth_GetArmControl()->brake || Bluetooth_GetArmControl()->disable);
    joystick_sequence = seq;
    if (ArmTrimBench_IsActive()) {
        if (brake) ArmTrimBench_Cancel();
        ArmTrimBench_Process();
        return;
    }
    if (ArmTrimBluetooth_OwnsMotion()) {
        if (brake) ArmTrimBluetooth_Cancel();
        ArmTrimBluetooth_Process();
        return;
    }
    if (!session) return;
    if (brake) { Stop(); Reply("ARM STOP JOYSTICK\r\n"); return; }
    if (dual_restart_pending) {
        if (s.state == ARM_STOPPING) return;
        if (s.state == ARM_FAULT) {
            dual_restart_pending = false;
            dual_restart_tick = 0U;
            Reply("ARM DUAL RESTART FAULT; CHECK_HARDWARE_THEN_CLEAR\r\n");
            return;
        }
        if ((uint32_t)(HAL_GetTick() - dual_restart_tick) <
            ARM_TUNER_DUAL_RESTART_SETTLE_MS)
            return;
        BeginDualStart();
        return;
    }
    if (dual_start_waiting) {
        ArmResult_t start_result;
        start_result = StartDualReferenceMove();
        if (start_result != ARM_OK) {
            dual_start_waiting = false;
            finite_setup = false;
            Bluetooth_SetExtended(0U);
            Reply("ARM DUAL START RESULT=%u; NO_MOTION\r\n",
                  (unsigned)start_result);
            return;
        }
        dual_start_waiting = false;
        dual_start_moving = true;
        heartbeat_tick = HAL_GetTick();
        Reply("ARM DUAL START RESULT=0; MOVING_REFERENCE P=%u,%u,%u "
              "G=%u T=%u\r\n",
              ARM_REFERENCE_P0, ARM_REFERENCE_P1, ARM_REFERENCE_P2,
              ARM_REFERENCE_P3, ARM_TUNER_DUAL_REFERENCE_MS);
        return;
    }
    if (dual_start_moving && s.motion_allowed &&
        s.state == ARM_COMPLETE_ESTIMATED) {
        FinishDualReferenceMove();
        return;
    }
    /* SETUP executes one bounded absolute command, then revokes permission.
     * It is not a continuous/queued joystick mode and never assumes a pose. */
    if (!brake && finite_setup && !finite_pending && s.motion_allowed &&
        s.state == ARM_COMPLETE_ESTIMATED) {
        (void)Arm_SetMotionAllowed(false);
        Reply("ARM SETUP COMPLETE_ESTIMATED; REARM_FOR_NEXT_MOVE\r\n");
        return;
    }
    /* The phone app emits dual packets on value changes instead of streaming
     * continuously while idle. A dual Cartesian command is already a bounded
     * 100 ms segment, so an idle packet gap is not an active motion hazard.
     * Keep the reference/permission while idle; every later segment still
     * requires a fresh, valid packet and passes WorkspaceReady in JogCartesian.
     * Chassis motor state is intentionally outside the arm safety domain. */
    if (dual_start_moving)
        bounded_motion_timeout = ARM_TUNER_DUAL_REFERENCE_MS + 500U;
    else if (finite_setup && s.state == ARM_RUNNING)
        bounded_motion_timeout = (uint32_t)move_ms + 500U;
    if (!dual_enabled && (s.motion_allowed || s.state == ARM_RUNNING) &&
        (!WorkspaceReady() ||
         (uint32_t)(HAL_GetTick()-heartbeat_tick) > bounded_motion_timeout)) {
        Stop();
        Reply("ARM STOP INTERLOCK_OR_HEARTBEAT; REARM_REQUIRED\r\n");
        return;
    }
    /* Legacy continuous remote mode keeps the strict stream lease. Dual mode
     * uses only bounded commands and the short repeat latch below, so an idle
     * event-driven phone must not lose its arm reference at 500 ms. */
    if (remote_enabled) {
        if (!dual_enabled &&
            (s.motion_allowed || s.state == ARM_RUNNING) &&
            (!Bluetooth_ArmIsConnected() ||
             (uint32_t)(HAL_GetTick() - remote_rx_tick) > BT_FAILSAFE_TIMEOUT_MS)) {
            Stop();
            Reply("ARM STOP REMOTE_RX_TIMEOUT; REARM_REQUIRED\r\n");
            return;
        }
        if (fresh && Bluetooth_ArmIsConnected()) {
            const BtArmControl_t *control = Bluetooth_GetArmControl();
            uint16_t preset_buttons = (uint16_t)(
                Bluetooth_GetControl()->frame.servo_buttons & 0x00FFU);
            uint16_t new_preset_buttons = (uint16_t)(
                preset_buttons & (uint16_t)~remote_preset_buttons_latch);
            ArmStatus_t current;
            remote_rx_tick = control->last_rx_tick;
            remote_preset_buttons_latch = preset_buttons;
            current = Arm_GetStatus();
            if (!current.motion_allowed && RemoteCentered(control)) {
                remote_center_seen = true;
                if (dual_auto_arm_pending) {
                    if (dual_center_samples < ARM_TUNER_DUAL_CENTER_SAMPLES)
                        ++dual_center_samples;
                    if (dual_center_samples >= ARM_TUNER_DUAL_CENTER_SAMPLES) {
                        ArmResult_t grant_result;
                        dual_auto_arm_pending = false;
                        dual_center_samples = 0U;
                        if (!cartesian_known || !CartesianConfigurationValid() ||
                            !WorkspaceReady() || Active()) {
                            Reply("ARM DUAL AUTO_GRANT REJECTED; RESTART_REQUIRED\r\n");
                            return;
                        }
                        grant_result = Arm_SetMotionAllowed(true);
                        if (grant_result == ARM_OK) {
                            heartbeat_tick = control->last_rx_tick;
                            remote_step_tick = HAL_GetTick() -
                                               ARM_TUNER_REMOTE_MIN_PERIOD_MS;
                        }
                        Reply("ARM DUAL AUTO_GRANT RESULT=%u; JOYSTICKS_ACTIVE\r\n",
                              (unsigned)grant_result);
                    }
                }
            } else if (!current.motion_allowed && dual_auto_arm_pending) {
                dual_center_samples = 0U;
            }
            current = Arm_GetStatus();
            if (current.motion_allowed) {
                heartbeat_tick = control->last_rx_tick;
                if (dual_enabled && new_preset_buttons != 0U &&
                    preset_buttons == new_preset_buttons &&
                    (new_preset_buttons &
                     (uint16_t)(new_preset_buttons - 1U)) == 0U) {
                    unsigned preset_index = 0U;
                    while ((new_preset_buttons &
                            (uint16_t)(1U << preset_index)) == 0U)
                        ++preset_index;
                    StartPreset(preset_index);
                    return; /* Consume the edge whether accepted or rejected. */
                }
                if (dual_enabled || cartesian_known) RemoteJog(control);
            }
        }
        /* The phone emits joystick packets on value changes, not at a stable
         * periodic rate. Continue only the most recent Cartesian vector for a
         * short bounded lease, chaining 200 ms interpolation segments.
         * A centered packet cancels the latch immediately; a lost release can
         * therefore move at most ARM_TUNER_REMOTE_AXIS_HOLD_MS. */
        if (dual_enabled && Bluetooth_ArmIsConnected() &&
            Arm_GetStatus().motion_allowed && cartesian_known &&
            !Active() && (remote_arm_x != 0 || remote_arm_y != 0) &&
            (uint32_t)(HAL_GetTick() - remote_axis_tick) <=
                ARM_TUNER_REMOTE_AXIS_HOLD_MS) {
            RemoteCartesianDelta(RemoteAxisStep(remote_arm_x),
                                 RemoteAxisStep(remote_arm_y));
        }
    }
    if (s.state == ARM_FAULT || s.state == ARM_STOPPING)
    {
        memset(target_known, 0, sizeof(target_known));
        cartesian_known = false;
        remote_center_seen = false;
        dual_auto_arm_pending = false;
        dual_center_samples = 0U;
    }
}

static void HandleLine(const char *line, uint32_t tick)
{
    char copy[80], *word[6];
    size_t count = 0U, i;
    unsigned a = 0U, b = 0U, c = 0U;
    int signed_a = 0, signed_b = 0, signed_c = 0;
    ArmResult_t result;
    if (line == NULL) {
        if (ArmTrimBench_HandleLine(NULL, tick, session)) return;
        if (ArmTrimBluetooth_HandleLine(NULL, tick, session)) return;
        if (session && dual_enabled && remote_enabled) {
            /* A UART receive recovery invalidates the packet stream, but every
             * dual-mode command is already a bounded segment. Do not erase the
             * Cartesian reference and strand only the arm while the chassis
             * later reconnects. Cancel the short repeat latch; the accepted
             * segment may finish and no new segment starts without a new valid
             * packet. Explicit shared brake/disable still calls Stop(). */
            remote_arm_x = remote_arm_y = 0;
            remote_direction_latch = BT_DIRECTION_NONE;
            Reply("ARM DUAL RX_LOSS; REPEAT_CANCELLED CURRENT_SEGMENT_BOUNDED\r\n");
        } else if (session) {
            Stop();
            Reply("ARM STOP RX_LOSS; REARM_REQUIRED\r\n");
        }
        return;
    }
    if (ArmTrimBench_HandleLine(line, tick, session)) return;
    if (strncmp(line, "@ARM ", 5U) != 0) return;
    if ((uint32_t)(HAL_GetTick() - tick) > BT_FAILSAFE_TIMEOUT_MS) return;
    if (ArmTrimBench_IsActive() && ArmTrimBluetooth_OwnsMotion() &&
        strncmp(line, "@ARM TRIM ", 10U) == 0 &&
        strcmp(line, "@ARM TRIM STATUS") != 0 &&
        strncmp(line, "@ARM TRIM KEEP ", 15U) != 0 &&
        strcmp(line, "@ARM TRIM RELEASE") != 0) {
        if (strcmp(line, "@ARM TRIM STOP") == 0) ArmTrimBench_Cancel();
        else { Reply("TRIM ERR BENCH_BUSY\r\n"); return; }
    }
    if (ArmTrimBluetooth_HandleLine(line, tick, session || ArmTrimBench_IsActive())) return;
    if (ArmTrimBluetooth_OwnsMotion() || ArmTrimBench_IsActive()) {
        if (strcmp(line, "@ARM STOP") == 0) {
            Stop();
            Reply("ARM STOP REQUESTED\r\n");
        } else Reply("ARM ERR TEST_OWNS_MOTION; END_TRIM_OR_WAIT_BENCH\r\n");
        return;
    }
    ArmTuner_Process(); /* Expired sessions cannot be revived by a late PING. */
    if (strlen(line) >= sizeof(copy) || (uint32_t)(HAL_GetTick()-tick)>BT_FAILSAFE_TIMEOUT_MS)
        return;
    strcpy(copy, line+5U);
    for (i = 0U; copy[i] != '\0'; ) {
        while (copy[i] == ' ') ++i;
        if (copy[i] == '\0') break;
        if (count == sizeof(word)/sizeof(word[0])) { Reply("ARM ERR SYNTAX\r\n"); return; }
        word[count++] = &copy[i];
        while (copy[i] != '\0' && copy[i] != ' ') ++i;
        if (copy[i] != '\0') copy[i++] = '\0';
    }
    if (count == 0U) { Reply("ARM ERR SYNTAX\r\n"); return; }
#define CMD(name, n) (strcmp(word[0], (name)) == 0 && count == (n))
    if (CMD("SHOW", 1U)) { Show(); return; }
    if (CMD("TARGETS", 1U)) { ShowTargets(); return; }
    if (CMD("LINK", 1U)) { ShowLink(); return; }
    if (CMD("IK", 4U) &&
        SignedNumber(word[1], -ARM_TUNER_COORDINATE_LIMIT_MM,
                     ARM_TUNER_COORDINATE_LIMIT_MM, &signed_a) &&
        SignedNumber(word[2], -ARM_TUNER_COORDINATE_LIMIT_MM,
                     ARM_TUNER_COORDINATE_LIMIT_MM, &signed_b) &&
        SignedNumber(word[3], -180, 180, &signed_c)) {
        PreviewInverseKinematics(signed_a, signed_b, signed_c);
        return;
    }
    if (strcmp(word[0], "IK") == 0) {
        Reply("ARM ERR SYNTAX; IK X_MM GROUND_Z_MM PHI_DEG\r\n");
        return;
    }
    if (CMD("STOP", 1U)) { Stop(); Reply("ARM STOP REQUESTED\r\n"); return; }
    if (CMD("DUAL", 2U) && strcmp(word[1], "START") == 0) {
        ArmStatus_t start_status;
        if (CAR_PD10_STANDALONE_TEST || CAR_MECANUM_TEST_MODE || CAR_HEADING_TEST_MODE) {
            Reply("ARM ERR DIAGNOSTIC_IMAGE\r\n"); return;
        }
        start_status = Arm_GetStatus();
        if (!session && (Active() || start_status.motion_allowed)) {
            Reply("ARM ERR PRESET_BUSY\r\n"); return;
        }
        if (start_status.state == ARM_FAULT) {
            Reply("ARM ERR DUAL_START_FAULT; CHECK_HARDWARE_THEN_CLEAR\r\n");
            return;
        }
        if (Active()) {
            /* Make START idempotent for recovery: stop the previous bounded
             * session, then begin the staged safe reference automatically. */
            session = true;
            Bluetooth_SetExtended(1U);
            Stop();
            dual_restart_pending = true;
            dual_restart_tick = HAL_GetTick();
            Reply("ARM DUAL START; STOPPING_PREVIOUS AUTO_RESTART_PENDING\r\n");
            return;
        }
        if (start_status.motion_allowed)
            (void)Arm_SetMotionAllowed(false);
        BeginDualStart();
        return;
    }
    if (CMD("DUAL", 2U) &&
        (strcmp(word[1], "ON") == 0 || strcmp(word[1], "OFF") == 0)) {
        if (CAR_PD10_STANDALONE_TEST || CAR_MECANUM_TEST_MODE || CAR_HEADING_TEST_MODE) {
            Reply("ARM ERR DIAGNOSTIC_IMAGE\r\n"); return;
        }
        if (Active() || Arm_GetStatus().motion_allowed) {
            Reply("ARM ERR STOP_BEFORE_DUAL\r\n"); return;
        }
        dual_enabled = strcmp(word[1], "ON") == 0;
        Bluetooth_SetExtended((uint8_t)dual_enabled);
        session = true;
        finite_setup = false;
        remote_enabled = dual_enabled;
        remote_center_seen = false;
        dual_start_waiting = dual_start_moving = false;
        dual_auto_arm_pending = false;
        dual_restart_pending = false;
        dual_restart_tick = 0U;
        dual_center_samples = 0U;
        remote_direction_latch = BT_DIRECTION_NONE;
        move_ms = ARM_TUNER_REMOTE_MIN_PERIOD_MS;
        joystick_sequence = Bluetooth_GetArmSequence();
        Reply("ARM DUAL=%u SHORTS=%u TIME=%u NO_MOTION; SYNC_AND_CENTER_THEN_ARM\r\n",
              (unsigned)dual_enabled, dual_enabled ? 16U : 13U,
              (unsigned)move_ms);
        return;
    }
    if (CMD("SETUP", 1U)) {
        if (CAR_PD10_STANDALONE_TEST || CAR_MECANUM_TEST_MODE || CAR_HEADING_TEST_MODE) {
            Reply("ARM ERR DIAGNOSTIC_IMAGE\r\n"); return;
        }
        if (Active() || Arm_GetStatus().motion_allowed) {
            Reply("ARM ERR STOP_BEFORE_SETUP\r\n"); return;
        }
        session = finite_setup = true;
        dual_enabled = remote_enabled = remote_center_seen = false;
        dual_start_waiting = dual_start_moving = false;
        dual_auto_arm_pending = false;
        dual_restart_pending = false;
        dual_restart_tick = 0U;
        dual_center_samples = 0U;
        remote_direction_latch = BT_DIRECTION_NONE;
        move_ms = 2000U;
        joystick_sequence = Bluetooth_GetArmSequence();
        Reply("ARM SETUP=1 TIME=2000 SINGLE_MOVE NO_PING; SELECT_ARM_MOVE\r\n");
        return;
    }
    if (CMD("REMOTE_ENTER", 1U)) {
        if (CAR_PD10_STANDALONE_TEST || CAR_MECANUM_TEST_MODE || CAR_HEADING_TEST_MODE) {
            Reply("ARM ERR DIAGNOSTIC_IMAGE\r\n"); return;
        }
        if (Active() || Arm_GetStatus().motion_allowed) {
            Reply("ARM ERR STOP_BEFORE_REMOTE\r\n"); return;
        }
        dual_enabled = Bluetooth_IsExtended() != 0U;
        session = remote_enabled = true;
        finite_setup = false;
        remote_center_seen = false;
        dual_start_waiting = dual_start_moving = false;
        dual_auto_arm_pending = false;
        dual_restart_pending = false;
        dual_restart_tick = 0U;
        dual_center_samples = 0U;
        remote_direction_latch = BT_DIRECTION_NONE;
        joystick_sequence = Bluetooth_GetArmSequence();
        move_ms = ARM_TUNER_REMOTE_MIN_PERIOD_MS;
        Reply("ARM MODE=1 REMOTE=1 TIME=%u NO_MOTION; SYNC_AND_CENTER_THEN_ARM\r\n",
              (unsigned)move_ms);
        return;
    }
    if (CMD("ENTER", 1U)) {
        if (CAR_PD10_STANDALONE_TEST || CAR_MECANUM_TEST_MODE || CAR_HEADING_TEST_MODE) {
            Reply("ARM ERR DIAGNOSTIC_IMAGE\r\n"); return;
        }
        if (!session) {
            if (Active() || Arm_GetStatus().motion_allowed) {
                Reply("ARM ERR BUSY\r\n"); return;
            }
            (void)Arm_SetMotionAllowed(false);
            session = true;
            joystick_sequence = Bluetooth_GetArmSequence();
        }
        Reply("ARM MODE=1; CHASSIS_UNCHANGED; CHECK_ARM_CLEARANCE\r\n");
        return;
    }
    if (!session) { Reply("ARM ERR ENTER_REQUIRED\r\n"); return; }
    if (CMD("PING", 1U)) { heartbeat_tick = tick; return; } /* No reply flood. */
    if (CMD("EXIT", 1U)) {
        if (Active() || Arm_GetStatus().state == ARM_FAULT) {
            Reply("ARM ERR STOP_OR_CLEAR_FIRST\r\n"); return;
        }
        (void)Arm_SetMotionAllowed(false);
        memset(target_known, 0, sizeof(target_known));
        cartesian_known = false;
        remote_enabled = remote_center_seen = false;
        dual_enabled = finite_setup = false;
        dual_start_waiting = dual_start_moving = false;
        dual_auto_arm_pending = false;
        dual_restart_pending = false;
        dual_restart_tick = 0U;
        dual_center_samples = 0U;
        remote_direction_latch = BT_DIRECTION_NONE;
        session = false;
        Reply("ARM MODE=0; CAR_CENTER_REARM_REQUIRED\r\n"); return;
    }
    if (CMD("CLEAR", 1U)) {
        result = Arm_ClearFault();
        Reply("ARM CLEAR RESULT=%u; CHECK_HARDWARE_BEFORE_REARM\r\n", (unsigned)result);
        return;
    }
    if (CMD("ARM", 1U)) {
        const BtArmControl_t *control = Bluetooth_GetArmControl();
        if (Bluetooth_ArmIsConnected() && (control->brake || control->disable)) {
            Reply("ARM ERR RELEASE_JOYSTICK_BRAKE\r\n"); return;
        }
        if (!WorkspaceReady() || Active() || !config.joints[selected].enabled || move_ms == 0U) {
            Reply("ARM ERR PARK_CONFIG_TIME_REQUIRED\r\n"); return;
        }
        if (remote_enabled && (!cartesian_known || !remote_center_seen ||
            !Bluetooth_ArmIsConnected() || !RemoteCentered(control) ||
            !CartesianConfigurationValid() ||
            move_ms > ARM_TUNER_REMOTE_MAX_MOVE_MS)) {
            Reply("ARM ERR REMOTE_SYNC_CENTER_TIME_REQUIRED\r\n"); return;
        }
        result = Arm_SetMotionAllowed(true);
        if (result == ARM_OK) {
            dual_auto_arm_pending = false;
            dual_center_samples = 0U;
            if (finite_setup) {
                finite_pending = true;
                Reply("ARM GRANT RESULT=0; SINGLE_MOVE_NO_PING\r\n");
                heartbeat_tick = tick;
                return;
            }
            heartbeat_tick = tick;
            remote_step_tick = HAL_GetTick() - ARM_TUNER_REMOTE_MIN_PERIOD_MS;
            remote_rx_tick = control->last_rx_tick;
        }
        Reply("ARM GRANT RESULT=%u; %s\r\n", (unsigned)result,
              remote_enabled ? "REMOTE_STREAM_REQUIRED" : "PING_REQUIRED"); return;
    }
    if (CMD("SLOT", 2U) && Number(word[1], ARM_TUNER_SLOT_COUNT-1U, &a)) {
        Reply("ARM J=%u SLOT=%u VALID=%u P=%u RAM_ONLY; NOT_FEEDBACK\r\n",
              selected, a, (unsigned)slot_valid[selected][a], (unsigned)slots[selected][a]); return;
    }
    if (Active()) { Reply("ARM ERR BUSY\r\n"); return; }
    if (CMD("GRIP", 2U) && Number(word[1], 2500U, &a)) {
        uint16_t grip_move_ms = dual_enabled ?
                                ARM_TUNER_REMOTE_GRIP_MOVE_MS : move_ms;
        if (!WorkspaceReady() || move_ms == 0U) {
            Reply("ARM ERR PARK_TIME_REQUIRED\r\n"); return;
        }
        result = Arm_MoveJoint(ARM_JOINT_GRIPPER, (uint16_t)a,
                               grip_move_ms, 0U);
        if (result == ARM_OK && finite_setup) {
            heartbeat_tick = tick;
            finite_pending = false;
        }
        if (result == ARM_OK) {
            target[ARM_JOINT_GRIPPER] = (uint16_t)a;
            target_known[ARM_JOINT_GRIPPER] = true;
        }
        Reply("ARM GRIP RESULT=%u P=%u %s\r\n", (unsigned)result, a,
              result == ARM_OK ? "ACCEPTED_NOT_ARRIVED" : "REJECTED");
        return;
    }
    if (CMD("PRESET", 2U) && strcmp(word[1], "NEXT") == 0) {
        StartPreset(next_preset);
        return;
    }
    if (CMD("PRESET", 2U) && Number(word[1], SERVO_POSE_COUNT - 1U, &a)) {
        StartPreset(a);
        return;
    }
    if (strcmp(word[0], "PRESET") == 0) {
        Reply("ARM ERR SYNTAX; PRESET 0_TO_%u OR PRESET NEXT\r\n",
              SERVO_POSE_COUNT - 1U);
        return;
    }
    if (CMD("REMOTE", 2U) && strcmp(word[1], "SHOW") == 0) {
        ArmStatus_t remote_status = Arm_GetStatus();
        Reply("ARM REMOTE=%u REF=%u CENTER=%u CONNECTED=%u TIME=%u DUAL=%u "
              "SHORTS=%u STATE=%s ALLOW=%u ERR=%u\r\n",
              (unsigned)remote_enabled, (unsigned)cartesian_known,
              (unsigned)remote_center_seen, (unsigned)Bluetooth_ArmIsConnected(),
              (unsigned)move_ms, (unsigned)dual_enabled,
              Bluetooth_IsExtended() ? 16U : 13U,
              StateName(remote_status.state),
              (unsigned)remote_status.motion_allowed,
              (unsigned)remote_status.error);
        return;
    }
    if (CMD("REMOTE", 2U) &&
        (strcmp(word[1], "ON") == 0 || strcmp(word[1], "OFF") == 0)) {
        if (Arm_GetStatus().motion_allowed) {
            Reply("ARM ERR STOP_BEFORE_REMOTE\r\n"); return;
        }
        remote_enabled = strcmp(word[1], "ON") == 0;
        remote_center_seen = false;
        joystick_sequence = Bluetooth_GetArmSequence();
        Reply("ARM REMOTE=%u NO_MOTION; FRESH_CENTER_THEN_ARM\r\n",
              (unsigned)remote_enabled);
        return;
    }
    if (CMD("SYNC", 2U) && strcmp(word[1], "LAST") == 0) {
        if (!target_known[0] || !target_known[1] || !target_known[2]) {
            Reply("ARM ERR SYNC_LAST_REQUIRES_KNOWN_P0_P1_P2\r\n");
            return;
        }
        SynchronizeCartesian(target[0], target[1], target[2]);
        return;
    }
    if (CMD("SYNC", 4U) && Number(word[1], 2500U, &a) &&
        Number(word[2], 2500U, &b) && Number(word[3], 2500U, &c)) {
        SynchronizeCartesian(a, b, c);
        return;
    }
    if (strcmp(word[0], "SYNC") == 0) {
        Reply("ARM ERR SYNTAX; SYNC P000 P001 P002 OR SYNC LAST\r\n");
        return;
    }
    if (CMD("JOG", 4U) &&
        SignedNumber(word[1], -ARM_TUNER_MAX_JOG_MM,
                     ARM_TUNER_MAX_JOG_MM, &signed_a) &&
        SignedNumber(word[2], -ARM_TUNER_MAX_JOG_MM,
                     ARM_TUNER_MAX_JOG_MM, &signed_b) &&
        SignedNumber(word[3], -ARM_TUNER_MAX_JOG_DEG,
                     ARM_TUNER_MAX_JOG_DEG, &signed_c)) {
        if (finite_setup) { Reply("ARM ERR USE_MOVE_IN_SETUP\r\n"); return; }
        if (remote_enabled) { Reply("ARM ERR REMOTE_ACTIVE\r\n"); return; }
        JogCartesian(signed_a, signed_b, signed_c);
        return;
    }
    if (strcmp(word[0], "JOG") == 0) {
        Reply("ARM ERR SYNTAX; JOG DX_MM DZ_MM DPHI_DEG\r\n");
        return;
    }
    if (CMD("SELECT", 2U) && Number(word[1], ARM_JOINT_COUNT-1U, &a)) {
        (void)Arm_SetMotionAllowed(false);
        selected = a;
        Show(); return;
    }
    if (CMD("LIMIT", 4U) && Number(word[1], 254U, &a) &&
        Number(word[2], 2500U, &b) && Number(word[3], 2500U, &c)) {
        ArmConfig_t candidate = config;
        ArmJointConfig_t *j = &candidate.joints[selected];
        if (Arm_GetStatus().motion_allowed) { Reply("ARM ERR STOP_BEFORE_CONFIG\r\n"); return; }
        j->enabled = j->calibrated = true;
        j->servo_id = (uint16_t)a; j->min_position = (uint16_t)b; j->max_position = (uint16_t)c;
        result = Arm_Configure(&candidate);
        if (result == ARM_OK) {
            config = candidate;
            target_known[selected] = false;
            cartesian_known = false;
            memset(slot_valid[selected], 0, sizeof(slot_valid[selected]));
        }
        Reply("ARM LIMIT RESULT=%u RAM_ONLY; USER_CONFIRMED_RANGE\r\n", (unsigned)result); return;
    }
    if (CMD("TIME", 2U) && Number(word[1], 9999U, &a) && a >= ARM_TUNER_MIN_MOVE_MS) {
        if (remote_enabled && a > ARM_TUNER_REMOTE_MAX_MOVE_MS) {
            Reply("ARM ERR REMOTE_TIME_100_TO_500_MS\r\n"); return;
        }
        move_ms = (uint16_t)a; Reply("ARM TIME=%u\r\n", a); return;
    }
    if (CMD("STEP", 2U) && Number(word[1], ARM_TUNER_MAX_STEP, &a) && a != 0U) {
        step_size = (uint16_t)a; Reply("ARM STEP=%u\r\n", a); return;
    }
    if (CMD("MARK", 2U) && Number(word[1], ARM_TUNER_SLOT_COUNT-1U, &a)) {
        if (!target_known[selected] || Arm_GetStatus().state != ARM_COMPLETE_ESTIMATED) {
            Reply("ARM ERR NO_COMPLETED_TARGET\r\n"); return;
        }
        slots[selected][a] = target[selected]; slot_valid[selected][a] = true;
        Reply("ARM J=%u SLOT=%u P=%u RAM_ONLY; USER_CONFIRMED_NOT_FEEDBACK\r\n",
              selected, a, (unsigned)target[selected]); return;
    }
    if ((CMD("MOVE", 2U) && Number(word[1], 2500U, &a)) || CMD("+", 1U) || CMD("-", 1U)) {
        if (remote_enabled) { Reply("ARM ERR REMOTE_ACTIVE\r\n"); return; }
        if (!WorkspaceReady() || move_ms == 0U) {
            Reply("ARM ERR PARK_TIME_REQUIRED\r\n"); return;
        }
        if (count == 1U) {
            if (!target_known[selected]) { Reply("ARM ERR MOVE_FIRST\r\n"); return; }
            a = strcmp(word[0], "+") == 0 ? target[selected]+step_size : target[selected]-step_size;
        }
        result = Arm_MoveJoint((ArmJoint_t)selected, (uint16_t)a, move_ms, 0U);
        if (result == ARM_OK) {
            if (finite_setup) {
                heartbeat_tick = tick;
                finite_pending = false;
            }
            target[selected] = (uint16_t)a;
            target_known[selected] = true;
            cartesian_known = false;
        }
        Reply("ARM MOVE RESULT=%u J=%u P=%u; %s\r\n", (unsigned)result, selected, a,
              result == ARM_OK ? "ACCEPTED_NOT_ARRIVED" : "REJECTED");
        return;
    }
    Reply("ARM ERR SYNTAX\r\n");
#undef CMD
}

void ArmTuner_Init(void)
{
    if (ArmTrimBluetooth_OwnsMotion() || ArmTrimBench_IsActive()) return;
    ArmTrimBluetooth_Init();
    Arm_ProjectConfig(&config);
    if (Arm_Configure(&config) != ARM_OK) Arm_DefaultConfig(&config);
    session = false; selected = 0U; move_ms = 0U; step_size = 5U;
    memset(target, 0, sizeof(target));
    memset(target_known, 0, sizeof(target_known));
    memset(slots, 0, sizeof(slots));
    memset(slot_valid, 0, sizeof(slot_valid));
    memset(cartesian_position, 0, sizeof(cartesian_position));
    memset(&cartesian_pose, 0, sizeof(cartesian_pose));
    cartesian_known = false;
    remote_enabled = remote_center_seen = false;
    dual_enabled = finite_setup = false;
    finite_pending = false;
    dual_start_waiting = dual_start_moving = dual_auto_arm_pending = false;
    dual_restart_pending = false;
    dual_restart_tick = 0U;
    dual_center_samples = 0U;
    remote_arm_x = remote_arm_y = 0;
    remote_direction_latch = BT_DIRECTION_NONE;
    next_preset = 0U;
    remote_preset_buttons_latch = 0U;
    remote_axis_tick = HAL_GetTick();
    remote_step_tick = HAL_GetTick();
    heartbeat_tick = HAL_GetTick();
    joystick_sequence = Bluetooth_GetArmSequence();
    Bluetooth_SetTextHandler(HandleLine);
    Debug_Log("ARM TRIM FW=V4.5 L2_MM=84.75 WT=HOLD_JOG BYTE0=0_OR_84\r\n");
}

uint8_t ArmTuner_IsSessionActive(void)
{
    return (uint8_t)(session || ArmTrimBluetooth_OwnsMotion() || ArmTrimBench_IsActive());
}
