#include "car_control.h"
#include "bluetooth_driver.h"
#include "motor_driver.h"
#include "serial_io.h"
#include "board_app.h"
#include "mecanum.h"
#include "mecanum_test.h"
#include "heading_control.h"
#include "turn_right.h"
#include "turn_config.h"
#include "maxicam.h"
#include "vision_config.h"
#include "laser.h"
#include "laser_config.h"
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include <math.h>

static CarState_t state;
static uint8_t previous_disable, previous_stop, powered, recovery_center;
static uint8_t boot_enable_pending, boot_wait_first_packet;
static uint8_t prev_right_90, prev_right_180;
static uint8_t pending_right_90, pending_right_180;
static uint8_t active_right_90, active_right_180;
static uint8_t prev_left_90, pending_left_90, active_left_90;
static uint8_t turn_wait_logged;
static uint8_t button_was_moving;
static uint8_t turn_conflict_logged, move_conflict_logged;
static uint32_t last_sequence, output_tick, release_sequence;
static uint8_t local_low;
static uint32_t local_change_tick;
static int16_t log_vx, log_vy, log_rotation;
static int8_t previous_vx_sign, previous_vy_sign;
static uint8_t heading_updated;
static const char *logged_route;
static uint32_t route_log_tick;
static uint32_t control_diag_ms;
static uint8_t vision_active, vision_is_shot, vision_from_bt, vision_enable_queued;
static uint8_t vision_button_raw, vision_button_stable, vision_fault_rearm;
static uint8_t shot_button_raw, shot_button_stable;
static uint8_t shot_request_pending;
static int16_t previous_cam_t, previous_shot;
static int8_t vision_direction, vision_motion;
static uint32_t vision_button_tick, vision_sequence, vision_frame_tick;
static uint32_t shot_button_tick, shot_fire_tick;
static uint32_t vision_settle_tick, vision_sample_tick;
static int16_t vision_samples[VISION_STABLE_FRAMES];
static uint8_t vision_has_target, vision_settle_started, vision_sample_count;
volatile uint32_t heading_test_action, heading_test_request;
static uint32_t test_seen;

static void Transition(CarState_t next)
{
    static const char *const names[] = {
        "OFF", "WAIT_CENTER", "READY", "RUNNING", "BRAKE_LOCK",
        "LINK_LOST", "FAULT", "LOCAL_STARTING", "TURNING",
        "VISION_STARTING", "VISION_TRACKING", "SHOT_ALIGNING", "SHOT_FIRING"
    };
    char line[80];
    if (state == next) return;
    (void)snprintf(line, sizeof(line), "[CAR] %s -> %s\r\n", names[state], names[next]);
    Debug_Log(line);
    state = next;
}

static void LogControl(const char *route)
{
    char line[64];
    uint32_t now = HAL_GetTick();
    if (logged_route && strcmp(route, logged_route) == 0 &&
        (uint32_t)(now - route_log_tick) < 1000U) return;
    logged_route = route;
    route_log_tick = now;
    (void)snprintf(line, sizeof(line), "BT CONTROL: %s\r\n", route);
    Debug_Log(line);
}

static void LogInputEvent(const char *event, const BluetoothControlFrame *c)
{
    char line[80];
    uint32_t age = HAL_GetTick() - Bluetooth_GetLastRxTick();
    (void)snprintf(line, sizeof(line), "[CTRL] %s seq=%lu age=%lu state=%u\r\n",
                   event, (unsigned long)Bluetooth_GetSequence(),
                   (unsigned long)age, (unsigned)state);
    Debug_Log(line);
    (void)snprintf(line, sizeof(line),
                   "[CTRL] F=%u B=%u L=%u R=%u STOP=%u X=%d Y=%d\r\n",
                   (unsigned)c->forward, (unsigned)c->backward,
                   (unsigned)c->strafe_left, (unsigned)c->strafe_right,
                   (unsigned)c->stop, c->joy_x, c->joy_y);
    Debug_Log(line);
}

static long ScaledHundred(float value)
{
    if (!isfinite(value)) return LONG_MAX;
    if (value >= (float)(LONG_MAX / 100L)) return LONG_MAX;
    if (value <= (float)(LONG_MIN / 100L)) return LONG_MIN;
    return (long)(value * 100.0f);
}

static void LogControlDiagnostics(const BluetoothControlFrame *c)
{
    char line[80];
    const HeadingPIDParameters *pid = Heading_GetPID();
    const HeadingStatus *heading = Heading_GetStatus();
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - control_diag_ms) < 1000U || !Debug_CanLog(2U)) return;
    control_diag_ms = now;
    (void)snprintf(line, sizeof(line),
                   "[CTRL] st=%u seq=%lu age=%lu FBLR=%u%u%u%u S=%u X=%d Y=%d\r\n",
                   (unsigned)state, (unsigned long)Bluetooth_GetSequence(),
                   (unsigned long)(now - Bluetooth_GetLastRxTick()),
                   (unsigned)c->forward, (unsigned)c->backward,
                   (unsigned)c->strafe_left, (unsigned)c->strafe_right,
                   (unsigned)c->stop, c->joy_x, c->joy_y);
    Debug_Log(line);
    (void)snprintf(line, sizeof(line),
                   "[CTRL] P100=%ld I100=%ld D100=%ld err100=%ld om=%d\r\n",
                   ScaledHundred(pid->kp), ScaledHundred(pid->ki),
                   ScaledHundred(pid->kd),
                   ScaledHundred(heading->yaw_error), log_rotation);
    Debug_Log(line);
}

static int16_t JoystickRPM(int16_t value, int16_t maximum_rpm)
{
    if (value >= -JOY_DEADZONE && value <= JOY_DEADZONE) return 0;
    return (int16_t)((int32_t)value * maximum_rpm / JOY_RANGE);
}

static int8_t Sign(int16_t value)
{
    return value > 0 ? 1 : (value < 0 ? -1 : 0);
}

static uint8_t Centered(const BluetoothControlFrame *c)
{
    return (uint8_t)(c->joy_x >= -JOY_DEADZONE && c->joy_x <= JOY_DEADZONE &&
                     c->joy_y >= -JOY_DEADZONE && c->joy_y <= JOY_DEADZONE);
}

static uint8_t ButtonsReleased(const BluetoothControlFrame *c)
{
    return (uint8_t)(!c->forward && !c->backward && !c->strafe_left &&
                     !c->strafe_right && !c->right_90 && !c->right_180 &&
                     !c->left_90 &&
                     !c->stop && !c->brake && !c->disable);
}

static void ResetHeading(void)
{
    (void)Heading_Update(false);
    Heading_ClearReference();
    heading_updated = 1U;
    previous_vx_sign = previous_vy_sign = 0;
    log_rotation = 0;
}

static int16_t ManualHeadingCorrection(int16_t vx, int16_t vy)
{
    int8_t vx_sign = Sign(vx), vy_sign = Sign(vy);
    int16_t correction;
    int32_t x_magnitude = vx < 0 ? -(int32_t)vx : (int32_t)vx;
    int32_t y_magnitude = vy < 0 ? -(int32_t)vy : (int32_t)vy;
    int32_t maximum = x_magnitude > y_magnitude ? x_magnitude : y_magnitude;
    int32_t limit = maximum * MANUAL_HEADING_LIMIT_PERCENT / 100;
    if (maximum != 0 && limit == 0) limit = 1;
    if (vx_sign != previous_vx_sign || vy_sign != previous_vy_sign)
        (void)Heading_Update(false);
    previous_vx_sign = vx_sign;
    previous_vy_sign = vy_sign;
    heading_updated = 1U;
    correction = CAR_YAW_HOLD_ENABLE ? Heading_Update(true) : Heading_Update(false);
    if (correction > limit) correction = (int16_t)limit;
    if (correction < -limit) correction = (int16_t)-limit;
    return correction;
}

static void ClearTurn(void)
{
    TurnRight_Cancel();
    pending_right_90 = pending_right_180 = 0U;
    active_right_90 = active_right_180 = 0U;
    pending_left_90 = active_left_90 = 0U;
    turn_wait_logged = 0U;
}

static void NormalStop(void)
{
    if (state == CAR_RUNNING || state == CAR_TURNING) (void)brake();
    ResetHeading();
    button_was_moving = 0U;
    turn_conflict_logged = move_conflict_logged = 0U;
    if (state == CAR_RUNNING || state == CAR_TURNING) Transition(CAR_READY);
    output_tick = HAL_GetTick();
}

static void VisionExit(CarState_t next)
{
    if (!vision_active) return;
    Laser_Disable();
    vision_active = vision_is_shot = vision_from_bt = vision_enable_queued = 0U;
    vision_settle_started = vision_sample_count = 0U;
    vision_has_target = 0U;
    vision_direction = vision_motion = 0;
    ClearTurn();
    (void)Motor_EStopAll();
    ResetHeading();
    if (next == CAR_OFF || next == CAR_FAULT) {
        (void)Motor_DisableAll();
        powered = 0U;
    }
    Transition(next);
    output_tick = HAL_GetTick();
    Debug_Log("[VISION] exit\r\n");
}

static HAL_StatusTypeDef VisionEnter(uint8_t from_bt, uint8_t is_shot,
                                     const BluetoothControlFrame *c)
{
    uint32_t sequence;
    HAL_StatusTypeDef status;
    if (vision_active && vision_is_shot == is_shot) return HAL_OK;
    if (Motor_HasFault() || state == CAR_FAULT ||
        state == CAR_BRAKE_LOCK || vision_fault_rearm ||
        (Bluetooth_IsConnected() && (c->stop || c->brake || c->disable))) return HAL_ERROR;
    if (from_bt && !Bluetooth_IsConnected()) return HAL_ERROR;
    if (vision_active)
        VisionExit(Bluetooth_IsConnected() ? CAR_WAIT_CENTER : CAR_OFF);
    ClearTurn();
    (void)Motor_EStopAll();
    Laser_Disable();
    ResetHeading();
    boot_enable_pending = boot_wait_first_packet = 0U;
    button_was_moving = 0U;
    status = MaxiCam_SendMode(is_shot ? MODE_CMD_AIM : MODE_CMD_OBJECT);
    if (status != HAL_OK) {
        if (Bluetooth_IsConnected()) Transition(CAR_WAIT_CENTER);
        else {
            (void)Motor_DisableAll();
            powered = 0U;
            Transition(CAR_OFF);
        }
        Debug_Log("[VISION] camera mode TX failed\r\n");
        return status;
    }
    vision_active = 1U;
    vision_is_shot = is_shot;
    vision_from_bt = from_bt;
    vision_enable_queued = vision_has_target = 0U;
    vision_settle_started = vision_sample_count = 0U;
    vision_direction = vision_motion = 0;
    MaxiCam_GetTargetData(NULL, &sequence);
    vision_sequence = sequence; /* Reject frames from before the entry request. */
    vision_frame_tick = HAL_GetTick();
    Transition(CAR_VISION_STARTING);
    Debug_Log(is_shot ? "[SHOT] enter\r\n" : "[VISION] enter\r\n");
    return HAL_OK;
}

static void VisionButtonProcess(const BluetoothControlFrame *c)
{
    uint8_t low = Board_VisionButtonIsLow();
    uint32_t now = HAL_GetTick();
    if (low != vision_button_raw) {
        vision_button_raw = low;
        vision_button_tick = now;
    }
    if (low == vision_button_stable ||
        (uint32_t)(now - vision_button_tick) < VISION_BUTTON_DEBOUNCE_MS) return;
    vision_button_stable = low;
    if (!low) return;
    if (vision_active && !vision_is_shot)
        VisionExit(Bluetooth_IsConnected() ? CAR_WAIT_CENTER : CAR_OFF);
    else (void)VisionEnter(0U, 0U, c);
}

static void ShotButtonProcess(const BluetoothControlFrame *c)
{
    uint8_t low = Board_ShotButtonIsLow();
    uint32_t now = HAL_GetTick();
    if (low != shot_button_raw) {
        shot_button_raw = low;
        shot_button_tick = now;
    }
    if (low == shot_button_stable ||
        (uint32_t)(now - shot_button_tick) < SHOT_BUTTON_DEBOUNCE_MS) return;
    shot_button_stable = low;
    if (!low) return;
    if (vision_active && vision_is_shot)
        VisionExit(Bluetooth_IsConnected() ? CAR_WAIT_CENTER : CAR_OFF);
    else (void)VisionEnter(0U, 1U, c);
}

static int8_t VisionFrameDirection(int16_t offset)
{
    int16_t maximum = vision_is_shot ? MAXICAM_SHOT_MAX_X : MAXICAM_ALIGN_MAX_X;
    int16_t minimum = vision_is_shot ? MAXICAM_SHOT_MIN_X : MAXICAM_ALIGN_MIN_X;
    if (offset > maximum) return 1;
    if (offset < minimum) return -1;
    return 0;
}

static void VisionStopMotion(void)
{
    if (vision_motion != 0) (void)Motor_EStopAll();
    vision_motion = 0;
    vision_settle_started = vision_sample_count = 0U;
}

/* Only samples taken after the mechanical settling interval count. */
static int8_t VisionSampleDecision(int16_t offset, uint32_t now)
{
    int16_t minimum, maximum;
    uint8_t i;
    int8_t direction;
    if (vision_sample_count != 0U &&
        (uint32_t)(now - vision_sample_tick) < VISION_SAMPLE_INTERVAL_MS) return 0;
    vision_samples[vision_sample_count++] = offset;
    vision_sample_tick = now;
    if (vision_sample_count < VISION_STABLE_FRAMES) return 0;
    minimum = maximum = vision_samples[0];
    direction = VisionFrameDirection(vision_samples[0]);
    for (i = 1U; i < VISION_STABLE_FRAMES; ++i)
    {
        int8_t next = VisionFrameDirection(vision_samples[i]);
        if (vision_samples[i] < minimum) minimum = vision_samples[i];
        if (vision_samples[i] > maximum) maximum = vision_samples[i];
        if (next != direction) direction = 0;
    }
    vision_sample_count = 0U;
    if (direction != 0) return direction;
    if (VisionFrameDirection(minimum) == 0 &&
        VisionFrameDirection(maximum) == 0 &&
        (int32_t)maximum - minimum <= VISION_STABLE_SPREAD_X) return 2;
    return 0;
}

static void VisionProcess(const BluetoothControlFrame *c)
{
    MaxiCamTargetData_t target;
    uint32_t sequence, now = HAL_GetTick();
    int8_t decision = 0;
    uint8_t new_frame;
    HAL_StatusTypeDef result;
    if (Motor_HasFault()) {
        VisionExit(CAR_FAULT);
        vision_fault_rearm = 1U;
        return;
    }
    if (Bluetooth_IsConnected() && (c->stop || c->brake || c->disable)) {
        VisionExit(c->disable ? CAR_OFF : CAR_BRAKE_LOCK);
        return;
    }
    if (vision_from_bt && !Bluetooth_IsConnected()) {
        VisionExit(CAR_LINK_LOST);
        return;
    }
    MaxiCam_GetTargetData(&target, &sequence);
    new_frame = (uint8_t)(sequence != vision_sequence);
    if (new_frame) {
        vision_sequence = sequence;
        vision_frame_tick = now;
        vision_has_target = (uint8_t)(target.target_valid &&
            (!vision_is_shot || target.type == (uint8_t)DETECT_TARGET));
        vision_direction = vision_has_target ? VisionFrameDirection(target.offset_x) : 0;
        if (state == CAR_SHOT_FIRING && (!vision_has_target || vision_direction != 0)) {
            VisionExit(Bluetooth_IsConnected() ? CAR_WAIT_CENTER : CAR_OFF);
            return;
        }
        if (!vision_has_target) VisionStopMotion();
        else if (vision_motion != 0 && vision_direction != vision_motion)
            VisionStopMotion();
    }
    if (vision_has_target &&
        (uint32_t)(now - vision_frame_tick) >= VISION_FRAME_TIMEOUT_MS) {
        if (vision_is_shot && state == CAR_SHOT_FIRING) {
            VisionExit(Bluetooth_IsConnected() ? CAR_WAIT_CENTER : CAR_OFF);
            return;
        }
        vision_has_target = 0U;
        vision_direction = 0;
        VisionStopMotion();
    }
    if (state == CAR_VISION_STARTING) {
        if (!Motor_IsIdle()) return;
        if (!vision_enable_queued) {
            result = Motor_EnableAll();
            if (result == HAL_OK) {
                vision_enable_queued = 1U;
                powered = 1U;
            } else if (result != HAL_BUSY) {
                VisionExit(CAR_FAULT);
                vision_fault_rearm = 1U;
            }
            return;
        }
        Transition(vision_is_shot ? CAR_SHOT_ALIGNING : CAR_VISION_TRACKING);
    }
    if (state == CAR_SHOT_FIRING) {
        if ((uint32_t)(now - shot_fire_tick) >= LASER_HOLD_MS)
            VisionExit(Bluetooth_IsConnected() ? CAR_WAIT_CENTER : CAR_OFF);
        return;
    }
    if (vision_motion != 0 || !vision_has_target) return;
    if (!Motor_IsIdle()) {
        vision_settle_started = vision_sample_count = 0U;
        return;
    }
    if (!vision_settle_started) {
        vision_settle_started = 1U;
        vision_settle_tick = now;
        return;
    }
    if ((uint32_t)(now - vision_settle_tick) < VISION_SETTLE_MS || !new_frame) return;
    /* The current frame was accepted above; never reuse it on later loops. */
    decision = VisionSampleDecision(target.offset_x, now);
    if (decision == 0) return;
    if (decision == 2) {
        if (vision_is_shot) {
            Laser_Enable();
            shot_fire_tick = now;
            Transition(CAR_SHOT_FIRING);
        }
        return;
    }
    result = mecanum_drive((int16_t)(decision * VISION_FOLLOW_RPM), 0, 0);
    if (result == HAL_OK) {
        vision_motion = decision;
        vision_settle_started = vision_sample_count = 0U;
        output_tick = now;
        log_vx = (int16_t)(decision * VISION_FOLLOW_RPM);
    } else if (result != HAL_BUSY) {
        VisionExit(CAR_FAULT);
        vision_fault_rearm = 1U;
    }
}

void Car_Control_Init(void)
{
    state = CAR_OFF;
    previous_disable = previous_stop = powered = recovery_center = 0U;
    prev_right_90 = prev_right_180 = 0U;
    pending_right_90 = pending_right_180 = 0U;
    active_right_90 = active_right_180 = 0U;
    prev_left_90 = pending_left_90 = active_left_90 = 0U;
    turn_wait_logged = 0U;
    button_was_moving = 0U;
    last_sequence = Bluetooth_GetSequence();
    output_tick = HAL_GetTick();
    release_sequence = last_sequence;
    local_low = 0U;
    local_change_tick = output_tick;
    Mecanum_Test_Init();
    Heading_Init();
    log_vx = log_vy = log_rotation = 0;
    previous_vx_sign = previous_vy_sign = 0;
    heading_updated = 0U;
    logged_route = NULL;
    route_log_tick = HAL_GetTick() - 1000U;
    control_diag_ms = HAL_GetTick();
    vision_active = vision_is_shot = vision_from_bt = vision_enable_queued = 0U;
    vision_fault_rearm = vision_has_target = 0U;
    vision_button_raw = vision_button_stable = Board_VisionButtonIsLow();
    vision_button_tick = HAL_GetTick();
    shot_button_raw = shot_button_stable = Board_ShotButtonIsLow();
    shot_button_tick = HAL_GetTick();
    shot_request_pending = 0U;
    previous_cam_t = previous_shot = 0;
    vision_direction = vision_motion = 0;
    vision_sequence = vision_frame_tick = 0U;
    vision_settle_started = vision_sample_count = 0U;
    vision_settle_tick = vision_sample_tick = shot_fire_tick = 0U;
    heading_test_action = heading_test_request = test_seen = 0U;
    Debug_Log("[CAR] OFF\r\n");
    boot_enable_pending = (uint8_t)(CAR_BOOT_AUTO_ENABLE && !CAR_PD10_STANDALONE_TEST);
    boot_wait_first_packet = boot_enable_pending;
    if (boot_enable_pending) Transition(CAR_WAIT_CENTER);
}

CarState_t Car_Control_GetState(void) { return state; }

static uint8_t AngleCommandAllowed(void)
{
    const BtControl_t *control = Bluetooth_GetControl();
    const BluetoothControlFrame *c = &control->frame;
    return (uint8_t)((state == CAR_READY || state == CAR_RUNNING) &&
        Bluetooth_IsConnected() && !c->stop && !c->brake && !c->disable &&
        !Motor_HasFault() && !CAR_PD10_STANDALONE_TEST && !CAR_MECANUM_TEST_MODE);
}

HAL_StatusTypeDef Car_Control_SetTargetYaw(float target_degrees)
{
    return AngleCommandAllowed() && Heading_SetTarget(target_degrees) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef Car_Control_AdjustTargetYaw(float delta_degrees)
{
    return AngleCommandAllowed() && Heading_AdjustTarget(delta_degrees) ? HAL_OK : HAL_ERROR;
}

static void Local_Process(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t low = Board_PD10IsLow();
    if (low != local_low) { local_low = low; local_change_tick = now; }
    if (Motor_HasFault() && state != CAR_FAULT) {
        (void)Motor_EStopAll();
        (void)Motor_DisableAll();
        Transition(CAR_FAULT);
        return;
    }
    if (state == CAR_FAULT) {
        if (!low && Motor_IsIdle()) { Motor_ClearFault(); Transition(CAR_OFF); }
        return;
    }
    if (!low) {
        if (state != CAR_OFF) {
            (void)Motor_EStopAll();
            (void)Motor_DisableAll();
            Transition(CAR_OFF);
        }
        return;
    }
    if ((uint32_t)(now - local_change_tick) < CAR_PD10_DEBOUNCE_MS) return;
    if (state == CAR_OFF) {
        if (Motor_EnableAll() == HAL_OK) Transition(CAR_LOCAL_STARTING);
        return;
    }
    if (state == CAR_LOCAL_STARTING ||
        (uint32_t)(now - output_tick) >= CAR_CONTROL_PERIOD_MS) {
        if (mecanum_drive(CAR_PD10_FORWARD_RPM, 0, 0) == HAL_OK) {
            output_tick = now;
            Transition(CAR_RUNNING);
        }
    }
}

static uint8_t BluetoothSafety(const BluetoothControlFrame *c, uint8_t fresh)
{
    const BtControl_t *control = Bluetooth_GetControl();
    uint32_t now = HAL_GetTick();
    uint8_t disable_edge = (uint8_t)(fresh && c->disable && !previous_disable);
    if (fresh) previous_disable = (uint8_t)c->disable;
    if (boot_wait_first_packet && Motor_HasFault()) {
        boot_enable_pending = boot_wait_first_packet = 0U;
        (void)Motor_EStopAll();
        Transition(CAR_FAULT);
        return 1U;
    }
    if (boot_enable_pending && !(control->valid && (c->stop || c->brake || c->disable)) &&
        Motor_EnableAll() == HAL_OK) {
        boot_enable_pending = 0U;
        powered = 1U;
        Debug_Log("[CAR] POWER-ON ENABLE\r\n");
    }
    if (!Bluetooth_IsConnected()) {
        if (boot_wait_first_packet && !control->valid) return 1U;
        boot_enable_pending = boot_wait_first_packet = 0U;
        if (state != CAR_OFF && state != CAR_LINK_LOST && state != CAR_FAULT) {
            LogInputEvent("LINK_LOST", c);
            ClearTurn();
            (void)Motor_EStopAll();
            ResetHeading();
            Transition(CAR_LINK_LOST);
            Debug_Log("[BT] link lost\r\n");
        }
        recovery_center = 0U;
        return 1U;
    }
    boot_wait_first_packet = 0U;
    if (disable_edge) {
        boot_enable_pending = 0U;
        ClearTurn();
        (void)Motor_EStopAll();
        (void)Motor_DisableAll();
        ResetHeading();
        button_was_moving = 0U;
        powered = recovery_center = 0U;
        Transition(CAR_OFF);
        LogControl("DISABLE");
        return 1U;
    }
    if (c->disable) return 1U;
    if (Motor_HasFault() && state != CAR_FAULT) {
        ClearTurn();
        (void)Motor_EStopAll();
        ResetHeading();
        Transition(CAR_FAULT);
        recovery_center = 0U;
        return 1U;
    }
    if (state == CAR_OFF || state == CAR_LINK_LOST || state == CAR_FAULT) {
        if (fresh && ButtonsReleased(c) && Centered(c)) {
            if (recovery_center && Motor_IsIdle()) {
                Motor_ClearFault();
                if (Motor_EnableAll() == HAL_OK) {
                    powered = 1U;
                    Transition(CAR_WAIT_CENTER);
                    recovery_center = 0U;
                }
            } else recovery_center = 1U;
        } else if (fresh) recovery_center = 0U;
        return 1U;
    }
    if (c->stop || c->brake) {
        boot_enable_pending = 0U;
        recovery_center = 0U;
        if (state != CAR_BRAKE_LOCK) {
            ClearTurn();
            (void)Motor_EStopAll();
            ResetHeading();
            button_was_moving = 0U;
            Transition(CAR_BRAKE_LOCK);
        }
        LogControl("BRAKE");
        return 1U;
    }
    if (state == CAR_BRAKE_LOCK) {
        if (fresh && ButtonsReleased(c) && Centered(c) && Motor_IsIdle()) {
            Transition(powered ? CAR_READY : CAR_OFF);
            output_tick = now;
            Debug_Log("[CAR] BRAKE LOCK RELEASED\r\n");
        }
        return 1U;
    }
    if (state == CAR_WAIT_CENTER) {
        if (fresh && ButtonsReleased(c) && Centered(c) && Motor_IsIdle()) {
            Transition(CAR_READY);
            output_tick = now;
        }
        return 1U;
    }
    return 0U;
}

static uint8_t ProcessTurn(const BluetoothControlFrame *c, uint8_t edge_90,
                           uint8_t edge_180, uint8_t edge_left)
{
    HAL_StatusTypeDef result;
    const TurnRightStatus_t *turn;
    if ((c->right_90 && c->right_180) ||
        (c->left_90 && (c->right_90 || c->right_180))) {
        ClearTurn();
        NormalStop();
        if (!turn_conflict_logged) Debug_Log("BT TURN CONFLICT\r\n");
        turn_conflict_logged = 1U;
        LogControl("CONFLICT");
        return 1U;
    }
    if ((pending_right_90 && (edge_180 || edge_left)) ||
        (pending_right_180 && (edge_90 || edge_left)) ||
        (pending_left_90 && (edge_90 || edge_180)) ||
        (active_left_90 && (edge_90 || edge_180)) ||
        ((active_right_90 || active_right_180) && edge_left)) {
        ClearTurn();
        NormalStop();
        Debug_Log("BT TURN CONFLICT\r\n");
        LogControl("CONFLICT");
        return 1U;
    }
    turn_conflict_logged = 0U;
    if (active_right_90 || active_right_180 || active_left_90) {
        TurnRight_Process(true);
        turn = TurnRight_GetStatus();
        if (turn->state == TURN_RIGHT_DONE) {
            ClearTurn();
            NormalStop();
            LogControl("BRAKE");
        } else if (turn->state == TURN_RIGHT_FAULT ||
                   turn->state == TURN_RIGHT_CANCELLED) {
            ClearTurn();
            (void)Motor_EStopAll();
            ResetHeading();
            Transition(CAR_BRAKE_LOCK);
        }
        return 1U;
    }
    if (edge_90) pending_right_90 = 1U;
    if (edge_180) pending_right_180 = 1U;
    if (edge_left) pending_left_90 = 1U;
    if (!pending_right_90 && !pending_right_180 && !pending_left_90)
        return (uint8_t)(c->right_90 || c->right_180 || c->left_90);
    if (state == CAR_RUNNING) NormalStop();
    if (!Motor_IsIdle()) {
        if (!turn_wait_logged) Debug_Log("[TURN] WAIT motor_busy\r\n");
        turn_wait_logged = 1U;
        return 1U;
    }
    turn_wait_logged = 0U;
    result = pending_right_90 ? right90(TURN_RIGHT_90_RPM) :
             pending_right_180 ? right180(TURN_RIGHT_180_RPM) :
                                 left90(TURN_LEFT_90_RPM);
    if (result == HAL_BUSY) return 1U;
    if (result != HAL_OK) {
        pending_right_90 = pending_right_180 = pending_left_90 = 0U;
        NormalStop();
        LogControl("BRAKE");
        return 1U;
    }
    active_right_90 = pending_right_90;
    active_right_180 = pending_right_180;
    active_left_90 = pending_left_90;
    pending_right_90 = pending_right_180 = pending_left_90 = 0U;
    ResetHeading();
    Transition(CAR_TURNING);
    LogControl(active_left_90 ? "LEFT90" :
               active_right_90 ? "RIGHT90" : "RIGHT180");
    return 1U;
}

static uint8_t ProcessButtons(const BluetoothControlFrame *c, uint8_t fresh)
{
    uint8_t count = (uint8_t)(c->forward + c->backward + c->strafe_left + c->strafe_right);
    int16_t vx = 0, vy = 0, yaw_correction;
    if (count > 1U) {
        NormalStop();
        if (!move_conflict_logged) Debug_Log("BT CONTROL CONFLICT\r\n");
        move_conflict_logged = 1U;
        LogControl("CONFLICT");
        return 1U;
    }
    move_conflict_logged = 0U;
    if (count == 0U) {
        if (button_was_moving) {
            LogInputEvent("BUTTON_RELEASE", c);
            NormalStop();
            LogControl("BRAKE");
            release_sequence = Bluetooth_GetSequence();
            return 1U;
        }
        if (Bluetooth_GetSequence() == release_sequence) return 1U;
        return 0U;
    }
    if (CAR_MECANUM_TEST_MODE) return 1U;
    button_was_moving = 1U;
    if (c->forward) { vx = BLUETOOTH_TEST_MOVE_RPM; LogControl("FORWARD"); }
    else if (c->backward) { vx = -BLUETOOTH_TEST_MOVE_RPM; LogControl("BACKWARD"); }
    else if (c->strafe_left) { vy = -BLUETOOTH_TEST_MOVE_RPM; LogControl("STRAFE_LEFT"); }
    else { vy = BLUETOOTH_TEST_MOVE_RPM; LogControl("STRAFE_RIGHT"); }
    yaw_correction = ManualHeadingCorrection(vx, vy);
    log_vx = vx;
    log_vy = vy;
    if ((uint32_t)(HAL_GetTick() - output_tick) >= CAR_CONTROL_PERIOD_MS &&
        mecanum_drive(vx, vy, yaw_correction) == HAL_OK) {
        output_tick = HAL_GetTick();
        log_rotation = yaw_correction;
        Transition(CAR_RUNNING);
    }
    (void)fresh;
    return 1U;
}

static void ProcessJoystick(const BluetoothControlFrame *c)
{
    int16_t translation_limit = CAR_HEADING_TEST_MODE ? HEADING_TEST_TRANSLATION_RPM : MOTOR_MAX_RPM;
    int16_t vx = JoystickRPM(c->joy_y, translation_limit);
    int16_t vy = JoystickRPM(c->joy_x, translation_limit);
    int16_t yaw_correction;
    if (CAR_HEADING_TEST_MODE) {
        int32_t sum = (vx < 0 ? -(int32_t)vx : vx) + (vy < 0 ? -(int32_t)vy : vy);
        if (sum > HEADING_TEST_TRANSLATION_RPM) {
            vx = (int16_t)((int32_t)vx * HEADING_TEST_TRANSLATION_RPM / sum);
            vy = (int16_t)((int32_t)vy * HEADING_TEST_TRANSLATION_RPM / sum);
        }
        if (test_seen != heading_test_request) {
            test_seen = heading_test_request;
            if (heading_test_action == 0U || heading_test_action > 3U) {
                (void)Motor_EStopAll();
                ResetHeading();
                Transition(CAR_BRAKE_LOCK);
                return;
            }
            if (heading_test_action == 3U) {
                if (!Heading_RequestReference()) Debug_Log("[TEST] reference rejected\r\n");
            } else {
                float step = heading_test_action == 1U ? ANGLE_ADJUST_STEP_DEG : -ANGLE_ADJUST_STEP_DEG;
                if (Car_Control_AdjustTargetYaw(step) != HAL_OK)
                    Debug_Log("[TEST] angle rejected\r\n");
            }
        }
    }
    if (vx == 0 && vy == 0) {
        if (state == CAR_RUNNING) {
            LogInputEvent("JOYSTICK_CENTER", c);
            NormalStop(); LogControl("BRAKE");
        }
        else { ResetHeading(); LogControl("BRAKE"); }
        return;
    }
    LogControl("JOYSTICK");
    yaw_correction = ManualHeadingCorrection(vx, vy);
    log_vx = vx;
    log_vy = vy;
    if ((uint32_t)(HAL_GetTick() - output_tick) >= CAR_CONTROL_PERIOD_MS &&
        mecanum_drive(vx, vy, yaw_correction) == HAL_OK) {
        output_tick = HAL_GetTick();
        log_rotation = yaw_correction;
        Transition(CAR_RUNNING);
    }
}

void Car_Control_Process(void)
{
    const BtControl_t *control;
    const BluetoothControlFrame *c;
    uint32_t sequence;
    uint8_t fresh, edge_90, edge_180, edge_left, allow, was_vision_active;
    heading_updated = 0U;
    log_vx = log_vy = 0;
    if (CAR_PD10_STANDALONE_TEST) {
        Local_Process();
        goto finish;
    }
    control = Bluetooth_GetControl();
    c = &control->frame;
    sequence = Bluetooth_GetSequence();
    fresh = (uint8_t)(sequence != last_sequence);
    if (fresh && c->stop && !previous_stop) LogInputEvent("STOP", c);
    if (fresh) previous_stop = (uint8_t)c->stop;
    edge_90 = (uint8_t)(fresh && c->right_90 && !prev_right_90);
    edge_180 = (uint8_t)(fresh && c->right_180 && !prev_right_180);
    edge_left = (uint8_t)(fresh && c->left_90 && !prev_left_90);
    if (fresh) {
        last_sequence = sequence;
        prev_right_90 = (uint8_t)c->right_90;
        prev_right_180 = (uint8_t)c->right_180;
        prev_left_90 = (uint8_t)c->left_90;
    }
    was_vision_active = vision_active;
    VisionButtonProcess(c);
    ShotButtonProcess(c);
    if (vision_fault_rearm && !vision_button_stable &&
        !shot_button_stable && Motor_IsIdle()) {
        Motor_ClearFault();
        vision_fault_rearm = 0U;
        Transition(CAR_OFF);
    }
    if (fresh) {
        uint8_t cam_changed = (uint8_t)(c->Cam_T != previous_cam_t);
        uint8_t shot_changed = (uint8_t)(c->Shot != previous_shot);
        uint8_t shot_exited = 0U;
        previous_cam_t = c->Cam_T;
        previous_shot = c->Shot;
        if (shot_changed) shot_request_pending = (uint8_t)(c->Shot != 0);
        if (vision_active && vision_from_bt && vision_is_shot && !c->Shot) {
            VisionExit(Bluetooth_IsConnected() ? CAR_WAIT_CENTER : CAR_OFF);
            shot_exited = 1U;
        } else if (vision_active && vision_from_bt && !vision_is_shot &&
                   cam_changed && !c->Cam_T) {
            VisionExit(Bluetooth_IsConnected() ? CAR_WAIT_CENTER : CAR_OFF);
        }
        if (!shot_exited && !c->Shot && cam_changed && c->Cam_T)
            (void)VisionEnter(1U, 0U, c);
    }
    if (shot_request_pending && !Bluetooth_IsConnected())
        shot_request_pending = 0U;
    if (shot_request_pending && fresh) {
        if (!Bluetooth_IsConnected() || c->stop || c->brake || c->disable ||
            Motor_HasFault() || state == CAR_FAULT || state == CAR_BRAKE_LOCK ||
            vision_fault_rearm)
            shot_request_pending = 0U;
        else if (VisionEnter(1U, 1U, c) != HAL_BUSY)
            shot_request_pending = 0U;
    }
    if (vision_active) {
        VisionProcess(c);
        goto finish;
    }
    if (was_vision_active) goto finish;
    if (BluetoothSafety(c, fresh)) {
        if (CAR_MECANUM_TEST_MODE) Mecanum_Test_Process(0U);
        goto finish;
    }
    if (state != CAR_READY && state != CAR_RUNNING && state != CAR_TURNING) goto finish;
    if (CAR_MECANUM_TEST_MODE && !ButtonsReleased(c)) {
        Mecanum_Test_Process(0U);
        if (state == CAR_RUNNING) NormalStop();
        goto finish;
    }
    if (ProcessTurn(c, edge_90, edge_180, edge_left)) goto finish;
    if (ProcessButtons(c, fresh)) goto finish;
    if (CAR_MECANUM_TEST_MODE) {
        allow = (uint8_t)(powered && Bluetooth_IsConnected() &&
            !Motor_HasFault() && Centered(c));
        if (!allow && mecanum_test_active && (state == CAR_READY || state == CAR_RUNNING))
            (void)Motor_EStopAll();
        Mecanum_Test_Process(allow);
        Transition(mecanum_test_active ? CAR_RUNNING : CAR_READY);
    } else if (ButtonsReleased(c)) ProcessJoystick(c);
finish:
    if (!heading_updated) {
        (void)Heading_Update(false);
        test_seen = heading_test_request;
    }
    if (!CAR_PD10_STANDALONE_TEST)
        LogControlDiagnostics(&Bluetooth_GetControl()->frame);
    Heading_Log(log_vx, log_vy, log_rotation);
}
