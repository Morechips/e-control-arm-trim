#include "car_control.h"
#include "motor_driver.h"
#include "serial_io.h"
#include "mecanum.h"
#include "mecanum_test.h"
#include "heading_control.h"
#include "turn_right.h"
#include "turn_config.h"
#include "remote_heading.h"
#include "maxicam.h"
#include "vision_config.h"
#include "laser.h"
#include "laser_config.h"
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include <math.h>

static CarRemoteInput_t remote_input;
static volatile uint8_t remote_invalidated;
static CarLocalInput_t local_input;
void Car_Control_SubmitRemoteInput(const CarRemoteInput_t *input)
{
    uint32_t mask;
    if (input == NULL) return;
    mask = __get_PRIMASK(); __disable_irq();
    remote_input = *input;
    remote_invalidated = (uint8_t)!input->valid;
    __set_PRIMASK(mask);
}
void Car_Control_InvalidateRemoteInput(void) { remote_invalidated = 1U; }
void Car_Control_SubmitLocalInput(const CarLocalInput_t *input)
{
    if (input == NULL) return;
    local_input.vision_held = input->vision_held;
    local_input.shot_held = input->shot_held;
    local_input.vision_press |= input->vision_press;
    local_input.shot_press |= input->shot_press;
    local_input.pd10_low = input->pd10_low;
    local_input.pd10_ready = input->pd10_ready;
}
static uint8_t RemoteValid(void) { return (uint8_t)(remote_input.valid && !remote_invalidated); }
static uint8_t RemoteConnected(void)
{
    return (uint8_t)(RemoteValid() &&
        (uint32_t)(HAL_GetTick() - remote_input.received_tick) <= BT_FAILSAFE_TIMEOUT_MS);
}
static CarState_t state;
static uint8_t previous_disable, previous_stop, powered, recovery_center;
static uint8_t boot_enable_pending, boot_wait_first_packet;
static uint32_t boot_wait_log_tick;
static uint8_t prev_right_90, prev_right_180;
static uint8_t pending_right_90, pending_right_180;
static uint8_t active_right_90, active_right_180;
static uint8_t prev_left_90, pending_left_90, active_left_90;
static uint8_t turn_wait_logged;
static uint8_t button_was_moving;
static uint8_t turn_conflict_logged, move_conflict_logged;
static uint32_t last_sequence, output_tick, release_sequence;
static int16_t log_vx, log_vy, log_rotation;
static uint8_t remote_drive; /* 0=stopped, 1=translation, 2=alignment */
static uint8_t heading_updated;
static const char *logged_route;
static uint32_t route_log_tick;
static uint32_t control_diag_ms;
static uint8_t vision_active, vision_is_shot, vision_from_remote, vision_enable_queued;
static uint8_t vision_fault_rearm;
static uint8_t shot_request_pending;
static int16_t previous_cam_t, previous_shot;
static int8_t vision_direction, vision_motion;
static uint32_t vision_sequence, vision_frame_tick;
static uint32_t shot_fire_tick;
static uint32_t vision_settle_tick, vision_sample_tick;
static int16_t vision_samples[VISION_STABLE_FRAMES];
static uint8_t vision_has_target, vision_settle_started, vision_sample_count;

static void Transition(CarState_t next)
{
    static const char *const names[] = {
        "OFF", "WAIT_CENTER", "READY", "RUNNING", "BRAKE_LOCK",
        "LINK_LOST", "FAULT", "LOCAL_STARTING", "TURNING",
        "VISION_STARTING", "VISION_TRACKING", "SHOT_ALIGNING", "SHOT_FIRING"
    };
    char line[80];
    if (state == next) return;
    if (next == CAR_OFF || next == CAR_LINK_LOST) RemoteHeading_Reset();
    (void)snprintf(line, sizeof(line), "[CAR] %s -> %s\r\n", names[state], names[next]);
    Debug_Log(line);
    state = next;
}

/* Bench instrument for "how long until the remote is usable". */
static void LogReady(void)
{
    char line[48];
    (void)snprintf(line, sizeof(line), "[CAR] READY t=%lu\r\n",
                   (unsigned long)HAL_GetTick());
    Debug_Log(line);
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

static void LogInputEvent(const char *event, const CarCommand_t *c)
{
    char line[80];
    uint32_t age = HAL_GetTick() - remote_input.received_tick;
    (void)snprintf(line, sizeof(line), "[CTRL] %s seq=%lu age=%lu state=%u\r\n",
                   event, (unsigned long)remote_input.sequence,
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

static void LogControlDiagnostics(const CarCommand_t *c)
{
    char line[80];
    const HeadingPIDParameters *pid = Heading_GetPID();
    const HeadingStatus *heading = Heading_GetStatus();
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - control_diag_ms) < 1000U || !Debug_CanLog(2U)) return;
    control_diag_ms = now;
    (void)snprintf(line, sizeof(line),
                   "[CTRL] st=%u seq=%lu age=%lu FBLR=%u%u%u%u S=%u X=%d Y=%d\r\n",
                   (unsigned)state, (unsigned long)remote_input.sequence,
                   (unsigned long)(now - remote_input.received_tick),
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

static uint8_t ButtonsReleased(const CarCommand_t *c)
{
    return (uint8_t)(!c->forward && !c->backward && !c->strafe_left &&
                     !c->strafe_right && !c->right_90 && !c->right_180 &&
                     !c->left_90 &&
                     !c->stop && !c->brake && !c->disable);
}

/* Bench inputs are compiled out of the production image, where the only
 * physical input is the start key. */
static uint8_t PhysicalInputsIdle(void)
{
#if CAR_TEST_INPUTS_ENABLE
    return (uint8_t)(!local_input.vision_held && !local_input.shot_held);
#else
    return 1U;
#endif
}

static void ResetHeading(void)
{
    remote_drive = 0U;
    RemoteHeading_Suspend();
    heading_updated = 1U;
    log_rotation = 0;
}

static void ClearTurn(void)
{
    TurnRight_Cancel();
    if (RemoteHeading_GetStatus()->phase == REMOTE_TURNING) RemoteHeading_Suspend();
    pending_right_90 = pending_right_180 = 0U;
    active_right_90 = active_right_180 = 0U;
    pending_left_90 = active_left_90 = 0U;
    turn_wait_logged = 0U;
}

static void NormalStop(void)
{
    remote_drive = 0U;
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
    RemoteHeading_Reset();
    Laser_Disable();
    vision_active = vision_is_shot = vision_from_remote = vision_enable_queued = 0U;
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
                                     const CarCommand_t *c)
{
    uint32_t sequence;
    if (vision_active && vision_is_shot == is_shot) return HAL_OK;
    if (Motor_HasFault() || state == CAR_FAULT ||
        state == CAR_BRAKE_LOCK || vision_fault_rearm ||
        (RemoteConnected() && (c->stop || c->brake || c->disable))) return HAL_ERROR;
    if (from_bt && !RemoteConnected()) return HAL_ERROR;
    RemoteHeading_Reset();
    if (vision_active)
        VisionExit(RemoteConnected() ? CAR_WAIT_CENTER : CAR_OFF);
    ClearTurn();
    (void)Motor_EStopAll();
    Laser_Disable();
    ResetHeading();
    boot_enable_pending = boot_wait_first_packet = 0U;
    button_was_moving = 0U;
    /* The camera only accepts the mode byte after a QR notification, so the
     * request is recorded here and sent by MaxiCam_Process(). Entering vision
     * mode no longer depends on that transmission succeeding. */
    MaxiCam_RequestMode(is_shot ? MAXICAM_MODE_AIM : MAXICAM_MODE_OBJECT);
    vision_active = 1U;
    vision_is_shot = is_shot;
    vision_from_remote = from_bt;
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

#if CAR_TEST_INPUTS_ENABLE
static void LocalVisionRequests(const CarCommand_t *c)
{
    uint8_t vision_press = local_input.vision_press, shot_press = local_input.shot_press;
    local_input.vision_press = local_input.shot_press = 0U;
    if (vision_press) {
        if (vision_active && !vision_is_shot)
            VisionExit(RemoteConnected() ? CAR_WAIT_CENTER : CAR_OFF);
        else (void)VisionEnter(0U, 0U, c);
    }
    if (shot_press) {
        if (vision_active && vision_is_shot)
            VisionExit(RemoteConnected() ? CAR_WAIT_CENTER : CAR_OFF);
        else (void)VisionEnter(0U, 1U, c);
    }
}
#endif

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

static void VisionProcess(const CarCommand_t *c)
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
    if (RemoteConnected() && (c->stop || c->brake || c->disable)) {
        VisionExit(c->disable ? CAR_OFF : CAR_BRAKE_LOCK);
        return;
    }
    if (vision_from_remote && !RemoteConnected()) {
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
            VisionExit(RemoteConnected() ? CAR_WAIT_CENTER : CAR_OFF);
            return;
        }
        if (!vision_has_target) VisionStopMotion();
        else if (vision_motion != 0 && vision_direction != vision_motion)
            VisionStopMotion();
    }
    if (vision_has_target &&
        (uint32_t)(now - vision_frame_tick) >= VISION_FRAME_TIMEOUT_MS) {
        if (vision_is_shot && state == CAR_SHOT_FIRING) {
            VisionExit(RemoteConnected() ? CAR_WAIT_CENTER : CAR_OFF);
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
            VisionExit(RemoteConnected() ? CAR_WAIT_CENTER : CAR_OFF);
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
    memset(&remote_input, 0, sizeof(remote_input));
    memset(&local_input, 0, sizeof(local_input));
    remote_invalidated = 0U;
    state = CAR_OFF;
    previous_disable = previous_stop = powered = recovery_center = 0U;
    prev_right_90 = prev_right_180 = 0U;
    pending_right_90 = pending_right_180 = 0U;
    active_right_90 = active_right_180 = 0U;
    prev_left_90 = pending_left_90 = active_left_90 = 0U;
    turn_wait_logged = 0U;
    button_was_moving = 0U;
    last_sequence = remote_input.sequence;
    output_tick = HAL_GetTick();
    release_sequence = last_sequence;
    Mecanum_Test_Init();
    Heading_Init();
    RemoteHeading_Reset();
    log_vx = log_vy = log_rotation = 0;
    remote_drive = 0U;
    heading_updated = 0U;
    logged_route = NULL;
    route_log_tick = HAL_GetTick() - 1000U;
    control_diag_ms = HAL_GetTick();
    vision_active = vision_is_shot = vision_from_remote = vision_enable_queued = 0U;
    vision_fault_rearm = vision_has_target = 0U;
    shot_request_pending = 0U;
    previous_cam_t = previous_shot = 0;
    vision_direction = vision_motion = 0;
    vision_sequence = vision_frame_tick = 0U;
    vision_settle_started = vision_sample_count = 0U;
    vision_settle_tick = vision_sample_tick = shot_fire_tick = 0U;
    Debug_Log("[CAR] OFF\r\n");
    boot_enable_pending = (uint8_t)(CAR_BOOT_AUTO_ENABLE && !CAR_PD10_STANDALONE_TEST);
    boot_wait_first_packet = boot_enable_pending;
    boot_wait_log_tick = HAL_GetTick();
    if (boot_enable_pending) Transition(CAR_WAIT_CENTER);
}

CarState_t Car_Control_GetState(void) { return state; }

static uint8_t AngleCommandAllowed(void)
{
    const CarCommand_t *c = &remote_input.command;
    return (uint8_t)(CAR_HEADING_TEST_MODE && (state == CAR_READY || state == CAR_RUNNING) &&
        RemoteConnected() && !c->stop && !c->brake && !c->disable &&
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

#if CAR_PD10_STANDALONE_TEST
/* Bench-only standalone path; requires the bench inputs (see car_config.h). */
static void Local_Process(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t low = local_input.pd10_low;
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
    if (!local_input.pd10_ready) return;
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
#endif

static uint8_t RemoteSafety(const CarCommand_t *c, uint8_t fresh)
{
    uint32_t now = HAL_GetTick();
    uint8_t disable_edge = (uint8_t)(fresh && c->disable && !previous_disable);
    if (fresh) previous_disable = (uint8_t)c->disable;
    if (RemoteValid()) boot_wait_first_packet = 0U;
    if (state == CAR_WAIT_CENTER) {
        if ((uint32_t)(now - boot_wait_log_tick) >= 1000U) {
            boot_wait_log_tick = now;
            Debug_Log(!RemoteValid() ? "[CAR] boot wait: valid control frame\r\n" :
                      !ButtonsReleased(c) ? "[CAR] boot wait: release controls\r\n" :
                      "[CAR] boot wait: motor TX idle / fresh frame\r\n");
        }
    }
    if (boot_wait_first_packet && Motor_HasFault()) {
        boot_enable_pending = boot_wait_first_packet = 0U;
        (void)Motor_EStopAll();
        Transition(CAR_FAULT);
        return 1U;
    }
    /* The phone's first frame after a reconnect often still carries `stop`.
     * Before the car has ever been enabled that must not cancel the boot
     * enable. The exemption ends immediately when enabling succeeds. */
    if (boot_enable_pending) {
        if (!c->disable && (c->stop || c->brake) && RemoteConnected() && !Motor_HasFault()) {
            /* Only pending boot enable is exempt: wait for neutral input. */
            return 1U;
        }
        /* Energize at power-up as before, but DISABLE is the highest-priority
         * safety input: never energize into it. */
        if (!c->disable && !c->stop && !c->brake && Motor_EnableAll() == HAL_OK) {
            boot_enable_pending = 0U;
            powered = 1U;
            Debug_Log("[CAR] POWER-ON ENABLE\r\n");
        }
    }
    if (!RemoteConnected()) {
        if (boot_wait_first_packet && !RemoteValid()) return 1U;
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
    if (disable_edge || (c->disable && boot_enable_pending)) {
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
        if (fresh && ButtonsReleased(c)) {
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
        recovery_center = 0U;
        boot_enable_pending = 0U;
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
        if (fresh && ButtonsReleased(c) && Motor_IsIdle()) {
            Transition(powered ? CAR_READY : CAR_OFF);
            LogReady();
            output_tick = now;
            Debug_Log("[CAR] BRAKE LOCK RELEASED\r\n");
        }
        return 1U;
    }
    if (state == CAR_WAIT_CENTER) {
        if (fresh && ButtonsReleased(c) && Motor_IsIdle()) {
            Transition(CAR_READY);
            output_tick = now;
            LogReady();
        }
        return 1U;
    }
    return 0U;
}

/* Called before translation or a new remote turn, including at rest. */
static uint8_t RemoteHeadingGate(void)
{
    const RemoteHeadingStatus_t *heading = RemoteHeading_GetStatus();
    if (heading->phase == REMOTE_ALIGNING) {
        if (remote_drive == 1U) { NormalStop(); return 1U; }
        if ((uint32_t)(HAL_GetTick() - output_tick) >= CAR_CONTROL_PERIOD_MS &&
            mecanum_drive(0, 0, heading->correction_rpm) == HAL_OK) {
            remote_drive = 2U;
            output_tick = HAL_GetTick();
            log_rotation = heading->correction_rpm;
            Transition(CAR_RUNNING);
        }
        return 1U;
    }
    if (remote_drive == 2U) { NormalStop(); return 1U; }
    return (uint8_t)(heading->phase != REMOTE_ALIGNED &&
                     heading->phase != REMOTE_DRIFTING &&
                     heading->phase != REMOTE_NO_IMU);
}

static uint8_t ProcessTurn(const CarCommand_t *c, uint8_t edge_90,
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
            RemoteHeading_CompleteTurn(active_left_90 ? -1 : active_right_180 ? 2 : 1);
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
    if (!pending_right_90 && !pending_right_180 && !pending_left_90) {
        if (c->right_90 || c->right_180 || c->left_90) {
            (void)RemoteHeadingGate();
            return 1U;
        }
        return 0U;
    }
    if (RemoteHeading_GetStatus()->phase == REMOTE_NO_IMU) {
        if (remote_drive != 0U) NormalStop();
        pending_right_90 = pending_right_180 = pending_left_90 = 0U;
        Debug_Log("[TURN] remote rejected: no IMU\r\n");
        return 1U;
    }
    if (RemoteHeadingGate()) return 1U;
    if (state == CAR_RUNNING) NormalStop();
    if (!Motor_IsIdle()) {
        if (!turn_wait_logged) Debug_Log("[TURN] WAIT motor_busy\r\n");
        turn_wait_logged = 1U;
        return 1U;
    }
    turn_wait_logged = 0U;
    result = TurnRight_StartRemote(pending_right_90 ? 90 : pending_right_180 ? 180 : -90,
                                  pending_right_180 ? TURN_RIGHT_180_RPM : TURN_RIGHT_90_RPM);
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
    remote_drive = 0U;
    RemoteHeading_BeginTurn();
    Transition(CAR_TURNING);
    LogControl(active_left_90 ? "LEFT90" :
               active_right_90 ? "RIGHT90" : "RIGHT180");
    return 1U;
}

static uint8_t ProcessButtons(const CarCommand_t *c, uint8_t fresh)
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
    if (!CAR_MECANUM_TEST_MODE && RemoteHeadingGate()) return 1U;
    if (count == 0U) {
        if (button_was_moving) {
            LogInputEvent("BUTTON_RELEASE", c);
            NormalStop();
            LogControl("BRAKE");
            release_sequence = remote_input.sequence;
            return 1U;
        }
        if (remote_input.sequence == release_sequence) return 1U;
        return 0U;
    }
    if (CAR_MECANUM_TEST_MODE) return 1U;
    button_was_moving = 1U;
    if (c->forward) { vx = BLUETOOTH_TEST_MOVE_RPM; LogControl("FORWARD"); }
    else if (c->backward) { vx = -BLUETOOTH_TEST_MOVE_RPM; LogControl("BACKWARD"); }
    else if (c->strafe_left) { vy = -BLUETOOTH_TEST_MOVE_RPM; LogControl("STRAFE_LEFT"); }
    else { vy = BLUETOOTH_TEST_MOVE_RPM; LogControl("STRAFE_RIGHT"); }
    log_vx = vx;
    log_vy = vy;
    yaw_correction = RemoteHeading_GetStatus()->phase == REMOTE_DRIFTING ?
        RemoteHeading_GetStatus()->correction_rpm : 0;
    if ((uint32_t)(HAL_GetTick() - output_tick) >= CAR_CONTROL_PERIOD_MS &&
        mecanum_drive(vx, vy, yaw_correction) == HAL_OK) {
        output_tick = HAL_GetTick();
        log_rotation = yaw_correction;
        remote_drive = 1U;
        Transition(CAR_RUNNING);
    }
    (void)fresh;
    return 1U;
}

void Car_Control_Process(void)
{
    const CarCommand_t *c;
    uint32_t sequence;
    uint8_t fresh, edge_90, edge_180, edge_left, allow, was_vision_active;
    heading_updated = 0U;
    log_vx = log_vy = 0;
#if CAR_PD10_STANDALONE_TEST
    /* Bench image: the standalone PD10 path owns the whole control loop. */
    Local_Process();
    goto finish;
#endif
    c = &remote_input.command;
    sequence = remote_input.sequence;
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
#if CAR_TEST_INPUTS_ENABLE
    LocalVisionRequests(c);
#endif
    if (vision_fault_rearm && PhysicalInputsIdle() && Motor_IsIdle()) {
        Motor_ClearFault();
        vision_fault_rearm = 0U;
        Transition(CAR_OFF);
    }
    if (fresh) {
        uint8_t cam_changed = (uint8_t)(c->vision_follow != previous_cam_t);
        uint8_t shot_changed = (uint8_t)(c->shot != previous_shot);
        uint8_t shot_exited = 0U;
        previous_cam_t = c->vision_follow;
        previous_shot = c->shot;
        if (shot_changed) shot_request_pending = (uint8_t)(c->shot != 0);
        if (vision_active && vision_from_remote && vision_is_shot && !c->shot) {
            VisionExit(RemoteConnected() ? CAR_WAIT_CENTER : CAR_OFF);
            shot_exited = 1U;
        } else if (vision_active && vision_from_remote && !vision_is_shot &&
                   cam_changed && !c->vision_follow) {
            VisionExit(RemoteConnected() ? CAR_WAIT_CENTER : CAR_OFF);
        }
        if (!shot_exited && !c->shot && cam_changed && c->vision_follow)
            (void)VisionEnter(1U, 0U, c);
    }
    if (shot_request_pending && !RemoteConnected())
        shot_request_pending = 0U;
    if (shot_request_pending && fresh) {
        if (!RemoteConnected() || c->stop || c->brake || c->disable ||
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
    if (RemoteSafety(c, fresh)) {
        if (CAR_MECANUM_TEST_MODE) Mecanum_Test_Process(0U);
        goto finish;
    }
    if (state != CAR_READY && state != CAR_RUNNING && state != CAR_TURNING) goto finish;
    if (!CAR_MECANUM_TEST_MODE) {
        RemoteHeading_Update(remote_drive == 1U &&
            !pending_right_90 && !pending_right_180 && !pending_left_90 &&
            !c->right_90 && !c->right_180 && !c->left_90);
        heading_updated = 1U;
    }
    if (CAR_MECANUM_TEST_MODE && !ButtonsReleased(c)) {
        Mecanum_Test_Process(0U);
        if (state == CAR_RUNNING) NormalStop();
        goto finish;
    }
    if (ProcessTurn(c, edge_90, edge_180, edge_left)) goto finish;
    if (ProcessButtons(c, fresh)) goto finish;
    if (CAR_MECANUM_TEST_MODE) {
        allow = (uint8_t)(powered && RemoteConnected() &&
            !Motor_HasFault());
        if (!allow && mecanum_test_active && (state == CAR_READY || state == CAR_RUNNING))
            (void)Motor_EStopAll();
        Mecanum_Test_Process(allow);
        Transition(mecanum_test_active ? CAR_RUNNING : CAR_READY);
    }
finish:
    if (!heading_updated) {
        (void)Heading_Update(false);
    }
    if (!CAR_PD10_STANDALONE_TEST)
        LogControlDiagnostics(&remote_input.command);
    Heading_Log(log_vx, log_vy, log_rotation);
    if (!CAR_PD10_STANDALONE_TEST && !CAR_MECANUM_TEST_MODE) RemoteHeading_Log();
}
