#include "mission_fsm.h"
#include "action_fsm.h"
#include "car_config.h"
#include "laser_config.h"
#include "maxicam.h"
#include "mecanum.h"
#include "turn_right.h"
#include <string.h>

static MissionState state;
static VisionTask_t task_data;
static bool task_data_valid;
static uint8_t bucket_detect_type = (uint8_t)DETECT_BARREL;
static uint8_t motion_started, settle_started, aligned_frames, target_lost_count;
static bool target_ever_seen, reacquiring;
static uint8_t servo_started, align_stop_pending, align_lost_pending;
static int8_t align_direction;
static uint8_t turn_started;
static uint32_t settle_tick, frame_sequence, laser_tick;
static uint8_t laser_started, final_brake_sent;
static volatile bool servo_done;
static ServoAction servo_action;

static uint8_t DetectColor(VisionColor_t color)
{
    switch (color)
    {
        case VISION_COLOR_RED: return (uint8_t)DETECT_REDBALL;
        case VISION_COLOR_BLUE: return (uint8_t)DETECT_BLUEBALL;
        case VISION_COLOR_GREEN: return (uint8_t)DETECT_GREANBALL;
        default: return (uint8_t)DETECT_UNKNOWN;
    }
}

static uint8_t ExpectedType(void)
{
    switch (state)
    {
        case TASK1_SEARCH_OBJECT: case TASK1_ALIGN_OBJECT:
            return DetectColor(task_data.explosive_color);
        case TASK1_SEARCH_BUCKET: case TASK1_ALIGN_BUCKET:
            return bucket_detect_type;
        case TASK2_SEARCH: case TASK2_ALIGN: case TASK2_CONFIRM:
            return (uint8_t)DETECT_TARGET;
        case TASK3_SEARCH: case TASK3_ALIGN: case TASK3_CONFIRM:
            return (uint8_t)(DETECT_HOSTAGE_1 + task_data.rescue_target_shape - VISION_SHAPE_CYLINDER);
        default: return (uint8_t)DETECT_UNKNOWN;
    }
}

static uint16_t StateRPM(void)
{
    switch (state)
    {
        case TASK1_SEARCH_OBJECT: case TASK1_SEARCH_BUCKET:
        case TASK1_ALIGN_OBJECT: case TASK1_ALIGN_BUCKET:
        case TASK2_ALIGN: case TASK3_ALIGN:
            return SPEED_SLOW_RPM;
        case TASK2_SEARCH: case TASK3_SEARCH:
            return SPEED_MEDIUM_RPM;
        default: return SPEED_FULL_RPM;
    }
}

static bool IsSearch(void)
{
    return state == TASK1_SEARCH_OBJECT || state == TASK1_SEARCH_BUCKET ||
           state == TASK2_SEARCH || state == TASK3_SEARCH;
}

static bool IsAlign(void)
{
    return state == TASK1_ALIGN_OBJECT || state == TASK1_ALIGN_BUCKET ||
           state == TASK2_ALIGN || state == TASK3_ALIGN;
}

static void EnterWithTracking(MissionState next, bool preserve_target)
{
    MaxiCamTargetData_t target;
    state = next;
    motion_started = settle_started = aligned_frames = 0U;
    if (!preserve_target && (next == TASK1_SEARCH_OBJECT || next == TASK1_SEARCH_BUCKET ||
        next == TASK2_SEARCH || next == TASK3_SEARCH))
    {
        target_ever_seen = reacquiring = false;
        target_lost_count = 0U;
    }
    servo_started = align_stop_pending = align_lost_pending = 0U;
    turn_started = 0U;
    align_direction = 0;
    MaxiCam_GetTargetData(&target, &frame_sequence);
    if (next == MISSION_DONE)
    {
        Laser_Disable();
        final_brake_sent = (uint8_t)(brake() == HAL_OK);
    }
}

static void Enter(MissionState next) { EnterWithTracking(next, false); }

static bool StopAndSettle(void)
{
    if (!Motor_IsIdle()) { settle_started = 0U; return false; }
    if (!settle_started)
    {
        settle_started = 1U;
        settle_tick = HAL_GetTick();
        return false;
    }
    return (uint32_t)(HAL_GetTick() - settle_tick) >= ACTION_STOP_SETTLE_MS;
}

static HAL_StatusTypeDef StartSearchMotion(void)
{
    const int16_t rpm = (int16_t)StateRPM();
    return reacquiring ? left(rpm) : right(rpm);
}

static void UpdateSearch(void)
{
    MaxiCamTargetData_t target;
    uint32_t sequence;
    HAL_StatusTypeDef result;
    const uint8_t expected_type = ExpectedType();
    if (expected_type == (uint8_t)DETECT_UNKNOWN)
    {
        state = MISSION_ERROR;
        return;
    }
    MaxiCam_GetTargetData(&target, &sequence);
    if (sequence != frame_sequence && target.target_valid && target.type == expected_type)
    {
        target_ever_seen = true;
        target_lost_count = 0U;
        reacquiring = false;
        if (brake() != HAL_OK) { state = MISSION_ERROR; return; }
        Enter(state == TASK1_SEARCH_OBJECT ? TASK1_ALIGN_OBJECT :
              state == TASK1_SEARCH_BUCKET ? TASK1_ALIGN_BUCKET :
              state == TASK2_SEARCH ? TASK2_ALIGN : TASK3_ALIGN);
        return;
    }
    frame_sequence = sequence;
    if (motion_started) return;
    result = StartSearchMotion();
    if (result == HAL_OK) motion_started = 1U;
    else if (result != HAL_BUSY) state = MISSION_ERROR;
}

static void UpdateAlign(void)
{
    MaxiCamTargetData_t target;
    uint32_t sequence;
    HAL_StatusTypeDef result;
    const int16_t minimum = state == TASK2_ALIGN ? MAXICAM_SHOT_MIN_X : MAXICAM_ALIGN_MIN_X;
    const int16_t maximum = state == TASK2_ALIGN ? MAXICAM_SHOT_MAX_X : MAXICAM_ALIGN_MAX_X;
    if (align_stop_pending || align_lost_pending)
    {
        if (!StopAndSettle()) return;
        if (align_lost_pending)
        {
            EnterWithTracking(state == TASK1_ALIGN_OBJECT ? TASK1_SEARCH_OBJECT :
                              state == TASK1_ALIGN_BUCKET ? TASK1_SEARCH_BUCKET :
                              state == TASK2_ALIGN ? TASK2_SEARCH : TASK3_SEARCH, true);
            reacquiring = true;
        }
        else if (state == TASK1_ALIGN_OBJECT) Enter(TASK1_PICK);
        else if (state == TASK1_ALIGN_BUCKET) Enter(TASK1_RELEASE);
        else if (state == TASK2_ALIGN) Enter(TASK2_CONFIRM);
        else Enter(TASK3_CONFIRM);
        return;
    }
    MaxiCam_GetTargetData(&target, &sequence);
    if (sequence == frame_sequence) return;
    if (!target.target_valid || target.type != ExpectedType())
    {
        frame_sequence = sequence;
        aligned_frames = 0U;
        if (state == TASK2_ALIGN)
        {
            /* Never continue an aim correction without a valid ring frame. */
            align_lost_pending = 1U;
            Laser_Disable();
            if (brake() != HAL_OK) state = MISSION_ERROR;
            return;
        }
        if (target.type != (uint8_t)DETECT_UNKNOWN || target.target_valid)
        {
            target_lost_count = 0U;
            return;
        }
        if (target_lost_count < UINT8_MAX) ++target_lost_count;
        if (target_ever_seen && target_lost_count >= MAXICAM_TARGET_LOST_FRAMES)
        {
            align_lost_pending = 1U;
            if (brake() != HAL_OK) state = MISSION_ERROR;
        }
        return;
    }
    target_lost_count = 0U;
    if (target.offset_x > maximum)
    {
        aligned_frames = 0U;
        if (align_direction == 1) return;
        result = up((int16_t)SPEED_SLOW_RPM);
        if (result == HAL_BUSY) return;
        if (result != HAL_OK) state = MISSION_ERROR;
        else if (result == HAL_OK) align_direction = 1;
        frame_sequence = sequence;
        return;
    }
    if (target.offset_x < minimum)
    {
        aligned_frames = 0U;
        if (align_direction == -1) return;
        result = down((int16_t)SPEED_SLOW_RPM);
        if (result == HAL_BUSY) return;
        if (result != HAL_OK) state = MISSION_ERROR;
        else if (result == HAL_OK) align_direction = -1;
        frame_sequence = sequence;
        return;
    }
    frame_sequence = sequence;
    if (state == TASK2_ALIGN)
    {
        /* Confirm only after braking and settling, using fresh ring frames. */
        align_stop_pending = 1U;
        if (brake() != HAL_OK) state = MISSION_ERROR;
        return;
    }
    if (aligned_frames < UINT8_MAX) ++aligned_frames;
    if (aligned_frames < MAXICAM_ALIGN_CONFIRM_FRAMES) return;
    align_stop_pending = 1U;
    if (brake() != HAL_OK) state = MISSION_ERROR;
}

void MissionFSM_Init(void)
{
    memset(&task_data, 0, sizeof(task_data));
    task_data_valid = false;
    task3_object_acquired = final_route_done = false;
    task1_done = task2_done = task3_done = false;
    bucket_detect_type = (uint8_t)DETECT_BARREL;
    servo_done = false;
    Laser_Disable();
    laser_started = final_brake_sent = 0U;
    target_ever_seen = reacquiring = false;
    target_lost_count = 0U;
    state = MISSION_IDLE;
}

void MissionFSM_SetTaskData(const VisionTask_t *task)
{
    if (task == NULL) { task_data_valid = false; return; }
    if (task->explosive_color < VISION_COLOR_RED || task->explosive_color > VISION_COLOR_BLUE ||
        task->counterterror_target_color < VISION_COLOR_RED ||
        task->counterterror_target_color > VISION_COLOR_BLUE ||
        task->rescue_target_shape < VISION_SHAPE_CYLINDER ||
        task->rescue_target_shape > VISION_SHAPE_WAIST_DRUM)
    {
        task_data_valid = false;
        return;
    }
    task_data = *task;
    task_data_valid = true;
}

void MissionFSM_SetBucketDetectType(uint8_t type)
{
    /* The protocol identifies the barrel as type 6; keep an explicit override. */
    bucket_detect_type = type < (uint8_t)DETECT_UNKNOWN ? type : (uint8_t)DETECT_UNKNOWN;
}

void MissionFSM_StartTask1(void) { Enter(TASK1_SEARCH_OBJECT); }
void MissionFSM_StartTask2(void) { Enter(TASK2_SEARCH); }
void MissionFSM_StartTask3(void) { task3_object_acquired = false; Enter(TASK3_SEARCH); }
void MissionFSM_EnterRouteToRightTurn(void) { Enter(ROUTE_TO_RIGHT_TURN); }
void MissionFSM_EnterRightTurn(void) { Enter(RIGHT_TURN); }
void MissionFSM_NotifyFinalRouteDone(void) { final_route_done = true; }
MissionState MissionFSM_GetState(void) { return state; }
bool MissionFSM_Task3ObjectAcquired(void) { return task3_object_acquired; }
bool MissionFSM_FinalRouteDone(void) { return final_route_done; }

void Servo_Start(ServoAction action)
{
    /* TODO: dispatch this event to the confirmed servo hardware adapter. */
    servo_action = action;
    servo_done = false;
}
bool Servo_IsDone(void) { return servo_done; }
void Servo_NotifyDone(void) { servo_done = true; }
ServoAction Servo_GetRequestedAction(void) { return servo_action; }
MissionState MissionFSM_Update(void)
{
    MaxiCamTargetData_t target;
    uint32_t sequence;
    if (state == MISSION_IDLE) return state;
    if (state == MISSION_ERROR)
    {
        Laser_Disable();
        (void)brake();
        return state;
    }
    if (state == MISSION_DONE)
    {
        if (!final_brake_sent && brake() == HAL_OK) final_brake_sent = 1U;
        Laser_Disable();
        return state;
    }
    if (!task_data_valid) { state = MISSION_ERROR; return state; }
    if (IsSearch()) { UpdateSearch(); return state; }
    if (IsAlign()) { UpdateAlign(); return state; }
    switch (state)
    {
        case TASK1_PICK:
            if (!servo_started)
            {
                Servo_Start(SERVO_ACTION_PICK);
                servo_started = 1U;
            }
            else if (Servo_IsDone()) Enter(TASK1_ROTATE_180);
            break;
        case TASK1_ROTATE_180:
            if (!turn_started)
            {
                HAL_StatusTypeDef result = right180((int16_t)SPEED_FULL_RPM);
                if (result == HAL_ERROR) { state = MISSION_ERROR; break; }
                if (result == HAL_BUSY) break;
                turn_started = 1U;
            }
            TurnRight_Process(true);
            if (TurnRight_GetStatus()->state == TURN_RIGHT_DONE) Enter(TASK1_SEARCH_BUCKET);
            else if (TurnRight_GetStatus()->state == TURN_RIGHT_FAULT) state = MISSION_ERROR;
            break;
        case TASK1_RELEASE:
            if (!servo_started)
            {
                Servo_Start(SERVO_ACTION_RELEASE);
                servo_started = 1U;
                break;
            }
            if (Servo_IsDone())
            {
                task1_done = true;
                Enter(TASK1_DONE);
            }
            break;
        case TASK1_DONE: break;
        case TASK2_CONFIRM:
            MaxiCam_GetTargetData(&target, &sequence);
            if (sequence == frame_sequence) break;
            frame_sequence = sequence;
            if (!target.target_valid || target.type != (uint8_t)DETECT_TARGET)
            {
                aligned_frames = 0U;
                if (target.type == (uint8_t)DETECT_UNKNOWN &&
                    ++target_lost_count >= MAXICAM_TARGET_LOST_FRAMES)
                {
                    EnterWithTracking(TASK2_SEARCH, true);
                    reacquiring = true;
                }
                else if (target.type != (uint8_t)DETECT_UNKNOWN)
                    target_lost_count = 0U;
                break;
            }
            target_lost_count = 0U;
            if (target.offset_x < MAXICAM_SHOT_MIN_X ||
                target.offset_x > MAXICAM_SHOT_MAX_X)
            {
                Enter(TASK2_ALIGN);
                break;
            }
            if (++aligned_frames >= MAXICAM_ALIGN_CONFIRM_FRAMES)
            {
                Enter(TASK2_LASER_ON);
                Laser_Enable();
                laser_started = 1U;
                laser_tick = HAL_GetTick();
            }
            break;
        case TASK2_LASER_ON:
            MaxiCam_GetTargetData(&target, &sequence);
            if (sequence != frame_sequence)
            {
                frame_sequence = sequence;
                if (!target.target_valid || target.type != (uint8_t)DETECT_TARGET ||
                    target.offset_x < MAXICAM_SHOT_MIN_X ||
                    target.offset_x > MAXICAM_SHOT_MAX_X)
                {
                    Laser_Disable();
                    (void)brake();
                    state = MISSION_ERROR;
                    break;
                }
            }
            if (!laser_started) { Laser_Enable(); laser_started = 1U; laser_tick = HAL_GetTick(); }
            if ((uint32_t)(HAL_GetTick() - laser_tick) >= LASER_HOLD_MS)
            {
                Laser_Disable();
                task2_done = true;
                Enter(TASK2_DONE);
            }
            break;
        case TASK2_DONE: break;
        case TASK3_CONFIRM:
            MaxiCam_GetTargetData(&target, &sequence);
            if (sequence != frame_sequence && target.target_valid && target.type == ExpectedType())
            {
                frame_sequence = sequence;
                Enter(TASK3_PICK);
            }
            else if (sequence != frame_sequence && (!target.target_valid || target.type != ExpectedType()))
                frame_sequence = sequence;
            break;
        case TASK3_PICK:
            if (!servo_started)
            {
                Servo_Start(SERVO_ACTION_RESCUE_PICK);
                servo_started = 1U;
                break;
            }
            if (Servo_IsDone())
            {
                task3_object_acquired = true;
                Enter(TASK3_RETURN);
            }
            break;
        case TASK3_RETURN:
            if (final_route_done && task3_object_acquired &&
                task1_done && task2_done)
            {
                task3_done = true;
                Enter(MISSION_DONE);
            }
            break;
        default: break;
    }
    return state;
}
