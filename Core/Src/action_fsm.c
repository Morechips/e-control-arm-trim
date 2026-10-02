#include "action_fsm.h"
#include "mecanum.h"
#include "turn_right.h"
#include "serial_io.h"
#include "car_config.h"
#include "maxicam.h"
#include "mission_fsm.h"
#include <stdio.h>

static ActionType current_action;
static ExecState exec_state;
static uint8_t motion_started;
static uint8_t end_detected;
static uint8_t stop_wait_started;
static uint32_t stop_wait_tick;
static ExecState post_stop_state;
static SpeedMode current_speed_mode = SPEED_NORMAL;
static uint8_t speed_mode_initialized;
static AlignState align_state;
static uint8_t align_confirm_count;
static uint32_t align_frame_sequence;

typedef enum
{
    ALIGN_MOTION_STOPPED = 0,
    ALIGN_MOTION_FORWARD,
    ALIGN_MOTION_BACKWARD
} AlignMotion;

static AlignMotion align_motion;

volatile bool qr_success;
volatile bool task1_done;
volatile bool task2_done;
volatile bool task3_done;
volatile bool task3_object_acquired;
volatile bool final_route_done;
volatile bool turn_left_done;

uint16_t SpeedMode_GetRPM(SpeedMode mode)
{
    switch (mode)
    {
        case SPEED_FULL: return SPEED_FULL_RPM;
        case SPEED_MEDIUM: return SPEED_MEDIUM_RPM;
        case SPEED_SLOW:
        default: return SPEED_SLOW_RPM;
    }
}

static void SelectSpeedMode(SpeedMode mode)
{
    char text[48];
    if (speed_mode_initialized && current_speed_mode == mode) return;
    current_speed_mode = mode;
    speed_mode_initialized = 1U;
    (void)snprintf(text, sizeof(text), "[SPEED] %s %u RPM\r\n",
                   mode == SPEED_FULL ? "FULL" :
                   (mode == SPEED_MEDIUM ? "MEDIUM" : "SLOW"),
                   (unsigned)SpeedMode_GetRPM(mode));
    Debug_Log(text);
}

static const char *ActionName(ActionType action)
{
    switch (action)
    {
        case ACTION_FORWARD: return "FORWARD";
        case ACTION_WAIT_QR: return "WAIT_QR";
        case ACTION_TRANSLATE_LEFT: return "TRANSLATE_LEFT";
        case ACTION_TRANSLATE_RIGHT: return "TRANSLATE_RIGHT";
        case ACTION_TASK_1: return "TASK_1";
        case ACTION_TURN_LEFT: return "TURN_LEFT";
        case ACTION_FORWARD_TO_TASK_2: return "FORWARD_TO_TASK_2";
        case ACTION_TASK_2: return "TASK_2";
        case ACTION_FORWARD_AFTER_TASK_2: return "FORWARD_AFTER_TASK_2";
        case ACTION_TURN_RIGHT: return "TURN_RIGHT";
        case ACTION_QR_LEFT_TURN: return "QR_LEFT_TURN";
        case ACTION_FINISH: return "FINISH";
        case ACTION_ALIGN_TARGET: return "ALIGN_TARGET";
        case ACTION_TASK_3: return "TASK_3";
        case ACTION_RETURN_TO_FINISH: return "RETURN_TO_FINISH";
        default: return "INVALID";
    }
}

static const char *StateName(ExecState state)
{
    static const char *const names[] = {
        "ENTER", "RUN", "BRAKE", "WAIT_STOP", "DONE", "ERROR"
    };
    return (unsigned)state < (sizeof(names) / sizeof(names[0])) ?
           names[state] : "INVALID";
}

static void Transition(ExecState next)
{
    char text[64];
    if (exec_state == next) return;
    (void)snprintf(text, sizeof(text), "[ACTION] %s -> %s\r\n",
                   StateName(exec_state), StateName(next));
    Debug_Log(text);
    exec_state = next;
    if (current_action == ACTION_WAIT_QR)
    {
        if (next == EXEC_WAIT_STOP) Debug_Log("[QR] BRAKE -> WAIT_STOP\r\n");
        else if (next == EXEC_DONE) Debug_Log("[QR] DONE\r\n");
    }
    if (current_action == ACTION_ALIGN_TARGET)
    {
        if (next == EXEC_WAIT_STOP)
        {
            align_state = ALIGN_WAIT_STOP;
            Debug_Log("[ALIGN] BRAKE -> WAIT_STOP\r\n");
        }
        else if (next == EXEC_DONE)
        {
            align_state = ALIGN_DONE;
            Debug_Log("[ALIGN] DONE\r\n");
        }
        else if (next == EXEC_ERROR) align_state = ALIGN_ERROR;
    }
}

static void EnterError(void)
{
    if (current_action == ACTION_TURN_RIGHT || current_action == ACTION_QR_LEFT_TURN ||
        MissionFSM_GetState() == TASK1_ROTATE_180)
        TurnRight_Cancel();
    Laser_Disable();
    Transition(EXEC_ERROR);
}

void ActionFSM_Init(ActionType action)
{
    current_action = action;
    exec_state = EXEC_ENTER;
    motion_started = 0U;
    end_detected = 0U;
    stop_wait_started = 0U;
    if (action == ACTION_WAIT_QR) qr_success = false;
    stop_wait_tick = HAL_GetTick();
    post_stop_state = EXEC_DONE;
    if (action == ACTION_ALIGN_TARGET)
    {
        MaxiCam_GetTargetData(NULL, &align_frame_sequence);
        align_state = ALIGN_IDLE;
        align_confirm_count = 0U;
        align_motion = ALIGN_MOTION_STOPPED;
    }
    if (action == ACTION_TASK_1) task1_done = false;
    if (action == ACTION_TASK_2) task2_done = false;
    if (action == ACTION_TASK_3) task3_done = false;
    if (action == ACTION_TASK_1) MissionFSM_StartTask1();
    if (action == ACTION_TASK_2) MissionFSM_StartTask2();
    if (action == ACTION_TASK_3) MissionFSM_StartTask3();
    if (action == ACTION_FORWARD_AFTER_TASK_2) MissionFSM_EnterRouteToRightTurn();
    if (action == ACTION_TURN_RIGHT && task2_done) MissionFSM_EnterRightTurn();
    if (action == ACTION_TURN_LEFT) turn_left_done = false;
}

void ActionFSM_Abort(void)
{
    if ((current_action == ACTION_TURN_RIGHT || current_action == ACTION_QR_LEFT_TURN ||
         MissionFSM_GetState() == TASK1_ROTATE_180) &&
        exec_state != EXEC_DONE && exec_state != EXEC_ERROR)
        TurnRight_Cancel();
    if (exec_state != EXEC_DONE && exec_state != EXEC_ERROR) (void)brake();
    Laser_Disable();
    if (current_action == ACTION_ALIGN_TARGET) align_state = ALIGN_ERROR;
    exec_state = EXEC_ERROR;
}

ActionType ActionFSM_GetAction(void) { return current_action; }
ExecState ActionFSM_GetState(void) { return exec_state; }
AlignState ActionFSM_GetAlignState(void) { return align_state; }

void ActionFSM_SetQRSuccess(bool success) { qr_success = success; }
void ActionFSM_SetTask1Done(bool done) { task1_done = done; }
void ActionFSM_SetTask2Done(bool done) { task2_done = done; }
void ActionFSM_SetTurnLeftDone(bool done) { turn_left_done = done; }

static uint8_t IsPathEndAction(ActionType action)
{
    return (uint8_t)(action == ACTION_FORWARD ||
                     action == ACTION_TRANSLATE_LEFT ||
                     action == ACTION_TRANSLATE_RIGHT ||
                     action == ACTION_FORWARD_TO_TASK_2 ||
                     action == ACTION_FORWARD_AFTER_TASK_2 ||
                     action == ACTION_RETURN_TO_FINISH);
}

void ActionFSM_NotifyEndDetected(void)
{
    char text[64];
    if (exec_state != EXEC_RUN || !motion_started || !IsPathEndAction(current_action))
        return;
    if (end_detected) return;
    end_detected = 1U;
    (void)snprintf(text, sizeof(text), "[ACTION] %s end detected\r\n",
                   ActionName(current_action));
    Debug_Log(text);
}

static HAL_StatusTypeDef StartMotion(void)
{
    const int16_t rpm = (int16_t)SpeedMode_GetRPM(current_speed_mode);
    if (current_action == ACTION_FORWARD)
        return up(rpm);
    if (current_action == ACTION_TRANSLATE_LEFT)
        return left(rpm);
    if (current_action == ACTION_TRANSLATE_RIGHT)
        return right(rpm);
    if (current_action == ACTION_FORWARD_TO_TASK_2 ||
        current_action == ACTION_FORWARD_AFTER_TASK_2 ||
        current_action == ACTION_RETURN_TO_FINISH)
        return up(rpm);
    if (current_action == ACTION_TURN_LEFT)
    {
        /* TODO: call a confirmed existing-parameter left-turn controller.
         * Safe placeholder: issue only Brake and await turn_left_done. */
        Debug_Log("[ACTION] TURN_LEFT safe placeholder; waiting external done\r\n");
        return brake();
    }
    if (current_action == ACTION_TURN_RIGHT)
        return right90(rpm);
    if (current_action == ACTION_QR_LEFT_TURN)
        return left90(rpm);
    return HAL_ERROR;
}

static void UpdateAlignment(void)
{
    MaxiCamTargetData_t data;
    uint32_t sequence;
    HAL_StatusTypeDef result;
    char text[64];

    MaxiCam_GetTargetData(&data, &sequence);
    if (sequence == align_frame_sequence) return;

    if (!data.target_valid)
    {
        align_frame_sequence = sequence;
        align_confirm_count = 0U;
        if (align_motion != ALIGN_MOTION_STOPPED)
        {
            Debug_Log("[ALIGN] target lost -> BRAKE\r\n");
            align_state = ALIGN_BRAKE;
            post_stop_state = EXEC_RUN;
            Transition(EXEC_BRAKE);
        }
        return;
    }

    if (data.offset_x > MAXICAM_ALIGN_MAX_X)
    {
        align_confirm_count = 0U;
        if (align_motion != ALIGN_MOTION_FORWARD)
        {
            result = up((int16_t)SpeedMode_GetRPM(SPEED_PRECISE));
            if (result == HAL_BUSY) return;
            if (result != HAL_OK) { EnterError(); return; }
            align_motion = ALIGN_MOTION_FORWARD;
            (void)snprintf(text, sizeof(text), "[ALIGN] forward adjust x=%d\r\n",
                           (int)data.offset_x);
            Debug_Log(text);
        }
        align_frame_sequence = sequence;
        return;
    }

    if (data.offset_x < MAXICAM_ALIGN_MIN_X)
    {
        align_confirm_count = 0U;
        if (align_motion != ALIGN_MOTION_BACKWARD)
        {
            result = down((int16_t)SpeedMode_GetRPM(SPEED_PRECISE));
            if (result == HAL_BUSY) return;
            if (result != HAL_OK) { EnterError(); return; }
            align_motion = ALIGN_MOTION_BACKWARD;
            (void)snprintf(text, sizeof(text), "[ALIGN] backward adjust x=%d\r\n",
                           (int)data.offset_x);
            Debug_Log(text);
        }
        align_frame_sequence = sequence;
        return;
    }

    align_frame_sequence = sequence;
    ++align_confirm_count;
    (void)snprintf(text, sizeof(text), "[ALIGN] deadzone %u/%u\r\n",
                   (unsigned)align_confirm_count,
                   (unsigned)MAXICAM_ALIGN_CONFIRM_FRAMES);
    Debug_Log(text);
    if (align_confirm_count >= MAXICAM_ALIGN_CONFIRM_FRAMES)
    {
        Debug_Log("[ALIGN] aligned -> BRAKE\r\n");
        align_state = ALIGN_BRAKE;
        post_stop_state = EXEC_DONE;
        Transition(EXEC_BRAKE);
    }
}

ExecState ActionFSM_Update(void)
{
    if (exec_state == EXEC_ENTER)
    {
        if (current_action == ACTION_ALIGN_TARGET)
        {
            if (Motor_HasFault()) { EnterError(); return exec_state; }
            SelectSpeedMode(SPEED_PRECISE);
            align_state = ALIGN_TRACKING;
            Transition(EXEC_RUN);
            return exec_state;
        }
        if (current_action == ACTION_WAIT_QR)
        {
            if (Motor_HasFault()) { EnterError(); return exec_state; }
            if (!Motor_IsIdle()) return exec_state;
            Debug_Log("[QR] stationary scan\r\n");
            Transition(EXEC_RUN);
            return exec_state;
        }
        if (current_action == ACTION_FINISH)
        {
            post_stop_state = current_action == ACTION_FINISH ? EXEC_DONE : EXEC_RUN;
            Transition(EXEC_BRAKE);
        }
        else
        {
            if (IsPathEndAction(current_action) || current_action == ACTION_TURN_RIGHT ||
                current_action == ACTION_QR_LEFT_TURN)
                SelectSpeedMode(SPEED_NORMAL);
            Transition(EXEC_RUN);
        }
        return exec_state;
    }

    if (exec_state == EXEC_RUN)
    {
        HAL_StatusTypeDef result;
        if (Motor_HasFault()) { EnterError(); return exec_state; }
        if (current_action == ACTION_ALIGN_TARGET)
        {
            UpdateAlignment();
            return exec_state;
        }
        if (current_action == ACTION_WAIT_QR)
        {
            if (qr_success)
            {
                Debug_Log("[QR] success -> BRAKE\r\n");
                Transition(EXEC_BRAKE);
            }
            return exec_state;
        }
        if (current_action == ACTION_TASK_1)
        {
            (void)MissionFSM_Update();
            if (MissionFSM_GetState() == MISSION_ERROR) { EnterError(); return exec_state; }
            if (MissionFSM_GetState() == TASK1_DONE && task1_done)
            {
                Debug_Log("[TASK1] done\r\n");
                Transition(EXEC_DONE);
            }
            return exec_state;
        }
        if (current_action == ACTION_TASK_2)
        {
            (void)MissionFSM_Update();
            if (MissionFSM_GetState() == MISSION_ERROR) { EnterError(); return exec_state; }
            if (MissionFSM_GetState() == TASK2_DONE && task2_done)
            {
                Debug_Log("[TASK2] done\r\n");
                Transition(EXEC_DONE);
            }
            return exec_state;
        }
        if (current_action == ACTION_TASK_3)
        {
            (void)MissionFSM_Update();
            if (MissionFSM_GetState() == MISSION_ERROR) { EnterError(); return exec_state; }
            task3_object_acquired = MissionFSM_Task3ObjectAcquired();
            task3_done = (MissionFSM_GetState() == MISSION_DONE);
            final_route_done = MissionFSM_FinalRouteDone();
            if (MissionFSM_GetState() == TASK3_RETURN && task3_object_acquired)
                Transition(EXEC_DONE);
            return exec_state;
        }
        if (!motion_started)
        {
            result = StartMotion();
            if (result == HAL_BUSY) return exec_state;
            if (result != HAL_OK) { EnterError(); return exec_state; }
            motion_started = 1U;
            return exec_state;
        }
        if (current_action == ACTION_TURN_RIGHT || current_action == ACTION_QR_LEFT_TURN)
        {
            TurnRightState_t turn_state;
            TurnRight_Process(true);
            turn_state = TurnRight_GetStatus()->state;
            if (turn_state == TURN_RIGHT_DONE) Transition(EXEC_BRAKE);
            else if (turn_state == TURN_RIGHT_FAULT || turn_state == TURN_RIGHT_CANCELLED)
                EnterError();
        }
        else if ((current_action == ACTION_TURN_LEFT && turn_left_done) || end_detected)
        {
            post_stop_state = EXEC_DONE;
            Transition(EXEC_BRAKE);
        }
        return exec_state;
    }

    if (exec_state == EXEC_BRAKE)
    {
        if (brake() != HAL_OK) { EnterError(); return exec_state; }
        stop_wait_started = 0U;
        Transition(EXEC_WAIT_STOP);
        return exec_state;
    }

    if (exec_state == EXEC_WAIT_STOP)
    {
        if (Motor_HasFault()) { EnterError(); return exec_state; }
        if (!Motor_IsIdle())
        {
            stop_wait_started = 0U;
            return exec_state;
        }
        if (!stop_wait_started)
        {
            stop_wait_tick = HAL_GetTick();
            stop_wait_started = 1U;
            return exec_state;
        }
        if ((uint32_t)(HAL_GetTick() - stop_wait_tick) >= ACTION_STOP_SETTLE_MS)
        {
            if (current_action == ACTION_ALIGN_TARGET && post_stop_state == EXEC_RUN)
            {
                align_motion = ALIGN_MOTION_STOPPED;
                align_state = ALIGN_TRACKING;
            }
            Transition(post_stop_state);
            if (post_stop_state == EXEC_RUN)
            {
                if (current_action == ACTION_TASK_1) Debug_Log("[TASK1] waiting\r\n");
                else if (current_action == ACTION_TASK_2) Debug_Log("[TASK2] waiting\r\n");
            }
        }
    }
    return exec_state;
}
