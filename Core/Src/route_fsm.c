#include "route_fsm.h"
#include "mecanum.h"
#include "serial_io.h"
#include "mission_fsm.h"
#include "maxicam.h"
#include <stdio.h>

const RouteStep route[] = {
    {ACTION_FORWARD},
    {ACTION_TURN_RIGHT},
    {ACTION_WAIT_QR},
    {ACTION_QR_LEFT_TURN},
    {ACTION_TRANSLATE_LEFT},
    {ACTION_FORWARD},
    {ACTION_TRANSLATE_LEFT},
    {ACTION_TURN_RIGHT},
    {ACTION_TASK_1},
    {ACTION_TURN_LEFT},
    {ACTION_FORWARD_TO_TASK_2},
    {ACTION_TASK_2},
    {ACTION_FORWARD_AFTER_TASK_2},
    {ACTION_TURN_RIGHT},
    {ACTION_TASK_3},
    {ACTION_RETURN_TO_FINISH},
    {ACTION_FINISH}
};
const size_t route_length = sizeof(route) / sizeof(route[0]);
size_t route_index;

static RouteState route_state;
static uint8_t initialized;
static bool qr_stage_done;

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
        case ACTION_TASK_3: return "TASK_3";
        case ACTION_RETURN_TO_FINISH: return "RETURN_TO_FINISH";
        default: return "INVALID";
    }
}

static void LogStep(void)
{
    char text[64];
    (void)snprintf(text, sizeof(text), "[ROUTE] step=%lu %s\r\n",
                   (unsigned long)route_index, ActionName(route[route_index].action));
    Debug_Log(text);
}

static void HoldBrake(const char *state_log)
{
    (void)brake();
    Debug_Log(state_log);
    Debug_Log("[BRAKE] HOLD\r\n");
}

static void EnterError(void)
{
    if (route_state == ROUTE_ERROR) return;
    route_state = ROUTE_ERROR;
    Laser_Disable();
    HoldBrake("[ROUTE] ERROR\r\n");
}

void RouteFSM_Init(void)
{
    if (MissionFSM_GetState() == MISSION_DONE) return;
    /* Restart is explicit and always cancels any previous in-progress motion. */
    if (initialized && route_state == ROUTE_RUNNING) ActionFSM_Abort();
    ActionFSM_SetQRSuccess(false);
    MissionFSM_Init();
    task1_done = task2_done = task3_done = false;
    task3_object_acquired = final_route_done = false;
    route_index = 0U;
    qr_stage_done = false;
    route_state = ROUTE_INIT;
    initialized = 1U;
}

RouteState RouteFSM_GetState(void) { return route_state; }
bool RouteFSM_QRStageDone(void) { return qr_stage_done; }

void RouteFSM_NotifyActionEnd(void)
{
    if (initialized && route_state == ROUTE_RUNNING)
        ActionFSM_NotifyEndDetected();
}

void RouteFSM_Update(void)
{
    ExecState action_state;
    ActionType completed_action;
    char text[48];

    if (!initialized || route_state == ROUTE_FINISHED || route_state == ROUTE_ERROR)
        return;

    if (route_state == ROUTE_INIT)
    {
        if (route_length == 0U) { EnterError(); return; }
        Debug_Log("[ROUTE] INIT\r\n");
        route_index = 0U;
        ActionFSM_Init(route[route_index].action);
        route_state = ROUTE_RUNNING;
        LogStep();
        return;
    }

    if (route_index >= route_length) { EnterError(); return; }
    action_state = ActionFSM_Update();
    if (action_state == EXEC_ERROR) { EnterError(); return; }
    if (action_state != EXEC_DONE) return;

    completed_action = route[route_index].action;

    if (completed_action == ACTION_FINISH)
    {
        (void)snprintf(text, sizeof(text), "[ROUTE] step=%lu DONE\r\n",
                       (unsigned long)route_index);
        Debug_Log(text);
        (void)MissionFSM_Update();
        if (MissionFSM_GetState() != MISSION_DONE || !task1_done || !task2_done ||
            !task3_done || !final_route_done)
        {
            EnterError();
            return;
        }
        route_state = ROUTE_FINISHED;
        Debug_Log("[ROUTE] FINISHED\r\n");
        Debug_Log("[BRAKE] HOLD\r\n");
        return;
    }

    if (completed_action == ACTION_RETURN_TO_FINISH)
    {
        MissionFSM_NotifyFinalRouteDone();
        (void)MissionFSM_Update();
        task3_done = (MissionFSM_GetState() == MISSION_DONE);
        task3_object_acquired = MissionFSM_Task3ObjectAcquired();
        final_route_done = MissionFSM_FinalRouteDone();
    }

    if (completed_action == ACTION_QR_LEFT_TURN)
    {
        if (!qr_success) { EnterError(); return; }
        qr_stage_done = true;
    }

    if (completed_action == ACTION_TASK_1 || completed_action == ACTION_TASK_2)
    {
        /* The camera accepts the mode byte only after a QR notification and has
         * no acknowledge, so this records the request: MaxiCam_Process() sends
         * and retries it. A pending mode must not wedge the route. */
        MaxiCam_RequestMode(completed_action == ACTION_TASK_1 ? MAXICAM_MODE_AIM
                                                             : MAXICAM_MODE_OBJECT);
    }

    (void)snprintf(text, sizeof(text), "[ROUTE] step=%lu DONE\r\n",
                   (unsigned long)route_index);
    Debug_Log(text);

    ++route_index;
    if (route_index >= route_length) { EnterError(); return; }
    ActionFSM_Init(route[route_index].action);
    LogStep();
}
