#ifndef ACTION_FSM_H
#define ACTION_FSM_H

#include "main.h"
#include <stdbool.h>

typedef enum
{
    SPEED_SLOW = 0,
    SPEED_MEDIUM,
    SPEED_FULL,
    SPEED_PRECISE = SPEED_SLOW,
    SPEED_NORMAL = SPEED_FULL
} SpeedMode;

/* Fixed RPM lookup. An invalid mode falls back to SPEED_SLOW. */
uint16_t SpeedMode_GetRPM(SpeedMode mode);

typedef enum
{
    ACTION_FORWARD = 0,
    ACTION_WAIT_QR,
    ACTION_TRANSLATE_LEFT,
    ACTION_TRANSLATE_RIGHT,
    ACTION_TASK_1,
    ACTION_TURN_LEFT,
    ACTION_FORWARD_TO_TASK_2,
    ACTION_TASK_2,
    ACTION_FORWARD_AFTER_TASK_2,
    /* Retained for compatibility; not used by the current fixed route. */
    ACTION_TURN_RIGHT,
    ACTION_FINISH,
    /* Reusable MaxiCam action entry; intentionally absent from route[]. */
    ACTION_ALIGN_TARGET,
    ACTION_TASK_3,
    ACTION_RETURN_TO_FINISH,
    ACTION_QR_LEFT_TURN
} ActionType;

typedef enum
{
    ALIGN_IDLE = 0,
    ALIGN_TRACKING,
    ALIGN_BRAKE,
    ALIGN_WAIT_STOP,
    ALIGN_DONE,
    ALIGN_ERROR
} AlignState;

typedef enum
{
    EXEC_ENTER = 0,
    EXEC_RUN,
    EXEC_BRAKE,
    EXEC_WAIT_STOP,
    EXEC_DONE,
    EXEC_ERROR
} ExecState;

/* There is no validated mechanical-stop feedback in the motor transport.
 * WAIT_STOP therefore requires an empty TX queue plus this nonblocking hold. */
#define ACTION_STOP_SETTLE_MS 100U

void ActionFSM_Init(ActionType action);
ExecState ActionFSM_Update(void);
void ActionFSM_Abort(void);

/* Completion inputs are set only after an explicit result.
 * ACTION_WAIT_QR clears any earlier QR notice and waits stationary for a new
 * binary 0x80 after the preceding right turn has stopped. Success completes
 * through BRAKE / WAIT_STOP / DONE. MissionFSM owns task1/2/3
 * completion; SetTask1Done/SetTask2Done remain compatibility event setters
 * but cannot bypass mission completion checks. */
extern volatile bool qr_success;
extern volatile bool task1_done;
extern volatile bool task2_done;
extern volatile bool task3_done;
extern volatile bool task3_object_acquired;
extern volatile bool final_route_done;
extern volatile bool turn_left_done;
void ActionFSM_SetQRSuccess(bool success);
void ActionFSM_SetTask1Done(bool done);
void ActionFSM_SetTask2Done(bool done);
void ActionFSM_SetTurnLeftDone(bool done);

/* Reserved for the future path-end detector. The event is accepted only while
 * a started straight/translation segment is running. Distance data is
 * deliberately not converted to speed in this module. */
void ActionFSM_NotifyEndDetected(void);

ActionType ActionFSM_GetAction(void);
ExecState ActionFSM_GetState(void);
AlignState ActionFSM_GetAlignState(void);

#endif
