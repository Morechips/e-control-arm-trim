#ifndef MISSION_FSM_H
#define MISSION_FSM_H

#include "vision_data.h"
#include "detect_data.h"
#include "laser.h"
#include <stdbool.h>

typedef enum
{
    MISSION_IDLE = 0,
    TASK1_SEARCH_OBJECT, TASK1_ALIGN_OBJECT, TASK1_PICK, TASK1_ROTATE_180,
    TASK1_SEARCH_BUCKET, TASK1_ALIGN_BUCKET, TASK1_RELEASE, TASK1_DONE,
    TASK2_SEARCH, TASK2_ALIGN, TASK2_CONFIRM, TASK2_LASER_ON, TASK2_DONE,
    ROUTE_TO_RIGHT_TURN, RIGHT_TURN,
    TASK3_SEARCH, TASK3_ALIGN, TASK3_CONFIRM, TASK3_PICK, TASK3_RETURN,
    MISSION_DONE, MISSION_ERROR
} MissionState;

typedef enum
{
    SERVO_ACTION_PICK = 0,
    SERVO_ACTION_RELEASE,
    SERVO_ACTION_RESCUE_PICK
} ServoAction;

void MissionFSM_Init(void);
void MissionFSM_SetTaskData(const VisionTask_t *task);
/* Bucket type is hardware/protocol-specific; set it only from a confirmed map. */
void MissionFSM_SetBucketDetectType(uint8_t type);
void MissionFSM_StartTask1(void);
void MissionFSM_StartTask2(void);
void MissionFSM_StartTask3(void);
void MissionFSM_EnterRouteToRightTurn(void);
void MissionFSM_EnterRightTurn(void);
void MissionFSM_NotifyFinalRouteDone(void);
MissionState MissionFSM_Update(void);
MissionState MissionFSM_GetState(void);
bool MissionFSM_Task3ObjectAcquired(void);
bool MissionFSM_FinalRouteDone(void);

/* Servo hardware adapter remains an event-only TODO stub. */
void Servo_Start(ServoAction action);
bool Servo_IsDone(void);
void Servo_NotifyDone(void);
ServoAction Servo_GetRequestedAction(void);
#endif
