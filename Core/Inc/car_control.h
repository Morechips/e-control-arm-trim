#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H
#include "main.h"
typedef enum {
    CAR_OFF, CAR_WAIT_CENTER, CAR_READY, CAR_RUNNING,
    CAR_BRAKE_LOCK, CAR_LINK_LOST, CAR_FAULT, CAR_LOCAL_STARTING,
    CAR_TURNING, CAR_VISION_STARTING, CAR_VISION_TRACKING,
    CAR_SHOT_ALIGNING, CAR_SHOT_FIRING
} CarState_t;
void Car_Control_Init(void);
void Car_Control_Process(void);
CarState_t Car_Control_GetState(void);
/* Explicit angle commands; ordinary joystick motion never creates rotation.
 * Rejected while safety-locked or before a valid IMU reference exists. */
HAL_StatusTypeDef Car_Control_SetTargetYaw(float target_degrees);
HAL_StatusTypeDef Car_Control_AdjustTargetYaw(float delta_degrees);
/* Heading test image only: debugger writes action, then increments request.
 * 0=STOP/brake lock, 1=target +5 degrees, 2=target -5 degrees,
 * 3=set the current heading as software zero.
 * Rejected requests are consumed; never replayed after unlocking. */
extern volatile uint32_t heading_test_action, heading_test_request;
#endif
