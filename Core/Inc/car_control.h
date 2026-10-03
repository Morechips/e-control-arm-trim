#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H
#include "main.h"
typedef enum {
    CAR_OFF, CAR_WAIT_CENTER, CAR_READY, CAR_RUNNING,
    CAR_BRAKE_LOCK, CAR_LINK_LOST, CAR_FAULT, CAR_LOCAL_STARTING,
    CAR_TURNING, CAR_VISION_STARTING, CAR_VISION_TRACKING,
    CAR_SHOT_ALIGNING, CAR_SHOT_FIRING
} CarState_t;
typedef struct {
    int16_t joy_x, joy_y;
    int16_t forward, backward, stop, strafe_left, strafe_right;
    int16_t right_90, right_180, left_90;
    int16_t vision_follow, shot, brake, disable;
} CarCommand_t;
typedef struct {
    CarCommand_t command;
    uint32_t sequence, received_tick;
    uint8_t valid;
} CarRemoteInput_t;
typedef struct {
    uint8_t vision_held, shot_held, vision_press, shot_press;
    uint8_t pd10_low, pd10_ready;
} CarLocalInput_t;
/* Foreground submission copies the snapshot. Invalidation is ISR-safe and
 * only latches a flag; neither entrypoint sends motor commands. */
void Car_Control_SubmitRemoteInput(const CarRemoteInput_t *input);
void Car_Control_InvalidateRemoteInput(void);
void Car_Control_SubmitLocalInput(const CarLocalInput_t *input);
void Car_Control_Init(void);
void Car_Control_Process(void);
CarState_t Car_Control_GetState(void);
/* Explicit diagnostic-image angle commands. Production remote targets are
 * managed by remote_heading.h; these calls are rejected in production. */
HAL_StatusTypeDef Car_Control_SetTargetYaw(float target_degrees);
HAL_StatusTypeDef Car_Control_AdjustTargetYaw(float delta_degrees);
#endif
