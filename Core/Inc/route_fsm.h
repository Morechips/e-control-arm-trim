#ifndef ROUTE_FSM_H
#define ROUTE_FSM_H

#include "action_fsm.h"
#include <stddef.h>

typedef struct
{
    ActionType action;
} RouteStep;

typedef enum
{
    ROUTE_INIT = 0,
    ROUTE_RUNNING,
    ROUTE_FINISHED,
    ROUTE_ERROR
} RouteState;

extern const RouteStep route[];
extern const size_t route_length;
extern size_t route_index;

/* RouteFSM_Init explicitly starts a new route. Update is foreground-only and
 * must be the exclusive owner of motion commands while RUNNING. Callers must
 * continue JY61_Process before, and Motor_Process after, every update. */
void RouteFSM_Init(void);
void RouteFSM_Update(void);

/* Future vision/path-end adapter: report one completed straight/translation
 * segment. Reports outside a running eligible action are ignored. */
void RouteFSM_NotifyActionEnd(void);

RouteState RouteFSM_GetState(void);
bool RouteFSM_QRStageDone(void);

#endif
