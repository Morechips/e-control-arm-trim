#ifndef ARM_TRIM_BLUETOOTH_H
#define ARM_TRIM_BLUETOOTH_H

#include <stdbool.h>
#include <stdint.h>
#include "arm_trim.h"

/* Thin project adapter: foreground only; no receiver callback registration.
 * BEGIN requires caller to establish the named real pose and parked chassis.
 * This module neither moves to the reference nor controls joint 003. */
void ArmTrimBluetooth_Init(void);
bool ArmTrimBluetooth_HandleLine(const char *line, uint32_t arrival_tick,
                                 bool legacy_session);
void ArmTrimBluetooth_Process(void);
void ArmTrimBluetooth_Cancel(void);
bool ArmTrimBluetooth_OwnsMotion(void);
/* Lets the outer task capture the completed three-joint target before END.
 * It must not replay the original reach joints when commanding the gripper. */
ArmTrimStatus_t ArmTrimBluetooth_GetStatus(void);
/* Outer workflow uses these after its timed reference move, or before
 * switching reference. They do not send position commands. */
ArmTrimResult_t ArmTrimBluetooth_BeginProfile(const char *name, bool legacy_session);
ArmTrimResult_t ArmTrimBluetooth_End(void);

#endif
