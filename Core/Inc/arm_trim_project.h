#ifndef ARM_TRIM_PROJECT_H
#define ARM_TRIM_PROJECT_H

#include "arm_trim.h"

/* Installation parameters only; no HAL, transport, ownership or motion. */
void ArmTrimProject_Geometry(ArmKinematicsGeometry_t *geometry);
void ArmTrimProject_Calibrations(ArmServoCalibration_t calibration[3]);
void ArmTrimProject_Collision(ArmCollisionModel_t *collision);
void ArmTrimProject_DefaultConfig(ArmTrimConfig_t *config);

#endif
