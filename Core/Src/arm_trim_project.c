#include "arm_trim_project.h"
#include "arm_trim_project_config.h"
#include "arm_collision_config.h"
#include <string.h>

#define PROJECT_PI 3.14159265358979323846f

void ArmTrimProject_Geometry(ArmKinematicsGeometry_t *geometry)
{
    if (geometry == NULL) return;
    *geometry = (ArmKinematicsGeometry_t){
        ARM_TRIM_PROJECT_LINK_1_MM, ARM_TRIM_PROJECT_LINK_2_MM,
        ARM_TRIM_PROJECT_TOOL_X_MM, ARM_TRIM_PROJECT_TOOL_Z_MM,
        ARM_TRIM_PROJECT_TOOL_AXIS_OFFSET_RAD
    };
}

void ArmTrimProject_Calibrations(ArmServoCalibration_t calibration[3])
{
    if (calibration == NULL) return;
    calibration[0] = (ArmServoCalibration_t){
        ARM_TRIM_PROJECT_P0_MIN, ARM_TRIM_PROJECT_P0_MAX,
        ARM_TRIM_PROJECT_P0_A, ARM_TRIM_PROJECT_P0_B, 0.0f, PROJECT_PI / 4.0f
    };
    calibration[1] = (ArmServoCalibration_t){
        ARM_TRIM_PROJECT_P1_MIN, ARM_TRIM_PROJECT_P1_MAX,
        ARM_TRIM_PROJECT_P1_A, ARM_TRIM_PROJECT_P1_B, 0.0f, -PROJECT_PI / 2.0f
    };
    calibration[2] = (ArmServoCalibration_t){
        ARM_TRIM_PROJECT_P2_MIN, ARM_TRIM_PROJECT_P2_MAX,
        ARM_TRIM_PROJECT_P2_A, ARM_TRIM_PROJECT_P2_B, 0.0f, -PROJECT_PI / 2.0f
    };
}

void ArmTrimProject_Collision(ArmCollisionModel_t *collision)
{
    if (collision == NULL) return;
    memset(collision, 0, sizeof(*collision));
    collision->enabled = ARM_COLLISION_PROJECT_ENABLED != 0;
    collision->box_min[0] = ARM_REAR_BOX_X_MIN_MM;
    collision->box_max[0] = ARM_REAR_BOX_X_MAX_MM;
    collision->box_min[1] = ARM_REAR_BOX_Y_MIN_MM;
    collision->box_max[1] = ARM_REAR_BOX_Y_MAX_MM;
    collision->box_min[2] = ARM_REAR_BOX_Z_MIN_MM;
    collision->box_max[2] = ARM_REAR_BOX_Z_MAX_MM;
    collision->radius_mm[0] = ARM_COLLISION_LINK1_RADIUS_MM;
    collision->radius_mm[1] = ARM_COLLISION_LINK2_RADIUS_MM;
    collision->radius_mm[2] = ARM_COLLISION_TOOL_RADIUS_MM;
    collision->clearance_mm = ARM_COLLISION_CLEARANCE_MM;
    collision->sweep_resolution_mm = ARM_COLLISION_SWEEP_RESOLUTION_MM;
}

void ArmTrimProject_DefaultConfig(ArmTrimConfig_t *config)
{
    if (config == NULL) return;
    ArmTrim_DefaultConfig(config);
    ArmTrimProject_Geometry(&config->geometry);
    ArmTrimProject_Calibrations(config->calibration);
    ArmTrimProject_Collision(&config->collision);
    config->enabled_min_mm = ARM_TRIM_PROJECT_MIN_MM;
    config->enabled_max_mm = ARM_TRIM_PROJECT_MAX_MM;
}
