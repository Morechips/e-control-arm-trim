/* Compatibility model for the retired arm tuner and its host tests. */
#include "arm_kinematics.h"
#include "arm_config.h"
#include <stddef.h>

#define ARM_KINEMATICS_PI 3.14159265358979323846f

void ArmKinematics_ProjectGeometry(ArmKinematicsGeometry_t *geometry)
{
    if (geometry == NULL) return;
    geometry->link_1_mm = ARM_LINK_1_MM;
    geometry->link_2_mm = ARM_LINK_2_MM;
    geometry->tool_x_mm = ARM_TOOL_X_MM;
    geometry->tool_z_mm = ARM_TOOL_Z_MM;
    geometry->tool_axis_offset_rad = 0.0f;
}

void ArmKinematics_ProjectCalibrations(
    ArmServoCalibration_t calibrations[3])
{
    if (calibrations == NULL) return;
    calibrations[0] = (ArmServoCalibration_t){
        ARM_P0_MIN, ARM_P0_MAX,
        ARM_P0_HORIZONTAL_GRASP, ARM_P0_UP_45,
        0.0f, ARM_KINEMATICS_PI / 4.0f
    };
    calibrations[1] = (ArmServoCalibration_t){
        ARM_P1_MIN, ARM_P1_MAX,
        ARM_P1_ALIGNED_WITH_LINK_1, ARM_P1_TOWARD_GRASP_90,
        0.0f, -ARM_KINEMATICS_PI / 2.0f
    };
    calibrations[2] = (ArmServoCalibration_t){
        ARM_P2_MIN, ARM_P2_MAX,
        ARM_P2_HORIZONTAL_GRASP, ARM_P2_VERTICAL_DOWN,
        0.0f, -ARM_KINEMATICS_PI / 2.0f
    };
}

ArmKinematicsResult_t ArmKinematics_ProjectForward(
    const uint16_t positions[3],
    ArmPose2D_t *pose)
{
    ArmKinematicsGeometry_t geometry;
    ArmServoCalibration_t calibrations[3];
    ArmJointAngles_t joints;
    ArmKinematicsResult_t result;
    float *const angles[3] = {
        &joints.q0_rad, &joints.q1_rad, &joints.q2_rad
    };
    size_t index;
    if (positions == NULL || pose == NULL)
        return ARM_KINEMATICS_INVALID_ARGUMENT;
    ArmKinematics_ProjectGeometry(&geometry);
    ArmKinematics_ProjectCalibrations(calibrations);
    for (index = 0U; index < 3U; ++index) {
        result = ArmKinematics_PositionToAngle(&calibrations[index],
                                               positions[index],
                                               angles[index]);
        if (result != ARM_KINEMATICS_OK) return result;
    }
    return ArmKinematics_Forward(&geometry, &joints, pose);
}

ArmKinematicsResult_t ArmKinematics_ProjectInverse(
    const ArmPose2D_t *pose,
    ArmElbowBranch_t branch,
    uint16_t positions[3])
{
    ArmKinematicsGeometry_t geometry;
    ArmServoCalibration_t calibrations[3];
    ArmJointAngles_t joints;
    const float *angles[3];
    uint16_t candidate[3];
    ArmKinematicsResult_t result;
    size_t index;
    if (pose == NULL || positions == NULL)
        return ARM_KINEMATICS_INVALID_ARGUMENT;
    ArmKinematics_ProjectGeometry(&geometry);
    ArmKinematics_ProjectCalibrations(calibrations);
    result = ArmKinematics_Inverse(&geometry, pose, branch, &joints);
    if (result != ARM_KINEMATICS_OK) return result;
    angles[0] = &joints.q0_rad;
    angles[1] = &joints.q1_rad;
    angles[2] = &joints.q2_rad;
    for (index = 0U; index < 3U; ++index) {
        result = ArmKinematics_AngleToPosition(&calibrations[index],
                                               *angles[index],
                                               &candidate[index]);
        if (result != ARM_KINEMATICS_OK) return result;
    }
    for (index = 0U; index < 3U; ++index) positions[index] = candidate[index];
    return ARM_KINEMATICS_OK;
}
