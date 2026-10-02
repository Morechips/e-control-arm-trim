#ifndef ARM_KINEMATICS_H
#define ARM_KINEMATICS_H

#include <stdint.h>

typedef enum
{
    ARM_KINEMATICS_OK = 0,
    ARM_KINEMATICS_INVALID_ARGUMENT,
    ARM_KINEMATICS_UNREACHABLE,
    ARM_KINEMATICS_SINGULAR,
    ARM_KINEMATICS_OUT_OF_RANGE
} ArmKinematicsResult_t;

typedef enum
{
    ARM_ELBOW_NEGATIVE = -1,
    ARM_ELBOW_POSITIVE = 1
} ArmElbowBranch_t;

typedef struct
{
    float link_1_mm;
    float link_2_mm;
    /* Wrist-frame vector from joint 002 to the selected virtual grasp point. */
    float tool_x_mm;
    float tool_z_mm;
    /* Jaw/tool direction minus the wrist-frame X direction. */
    float tool_axis_offset_rad;
} ArmKinematicsGeometry_t;

typedef struct
{
    float q0_rad;
    float q1_rad;
    float q2_rad;
} ArmJointAngles_t;

typedef struct
{
    float x_mm;
    float z_mm;
    float phi_rad;
} ArmPose2D_t;

typedef struct
{
    uint16_t min_position;
    uint16_t max_position;
    uint16_t position_a;
    uint16_t position_b;
    float angle_a_rad;
    float angle_b_rad;
} ArmServoCalibration_t;

/* Pure calculations only: these functions never command hardware. */
ArmKinematicsResult_t ArmKinematics_Forward(
    const ArmKinematicsGeometry_t *geometry,
    const ArmJointAngles_t *joints,
    ArmPose2D_t *pose);

/* Solves one of the two planar elbow branches. Singular, unreachable and
 * invalid requests are rejected without returning a motion command. */
ArmKinematicsResult_t ArmKinematics_Inverse(
    const ArmKinematicsGeometry_t *geometry,
    const ArmPose2D_t *pose,
    ArmElbowBranch_t branch,
    ArmJointAngles_t *joints);

/* Linear two-point calibration. The range is numeric P order, independent of
 * whether increasing P moves a joint forward, backward, clockwise or
 * anticlockwise. Calibration points must lie inside that range. */
ArmKinematicsResult_t ArmKinematics_PositionToAngle(
    const ArmServoCalibration_t *calibration,
    uint16_t position,
    float *angle_rad);
ArmKinematicsResult_t ArmKinematics_AngleToPosition(
    const ArmServoCalibration_t *calibration,
    float angle_rad,
    uint16_t *position);

/* Project-specific measured model. These helpers remain pure calculations and
 * never transmit servo commands. Pose Z is relative to joint 000; add
 * ARM_BASE_AXIS_HEIGHT_MM when a ground-referenced height is required. */
void ArmKinematics_ProjectGeometry(ArmKinematicsGeometry_t *geometry);
void ArmKinematics_ProjectCalibrations(
    ArmServoCalibration_t calibrations[3]);
ArmKinematicsResult_t ArmKinematics_ProjectForward(
    const uint16_t positions[3],
    ArmPose2D_t *pose);
ArmKinematicsResult_t ArmKinematics_ProjectInverse(
    const ArmPose2D_t *pose,
    ArmElbowBranch_t branch,
    uint16_t positions[3]);

#endif

