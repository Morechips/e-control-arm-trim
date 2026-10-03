#include "arm_kinematics.h"
#include <math.h>
#include <stdbool.h>
#include <stddef.h>

#define ARM_KINEMATICS_COS_TOLERANCE 0.00001f
#define ARM_KINEMATICS_SINGULAR_SIN 0.0001f
#define ARM_KINEMATICS_CALIBRATION_EPSILON 0.000001f
#define ARM_KINEMATICS_PI 3.14159265358979323846f

static bool GeometryValid(const ArmKinematicsGeometry_t *geometry)
{
    return geometry != NULL && isfinite(geometry->link_1_mm) &&
           isfinite(geometry->link_2_mm) && isfinite(geometry->tool_x_mm) &&
           isfinite(geometry->tool_z_mm) &&
           isfinite(geometry->tool_axis_offset_rad) &&
           geometry->link_1_mm > 0.0f && geometry->link_2_mm > 0.0f;
}

static bool CalibrationValid(const ArmServoCalibration_t *calibration)
{
    if (calibration == NULL ||
        calibration->min_position >= calibration->max_position ||
        calibration->position_a < calibration->min_position ||
        calibration->position_a > calibration->max_position ||
        calibration->position_b < calibration->min_position ||
        calibration->position_b > calibration->max_position ||
        calibration->position_a == calibration->position_b ||
        !isfinite(calibration->angle_a_rad) ||
        !isfinite(calibration->angle_b_rad))
        return false;
    return fabsf(calibration->angle_b_rad - calibration->angle_a_rad) >
           ARM_KINEMATICS_CALIBRATION_EPSILON;
}

ArmKinematicsResult_t ArmKinematics_Forward(
    const ArmKinematicsGeometry_t *geometry,
    const ArmJointAngles_t *joints,
    ArmPose2D_t *pose)
{
    float first_angle, wrist_angle, cosine, sine;
    if (!GeometryValid(geometry) || joints == NULL || pose == NULL ||
        !isfinite(joints->q0_rad) || !isfinite(joints->q1_rad) ||
        !isfinite(joints->q2_rad))
        return ARM_KINEMATICS_INVALID_ARGUMENT;

    first_angle = joints->q0_rad + joints->q1_rad;
    wrist_angle = first_angle + joints->q2_rad;
    cosine = cosf(wrist_angle);
    sine = sinf(wrist_angle);
    pose->x_mm = geometry->link_1_mm * cosf(joints->q0_rad) +
                 geometry->link_2_mm * cosf(first_angle) +
                 cosine * geometry->tool_x_mm - sine * geometry->tool_z_mm;
    pose->z_mm = geometry->link_1_mm * sinf(joints->q0_rad) +
                 geometry->link_2_mm * sinf(first_angle) +
                 sine * geometry->tool_x_mm + cosine * geometry->tool_z_mm;
    pose->phi_rad = wrist_angle + geometry->tool_axis_offset_rad;
    return ARM_KINEMATICS_OK;
}

ArmKinematicsResult_t ArmKinematics_Inverse(
    const ArmKinematicsGeometry_t *geometry,
    const ArmPose2D_t *pose,
    ArmElbowBranch_t branch,
    ArmJointAngles_t *joints)
{
    ArmJointAngles_t solution;
    float wrist_angle, cosine, sine, wrist_x, wrist_z;
    float radius_squared, cosine_q1, sine_q1_squared, sine_q1;
    if (!GeometryValid(geometry) || pose == NULL || joints == NULL ||
        !isfinite(pose->x_mm) || !isfinite(pose->z_mm) ||
        !isfinite(pose->phi_rad) ||
        (branch != ARM_ELBOW_NEGATIVE && branch != ARM_ELBOW_POSITIVE))
        return ARM_KINEMATICS_INVALID_ARGUMENT;

    wrist_angle = pose->phi_rad - geometry->tool_axis_offset_rad;
    cosine = cosf(wrist_angle);
    sine = sinf(wrist_angle);
    wrist_x = pose->x_mm -
              (cosine * geometry->tool_x_mm - sine * geometry->tool_z_mm);
    wrist_z = pose->z_mm -
              (sine * geometry->tool_x_mm + cosine * geometry->tool_z_mm);
    radius_squared = wrist_x * wrist_x + wrist_z * wrist_z;
    cosine_q1 = (radius_squared - geometry->link_1_mm * geometry->link_1_mm -
                 geometry->link_2_mm * geometry->link_2_mm) /
                (2.0f * geometry->link_1_mm * geometry->link_2_mm);
    if (cosine_q1 < -1.0f - ARM_KINEMATICS_COS_TOLERANCE ||
        cosine_q1 > 1.0f + ARM_KINEMATICS_COS_TOLERANCE)
        return ARM_KINEMATICS_UNREACHABLE;
    if (cosine_q1 < -1.0f) cosine_q1 = -1.0f;
    if (cosine_q1 > 1.0f) cosine_q1 = 1.0f;
    sine_q1_squared = 1.0f - cosine_q1 * cosine_q1;
    if (sine_q1_squared < 0.0f) sine_q1_squared = 0.0f;
    sine_q1 = sqrtf(sine_q1_squared);
    if (sine_q1 < ARM_KINEMATICS_SINGULAR_SIN)
        return ARM_KINEMATICS_SINGULAR;
    if (branch == ARM_ELBOW_NEGATIVE) sine_q1 = -sine_q1;

    solution.q1_rad = atan2f(sine_q1, cosine_q1);
    solution.q0_rad = atan2f(wrist_z, wrist_x) -
                      atan2f(geometry->link_2_mm * sine_q1,
                             geometry->link_1_mm +
                                 geometry->link_2_mm * cosine_q1);
    solution.q2_rad = wrist_angle - solution.q0_rad - solution.q1_rad;
    *joints = solution;
    return ARM_KINEMATICS_OK;
}

ArmKinematicsResult_t ArmKinematics_PositionToAngle(
    const ArmServoCalibration_t *calibration,
    uint16_t position,
    float *angle_rad)
{
    float slope;
    if (!CalibrationValid(calibration) || angle_rad == NULL)
        return ARM_KINEMATICS_INVALID_ARGUMENT;
    if (position < calibration->min_position ||
        position > calibration->max_position)
        return ARM_KINEMATICS_OUT_OF_RANGE;
    slope = (calibration->angle_b_rad - calibration->angle_a_rad) /
            ((float)calibration->position_b -
             (float)calibration->position_a);
    *angle_rad = calibration->angle_a_rad +
                 slope * ((float)position - (float)calibration->position_a);
    return ARM_KINEMATICS_OK;
}

ArmKinematicsResult_t ArmKinematics_AngleToPosition(
    const ArmServoCalibration_t *calibration,
    float angle_rad,
    uint16_t *position)
{
    float raw_position;
    if (!CalibrationValid(calibration) || position == NULL ||
        !isfinite(angle_rad))
        return ARM_KINEMATICS_INVALID_ARGUMENT;
    raw_position = (float)calibration->position_a +
                   (angle_rad - calibration->angle_a_rad) *
                       ((float)calibration->position_b -
                        (float)calibration->position_a) /
                       (calibration->angle_b_rad -
                        calibration->angle_a_rad);
    if (raw_position < (float)calibration->min_position - 0.5f ||
        raw_position > (float)calibration->max_position + 0.5f)
        return ARM_KINEMATICS_OUT_OF_RANGE;
    if (raw_position < (float)calibration->min_position)
        raw_position = (float)calibration->min_position;
    if (raw_position > (float)calibration->max_position)
        raw_position = (float)calibration->max_position;
    *position = (uint16_t)(raw_position + 0.5f);
    return ARM_KINEMATICS_OK;
}
