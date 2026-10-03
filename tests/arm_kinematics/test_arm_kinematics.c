#include "arm_kinematics.h"
#include "arm_config.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define PI_F 3.14159265358979323846f
#define CHECK_TOLERANCE 0.0005f

static unsigned checks;

#define CHECK(condition) do { ++checks; if (!(condition)) { \
    fprintf(stderr, "FAIL line %u: %s\n", (unsigned)__LINE__, #condition); \
    exit(1); } } while (0)

static void CheckNear(float actual, float expected)
{
    CHECK(fabsf(actual - expected) <= CHECK_TOLERANCE);
}

static void TestForwardWithToolOffset(void)
{
    const ArmKinematicsGeometry_t geometry = {
        100.0f, 80.0f, 60.0f, -20.0f, 0.25f
    };
    const ArmJointAngles_t joints = {0.0f, 0.0f, 0.0f};
    ArmPose2D_t pose = {0};
    CHECK(ArmKinematics_Forward(&geometry, &joints, &pose) ==
          ARM_KINEMATICS_OK);
    CheckNear(pose.x_mm, 240.0f);
    CheckNear(pose.z_mm, -20.0f);
    CheckNear(pose.phi_rad, 0.25f);
}

static void TestInverseBranches(void)
{
    const ArmKinematicsGeometry_t geometry = {
        100.0f, 100.0f, 50.0f, 0.0f, 0.0f
    };
    const ArmPose2D_t target = {150.0f, 100.0f, 0.0f};
    ArmJointAngles_t positive = {0}, negative = {0};
    ArmPose2D_t reconstructed = {0};

    CHECK(ArmKinematics_Inverse(&geometry, &target, ARM_ELBOW_POSITIVE,
                                &positive) == ARM_KINEMATICS_OK);
    CheckNear(positive.q0_rad, 0.0f);
    CheckNear(positive.q1_rad, PI_F / 2.0f);
    CheckNear(positive.q2_rad, -PI_F / 2.0f);
    CHECK(ArmKinematics_Forward(&geometry, &positive, &reconstructed) ==
          ARM_KINEMATICS_OK);
    CheckNear(reconstructed.x_mm, target.x_mm);
    CheckNear(reconstructed.z_mm, target.z_mm);
    CheckNear(reconstructed.phi_rad, target.phi_rad);

    CHECK(ArmKinematics_Inverse(&geometry, &target, ARM_ELBOW_NEGATIVE,
                                &negative) == ARM_KINEMATICS_OK);
    CheckNear(negative.q0_rad, PI_F / 2.0f);
    CheckNear(negative.q1_rad, -PI_F / 2.0f);
    CheckNear(negative.q2_rad, 0.0f);
}

static void TestRoundTripWithOffset(void)
{
    const ArmKinematicsGeometry_t geometry = {
        104.85f, 88.65f, 63.0f, -25.4f, 0.2f
    };
    const ArmJointAngles_t original = {0.3f, -0.8f, 0.4f};
    ArmJointAngles_t solved = {0};
    ArmPose2D_t pose = {0}, reconstructed = {0};
    CHECK(ArmKinematics_Forward(&geometry, &original, &pose) ==
          ARM_KINEMATICS_OK);
    CHECK(ArmKinematics_Inverse(&geometry, &pose, ARM_ELBOW_NEGATIVE,
                                &solved) == ARM_KINEMATICS_OK);
    CheckNear(solved.q0_rad, original.q0_rad);
    CheckNear(solved.q1_rad, original.q1_rad);
    CheckNear(solved.q2_rad, original.q2_rad);
    CHECK(ArmKinematics_Forward(&geometry, &solved, &reconstructed) ==
          ARM_KINEMATICS_OK);
    CheckNear(reconstructed.x_mm, pose.x_mm);
    CheckNear(reconstructed.z_mm, pose.z_mm);
    CheckNear(reconstructed.phi_rad, pose.phi_rad);
}

static void TestRejectedSolutions(void)
{
    ArmKinematicsGeometry_t geometry = {
        100.0f, 100.0f, 50.0f, 0.0f, 0.0f
    };
    ArmPose2D_t pose = {1000.0f, 0.0f, 0.0f};
    ArmJointAngles_t joints = {9.0f, 9.0f, 9.0f};
    CHECK(ArmKinematics_Inverse(&geometry, &pose, ARM_ELBOW_POSITIVE,
                                &joints) == ARM_KINEMATICS_UNREACHABLE);
    CheckNear(joints.q0_rad, 9.0f);

    pose.x_mm = 250.0f;
    CHECK(ArmKinematics_Inverse(&geometry, &pose, ARM_ELBOW_POSITIVE,
                                &joints) == ARM_KINEMATICS_SINGULAR);
    CheckNear(joints.q0_rad, 9.0f);

    geometry.link_1_mm = 0.0f;
    CHECK(ArmKinematics_Inverse(&geometry, &pose, ARM_ELBOW_POSITIVE,
                                &joints) == ARM_KINEMATICS_INVALID_ARGUMENT);
    geometry.link_1_mm = 100.0f;
    CHECK(ArmKinematics_Inverse(&geometry, &pose, (ArmElbowBranch_t)0,
                                &joints) == ARM_KINEMATICS_INVALID_ARGUMENT);
    CHECK(ArmKinematics_Forward(NULL, &joints, &pose) ==
          ARM_KINEMATICS_INVALID_ARGUMENT);
}

static void TestServoCalibration(void)
{
    ArmServoCalibration_t calibration = {
        500U, 2500U, 2000U, 2400U, PI_F / 2.0f, 0.0f
    };
    float angle = 0.0f;
    uint16_t position = 0U;
    CHECK(ArmKinematics_PositionToAngle(&calibration, 2200U, &angle) ==
          ARM_KINEMATICS_OK);
    CheckNear(angle, PI_F / 4.0f);
    CHECK(ArmKinematics_AngleToPosition(&calibration, PI_F / 4.0f,
                                        &position) == ARM_KINEMATICS_OK);
    CHECK(position == 2200U);
    CHECK(ArmKinematics_PositionToAngle(&calibration, 499U, &angle) ==
          ARM_KINEMATICS_OUT_OF_RANGE);
    CHECK(ArmKinematics_AngleToPosition(&calibration, 8.0f, &position) ==
          ARM_KINEMATICS_OUT_OF_RANGE);

    calibration.position_b = calibration.position_a;
    CHECK(ArmKinematics_PositionToAngle(&calibration, 2000U, &angle) ==
          ARM_KINEMATICS_INVALID_ARGUMENT);
    calibration.position_b = 2400U;
    calibration.angle_b_rad = calibration.angle_a_rad;
    CHECK(ArmKinematics_AngleToPosition(&calibration, 0.0f, &position) ==
          ARM_KINEMATICS_INVALID_ARGUMENT);
}

static void TestMeasuredProjectPose(void)
{
    const uint16_t reference[3] = {
        ARM_P0_UP_45,
        ARM_P1_TOWARD_GRASP_90,
        ARM_P2_HORIZONTAL_GRASP
    };
    uint16_t solved[3] = {0U, 0U, 0U};
    ArmKinematicsGeometry_t geometry = {0};
    ArmPose2D_t pose = {0};
    float tool_length;

    ArmKinematics_ProjectGeometry(&geometry);
    tool_length = sqrtf(geometry.tool_x_mm * geometry.tool_x_mm +
                        geometry.tool_z_mm * geometry.tool_z_mm);
    CheckNear(tool_length, ARM_TOOL_REACH_MM);
    CheckNear(geometry.tool_z_mm, 52.4f);

    CHECK(ArmKinematics_ProjectForward(reference, &pose) ==
          ARM_KINEMATICS_OK);
    CheckNear(pose.x_mm, 0.70710678f *
              (ARM_LINK_1_MM + ARM_LINK_2_MM +
               ARM_TOOL_X_MM + ARM_TOOL_Z_MM));
    CheckNear(pose.z_mm, 0.70710678f *
              (ARM_LINK_1_MM - ARM_LINK_2_MM -
               ARM_TOOL_X_MM + ARM_TOOL_Z_MM));
    CheckNear(pose.phi_rad, -PI_F / 4.0f);

    CHECK(ArmKinematics_ProjectInverse(&pose, ARM_ELBOW_NEGATIVE, solved) ==
          ARM_KINEMATICS_OK);
    CHECK(solved[0] == reference[0]);
    CHECK(solved[1] == reference[1]);
    CHECK(solved[2] == reference[2]);
}

static void TestProjectRangeRejection(void)
{
    const uint16_t bad_position[3] = {
        ARM_P0_MIN - 1U, ARM_P1_ALIGNED_WITH_LINK_1,
        ARM_P2_HORIZONTAL_GRASP
    };
    uint16_t output[3] = {0U, 0U, 0U};
    ArmPose2D_t pose = {9.0f, 9.0f, 9.0f};
    CHECK(ArmKinematics_ProjectForward(bad_position, &pose) ==
          ARM_KINEMATICS_OUT_OF_RANGE);
    CheckNear(pose.x_mm, 9.0f);
    CHECK(ArmKinematics_ProjectForward(NULL, &pose) ==
          ARM_KINEMATICS_INVALID_ARGUMENT);
    CHECK(ArmKinematics_ProjectInverse(NULL, ARM_ELBOW_NEGATIVE,
                                       output) ==
          ARM_KINEMATICS_INVALID_ARGUMENT);
}

static void TestOperatorReferencePose(void)
{
    const uint16_t reference[3] = {
        ARM_REFERENCE_P0, ARM_REFERENCE_P1, ARM_REFERENCE_P2
    };
    uint16_t solved[3] = {0U, 0U, 0U};
    ArmPose2D_t pose = {0};
    CHECK(ArmKinematics_ProjectForward(reference, &pose) ==
          ARM_KINEMATICS_OK);
    CHECK(ArmKinematics_ProjectInverse(&pose, ARM_ELBOW_NEGATIVE, solved) ==
          ARM_KINEMATICS_OK);
    CHECK(solved[0] == reference[0]);
    CHECK(solved[1] == reference[1]);
    CHECK(solved[2] == reference[2]);
    CHECK(pose.x_mm > 230.0f && pose.x_mm < 240.0f);
    CHECK(pose.z_mm > 150.0f && pose.z_mm < 165.0f);
}

int main(void)
{
    TestForwardWithToolOffset();
    TestInverseBranches();
    TestRoundTripWithOffset();
    TestRejectedSolutions();
    TestServoCalibration();
    TestMeasuredProjectPose();
    TestProjectRangeRejection();
    TestOperatorReferencePose();
    printf("PASS arm kinematics: %u checks; 2-D tool offset, both IK branches, "
           "reachability, singularity and P-angle calibration.\n", checks);
    return 0;
}

