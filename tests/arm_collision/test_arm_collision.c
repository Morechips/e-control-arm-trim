#include "arm_trim_project.h"
#include "arm_collision.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define CHECK(c) do { ++checks; if (!(c)) { fprintf(stderr, "FAIL %u: %s\n", (unsigned)__LINE__, #c); exit(1); } } while (0)
static unsigned checks;

static ArmCollisionModel_t Model(void)
{
    ArmCollisionModel_t m = {0};
    m.enabled = true;
    m.box_min[0] = 8.0f; m.box_max[0] = 12.0f;
    m.box_min[1] = -1.0f; m.box_max[1] = 1.0f;
    m.box_min[2] = -2.0f; m.box_max[2] = 2.0f;
    m.radius_mm[0] = m.radius_mm[1] = m.radius_mm[2] = 0.2f;
    m.sweep_resolution_mm = 0.5f;
    return m;
}

static void TestGeometry(void)
{
    ArmKinematicsGeometry_t g = {10.0f, 1.0f, 1.0f, 0.0f, 0.0f};
    ArmJointAngles_t a = {1.04719755f, 0.0f, 0.0f}, b = {-1.04719755f, 0.0f, 0.0f};
    ArmJointAngles_t zero = {0}, invalid = {NAN, 0.0f, 0.0f};
    ArmCollisionModel_t m = Model();
    CHECK(ArmCollision_ModelValid(&m));
    CHECK(ArmCollision_CheckPose(&m, &g, &zero) == ARM_COLLISION_BLOCKED);
    CHECK(ArmCollision_CheckPose(&m, &g, &a) == ARM_COLLISION_CLEAR);
    CHECK(ArmCollision_CheckPose(&m, &g, &b) == ARM_COLLISION_CLEAR);
    CHECK(ArmCollision_CheckEdge(&m, &g, &a, &b) == ARM_COLLISION_BLOCKED);
    CHECK(ArmCollision_CheckEdge(&m, &g, &a, &a) == ARM_COLLISION_CLEAR);
    CHECK(ArmCollision_CheckPose(&m, &g, &invalid) == ARM_COLLISION_INVALID);
    CHECK(ArmCollision_CheckEdge(&m, &g, &a, &invalid) == ARM_COLLISION_INVALID);
    /* The box misses the motion plane, but thick bodies can still reach it. */
    m.box_min[1] = 2.0f; m.box_max[1] = 3.0f;
    CHECK(ArmCollision_CheckPose(&m, &g, &zero) == ARM_COLLISION_CLEAR);
    m.radius_mm[0] = 2.0f;
    CHECK(ArmCollision_CheckPose(&m, &g, &zero) == ARM_COLLISION_BLOCKED);
    /* Finite capsule thickness must be used; no centerline-only clearance. */
    m = Model(); m.box_min[0] = 5.0f; m.box_max[0] = 6.0f;
    m.box_min[2] = 1.0f; m.box_max[2] = 2.0f;
    CHECK(ArmCollision_CheckPose(&m, &g, &zero) == ARM_COLLISION_CLEAR);
    m.radius_mm[0] = 1.0f;
    CHECK(ArmCollision_CheckPose(&m, &g, &zero) == ARM_COLLISION_BLOCKED);
    m.radius_mm[0] = 0.2f; m.clearance_mm = 0.8f;
    CHECK(ArmCollision_CheckPose(&m, &g, &zero) == ARM_COLLISION_BLOCKED);
    /* Thin obstacle during rotation: no gap between sampling points is ignored. */
    m = Model(); m.box_min[0] = 9.999f; m.box_max[0] = 10.001f;
    m.box_min[2] = 0.073f; m.box_max[2] = 0.074f;
    m.radius_mm[0] = m.radius_mm[1] = m.radius_mm[2] = 0.0f;
    CHECK(ArmCollision_CheckEdge(&m, &g, &a, &b) == ARM_COLLISION_BLOCKED);
    /* Disabled model is explicit, and invalid enabled models fail closed. */
    m = Model(); m.enabled = false;
    CHECK(ArmCollision_CheckPose(&m, NULL, NULL) == ARM_COLLISION_CLEAR);
    CHECK(ArmCollision_CheckEdge(&m, NULL, NULL, NULL) == ARM_COLLISION_CLEAR);
    m.enabled = true; m.box_min[0] = m.box_max[0];
    CHECK(!ArmCollision_ModelValid(&m));
    CHECK(ArmCollision_CheckPose(&m, &g, &zero) == ARM_COLLISION_INVALID);
    m = Model(); m.radius_mm[1] = -1.0f;
    CHECK(!ArmCollision_ModelValid(&m));
    m = Model(); m.clearance_mm = NAN;
    CHECK(!ArmCollision_ModelValid(&m));
    CHECK(ArmCollision_CheckPose(NULL, &g, &zero) == ARM_COLLISION_INVALID);
}

static void TestProject(void)
{
    ArmCollisionModel_t m;
    ArmKinematicsGeometry_t g;
    ArmServoCalibration_t c[3];
    const uint16_t p[][3] = {
        {1356U,1850U,698U}, {1684U,2136U,785U}, {1566U,1896U,673U},
        {1800U,1855U,528U}
    };
    unsigned i, j;
    ArmTrimProject_Collision(&m);
    ArmTrimProject_Geometry(&g);
    ArmTrimProject_Calibrations(c);
    CHECK(m.enabled && ArmCollision_ModelValid(&m));
    CHECK(m.box_min[0] == -70.0f && m.box_max[0] == -30.0f);
    CHECK(fabsf(m.box_min[2]+31.2f) < 0.001f && fabsf(m.box_max[2]-38.8f) < 0.001f);
    for (i = 0U; i < 4U; ++i) {
        ArmJointAngles_t q;
        float *angle[] = {&q.q0_rad, &q.q1_rad, &q.q2_rad};
        for (j = 0U; j < 3U; ++j)
            CHECK(ArmKinematics_PositionToAngle(&c[j], p[i][j], angle[j]) == ARM_KINEMATICS_OK);
        CHECK(ArmCollision_CheckPose(&m, &g, &q) ==
            (i == 3U ? ARM_COLLISION_BLOCKED : ARM_COLLISION_CLEAR));
    }
}

int main(void)
{
    TestGeometry();
    TestProject();
    printf("PASS arm collision: %u checks (modeled capsules, not hardware)\n", checks);
    return 0;
}
