#include "arm_trim.h"
#include <stdio.h>

static unsigned sends;
static ArmTrim_t trim;
static bool Send(void *user, const uint16_t p[3], uint16_t ms)
{ (void)user; (void)p; (void)ms; ++sends; return false; }
static bool Stop(void *user, unsigned joint) { (void)user; (void)joint; return false; }
static uint32_t Now(void *user) { (void)user; return 0U; }

static ArmJointAngles_t Angles(const ArmTrimConfig_t *c, const uint16_t p[3])
{
    ArmJointAngles_t q = {0};
    float *angle[] = {&q.q0_rad, &q.q1_rad, &q.q2_rad};
    unsigned i;
    for (i = 0U; i < 3U; ++i)
        (void)ArmKinematics_PositionToAngle(&c->calibration[i], p[i], angle[i]);
    return q;
}

static float SkeletonGap(ArmCollisionModel_t m, const ArmKinematicsGeometry_t *g,
    const ArmJointAngles_t *q)
{
    float low = 0.0f, high = 1000.0f;
    unsigned i;
    m.radius_mm[0] = m.radius_mm[1] = m.radius_mm[2] = 0.0f;
    for (i = 0U; i < 24U; ++i) {
        float mid = (low + high)/2.0f;
        m.clearance_mm = mid;
        if (ArmCollision_CheckPose(&m, g, q) == ARM_COLLISION_CLEAR) low = mid;
        else high = mid;
    }
    return low;
}

int main(void)
{
    const char *name[] = {"BALL", "HOSTAGE", "BUCKET", "HOSTAGE_LIFT"};
    const uint16_t p[][3] = {{1356,1850,698},{1684,2136,785},{1566,1896,673},{1800,1855,528}};
    ArmTrimConfig_t c;
    ArmTrimIO_t io = {Send, Stop, Now, NULL, NULL};
    ArmCollisionModel_t box;
    unsigned i;
    ArmTrim_DefaultConfig(&c);
    ArmKinematics_ProjectGeometry(&c.geometry);
    ArmKinematics_ProjectCalibrations(c.calibration);
    ArmCollision_ProjectModel(&box);
    c.enabled_min_mm = -75.0f; c.enabled_max_mm = 75.0f;
    printf("MODEL ONLY; no hardware; box X[-70,-30] Y[-50,50] Z[-31.2,38.8] mm\n");
    printf("Provisional radii %.1f,%.1f,%.1f mm; extra clearance %.1f mm\n",
        (double)box.radius_mm[0], (double)box.radius_mm[1], (double)box.radius_mm[2], (double)box.clearance_mm);
    for (i = 0U; i < 4U; ++i) {
        ArmJointAngles_t q = Angles(&c, p[i]);
        unsigned enabled;
        printf("%s P=%u,%u,%u static=%s zero_thickness_centerline_gap=%.1f mm\n",
            name[i], p[i][0], p[i][1], p[i][2],
            ArmCollision_CheckPose(&box, &c.geometry, &q) == ARM_COLLISION_CLEAR ? "CLEAR_MODEL" : "BLOCKED_MODEL",
            (double)SkeletonGap(box, &c.geometry, &q));
        if (i == 3U) continue;
        for (enabled = 0U; enabled < 2U; ++enabled) {
            c.collision = box; c.collision.enabled = enabled != 0U;
            if (ArmTrim_Init(&trim, &c, &io) != ARM_TRIM_OK ||
                ArmTrim_Synchronize(&trim, p[i]) != ARM_TRIM_OK) return 1;
            printf("  %s range=%.0f..%.0f mm\n", enabled ? "REAR_BOX_GUARD" : "KINEMATICS_ONLY",
                (double)trim.status.model_min_mm, (double)trim.status.model_max_mm);
        }
    }
    {
        ArmJointAngles_t reach = Angles(&c, p[1]), lift = Angles(&c, p[3]);
        printf("HOSTAGE -> HOSTAGE_LIFT linear joint path result=%u (0 clear,1 blocked,2 invalid)\n",
            (unsigned)ArmCollision_CheckEdge(&box, &c.geometry, &reach, &lift));
    }
    printf("Servo sends=%u\n", sends);
    return sends != 0U;
}
