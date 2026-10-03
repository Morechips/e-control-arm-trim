#include "arm_collision.h"
#include <math.h>
#include <stddef.h>

#define COLLISION_MAX_EDGE_SAMPLES 4096U

typedef struct { float x, z; } Point_t;

bool ArmCollision_ModelValid(const ArmCollisionModel_t *m)
{
    unsigned i;
    if (m == NULL) return false;
    if (!m->enabled) return true;
    for (i = 0U; i < 3U; ++i)
        if (!isfinite(m->box_min[i]) || !isfinite(m->box_max[i]) ||
            m->box_min[i] >= m->box_max[i] ||
            !isfinite(m->radius_mm[i]) || m->radius_mm[i] < 0.0f)
            return false;
    return isfinite(m->clearance_mm) && m->clearance_mm >= 0.0f &&
        isfinite(m->sweep_resolution_mm) && m->sweep_resolution_mm >= 0.1f &&
        m->sweep_resolution_mm <= 2.0f;
}

static float PointSegmentSquared(Point_t p, Point_t a, Point_t b)
{
    float x = b.x - a.x, z = b.z - a.z, length2 = x*x + z*z;
    float u = length2 > 0.0f ? ((p.x-a.x)*x + (p.z-a.z)*z)/length2 : 0.0f;
    u = fmaxf(0.0f, fminf(1.0f, u));
    x = p.x - a.x - u*x;
    z = p.z - a.z - u*z;
    return x*x + z*z;
}

static float PointBoxSquared(Point_t p, const ArmCollisionModel_t *m)
{
    float x = fmaxf(m->box_min[0]-p.x, fmaxf(0.0f, p.x-m->box_max[0]));
    float z = fmaxf(m->box_min[2]-p.z, fmaxf(0.0f, p.z-m->box_max[2]));
    return x*x + z*z;
}

static float SegmentBoxSquared(Point_t a, Point_t b, const ArmCollisionModel_t *m)
{
    float starts[2] = {a.x, a.z}, deltas[2] = {b.x-a.x, b.z-a.z};
    float low[2] = {m->box_min[0], m->box_min[2]};
    float high[2] = {m->box_max[0], m->box_max[2]};
    float enter = 0.0f, leave = 1.0f, closest;
    bool intersects = true;
    unsigned i, j;
    for (i = 0U; i < 2U; ++i) {
        if (deltas[i] == 0.0f) {
            if (starts[i] < low[i] || starts[i] > high[i]) intersects = false;
        } else {
            float t1 = (low[i]-starts[i])/deltas[i];
            float t2 = (high[i]-starts[i])/deltas[i];
            enter = fmaxf(enter, fminf(t1, t2));
            leave = fminf(leave, fmaxf(t1, t2));
        }
    }
    if (intersects && enter <= leave) return 0.0f;
    closest = fminf(PointBoxSquared(a, m), PointBoxSquared(b, m));
    for (i = 0U; i < 2U; ++i)
        for (j = 0U; j < 2U; ++j) {
            Point_t corner = {i == 0U ? low[0] : high[0], j == 0U ? low[1] : high[1]};
            closest = fminf(closest, PointSegmentSquared(corner, a, b));
        }
    return closest;
}

static ArmCollisionResult_t PoseWithPadding(const ArmCollisionModel_t *m,
    const ArmKinematicsGeometry_t *g, const ArmJointAngles_t *q, float extra)
{
    ArmPose2D_t grasp;
    Point_t p[4] = {{0.0f, 0.0f}};
    float angle, dy;
    unsigned i;
    if (ArmKinematics_Forward(g, q, &grasp) != ARM_KINEMATICS_OK)
        return ARM_COLLISION_INVALID;
    p[1] = (Point_t){g->link_1_mm*cosf(q->q0_rad), g->link_1_mm*sinf(q->q0_rad)};
    angle = q->q0_rad + q->q1_rad;
    p[2] = (Point_t){p[1].x + g->link_2_mm*cosf(angle), p[1].z + g->link_2_mm*sinf(angle)};
    p[3] = (Point_t){grasp.x_mm, grasp.z_mm};
    /* Segment Y is constant zero, so the nearest box Y is independent of X/Z. */
    dy = fmaxf(m->box_min[1], fmaxf(0.0f, -m->box_max[1]));
    for (i = 0U; i < 3U; ++i) {
        float radius = m->radius_mm[i] + m->clearance_mm + extra + 0.001f;
        float distance2 = SegmentBoxSquared(p[i], p[i+1U], m) + dy*dy;
        if (!isfinite(distance2) || !isfinite(radius)) return ARM_COLLISION_INVALID;
        if (distance2 <= radius*radius) return ARM_COLLISION_BLOCKED;
    }
    return ARM_COLLISION_CLEAR;
}

ArmCollisionResult_t ArmCollision_CheckPose(const ArmCollisionModel_t *m,
    const ArmKinematicsGeometry_t *g, const ArmJointAngles_t *q)
{
    if (!ArmCollision_ModelValid(m)) return ARM_COLLISION_INVALID;
    if (!m->enabled) return ARM_COLLISION_CLEAR;
    return PoseWithPadding(m, g, q, 0.0f);
}

ArmCollisionResult_t ArmCollision_CheckEdge(const ArmCollisionModel_t *m,
    const ArmKinematicsGeometry_t *g, const ArmJointAngles_t *a, const ArmJointAngles_t *b)
{
    ArmPose2D_t pose;
    float d0, d1, d2, displacement, extra, count;
    unsigned n, i;
    if (!ArmCollision_ModelValid(m)) return ARM_COLLISION_INVALID;
    if (!m->enabled) return ARM_COLLISION_CLEAR;
    if (ArmKinematics_Forward(g, a, &pose) != ARM_KINEMATICS_OK ||
        ArmKinematics_Forward(g, b, &pose) != ARM_KINEMATICS_OK)
        return ARM_COLLISION_INVALID;
    d0 = b->q0_rad-a->q0_rad;
    d1 = b->q1_rad-a->q1_rad;
    d2 = b->q2_rad-a->q2_rad;
    /* |d position/du| is bounded by sum(length * |d absolute angle/du|).
     * Every centerline point is a convex combination of segment endpoints;
     * a single bound therefore covers all three capsules. */
    displacement = g->link_1_mm*fabsf(d0) + g->link_2_mm*fabsf(d0+d1) +
        hypotf(g->tool_x_mm, g->tool_z_mm)*fabsf(d0+d1+d2);
    count = ceilf(displacement/(2.0f*m->sweep_resolution_mm));
    if (!isfinite(count) || count > (float)COLLISION_MAX_EDGE_SAMPLES)
        return ARM_COLLISION_INVALID;
    n = count < 1.0f ? 1U : (unsigned)count;
    extra = displacement/(2.0f*(float)n);
    for (i = 0U; i <= n; ++i) {
        float u = (float)i/(float)n;
        ArmJointAngles_t q = {a->q0_rad+u*d0, a->q1_rad+u*d1, a->q2_rad+u*d2};
        ArmCollisionResult_t result = PoseWithPadding(m, g, &q, extra);
        if (result != ARM_COLLISION_CLEAR) return result;
    }
    return ARM_COLLISION_CLEAR;
}
