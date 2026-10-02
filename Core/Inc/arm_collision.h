#ifndef ARM_COLLISION_H
#define ARM_COLLISION_H

#include "arm_kinematics.h"
#include <stdbool.h>

typedef enum {
    ARM_COLLISION_CLEAR = 0,
    ARM_COLLISION_BLOCKED,
    ARM_COLLISION_INVALID
} ArmCollisionResult_t;

/* Coordinates relative to joint 000, +X toward the grasp object, +Z up.
 * The arm moves in Y=0. Each modeled body is a capsule around one of
 * 000->001, 001->002, 002->virtual grasp point, respectively. Radii must
 * cover the corresponding servo/bracket/tool bodies, not just the rods.
 * Real body coverage and installation accuracy remain caller responsibilities. */
typedef struct {
    bool enabled;
    float box_min[3], box_max[3]; /* X, Y, Z */
    float radius_mm[3];
    float clearance_mm;
    float sweep_resolution_mm;
} ArmCollisionModel_t;

bool ArmCollision_ModelValid(const ArmCollisionModel_t *model);
/* Disabled models return CLEAR; a malformed enabled model fails closed.
 * No allocation, HAL, feedback, self-collision or hardware commands. */
ArmCollisionResult_t ArmCollision_CheckPose(const ArmCollisionModel_t *model,
    const ArmKinematicsGeometry_t *geometry, const ArmJointAngles_t *joints);
/* Checks linear JOINT interpolation, including a displacement bound between
 * samples. This is conservative for these modeled capsules; it is not a
 * guarantee that the real arm follows the commanded joint interpolation. */
ArmCollisionResult_t ArmCollision_CheckEdge(const ArmCollisionModel_t *model,
    const ArmKinematicsGeometry_t *geometry, const ArmJointAngles_t *from,
    const ArmJointAngles_t *to);
void ArmCollision_ProjectModel(ArmCollisionModel_t *model);

#endif
