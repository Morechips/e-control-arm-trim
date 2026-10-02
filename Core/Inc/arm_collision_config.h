#ifndef ARM_COLLISION_CONFIG_H
#define ARM_COLLISION_CONFIG_H

/* Operator-confirmed rear box, 2026-10-02; millimeters relative to 000.
 * The board is 31.2 below 000, box height is 70, depth is 40, front gap is 30. */
#define ARM_COLLISION_PROJECT_ENABLED 1
#define ARM_REAR_BOX_X_MIN_MM (-70.0f)
#define ARM_REAR_BOX_X_MAX_MM (-30.0f)
#define ARM_REAR_BOX_Y_MIN_MM (-50.0f)
#define ARM_REAR_BOX_Y_MAX_MM 50.0f
#define ARM_REAR_BOX_Z_MIN_MM (-31.2f)
#define ARM_REAR_BOX_Z_MAX_MM 38.8f

/* Operator estimated arm thickness about 24mm, but the side-profile envelope
 * is not measured. Use 15/20mm radii as provisional padded estimates.
 * INITIAL ESTIMATES, not measured exterior envelopes. Tune these after
 * measuring the actual rods, servo housings, camera bracket and open gripper.
 * In particular the camera/tool must fit inside the third capsule. */
#define ARM_COLLISION_ENVELOPE_ESTIMATED 1
#define ARM_COLLISION_LINK1_RADIUS_MM 15.0f
#define ARM_COLLISION_LINK2_RADIUS_MM 20.0f
#define ARM_COLLISION_TOOL_RADIUS_MM 60.0f
#define ARM_COLLISION_CLEARANCE_MM 5.0f
/* Maximum added model displacement between an edge sample and its nearest
 * point in time. It is added to clearance, not used as an unguarded gap. */
#define ARM_COLLISION_SWEEP_RESOLUTION_MM 0.5f

#endif
