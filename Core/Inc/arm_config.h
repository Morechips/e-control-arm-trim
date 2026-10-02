#ifndef ARM_CONFIG_H
#define ARM_CONFIG_H

/* Three planar arm joints, one gripper and one optional future channel. */
#define ARM_JOINT_COUNT 5U
#define ARM_MAX_SEQUENCE_STEPS 16U
#define ARM_SERVO_ID_UNASSIGNED 65535U
#define ARM_MAX_HOLD_MS 60000U
/* Foreground service deadline, not a mechanical motion parameter. If service
 * resumes late, cancel instead of launching another unobserved action. */
#define ARM_SERVICE_TIMEOUT_MS 250U

/* Initial measured 3R planar model. Coordinates use joint 000 as the origin,
 * +X toward the object and +Z upward. These are bench measurements rather
 * than nominal CAD dimensions, so later calibration may refine them. */
#define ARM_LINK_1_MM 104.85f
/* 001 axis -> 002 axis, operator clarified on 2026-10-02:
 * add the two measurements first, then halve: (82 + 87.5) / 2 = 84.75 mm. */
#define ARM_LINK_2_MM 84.75f
#define ARM_BASE_AXIS_HEIGHT_MM 120.0f
#define ARM_TOOL_REACH_MM 132.0f
#define ARM_TOOL_X_MM 121.1538f
#define ARM_TOOL_Z_MM 52.4f

/* Operator-measured joint travel on 2026-10-02. These scalar bounds are
 * NOT collision-free pose or path limits. */
#define ARM_P0_MIN 915U
/* Operator requested restoring P1800 on 2026-10-02. Joint travel alone
 * does not prove that the restored pose or its path clears obstacles. */
#define ARM_P0_MAX 1800U
#define ARM_P1_MIN 947U
#define ARM_P1_MAX 2500U
#define ARM_P2_MIN 500U
#define ARM_P2_MAX 1874U

/* Measured after the complete arm was yawed 180 degrees on the chassis.
 * Joint angles remain local to the arm plane: q0=0 is horizontal toward the
 * current grasp side; q1/q2=-90 degrees bend toward the grasp side/down. */
#define ARM_P0_HORIZONTAL_GRASP 989U
#define ARM_P0_UP_45 1309U
#define ARM_P1_ALIGNED_WITH_LINK_1 1687U
#define ARM_P1_TOWARD_GRASP_90 2344U
#define ARM_P2_HORIZONTAL_GRASP 1232U
#define ARM_P2_VERTICAL_DOWN 571U

/* Operator-confirmed simultaneous pose after reinstall. This is a convenient
 * Cartesian seed, not the servo controller's stored power-on action and not
 * position feedback.  Keep this independent from the controller-board boot
 * pose, which must be edited/downloaded with its upper-computer software. */
#define ARM_REFERENCE_P0 1532U
#define ARM_REFERENCE_P1 2219U
#define ARM_REFERENCE_P2 1202U
/* Dual-control startup also establishes a known, half-open gripper reference.
 * This avoids the first button press jumping between the two end positions. */
#define ARM_REFERENCE_P3 1200U
#define ARM_GRIPPER_MIN_P 900U
/* Recorded fully-open task poses use P2192. */
#define ARM_GRIPPER_MAX_P 2192U

#endif

