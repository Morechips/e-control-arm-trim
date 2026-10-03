#ifndef ARM_TRIM_PROJECT_CONFIG_H
#define ARM_TRIM_PROJECT_CONFIG_H


/* Installation-specific values; independent of the legacy arm_config.h. */
#ifndef ARM_TRIM_PROJECT_LINK_1_MM
#define ARM_TRIM_PROJECT_LINK_1_MM 104.85f
#endif
#ifndef ARM_TRIM_PROJECT_LINK_2_MM
#define ARM_TRIM_PROJECT_LINK_2_MM 84.75f
#endif
#ifndef ARM_TRIM_PROJECT_TOOL_X_MM
#define ARM_TRIM_PROJECT_TOOL_X_MM 121.1538f
#endif
#ifndef ARM_TRIM_PROJECT_TOOL_Z_MM
#define ARM_TRIM_PROJECT_TOOL_Z_MM 52.4f
#endif
#ifndef ARM_TRIM_PROJECT_TOOL_AXIS_OFFSET_RAD
#define ARM_TRIM_PROJECT_TOOL_AXIS_OFFSET_RAD 0.0f
#endif
#ifndef ARM_TRIM_PROJECT_P0_A
#define ARM_TRIM_PROJECT_P0_A 989U
#endif
#ifndef ARM_TRIM_PROJECT_P0_B
#define ARM_TRIM_PROJECT_P0_B 1309U
#endif
#ifndef ARM_TRIM_PROJECT_P1_A
#define ARM_TRIM_PROJECT_P1_A 1687U
#endif
#ifndef ARM_TRIM_PROJECT_P1_B
#define ARM_TRIM_PROJECT_P1_B 2344U
#endif
#ifndef ARM_TRIM_PROJECT_P2_A
#define ARM_TRIM_PROJECT_P2_A 1232U
#endif
#ifndef ARM_TRIM_PROJECT_P2_B
#define ARM_TRIM_PROJECT_P2_B 571U
#endif

/* Joint 000 uses +X as zero; 001/002 are relative to the preceding link. */
#ifndef ARM_TRIM_PROJECT_Q0_A_DEG
#define ARM_TRIM_PROJECT_Q0_A_DEG 0.0f
#endif
#ifndef ARM_TRIM_PROJECT_Q0_B_DEG
#define ARM_TRIM_PROJECT_Q0_B_DEG 45.0f
#endif
#ifndef ARM_TRIM_PROJECT_Q1_A_DEG
#define ARM_TRIM_PROJECT_Q1_A_DEG 0.0f
#endif
#ifndef ARM_TRIM_PROJECT_Q1_B_DEG
#define ARM_TRIM_PROJECT_Q1_B_DEG (-90.0f)
#endif
#ifndef ARM_TRIM_PROJECT_Q2_A_DEG
#define ARM_TRIM_PROJECT_Q2_A_DEG 0.0f
#endif
#ifndef ARM_TRIM_PROJECT_Q2_B_DEG
#define ARM_TRIM_PROJECT_Q2_B_DEG (-90.0f)
#endif
#ifndef ARM_TRIM_PROJECT_SEARCH_MM
#define ARM_TRIM_PROJECT_SEARCH_MM 75.0f
#endif
#ifndef ARM_TRIM_PROJECT_SPEED_MM_S
#define ARM_TRIM_PROJECT_SPEED_MM_S 10.0f
#endif
#ifndef ARM_TRIM_PROJECT_ACCELERATION_MM_S2
#define ARM_TRIM_PROJECT_ACCELERATION_MM_S2 20.0f
#endif
#ifndef ARM_TRIM_PROJECT_MAX_SEGMENT_MM
#define ARM_TRIM_PROJECT_MAX_SEGMENT_MM 2.0f
#endif
#ifndef ARM_TRIM_PROJECT_UPDATE_PERIOD_MS
#define ARM_TRIM_PROJECT_UPDATE_PERIOD_MS 50U
#endif
#ifndef ARM_TRIM_PROJECT_SETTLE_MS
#define ARM_TRIM_PROJECT_SETTLE_MS 300U
#endif

/* Use the operator-confirmed joint travel.  The v4
 * jog can use the contiguous model range within the original 75mm search;
 * these scalar bounds do not represent collision checks or real feedback. */
#ifndef ARM_TRIM_PROJECT_P0_MIN
#define ARM_TRIM_PROJECT_P0_MIN 915U
#endif
#ifndef ARM_TRIM_PROJECT_P0_MAX
#define ARM_TRIM_PROJECT_P0_MAX 1800U
#endif
#ifndef ARM_TRIM_PROJECT_P1_MIN
#define ARM_TRIM_PROJECT_P1_MIN 947U
#endif
#ifndef ARM_TRIM_PROJECT_P1_MAX
#define ARM_TRIM_PROJECT_P1_MAX 2500U
#endif
#ifndef ARM_TRIM_PROJECT_P2_MIN
#define ARM_TRIM_PROJECT_P2_MIN 500U
#endif
#ifndef ARM_TRIM_PROJECT_P2_MAX
#define ARM_TRIM_PROJECT_P2_MAX 1874U
#endif
#ifndef ARM_TRIM_PROJECT_MIN_MM
#define ARM_TRIM_PROJECT_MIN_MM (-75.0f)
#endif
#ifndef ARM_TRIM_PROJECT_MAX_MM
#define ARM_TRIM_PROJECT_MAX_MM 75.0f
#endif
#ifndef ARM_TRIM_BALL_P0
#define ARM_TRIM_BALL_P0 1356U
#endif
#ifndef ARM_TRIM_BALL_P1
#define ARM_TRIM_BALL_P1 1850U
#endif
#ifndef ARM_TRIM_BALL_P2
#define ARM_TRIM_BALL_P2 698U
#endif
/* Previous installation's pose. Operator will re-record 000..002 for the
 * changed arm; retain these values until the replacement is supplied. */
#ifndef ARM_TRIM_HOSTAGE_P0
#define ARM_TRIM_HOSTAGE_P0 1684U
#endif
#ifndef ARM_TRIM_HOSTAGE_P1
#define ARM_TRIM_HOSTAGE_P1 2136U
#endif
#ifndef ARM_TRIM_HOSTAGE_P2
#define ARM_TRIM_HOSTAGE_P2 785U
#endif
#ifndef ARM_TRIM_BUCKET_P0
#define ARM_TRIM_BUCKET_P0 1566U
#endif
#ifndef ARM_TRIM_BUCKET_P1
#define ARM_TRIM_BUCKET_P1 1896U
#endif
#ifndef ARM_TRIM_BUCKET_P2
#define ARM_TRIM_BUCKET_P2 673U
#endif

/* User-confirmed gripper targets, independent of planar trim. */
#ifndef ARM_TRIM_GRIPPER_MIN_P
#define ARM_TRIM_GRIPPER_MIN_P 500U
#endif
#ifndef ARM_TRIM_GRIPPER_MAX_P
#define ARM_TRIM_GRIPPER_MAX_P 2500U
#endif
#ifndef ARM_TRIM_BENCH_CLOSE_P
#define ARM_TRIM_BENCH_CLOSE_P 500U
#endif
#ifndef ARM_TRIM_BENCH_OPEN_P
#define ARM_TRIM_BENCH_OPEN_P 1800U
#endif

#endif
