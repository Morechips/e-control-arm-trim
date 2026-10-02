#ifndef ARM_TRIM_PROJECT_CONFIG_H
#define ARM_TRIM_PROJECT_CONFIG_H

#include "arm_config.h"

/* Use the operator-confirmed joint travel.  The v4
 * jog can use the contiguous model range within the original 75mm search;
 * these scalar bounds do not represent collision checks or real feedback. */
#define ARM_TRIM_PROJECT_P0_MIN ARM_P0_MIN
#define ARM_TRIM_PROJECT_P0_MAX ARM_P0_MAX
#define ARM_TRIM_PROJECT_P1_MIN ARM_P1_MIN
#define ARM_TRIM_PROJECT_P1_MAX ARM_P1_MAX
#define ARM_TRIM_PROJECT_P2_MIN ARM_P2_MIN
#define ARM_TRIM_PROJECT_P2_MAX ARM_P2_MAX
#define ARM_TRIM_PROJECT_MIN_MM (-75.0f)
#define ARM_TRIM_PROJECT_MAX_MM 75.0f
#define ARM_TRIM_BALL_P0 1356U
#define ARM_TRIM_BALL_P1 1850U
#define ARM_TRIM_BALL_P2 698U
/* Previous installation's pose. Operator will re-record 000..002 for the
 * changed arm; retain these values until the replacement is supplied. */
#define ARM_TRIM_HOSTAGE_P0 1684U
#define ARM_TRIM_HOSTAGE_P1 2136U
#define ARM_TRIM_HOSTAGE_P2 785U
#define ARM_TRIM_BUCKET_P0 1566U
#define ARM_TRIM_BUCKET_P1 1896U
#define ARM_TRIM_BUCKET_P2 673U

/* User-confirmed gripper targets, independent of planar trim. */
#define ARM_TRIM_BENCH_CLOSE_P 500U
#define ARM_TRIM_BENCH_OPEN_P 1800U

#endif
