#ifndef ARM_TUNER_H
#define ARM_TUNER_H

#include <stdint.h>

#define ARM_TUNER_HEARTBEAT_MS 1500U
#define ARM_TUNER_SLOT_COUNT 4U
#define ARM_TUNER_MAX_STEP 50U
#define ARM_TUNER_MIN_MOVE_MS 100U
#define ARM_TUNER_MAX_JOG_MM 10
#define ARM_TUNER_MAX_JOG_DEG 5
#define ARM_TUNER_MAX_JOG_P_DELTA 80U
#define ARM_TUNER_REMOTE_DEADZONE 150
#define ARM_TUNER_REMOTE_STEP_MM 2
#define ARM_TUNER_REMOTE_MIN_PERIOD_MS 200U
#define ARM_TUNER_REMOTE_MAX_MOVE_MS 500U
#define ARM_TUNER_REMOTE_AXIS_HOLD_MS 600U
#define ARM_TUNER_REMOTE_WRIST_STEP_P 35U
#define ARM_TUNER_REMOTE_GRIP_STEP_P 40U
#define ARM_TUNER_REMOTE_GRIP_MOVE_MS 300U
#define ARM_TUNER_DUAL_REFERENCE_MS 2000U
#define ARM_TUNER_DUAL_RESTART_SETTLE_MS 300U
#define ARM_TUNER_DUAL_CENTER_SAMPLES 1U
#define ARM_TUNER_PRESET_MOVE_MS 1500U

/* Init once after Bluetooth_Init, PID_Tuner_Init and Arm_Init; silent.
 * No flash writes, position feedback, preset poses or task automation.
 * Foreground only. Process before Arm_Process on every main-loop iteration. */
void ArmTuner_Init(void);
void ArmTuner_Process(void);
uint8_t ArmTuner_IsSessionActive(void);

#endif

