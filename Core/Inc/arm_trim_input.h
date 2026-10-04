#ifndef ARM_TRIM_INPUT_H
#define ARM_TRIM_INPUT_H

#include <stdbool.h>
#include <stdint.h>
#include "servo.h"

/* Selectable project integration. Pure planner consumers need no input layer. */
#ifndef ARM_TRIM_ENABLE
#define ARM_TRIM_ENABLE 0
#endif

#define ARM_TRIM_BUTTON_BALL   0x01U
#define ARM_TRIM_BUTTON_HOSTAGE 0x02U
#define ARM_TRIM_BUTTON_BUCKET 0x04U
#define ARM_TRIM_BUTTON_JOG    0x08U
#define ARM_TRIM_BUTTON_CLOSE  0x10U
#define ARM_TRIM_BUTTON_OPEN   0x20U
#define ARM_TRIM_BUTTON_STATUS 0x40U
#define ARM_TRIM_BUTTON_STOP   0x80U

void ArmTrimInput_Init(void);
/* Foreground reception copies one request; actions run in the late loop. */
void ArmTrimInput_Submit(uint8_t buttons, int16_t direction, uint32_t arrival_tick);
/* The unified car packet and legacy pose buttons enter the same foreground
 * consumer. requests counts preset/GAP edges, so conflicting presses are
 * consumed once rather than replayed after a busy motion. */
void ArmTrimInput_SubmitCombined(uint8_t buttons, int16_t direction,
                                 ServoCode preset, uint16_t gap_pwm,
                                 unsigned requests, uint32_t arrival_tick);
void ArmTrimInput_HandleLine(const char *line, uint32_t arrival_tick);
/* ISR-safe latch; no parsing, logging or servo I/O. */
void ArmTrimInput_Invalidate(void);
void ArmTrimInput_Process(void);

#endif
