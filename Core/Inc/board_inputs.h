#ifndef BOARD_INPUTS_H
#define BOARD_INPUTS_H
#include <stdint.h>
void BoardInputs_Init(void);
void BoardInputs_ProcessControl(void);
uint8_t BoardInputs_TakeServoAimPress(void);
void BoardInputs_ProcessAux(void);
uint8_t BoardInputs_DisplayPressed(void);
/* Diagnostics only; supplied TX statistics do not trigger actions. */
void BoardInputs_TraceServo(uint32_t transmissions, unsigned last_result);
#endif
