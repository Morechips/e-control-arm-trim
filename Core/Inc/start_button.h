#ifndef START_BUTTON_H
#define START_BUTTON_H

#include "main.h"

/* Single debounced start key; the only physical input of the production image.
 * A press that is held through power-up must be released once before it can
 * produce an event. Sampling happens in Car_Control_Process(). */
void StartButton_Init(void);
void StartButton_Process(void);
/* One-shot: returns 1 once per debounced press, 0 afterwards. */
uint8_t StartButton_TakePressEvent(void);
uint8_t StartButton_IsHeld(void);

#endif
