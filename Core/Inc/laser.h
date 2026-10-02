#ifndef LASER_H
#define LASER_H

#include <stdbool.h>

/* PC3 is high impedance while inactive and sinks the active-low trigger. */
void Laser_Init(void);
/* Poll PB8 in the foreground; press is debounced, release is immediate. */
void Laser_ProcessButton(void);
/* These functions set and clear only the automatic task request. */
void Laser_Enable(void);
void Laser_Disable(void);
/* Effective PC3 trigger state, including a held manual button. */
bool Laser_IsEnabled(void);

#endif
