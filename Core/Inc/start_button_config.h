#ifndef START_BUTTON_CONFIG_H
#define START_BUTTON_CONFIG_H

#include "main.h"

/* The production firmware has exactly ONE physical input: the start key.
 * To move it to another pin, change these four macros (and nothing else).
 * The default is PD10, which the board already wires as a pull-up input. */
#ifndef START_BUTTON_GPIO_PORT
#define START_BUTTON_GPIO_PORT GPIOD
#endif
#ifndef START_BUTTON_GPIO_CLK_ENABLE
#define START_BUTTON_GPIO_CLK_ENABLE() __HAL_RCC_GPIOD_CLK_ENABLE()
#endif
#ifndef START_BUTTON_GPIO_PIN
#define START_BUTTON_GPIO_PIN GPIO_PIN_10
#endif
/* 1 = pressing connects the pin to ground (internal pull-up), as wired today. */
#ifndef START_BUTTON_ACTIVE_LOW
#define START_BUTTON_ACTIVE_LOW 1U
#endif

#ifndef START_BUTTON_DEBOUNCE_MS
#define START_BUTTON_DEBOUNCE_MS 20U
#endif

#endif
