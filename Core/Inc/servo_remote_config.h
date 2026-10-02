#ifndef SERVO_REMOTE_CONFIG_H
#define SERVO_REMOTE_CONFIG_H

#include "main.h"

/* PE4 button: internal pull-up, pressed to ground. */
#define SERVO_BUTTON_GPIO_PORT GPIOE
#define SERVO_BUTTON_GPIO_PIN GPIO_PIN_4
#define SERVO_BUTTON_GPIO_CLK_ENABLE() __HAL_RCC_GPIOE_CLK_ENABLE()
#define SERVO_BUTTON_DEBOUNCE_MS 20U

#endif
