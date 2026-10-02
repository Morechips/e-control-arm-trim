#ifndef VISION_CONFIG_H
#define VISION_CONFIG_H

/* UART4 MaxiCam port: PC10 TX / PC11 RX, AF8, 8N1, no flow control. */
#ifndef MAXICAM_UART_BAUD_RATE
#define MAXICAM_UART_BAUD_RATE 115200U
#endif

/* Compatibility name retained for existing project configuration. */
#ifndef VISION_UART_BAUD_RATE
#define VISION_UART_BAUD_RATE MAXICAM_UART_BAUD_RATE

#define MAXICAM_MODE_TX_TIMEOUT_MS 5U
#endif

/* PE0 is active low; the test button connects the pin to ground. */
#define VISION_BUTTON_GPIO_PORT GPIOE
#define VISION_BUTTON_GPIO_PIN GPIO_PIN_0
#define VISION_BUTTON_GPIO_CLK_ENABLE() __HAL_RCC_GPIOE_CLK_ENABLE()
#define VISION_BUTTON_DEBOUNCE_MS 20U
#define VISION_FOLLOW_RPM 8
#define VISION_FRAME_TIMEOUT_MS 200U
#define VISION_SETTLE_MS 300U
#define VISION_SAMPLE_INTERVAL_MS 100U
#define VISION_STABLE_FRAMES 3U
#define VISION_STABLE_SPREAD_X 4

/* PC1 uses the same active-low, pull-up wiring as the PE0 test button. */
#define SHOT_BUTTON_GPIO_PORT GPIOC
#define SHOT_BUTTON_GPIO_PIN GPIO_PIN_1
#define SHOT_BUTTON_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()
#define SHOT_BUTTON_DEBOUNCE_MS 20U

#endif
