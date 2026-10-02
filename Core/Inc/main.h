#ifndef MAIN_H
#define MAIN_H

#include "stm32f4xx_hal.h"

/* Change this value if the Bluetooth module uses another data-mode baud rate. */
#ifndef BT_UART_BAUD_RATE
#define BT_UART_BAUD_RATE 9600U
#endif

/* USART1 debug log / optional diagnostic bridge: PA9=TX, PA10=RX. */
#ifndef UART1_BAUD_RATE
#define UART1_BAUD_RATE 115200U
#endif

#define BUTTON1_GPIO_PORT GPIOD
#define BUTTON1_GPIO_PINS (GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_14 | GPIO_PIN_15)
#define BUTTON1_DEBOUNCE_MS 20U

#define MOTOR1_ADDRESS 1U
#define MOTOR1_UART_BAUD_RATE 115200U
#define MOTOR1_START_DELAY_MS 30U

/* Override these for modules that use a different AT command dialect. */
#define BT_DEVICE_NAME "AIOTCAR"
#ifndef BT_AT_NAME_COMMAND
#define BT_AT_NAME_COMMAND "AT+NAME=" BT_DEVICE_NAME "\r\n"
#endif

#ifndef BT_AT_STARTUP_DELAY_MS
#define BT_AT_STARTUP_DELAY_MS 500U
#endif

#ifndef BT_AT_RESPONSE_TIMEOUT_MS
#define BT_AT_RESPONSE_TIMEOUT_MS 300U
#endif

#ifndef BT_AT_RESPONSE_INTERBYTE_TIMEOUT_MS
#define BT_AT_RESPONSE_INTERBYTE_TIMEOUT_MS 20U
#endif

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;
void Error_Handler(void);

#endif
