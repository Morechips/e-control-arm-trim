#ifndef MAIN_H
#define MAIN_H

#include "stm32f4xx_hal.h"

/* A 41-byte frame at 9600 8N1 takes 42.7 ms: 20 Hz uses about 85% of
 * the wire. Consider 115200 only after hardware measurement; keep 9600 here. */
#ifndef BT_UART_BAUD_RATE
#define BT_UART_BAUD_RATE 9600U
#endif

/* USART1 debug log / optional diagnostic bridge: PA9=TX, PA10=RX. */
#ifndef UART1_BAUD_RATE
#define UART1_BAUD_RATE 115200U
#endif

#define MOTOR1_ADDRESS 1U
#define MOTOR1_UART_BAUD_RATE 115200U
#define MOTOR1_START_DELAY_MS 30U

/* Override these for modules that use a different AT command dialect. */
#define BT_DEVICE_NAME "AIOTCAR"
#ifndef BT_AT_NAME_COMMAND
#define BT_AT_NAME_COMMAND "AT+NAME=" BT_DEVICE_NAME "\r\n"
#endif

/* One-shot factory provisioning only. The advertising name is stored in the
 * module, so renaming on every boot is unnecessary and it used to block the
 * control loop for ~0.8-1.4 s before USART6 reception was armed. Leave this 0
 * for normal operation: set it to 1, flash, power on once, then set it back. */
#ifndef BT_AT_NAME_ENABLE
#define BT_AT_NAME_ENABLE 0U
#endif

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;
void Error_Handler(void);

#endif
