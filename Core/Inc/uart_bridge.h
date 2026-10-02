#ifndef UART_BRIDGE_H
#define UART_BRIDGE_H
#include "stm32f4xx_hal.h"
/* Exclusive UART ownership; call Process repeatedly from foreground.
 * Uses the existing UART IRQ handlers. No X42 calls while active. */
HAL_StatusTypeDef UART_Bridge_Init(UART_HandleTypeDef *u1, UART_HandleTypeDef *motor);
void UART_Bridge_Process(void);
/* Second UART RX is supplied by DMA; bridge never arms its RX interrupt. */
HAL_StatusTypeDef UART_Bridge_InitDMA(UART_HandleTypeDef *u1, UART_HandleTypeDef *u2);
void UART_Bridge_FeedDMA(const uint8_t *data, uint16_t size);
extern volatile uint32_t usart1_to_usart2_bytes;
extern volatile uint32_t usart2_to_usart1_bytes;
extern volatile uint32_t usart1_to_uart5_bytes;
extern volatile uint32_t uart5_to_usart1_bytes;
extern volatile uint32_t bridge_dropped_bytes;
extern volatile uint32_t bridge_uart_errors;
#endif
