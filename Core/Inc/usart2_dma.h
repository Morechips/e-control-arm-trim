#ifndef USART2_DMA_H
#define USART2_DMA_H
#include "main.h"

#define USART2_RX_BUFFER_SIZE 256U
#ifndef USART2_RX_PERIOD_MS
#define USART2_RX_PERIOD_MS 5U
#endif
#if USART2_RX_PERIOD_MS != 5U && USART2_RX_PERIOD_MS != 2U
#error USART2_RX_PERIOD_MS_must_be_5_or_2
#endif
#define USART2_BAUD_RATE 9600U

extern UART_HandleTypeDef huart2;
extern volatile uint32_t usart2_rx_periods;
extern volatile uint32_t usart2_rx_bytes;
/* Suspected overflow/full-ring events, not lost bytes. Multiple wraps between
 * polls cannot be counted exactly; some overflows are undetectable. */
extern volatile uint32_t usart2_rx_overflows;
extern volatile uint32_t usart2_rx_errors;
extern volatile uint32_t usart2_rx_unread_batches;
extern volatile uint32_t usart2_rx_dma_errors, usart2_rx_uart_errors;
extern volatile uint32_t usart2_rx_ore_errors, usart2_rx_ne_errors;
extern volatile uint32_t usart2_rx_fe_errors, usart2_rx_pe_errors;
extern volatile uint32_t usart2_last_dma_flags, usart2_last_uart_flags;

void USART2_DMA_Init(void);
void USART2_DMA_Process(void);
void USART2_DMA_TimerIRQ(void);
/* Foreground, single consumer. Returns bytes copied, retains any unread suffix.
 * Latest nonempty batch replaces unread data; see unread_batches counter. */
uint16_t USART2_DMA_Read(uint8_t *destination, uint16_t capacity);
uint16_t USART2_DMA_ReadTimed(uint8_t *destination, uint16_t capacity, uint32_t *tick);
/* Software read barrier only; RX DMA is never stopped. */
void USART2_DMA_DiscardPending(void);
uint16_t USART2_DMA_Remaining(void);
#endif
