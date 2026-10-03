#include "usart2_dma.h"
#include <assert.h>
#include <string.h>
UART_HandleTypeDef huart2;
volatile uint32_t usart2_rx_periods,usart2_rx_bytes,usart2_rx_overflows,usart2_rx_errors,usart2_rx_unread_batches;
volatile uint32_t usart2_rx_dma_errors,usart2_rx_uart_errors;
volatile uint32_t usart2_rx_ore_errors,usart2_rx_ne_errors,usart2_rx_fe_errors,usart2_rx_pe_errors;
volatile uint32_t usart2_last_dma_flags,usart2_last_uart_flags;
unsigned mock_reset_count, mock_discard_count;
HAL_StatusTypeDef mock_reset_result;
uint8_t mock_reset_bytes[5];
void USART2_DMA_Init(void) {}
void USART2_DMA_Process(void) {}
void USART2_DMA_DiscardPending(void) { mock_discard_count++; }
uint16_t USART2_DMA_Remaining(void) { return USART2_RX_BUFFER_SIZE; }
uint16_t USART2_DMA_ReadTimed(uint8_t *p,uint16_t capacity,uint32_t *tick)
{ (void)p;(void)capacity;*tick=HAL_GetTick();return 0; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n,uint32_t timeout)
{
    assert(u==&huart2 && n==5 && timeout==10);
    memcpy(mock_reset_bytes,p,5);mock_reset_count++;return mock_reset_result;
}
