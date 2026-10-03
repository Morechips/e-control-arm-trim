#ifndef UART_DRIVER_H
#define UART_DRIVER_H
#include <stddef.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
#ifndef UART_RX_CAPACITY
#define UART_RX_CAPACITY 256U
#endif
#ifndef UART_BINDING_CAPACITY
#define UART_BINDING_CAPACITY 8U
#endif

typedef enum { UART_TX_BLOCKING, UART_TX_INTERRUPT } UartTxMode_t;
typedef struct {
    UART_HandleTypeDef *uart;
    uint8_t byte, data[UART_RX_CAPACITY];
    uint32_t tick[UART_RX_CAPACITY];
    volatile uint16_t head, tail;
    volatile uint8_t broken;
    volatile uint32_t bytes;
} UartRx_t;
typedef uint16_t (*UartDmaRead_t)(uint8_t *, uint16_t, uint32_t *);
typedef void (*UartHook_t)(UART_HandleTypeDef *);
struct UartTxQueue;
typedef struct {
    UART_HandleTypeDef *uart;
    UartRx_t *rx;
    UartDmaRead_t dma_read;
    struct UartTxQueue *tx;
    UartHook_t rx_ready, error;
    uint32_t last_read_tick;
} UartBinding_t;
/* Binary lengths, never strlen. IT callers retain storage until completion. */
HAL_StatusTypeDef UART_SEND(const uint8_t *data, size_t length, UART_HandleTypeDef *uart,
                             UartTxMode_t mode, uint32_t timeout_ms);
/* ticks is optional, otherwise capacity entries. Empty reads succeed with 0 bytes. */
HAL_StatusTypeDef UART_RECV(uint8_t *data, size_t capacity, UART_HandleTypeDef *uart,
                             size_t *received, uint32_t *ticks);
HAL_StatusTypeDef UART_BindRx(UartRx_t *rx, UART_HandleTypeDef *uart,
                               UartHook_t ready, UartHook_t error);
HAL_StatusTypeDef UART_BindDma(UART_HandleTypeDef *uart, UartDmaRead_t read);
HAL_StatusTypeDef UART_BindTx(UART_HandleTypeDef *uart, struct UartTxQueue *queue);
void UART_UnbindTx(UART_HandleTypeDef *uart, struct UartTxQueue *queue);
uint8_t UART_Recover(UART_HandleTypeDef *uart);
uint32_t UART_RxTick(UART_HandleTypeDef *uart);
uint32_t UART_RxBytes(UART_HandleTypeDef *uart);
HAL_StatusTypeDef UART_AbortTx(UART_HandleTypeDef *uart);
HAL_StatusTypeDef UART_Claim(UART_HandleTypeDef *uart, const UartBinding_t *binding,
                              UartBinding_t *previous);
void UART_Restore(const UartBinding_t *previous);
void UART_RxCallback(UART_HandleTypeDef *uart);
void UART_TxCallback(UART_HandleTypeDef *uart);
void UART_ErrorCallback(UART_HandleTypeDef *uart);
#endif
