#ifndef UART_TX_QUEUE_H
#define UART_TX_QUEUE_H
#include "uart_driver.h"
typedef enum { UART_QUEUE_FIFO, UART_QUEUE_LATEST } UartQueuePolicy_t;
typedef enum { UART_QUEUE_STARTED, UART_QUEUE_COMPLETED, UART_QUEUE_START_FAILED,
               UART_QUEUE_TIMED_OUT } UartQueueEvent_t;
struct UartTxQueue;
typedef void (*UartQueueNotify_t)(struct UartTxQueue *, UartQueueEvent_t,
                                  const uint8_t *, uint16_t, uint32_t, HAL_StatusTypeDef);
typedef struct {
    UartQueuePolicy_t policy;
    uint16_t capacity, frame_size;
    uint32_t timeout_ms, gap_ms;
    uint8_t immediate, continue_in_irq, retain_failed, count_active;
    uint8_t timeout_inclusive, abort_failure_keeps_active, defer_binding;
    UartQueueNotify_t notify;
} UartTxQueueConfig_t;
typedef struct UartTxQueue {
    UART_HandleTypeDef *uart;
    UartTxQueueConfig_t config;
    uint8_t *storage, *current;
    uint16_t *lengths;
    uint32_t *tags;
    volatile uint16_t head, tail, count;
    volatile uint8_t active, reserved, starting, completion_pending;
    uint16_t current_length;
    uint32_t current_tag, started_tick, completed_tick;
} UartTxQueue_t;
/* Static caller-owned storage: capacity*frame_size plus one current frame.
 * lengths may be NULL only for one-byte frames; tags may be NULL for tag zero.
 * Event hooks are bounded, may run in IRQ context, and must not format logs.
 * One producer per instance. Foreground and TC consumer transitions are guarded.
 */
HAL_StatusTypeDef UART_TxQueue_Init(UartTxQueue_t *q, UART_HandleTypeDef *uart,
    const UartTxQueueConfig_t *config, uint8_t *storage, uint16_t *lengths,
    uint32_t *tags, uint8_t *current);
HAL_StatusTypeDef UART_TxQueue_Submit(UartTxQueue_t *q, const uint8_t *data,
                                      size_t length, uint32_t tag);
/* Reserve/Commit are a single producer operation; never expose a half-built frame. */
uint8_t *UART_TxQueue_Reserve(UartTxQueue_t *q);
HAL_StatusTypeDef UART_TxQueue_Commit(UartTxQueue_t *q, size_t length, uint32_t tag);
HAL_StatusTypeDef UART_TxQueue_Process(UartTxQueue_t *q);
void UART_TxQueue_Complete(UartTxQueue_t *q);
void UART_TxQueue_CancelPending(UartTxQueue_t *q, uint8_t bypass_gap);
uint8_t UART_TxQueue_IsIdle(const UartTxQueue_t *q);
uint8_t UART_TxQueue_IsSending(const UartTxQueue_t *q);
uint16_t UART_TxQueue_Pending(const UartTxQueue_t *q);
uint16_t UART_TxQueue_Free(const UartTxQueue_t *q);
const uint8_t *UART_TxQueue_Peek(const UartTxQueue_t *q);
#endif
