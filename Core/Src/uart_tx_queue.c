#include "uart_tx_queue.h"
#include <string.h>

static void Notify(UartTxQueue_t *q, UartQueueEvent_t event, HAL_StatusTypeDef status)
{
    if (q->config.notify != NULL)
        q->config.notify(q, event, q->current, q->current_length, q->current_tag, status);
}

HAL_StatusTypeDef UART_TxQueue_Init(UartTxQueue_t *q, UART_HandleTypeDef *uart,
    const UartTxQueueConfig_t *config, uint8_t *storage, uint16_t *lengths,
    uint32_t *tags, uint8_t *current)
{
    if (q == NULL || uart == NULL || config == NULL || storage == NULL || current == NULL ||
        config->capacity == 0U || config->frame_size == 0U ||
        (config->policy != UART_QUEUE_FIFO && config->policy != UART_QUEUE_LATEST) ||
        (lengths == NULL && config->frame_size != 1U) ||
        (config->policy == UART_QUEUE_LATEST && config->capacity != 1U)) return HAL_ERROR;
    memset(q, 0, sizeof(*q));
    q->uart = uart; q->config = *config;
    q->storage = storage; q->lengths = lengths; q->tags = tags; q->current = current;
    q->completed_tick = HAL_GetTick() - config->gap_ms;
    return config->defer_binding ? HAL_OK : UART_BindTx(uart, q);
}

uint8_t UART_TxQueue_IsSending(const UartTxQueue_t *q) { return q->active; }
uint16_t UART_TxQueue_Pending(const UartTxQueue_t *q) { return q->count; }
uint16_t UART_TxQueue_Free(const UartTxQueue_t *q)
{
    uint16_t occupied = (uint16_t)(q->count + (q->config.count_active ? q->active : 0U));
    return occupied >= q->config.capacity ? 0U : (uint16_t)(q->config.capacity - occupied);
}
uint8_t UART_TxQueue_IsIdle(const UartTxQueue_t *q)
{
    return (uint8_t)(q != NULL && !q->active && q->count == 0U && !q->reserved);
}
const uint8_t *UART_TxQueue_Peek(const UartTxQueue_t *q)
{
    return q->count == 0U ? NULL : q->storage + (size_t)q->tail * q->config.frame_size;
}

uint8_t *UART_TxQueue_Reserve(UartTxQueue_t *q)
{
    uint8_t *data = NULL;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (q != NULL && q->uart != NULL && !q->reserved &&
        (q->config.policy == UART_QUEUE_LATEST || UART_TxQueue_Free(q) != 0U)) {
        if (q->config.policy == UART_QUEUE_LATEST) {
            q->head = q->tail = q->count = 0U;
        }
        q->reserved = 1U;
        data = q->storage + (size_t)q->head * q->config.frame_size;
    }
    __set_PRIMASK(mask);
    return data;
}

HAL_StatusTypeDef UART_TxQueue_Commit(UartTxQueue_t *q, size_t length, uint32_t tag)
{
    uint8_t start;
    uint32_t mask = __get_PRIMASK();
    if (q == NULL || !q->reserved) return HAL_ERROR;
    if (length == 0U || length > q->config.frame_size || (tag != 0U && q->tags == NULL)) { q->reserved = 0U; return HAL_ERROR; }
    __disable_irq();
    if (q->lengths != NULL) q->lengths[q->head] = (uint16_t)length;
    if (q->tags != NULL) q->tags[q->head] = tag;
    __DMB();
    q->head = (uint16_t)((q->head + 1U) % q->config.capacity);
    ++q->count; q->reserved = 0U;
    start = (uint8_t)(q->config.immediate && !q->active);
    __set_PRIMASK(mask);
    return start ? UART_TxQueue_Process(q) : HAL_OK;
}
HAL_StatusTypeDef UART_TxQueue_Submit(UartTxQueue_t *q, const uint8_t *data,
                                      size_t length, uint32_t tag)
{
    uint8_t *destination;
    if (q == NULL || data == NULL || length == 0U || length > q->config.frame_size || (tag != 0U && q->tags == NULL)) return HAL_ERROR;
    destination = UART_TxQueue_Reserve(q);
    if (destination == NULL) return HAL_BUSY;
    memcpy(destination, data, length);
    return UART_TxQueue_Commit(q, length, tag);
}

void UART_TxQueue_CancelPending(UartTxQueue_t *q, uint8_t bypass_gap)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    q->head = q->tail = q->count = 0U; q->reserved = 0U;
    if (bypass_gap) q->completed_tick = HAL_GetTick() - q->config.gap_ms;
    __set_PRIMASK(mask);
}

HAL_StatusTypeDef UART_TxQueue_Process(UartTxQueue_t *q)
{
    HAL_StatusTypeDef status;
    uint32_t elapsed, mask;
    if (q == NULL || q->uart == NULL) return HAL_ERROR;
    mask = __get_PRIMASK();
    __disable_irq();
    elapsed = HAL_GetTick() - q->started_tick;
    if (q->active && q->config.timeout_ms != 0U &&
        (elapsed > q->config.timeout_ms ||
         (q->config.timeout_inclusive && elapsed == q->config.timeout_ms))) {
        status = UART_AbortTx(q->uart);
        if (status != HAL_OK && q->config.abort_failure_keeps_active) {
            __set_PRIMASK(mask); return status;
        }
        q->active = 0U;
        Notify(q, UART_QUEUE_TIMED_OUT, HAL_TIMEOUT);
    }
    if (q->active || q->count == 0U ||
        (uint32_t)(HAL_GetTick() - q->completed_tick) < q->config.gap_ms) {
        __set_PRIMASK(mask); return HAL_OK;
    }
    q->current_length = q->lengths == NULL ? 1U : q->lengths[q->tail];
    q->current_tag = q->tags == NULL ? 0U : q->tags[q->tail];
    memcpy(q->current, q->storage + (size_t)q->tail * q->config.frame_size, q->current_length);
    q->tail = (uint16_t)((q->tail + 1U) % q->config.capacity); --q->count;
    q->active = 1U; q->started_tick = HAL_GetTick();
    /* Publish before HAL: tests and real interrupts may complete immediately. */
    q->starting = 1U; q->completion_pending = 0U;
    status = UART_SEND(q->current, q->current_length, q->uart, UART_TX_INTERRUPT, 0U);
    q->starting = 0U;
    if (status != HAL_OK) {
        q->completion_pending = 0U;
        q->active = 0U;
        if (q->config.retain_failed) {
            q->tail = (uint16_t)((q->tail + q->config.capacity - 1U) % q->config.capacity);
            ++q->count;
        }
        Notify(q, UART_QUEUE_START_FAILED, status);
    } else {
        Notify(q, UART_QUEUE_STARTED, HAL_OK);
        if (q->completion_pending) UART_TxQueue_Complete(q);
    }
    __set_PRIMASK(mask);
    return status;
}

void UART_TxQueue_Complete(UartTxQueue_t *q)
{
    if (q == NULL || !q->active) return;
    if (q->starting) { q->completion_pending = 1U; return; }
    q->completion_pending = 0U;
    q->completed_tick = HAL_GetTick();
    q->active = 0U;
    Notify(q, UART_QUEUE_COMPLETED, HAL_OK);
    if (q->config.continue_in_irq) (void)UART_TxQueue_Process(q);
}
