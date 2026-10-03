#include "uart_driver.h"
#include "uart_tx_queue.h"
#include <string.h>
static UartBinding_t bindings[UART_BINDING_CAPACITY];

static UartBinding_t *Find(UART_HandleTypeDef *uart, uint8_t allocate)
{
    unsigned i;
    if (uart == NULL) return NULL;
    for (i = 0U; i < UART_BINDING_CAPACITY; ++i)
        if (bindings[i].uart == uart) return &bindings[i];
    if (allocate) for (i = 0U; i < UART_BINDING_CAPACITY; ++i) {
        if (bindings[i].uart == NULL) { bindings[i].uart = uart; return &bindings[i]; }
    }
    return NULL;
}
HAL_StatusTypeDef UART_SEND(const uint8_t *data, size_t length, UART_HandleTypeDef *uart,
                             UartTxMode_t mode, uint32_t timeout_ms)
{
    if (data == NULL || length == 0U || length > UINT16_MAX || uart == NULL) return HAL_ERROR;
    if (mode == UART_TX_INTERRUPT) return HAL_UART_Transmit_IT(uart, data, (uint16_t)length);
    if (mode == UART_TX_BLOCKING) return HAL_UART_Transmit(uart, data, (uint16_t)length, timeout_ms);
    return HAL_ERROR;
}
HAL_StatusTypeDef UART_AbortTx(UART_HandleTypeDef *uart) { return HAL_UART_AbortTransmit(uart); }
HAL_StatusTypeDef UART_BindRx(UartRx_t *rx, UART_HandleTypeDef *uart,
                               UartHook_t ready, UartHook_t error)
{
    UartBinding_t *binding = Find(uart, 1U);
    HAL_StatusTypeDef status;
    uint32_t mask;
    if (rx == NULL || binding == NULL) return HAL_ERROR;
    if (binding->rx != NULL && HAL_UART_AbortReceive(uart) != HAL_OK) return HAL_ERROR;
    memset(rx, 0, sizeof(*rx)); rx->uart = uart;
    mask = __get_PRIMASK(); __disable_irq();
    binding->rx = rx; binding->dma_read = NULL; binding->rx_ready = ready; binding->error = error;
    binding->last_read_tick = 0U;
    status = HAL_UART_Receive_IT(uart, &rx->byte, 1U);
    if (status != HAL_OK) rx->broken = 1U;
    __set_PRIMASK(mask);
    return status;
}
HAL_StatusTypeDef UART_BindDma(UART_HandleTypeDef *uart, UartDmaRead_t read)
{
    UartBinding_t *binding = Find(uart, 1U);
    if (binding == NULL || read == NULL) return HAL_ERROR;
    binding->rx = NULL; binding->dma_read = read;
    return HAL_OK;
}
HAL_StatusTypeDef UART_BindTx(UART_HandleTypeDef *uart, struct UartTxQueue *queue)
{
    UartBinding_t *binding = Find(uart, 1U);
    if (binding == NULL || queue == NULL) return HAL_ERROR;
    if (binding->tx != NULL && binding->tx != queue) return HAL_BUSY;
    binding->tx = queue; return HAL_OK;
}
void UART_UnbindTx(UART_HandleTypeDef *uart, struct UartTxQueue *queue)
{
    UartBinding_t *binding = Find(uart, 0U);
    if (binding != NULL && binding->tx == queue) binding->tx = NULL;
}
uint8_t UART_Recover(UART_HandleTypeDef *uart)
{
    UartBinding_t *binding = Find(uart, 0U);
    UartRx_t *rx;
    uint8_t broken;
    uint32_t mask;
    if (binding == NULL || binding->rx == NULL) return 0U;
    rx = binding->rx; mask = __get_PRIMASK(); __disable_irq();
    broken = rx->broken;
    if (broken) { rx->tail = rx->head; rx->broken = 0U; }
    if (uart->RxState == HAL_UART_STATE_READY &&
        HAL_UART_Receive_IT(uart, &rx->byte, 1U) != HAL_OK) rx->broken = 1U;
    __set_PRIMASK(mask); return broken;
}
uint32_t UART_RxTick(UART_HandleTypeDef *uart)
{
    UartBinding_t *binding = Find(uart, 0U);
    return binding == NULL ? 0U : binding->last_read_tick;
}
uint32_t UART_RxBytes(UART_HandleTypeDef *uart)
{
    UartBinding_t *binding = Find(uart, 0U);
    return binding != NULL && binding->rx != NULL ? binding->rx->bytes : 0U;
}
HAL_StatusTypeDef UART_RECV(uint8_t *data, size_t capacity, UART_HandleTypeDef *uart,
                             size_t *received, uint32_t *ticks)
{
    UartBinding_t *binding;
    UartRx_t *rx;
    if (received == NULL) return HAL_ERROR;
    *received = 0U;
    if (data == NULL || capacity == 0U || capacity > UINT16_MAX) return HAL_ERROR;
    binding = Find(uart, 0U);
    if (binding == NULL) return HAL_ERROR;
    if (binding->dma_read != NULL) {
        uint32_t tick = 0U;
        size_t i;
        *received = binding->dma_read(data, (uint16_t)capacity, &tick);
        if (*received > capacity) { *received = 0U; return HAL_ERROR; }
        if (*received != 0U) binding->last_read_tick = tick;
        if (ticks != NULL) for (i = 0U; i < *received; ++i) ticks[i] = tick;
        return HAL_OK;
    }
    rx = binding->rx;
    if (rx == NULL || rx->broken) return HAL_ERROR;
    while (*received < capacity && rx->tail != rx->head && !rx->broken) {
        __DMB(); data[*received] = rx->data[rx->tail];
        binding->last_read_tick = rx->tick[rx->tail];
        if (ticks != NULL) ticks[*received] = binding->last_read_tick;
        __DMB(); rx->tail = (uint16_t)((rx->tail + 1U) % UART_RX_CAPACITY); ++*received;
    }
    return HAL_OK;
}
void UART_RxCallback(UART_HandleTypeDef *uart)
{
    UartBinding_t *binding = Find(uart, 0U);
    UartRx_t *rx;
    uint16_t next;
    if (binding == NULL || binding->rx == NULL) return;
    rx = binding->rx; ++rx->bytes;
    next = (uint16_t)((rx->head + 1U) % UART_RX_CAPACITY);
    if (next == rx->tail) rx->broken = 1U;
    if (!rx->broken) {
        rx->data[rx->head] = rx->byte; rx->tick[rx->head] = HAL_GetTick();
        __DMB(); rx->head = next;
    }
    if (HAL_UART_Receive_IT(uart, &rx->byte, 1U) != HAL_OK) rx->broken = 1U;
    if (binding->rx_ready != NULL) binding->rx_ready(uart);
}
void UART_TxCallback(UART_HandleTypeDef *uart)
{
    UartBinding_t *binding = Find(uart, 0U);
    if (binding != NULL && binding->tx != NULL && binding->tx->uart == uart)
        UART_TxQueue_Complete(binding->tx);
}
void UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    UartBinding_t *binding = Find(uart, 0U);
    if (binding == NULL) return;
    if (binding->rx != NULL) { __HAL_UART_CLEAR_OREFLAG(uart); binding->rx->broken = 1U; }
    if (binding->error != NULL) binding->error(uart);
}
HAL_StatusTypeDef UART_Claim(UART_HandleTypeDef *uart, const UartBinding_t *replacement,
                              UartBinding_t *previous)
{
    UartBinding_t *binding = Find(uart, 1U);
    HAL_StatusTypeDef status = HAL_OK;
    uint32_t mask;
    if (binding == NULL || replacement == NULL || previous == NULL) return HAL_ERROR;
    mask = __get_PRIMASK(); __disable_irq();
    if (binding->tx != NULL && !UART_TxQueue_IsIdle(binding->tx)) {
        __set_PRIMASK(mask); return HAL_BUSY;
    }
    *previous = *binding;
    if (binding->rx != NULL && HAL_UART_AbortReceive(uart) != HAL_OK) {
        __set_PRIMASK(mask); return HAL_ERROR;
    }
    *binding = *replacement; binding->uart = uart;
    if (binding->rx == NULL && binding->dma_read == NULL) binding->dma_read = previous->dma_read;
    if (binding->rx != NULL) {
        memset(binding->rx, 0, sizeof(*binding->rx)); binding->rx->uart = uart;
        status = HAL_UART_Receive_IT(uart, &binding->rx->byte, 1U);
    }
    if (status != HAL_OK) UART_Restore(previous);
    __set_PRIMASK(mask);
    return status;
}
void UART_Restore(const UartBinding_t *previous)
{
    UartBinding_t *binding;
    uint32_t mask;
    if (previous == NULL || previous->uart == NULL) return;
    binding = Find(previous->uart, 0U);
    if (binding == NULL) return;
    mask = __get_PRIMASK(); __disable_irq();
    if (binding->rx != NULL) (void)HAL_UART_AbortReceive(binding->uart);
    *binding = *previous;
    if (binding->rx != NULL) {
        /* The aborted stream is discontinuous. Rearm now, but preserve the
         * recovery notice until the protocol owner discards its partial frame. */
        binding->rx->broken = 1U;
        if (binding->uart->RxState == HAL_UART_STATE_READY)
            (void)HAL_UART_Receive_IT(binding->uart, &binding->rx->byte, 1U);
    }
    __set_PRIMASK(mask);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart) { UART_RxCallback(uart); }
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart) { UART_TxCallback(uart); }
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart) { UART_ErrorCallback(uart); }
