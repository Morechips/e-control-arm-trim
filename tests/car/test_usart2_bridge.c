#include "uart_driver.h"
#include "uart_tx_queue.h"
#include "servo.h"
#include "uart_bridge.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static UART_HandleTypeDef u1, u2, other;
static uint8_t output[2][4096], fail_tx, fail_receive;
static unsigned lengths[2], callbacks;
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u, uint8_t *p, uint16_t n) {
    assert((u == &u1 || u == &u2 || u == &other) && n == 1U);
    if (u == &u2 && fail_receive) return HAL_ERROR;
    u->rx = p; u->RxState = 1U; return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *u) {
    u->RxState = HAL_UART_STATE_READY; return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u, const uint8_t *p, uint16_t n) {
    unsigned port = u == &u1 ? 0U : 1U;
    if (u == &other) { assert(n == 1U && !u->pending); u->pending = 1U; return HAL_OK; }
    assert((u == &u1 || u == &u2) && n == 1U && !u->pending);
    if (fail_tx) { fail_tx = 0U; return HAL_BUSY; }
    output[port][lengths[port]++] = *p; u->pending = 1U; return HAL_OK;
}
static void Received(UART_HandleTypeDef *u)
{
    uint8_t byte; size_t count;
    assert(u == &other);
    assert(UART_RECV(&byte, 1U, u, &count, NULL) == HAL_OK && count == 1U && byte == 77U);
    ++callbacks;
}
static void Error(UART_HandleTypeDef *u) { assert(u == &other); ++callbacks; }
static void TxEvent(UartTxQueue_t *q, UartQueueEvent_t event, const uint8_t *data,
                    uint16_t length, uint32_t tag, HAL_StatusTypeDef result)
{
    (void)q; (void)data; (void)length; (void)tag; (void)result;
    if (event == UART_QUEUE_COMPLETED) ++callbacks;
}
static void Drain(unsigned steps) {
    while (steps--) {
        UART_Bridge_Process();
        if (u1.pending) { u1.pending = 0U; HAL_UART_TxCpltCallback(&u1); }
        if (u2.pending) { u2.pending = 0U; HAL_UART_TxCpltCallback(&u2); }
    }
}
int main(void) {
    uint8_t data[512]; unsigned i;
    u1.RxState = u2.RxState = HAL_UART_STATE_READY;
    assert(UART_Bridge_InitDMA(NULL, &u2) == HAL_ERROR);
    fail_receive = 1U;
    assert(UART_Bridge_Init(&u1, &u2) == HAL_ERROR);
    assert(u1.RxState == HAL_UART_STATE_READY);
    fail_receive = 0U;
    assert(UART_Bridge_InitDMA(&u1, &u2) == HAL_OK);
    for (i = 0U; i < sizeof(data); i++) data[i] = (uint8_t)i;
    for (i = 0U; i < sizeof(data); i++) {
        *u1.rx = data[i]; u1.RxState = HAL_UART_STATE_READY; HAL_UART_RxCpltCallback(&u1);
    }
    UART_Bridge_FeedDMA(data, sizeof(data)); fail_tx = 1U; Drain(514U);
    assert(lengths[0] == 512U && lengths[1] == 512U);
    assert(memcmp(output[0], data, 512U) == 0 && memcmp(output[1], data, 512U) == 0);
    assert(usart1_to_usart2_bytes == 512U && usart2_to_usart1_bytes == 512U);
    assert(bridge_uart_errors == 1U && bridge_dropped_bytes == 0U);
    UART_Bridge_FeedDMA(data, 512U); UART_Bridge_FeedDMA(data, 512U);
    assert(bridge_dropped_bytes == 1U); Drain(1024U);
    assert(lengths[0] == 1535U);
    assert(memcmp(output[0] + 512U, data, 512U) == 0);
    assert(memcmp(output[0] + 1024U, data, 511U) == 0);
    HAL_UART_RxCpltCallback(&other); HAL_UART_TxCpltCallback(&other); HAL_UART_ErrorCallback(&other);
    assert(callbacks == 0U); /* Unknown UART events are ignored, without fan-out. */
    {
        UartRx_t rx;
        UartTxQueue_t q;
        uint8_t storage[1], current, byte = 4U;
        const UartTxQueueConfig_t config = {
            .policy = UART_QUEUE_FIFO, .capacity = 1U, .frame_size = 1U, .notify = TxEvent
        };
        assert(UART_BindRx(&rx, &other, Received, Error) == HAL_OK);
        assert(UART_TxQueue_Init(&q, &other, &config, storage, NULL, NULL, &current) == HAL_OK);
        *other.rx = 77U; HAL_UART_RxCpltCallback(&other);
        assert(UART_TxQueue_Submit(&q, &byte, 1U, 0U) == HAL_OK);
        assert(UART_TxQueue_Process(&q) == HAL_OK);
        other.pending = 0U; HAL_UART_TxCpltCallback(&other); HAL_UART_ErrorCallback(&other);
        assert(callbacks == 3U);
        UART_UnbindTx(&other, &q);
        UartBinding_t replacement = {.uart = &other}, previous;
        assert(UART_Claim(&other, &replacement, &previous) == HAL_OK);
    }
    u1.RxState = HAL_UART_STATE_READY; HAL_UART_ErrorCallback(&u1); Drain(1U);
    assert(u1.RxState != HAL_UART_STATE_READY && u2.rx == NULL);
    puts("PASS USART2 bridge: binary duplex / DMA ownership / TX retry / ring overflow / callback routing / RX recovery");
    return 0;
}
