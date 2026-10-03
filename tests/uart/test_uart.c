#include "uart_driver.h"
#include "uart_tx_queue.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static UART_HandleTypeDef a, b, dma, unknown;
static UartRx_t rx_a, rx_b;
static uint32_t tick;
static HAL_StatusTypeDef next_tx, next_rx, abort_result = HAL_OK;
static unsigned calls, aborts, receive_calls, errors, event_count;
static uint8_t immediate;
static const uint8_t *inflight;
static uint16_t inflight_length;
static uint8_t sent[32];
static struct { UartQueueEvent_t event; uint32_t tag; uint8_t byte; } events[64];
uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u, uint8_t *data, uint16_t n)
{
    HAL_StatusTypeDef result = next_rx;
    assert(n == 1U); ++receive_calls; next_rx = HAL_OK;
    if (result == HAL_OK) { u->rx = data; u->RxState = 1U; }
    return result;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *u)
{ u->RxState = HAL_UART_STATE_READY; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *u)
{
    ++aborts;
    if (abort_result == HAL_OK) { u->pending = 0U; u->gState = HAL_UART_STATE_READY; }
    return abort_result;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u, const uint8_t *data, uint16_t n)
{
    HAL_StatusTypeDef result = next_tx;
    ++calls; next_tx = HAL_OK;
    assert(n <= sizeof(sent)); memcpy(sent, data, n);
    if (result == HAL_OK) {
        assert(!u->pending); u->pending = 1U; inflight = data; inflight_length = n;
        if (immediate) { u->pending = 0U; HAL_UART_TxCpltCallback(u); }
    }
    return result;
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u, const uint8_t *data, uint16_t n, uint32_t timeout)
{
    HAL_StatusTypeDef result = next_tx;
    (void)u; assert(timeout == 7U); ++calls; next_tx = HAL_OK;
    memcpy(sent, data, n); return result;
}
static void Error(UART_HandleTypeDef *u) { assert(u == &a); ++errors; }
static void Event(UartTxQueue_t *q, UartQueueEvent_t event, const uint8_t *data,
                   uint16_t length, uint32_t tag, HAL_StatusTypeDef result)
{
    (void)q; (void)result;
    assert(length != 0U && event_count < 64U);
    events[event_count].event = event; events[event_count].tag = tag;
    events[event_count++].byte = data[0];
}
static void Complete(UART_HandleTypeDef *u)
{
    assert(u->pending); u->pending = 0U; HAL_UART_TxCpltCallback(u);
}
static void Inject(UART_HandleTypeDef *u, uint8_t byte, uint32_t t)
{
    tick = t; *u->rx = byte; u->RxState = HAL_UART_STATE_READY; HAL_UART_RxCpltCallback(u);
}
static uint16_t DmaRead(uint8_t *data, uint16_t capacity, uint32_t *time)
{
    uint16_t n = capacity < 3U ? capacity : 3U;
    for (uint16_t i = 0U; i < n; ++i) data[i] = (uint8_t)(20U + i);
    *time = 1234U; return n;
}
static void TestRx(void)
{
    uint8_t data[8]; uint32_t times[8]; size_t n;
    assert(UART_BindRx(&rx_a, &a, NULL, Error) == HAL_OK);
    assert(UART_BindRx(&rx_b, &b, NULL, NULL) == HAL_OK);
    assert(UART_RECV(data, sizeof(data), &a, &n, times) == HAL_OK && n == 0U);
    Inject(&a, 0U, 10U); Inject(&b, 99U, 11U); Inject(&a, 7U, 12U);
    assert(UART_RECV(data, 1U, &a, &n, times) == HAL_OK && n == 1U && data[0] == 0U && times[0] == 10U);
    assert(UART_RECV(data, 8U, &a, &n, times) == HAL_OK && n == 1U && data[0] == 7U && times[0] == 12U);
    assert(UART_RxTick(&a) == 12U && UART_RxBytes(&a) == 2U);
    assert(UART_RECV(data, 8U, &b, &n, NULL) == HAL_OK && n == 1U && data[0] == 99U);
    Inject(&a, 1U, 13U); HAL_UART_ErrorCallback(&a);
    assert(errors == 1U && UART_RECV(data, 8U, &a, &n, NULL) == HAL_ERROR && n == 0U);
    a.RxState = HAL_UART_STATE_READY;
    assert(UART_Recover(&a) && !UART_Recover(&a));
    assert(UART_RECV(data, 8U, &a, &n, NULL) == HAL_OK && n == 0U);
    for (unsigned i = 0U; i < UART_RX_CAPACITY; ++i) Inject(&a, (uint8_t)i, i);
    assert(UART_Recover(&a));
    Inject(&a, 42U, 1000U);
    assert(UART_RECV(data, 8U, &a, &n, times) == HAL_OK && n == 1U && data[0] == 42U);
    assert(UART_BindDma(&dma, DmaRead) == HAL_OK);
    assert(UART_RECV(data, 8U, &dma, &n, times) == HAL_OK && n == 3U && data[0] == 20U);
    assert(times[0] == 1234U && times[2] == 1234U && UART_RxTick(&dma) == 1234U);
    assert(UART_RECV(NULL, 8U, &a, &n, NULL) == HAL_ERROR && n == 0U);
    assert(UART_RECV(data, 0U, &a, &n, NULL) == HAL_ERROR);
    assert(UART_RECV(data, 8U, &unknown, &n, NULL) == HAL_ERROR);
    assert(UART_RECV(data, 8U, &a, NULL, NULL) == HAL_ERROR);
    unsigned before = receive_calls;
    HAL_UART_RxCpltCallback(&unknown); HAL_UART_TxCpltCallback(&unknown); HAL_UART_ErrorCallback(&unknown);
    assert(receive_calls == before && errors == 1U);
}
static void TestSend(void)
{
    const uint8_t bytes[] = {0U, 2U, 0U, 3U}; unsigned before = calls;
    assert(UART_SEND(NULL, 4U, &a, UART_TX_BLOCKING, 7U) == HAL_ERROR);
    assert(UART_SEND(bytes, 0U, &a, UART_TX_BLOCKING, 7U) == HAL_ERROR);
    assert(UART_SEND(bytes, (size_t)UINT16_MAX + 1U, &a, UART_TX_BLOCKING, 7U) == HAL_ERROR);
    assert(UART_SEND(bytes, 4U, NULL, UART_TX_BLOCKING, 7U) == HAL_ERROR);
    assert(UART_SEND(bytes, 4U, &a, (UartTxMode_t)99, 7U) == HAL_ERROR && calls == before);
    for (unsigned i = HAL_OK; i <= HAL_TIMEOUT; ++i) {
        next_tx = (HAL_StatusTypeDef)i;
        assert(UART_SEND(bytes, 4U, &a, UART_TX_BLOCKING, 7U) == (HAL_StatusTypeDef)i);
        assert(memcmp(sent, bytes, 4U) == 0);
    }
}
static void TestQueues(void)
{
    UartTxQueue_t q;
    uint8_t storage[2][4], current[4], first[] = {1U, 0U}, second[] = {2U, 0U};
    uint16_t lengths[2]; uint32_t tags[2];
    UartTxQueueConfig_t config = {
        .policy = UART_QUEUE_FIFO, .capacity = 2U, .frame_size = 4U,
        .retain_failed = 1U, .gap_ms = 2U, .timeout_ms = 20U, .notify = Event
    };
    tick = UINT32_MAX - 10U; event_count = 0U;
    assert(UART_TxQueue_Init(&q, &a, &config, &storage[0][0], lengths, tags, current) == HAL_OK);
    assert(UART_TxQueue_Submit(&q, first, 2U, 11U) == HAL_OK);
    assert(UART_TxQueue_Submit(&q, second, 2U, 22U) == HAL_OK);
    assert(UART_TxQueue_Submit(&q, second, 2U, 33U) == HAL_BUSY);
    first[0] = 99U;
    next_tx = HAL_BUSY;
    assert(UART_TxQueue_Process(&q) == HAL_BUSY && UART_TxQueue_Pending(&q) == 2U);
    assert(events[0].event == UART_QUEUE_START_FAILED && events[0].tag == 11U);
    assert(UART_TxQueue_Process(&q) == HAL_OK && sent[0] == 1U && inflight_length == 2U);
    assert(UART_TxQueue_IsSending(&q) && UART_TxQueue_Pending(&q) == 1U);
    UART_TxQueue_CancelPending(&q, 1U);
    assert(inflight[0] == 1U && !UART_TxQueue_IsIdle(&q));
    Complete(&a);
    assert(events[2].event == UART_QUEUE_COMPLETED && events[2].tag == 11U);
    assert(UART_TxQueue_Submit(&q, second, 2U, 22U) == HAL_OK);
    unsigned before = calls;
    assert(UART_TxQueue_Process(&q) == HAL_OK && calls == before);
    tick += 2U; assert(UART_TxQueue_Process(&q) == HAL_OK && sent[0] == 2U);
    tick += 20U; assert(UART_TxQueue_Process(&q) == HAL_OK && UART_TxQueue_IsSending(&q));
    ++tick; assert(UART_TxQueue_Process(&q) == HAL_OK && UART_TxQueue_IsIdle(&q) && aborts == 1U);
    assert(events[event_count-1U].event == UART_QUEUE_TIMED_OUT);
    /* RX remains alive after a TX timeout. */
    Inject(&a, 9U, tick); uint8_t byte; size_t n;
    assert(UART_RECV(&byte, 1U, &a, &n, NULL) == HAL_OK && n == 1U && byte == 9U);
    UART_UnbindTx(&a, &q);
    config.policy = UART_QUEUE_LATEST; config.capacity = 1U; config.gap_ms = 0U;
    config.timeout_ms = 0U; config.retain_failed = 0U; config.immediate = config.continue_in_irq = 1U;
    event_count = 0U; first[0] = 1U;
    assert(UART_TxQueue_Init(&q, &a, &config, &storage[0][0], lengths, tags, current) == HAL_OK);
    assert(UART_TxQueue_Submit(&q, first, 2U, 11U) == HAL_OK);
    HAL_UART_ErrorCallback(&a);
    assert(UART_TxQueue_IsSending(&q) && a.pending && inflight[0] == 1U);
    assert(UART_Recover(&a));
    assert(UART_TxQueue_Submit(&q, second, 2U, 22U) == HAL_OK);
    second[0] = 3U;
    assert(UART_TxQueue_Submit(&q, second, 2U, 33U) == HAL_OK && inflight[0] == 1U);
    Complete(&a); assert(a.pending && sent[0] == 3U);
    Complete(&a); assert(UART_TxQueue_IsIdle(&q));
    next_tx = HAL_BUSY;
    assert(UART_TxQueue_Submit(&q, first, 2U, 11U) == HAL_BUSY && UART_TxQueue_IsIdle(&q));
    immediate = 1U;
    assert(UART_TxQueue_Submit(&q, first, 2U, 11U) == HAL_OK && UART_TxQueue_IsIdle(&q));
    assert(events[event_count-2U].event == UART_QUEUE_STARTED && events[event_count-2U].tag == 11U);
    assert(events[event_count-1U].event == UART_QUEUE_COMPLETED && events[event_count-1U].tag == 11U);
    immediate = 0U;
    /* TC while a replacement is being built must not expose that half-frame. */
    assert(UART_TxQueue_Submit(&q, first, 2U, 11U) == HAL_OK);
    uint8_t *reserved = UART_TxQueue_Reserve(&q); assert(reserved != NULL);
    reserved[0] = 8U; reserved[1] = 0U;
    before = calls; Complete(&a); assert(calls == before);
    assert(UART_TxQueue_Commit(&q, 2U, 88U) == HAL_OK && sent[0] == 8U);
    Complete(&a); UART_UnbindTx(&a, &q);
    /* Bridge budget includes the in-flight frame. */
    config.policy = UART_QUEUE_FIFO; config.capacity = 2U; config.count_active = 1U;
    config.immediate = config.continue_in_irq = 0U;
    assert(UART_TxQueue_Init(&q, &a, &config, &storage[0][0], lengths, tags, current) == HAL_OK);
    assert(UART_TxQueue_Submit(&q, first, 2U, 0U) == HAL_OK); UART_TxQueue_Process(&q);
    assert(UART_TxQueue_Submit(&q, second, 2U, 0U) == HAL_OK);
    assert(UART_TxQueue_Submit(&q, second, 2U, 0U) == HAL_BUSY);
    Complete(&a); UART_TxQueue_CancelPending(&q, 0U);
    config.timeout_ms = 5U; config.timeout_inclusive = config.abort_failure_keeps_active = 1U;
    UART_UnbindTx(&a, &q);
    assert(UART_TxQueue_Init(&q, &a, &config, &storage[0][0], lengths, tags, current) == HAL_OK);
    UART_TxQueue_Submit(&q, first, 2U, 0U); UART_TxQueue_Process(&q); tick += 5U;
    abort_result = HAL_ERROR;
    assert(UART_TxQueue_Process(&q) == HAL_ERROR && UART_TxQueue_IsSending(&q));
    abort_result = HAL_OK;
    assert(UART_TxQueue_Process(&q) == HAL_OK && UART_TxQueue_IsIdle(&q));
    HAL_UART_TxCpltCallback(&a); assert(UART_TxQueue_IsIdle(&q));
    UART_UnbindTx(&a, &q);
}
static void TestClaimRollback(void)
{
    UartRx_t replacement_rx; UartBinding_t previous;
    UartBinding_t binding = {.uart = &a, .rx = &replacement_rx};
    next_rx = HAL_ERROR;
    assert(UART_Claim(&a, &binding, &previous) == HAL_ERROR);
    assert(UART_Recover(&a) == 1U);
    Inject(&a, 6U, tick);
    uint8_t byte; size_t n;
    assert(UART_RECV(&byte, 1U, &a, &n, NULL) == HAL_OK && n == 1U && byte == 6U);
    assert(UART_Claim(&a, &binding, &previous) == HAL_OK);
    UART_Restore(&previous);
    assert(UART_Recover(&a) == 1U);
    Inject(&a, 5U, tick);
    assert(UART_RECV(&byte, 1U, &a, &n, NULL) == HAL_OK && n == 1U && byte == 5U);
}
int main(void)
{
    TestRx(); TestSend(); TestQueues(); TestClaimRollback();
    puts("PASS shared UART/queue: binary IO, IT/DMA timestamps, isolation, recovery, FIFO/latest, tags, TC lifetime, overflow, timeout/wrap and ownership rollback");
    return 0;
}
