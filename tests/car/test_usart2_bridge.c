#include "uart_bridge.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static UART_HandleTypeDef u1, u2, other;
static uint8_t output[2][4096], fail_tx;
static unsigned lengths[2], callbacks;
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u, uint8_t *p, uint16_t n) {
    assert(u == &u1 && n == 1U); u->rx = p; u->RxState = 1U; return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *u) {
    u->RxState = HAL_UART_STATE_READY; return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u, const uint8_t *p, uint16_t n) {
    unsigned port = u == &u1 ? 0U : 1U;
    assert((u == &u1 || u == &u2) && n == 1U && !u->pending);
    if (fail_tx) { fail_tx = 0U; return HAL_BUSY; }
    output[port][lengths[port]++] = *p; u->pending = 1U; return HAL_OK;
}
void Bluetooth_RxCallback(UART_HandleTypeDef *u) { assert(u == &other); callbacks++; }
void Motor_RxCallback(UART_HandleTypeDef *u) { assert(u == &other); callbacks++; }
void Motor_TxCallback(UART_HandleTypeDef *u) { assert(u == &other); callbacks++; }
void Debug_TxCallback(UART_HandleTypeDef *u) { assert(u == &other); callbacks++; }
void PID_Tuner_TxCallback(UART_HandleTypeDef *u) { assert(u == &other); callbacks++; }
void Bluetooth_ErrorCallback(UART_HandleTypeDef *u) { assert(u == &other); callbacks++; }
void Motor_ErrorCallback(UART_HandleTypeDef *u) { assert(u == &other); callbacks++; }
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
    assert(callbacks == 7U);
    u1.RxState = HAL_UART_STATE_READY; HAL_UART_ErrorCallback(&u1); Drain(1U);
    assert(u1.RxState != HAL_UART_STATE_READY && u2.rx == NULL);
    puts("PASS USART2 bridge: binary duplex / DMA ownership / TX retry / ring overflow / callback routing / RX recovery");
    return 0;
}
