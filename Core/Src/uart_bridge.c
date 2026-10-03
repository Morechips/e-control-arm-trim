#include "uart_bridge.h"
#include "uart_driver.h"
#include "uart_tx_queue.h"
#include <string.h>
#define BRIDGE_QUEUE_SIZE 1023U
typedef struct {
    UART_HandleTypeDef *uart;
    UartRx_t rx;
    UartTxQueue_t queue;
    uint8_t storage[BRIDGE_QUEUE_SIZE], current;
    UartBinding_t previous;
} BridgePort;
static BridgePort ports[2];
static uint8_t enabled, dma_rx;
volatile uint32_t usart1_to_usart2_bytes, usart2_to_usart1_bytes;
volatile uint32_t usart1_to_uart5_bytes, uart5_to_usart1_bytes;
volatile uint32_t bridge_dropped_bytes, bridge_uart_errors;
static void Error(UART_HandleTypeDef *uart) { (void)uart; ++bridge_uart_errors; }
static void TxEvent(UartTxQueue_t *q, UartQueueEvent_t event, const uint8_t *data,
                    uint16_t length, uint32_t tag, HAL_StatusTypeDef result)
{
    (void)q; (void)data; (void)length; (void)tag; (void)result;
    if (event == UART_QUEUE_START_FAILED) ++bridge_uart_errors;
}
static void Feed(unsigned destination, uint8_t byte)
{
    if (UART_TxQueue_Submit(&ports[destination].queue, &byte, 1U, 0U) != HAL_OK) {
        ++bridge_dropped_bytes; return;
    }
    if (destination == 0U) {
        if (dma_rx) ++usart2_to_usart1_bytes; else ++uart5_to_usart1_bytes;
    } else {
        if (dma_rx) ++usart1_to_usart2_bytes; else ++usart1_to_uart5_bytes;
    }
}
static void Received(UART_HandleTypeDef *uart)
{
    uint8_t byte;
    size_t count;
    unsigned source = uart == ports[0].uart ? 0U : 1U;
    if (!enabled) return;
    while (UART_RECV(&byte, 1U, uart, &count, NULL) == HAL_OK && count != 0U)
        Feed(1U - source, byte);
}
static HAL_StatusTypeDef Init(UART_HandleTypeDef *u1, UART_HandleTypeDef *other, uint8_t use_dma)
{
    unsigned i;
    HAL_StatusTypeDef result;
    const UartTxQueueConfig_t config = {
        .policy = UART_QUEUE_FIFO, .capacity = BRIDGE_QUEUE_SIZE, .frame_size = 1U,
        .retain_failed = 1U, .count_active = 1U, .defer_binding = 1U, .notify = TxEvent
    };
    if (enabled || u1 == NULL || other == NULL || u1 == other) return HAL_ERROR;
    memset(ports, 0, sizeof(ports)); ports[0].uart = u1; ports[1].uart = other;
    dma_rx = use_dma;
    for (i = 0U; i < 2U; ++i)
        if (UART_TxQueue_Init(&ports[i].queue, ports[i].uart, &config, ports[i].storage,
                              NULL, NULL, &ports[i].current) != HAL_OK) return HAL_ERROR;
    enabled = 1U;
    for (i = 0U; i < 2U; ++i) {
        UartBinding_t binding = {
            .uart = ports[i].uart, .rx = use_dma && i == 1U ? NULL : &ports[i].rx,
            .tx = &ports[i].queue, .rx_ready = Received, .error = Error
        };
        result = UART_Claim(ports[i].uart, &binding, &ports[i].previous);
        if (result != HAL_OK) {
            enabled = 0U;
            while (i != 0U) UART_Restore(&ports[--i].previous);
            return result;
        }
    }
    return HAL_OK;
}
HAL_StatusTypeDef UART_Bridge_Init(UART_HandleTypeDef *u1, UART_HandleTypeDef *other)
{ return Init(u1, other, 0U); }
HAL_StatusTypeDef UART_Bridge_InitDMA(UART_HandleTypeDef *u1, UART_HandleTypeDef *other)
{ return Init(u1, other, 1U); }
void UART_Bridge_FeedDMA(const uint8_t *data, uint16_t size)
{
    uint16_t i;
    if (!enabled || !dma_rx || data == NULL) return;
    for (i = 0U; i < size; ++i) Feed(0U, data[i]);
}
void UART_Bridge_Process(void)
{
    unsigned i;
    if (!enabled) return;
    for (i = 0U; i < 2U; ++i) {
        if (!(dma_rx && i == 1U)) (void)UART_Recover(ports[i].uart);
        (void)UART_TxQueue_Process(&ports[i].queue);
    }
}
