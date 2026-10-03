#include "serial_io.h"
#include "uart_tx_queue.h"
#include <string.h>
static uint8_t logs[15][80], current[80];
static uint16_t lengths[15];
static UartTxQueue_t queue;
static uint8_t initialized;
static void Init(void)
{
    const UartTxQueueConfig_t config = {
        .policy = UART_QUEUE_FIFO, .capacity = 15U, .frame_size = 80U,
        .timeout_ms = 200U, .retain_failed = 1U
    };
    if (!initialized && UART_TxQueue_Init(&queue, &huart1, &config, &logs[0][0],
                                         lengths, NULL, current) == HAL_OK) initialized = 1U;
}
void Debug_Log(const char *text)
{
    size_t length = 0U;
    uint8_t *destination;
    if (text == NULL) return;
    Init(); if (!initialized) return;
    while (length < 79U && text[length] != '\0') ++length;
    if (length == 0U) return;
    destination = UART_TxQueue_Reserve(&queue);
    if (destination == NULL) return;
    memcpy(destination, text, length); destination[length] = '\0';
    (void)UART_TxQueue_Commit(&queue, length, 0U);
}
uint8_t Debug_CanLog(uint8_t count)
{
    Init(); return (uint8_t)(initialized && UART_TxQueue_Free(&queue) >= count);
}
void Debug_TxCallback(UART_HandleTypeDef *uart)
{
    if (initialized && uart == &huart1) UART_TxQueue_Complete(&queue);
}
void Debug_Process(void)
{
    Init(); if (initialized) (void)UART_TxQueue_Process(&queue);
}
