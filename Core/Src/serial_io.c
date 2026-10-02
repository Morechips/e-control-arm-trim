#include "serial_io.h"
#include <string.h>
static char logs[16][80];
static uint8_t log_head, log_tail;
static volatile uint8_t log_active;
static uint32_t log_tick;
void Serial_Init(SerialRx *rx, UART_HandleTypeDef *uart)
{
    memset(rx, 0, sizeof(*rx));
    rx->uart = uart;
    if (HAL_UART_Receive_IT(uart, &rx->byte, 1U) != HAL_OK) rx->broken = 1U;
}
void Serial_RxCallback(SerialRx *rx, UART_HandleTypeDef *uart)
{
    uint16_t next;
    if (rx->uart != uart) return;
    next = (uint16_t)((rx->head + 1U) % SERIAL_RX_SIZE);
    if (next == rx->tail) rx->broken = 1U;
    if (!rx->broken) {
        rx->data[rx->head] = rx->byte;
        rx->tick[rx->head] = HAL_GetTick();
        __DMB();
        rx->head = next;
    }
    if (HAL_UART_Receive_IT(uart, &rx->byte, 1U) != HAL_OK) rx->broken = 1U;
}
void Serial_ErrorCallback(SerialRx *rx, UART_HandleTypeDef *uart)
{
    if (rx->uart == uart) rx->broken = 1U;
}
uint8_t Serial_Recover(SerialRx *rx)
{
    uint8_t broken;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    broken = rx->broken;
    if (broken) {
        rx->tail = rx->head;
        rx->broken = 0U;
    }
    if (rx->uart->RxState == HAL_UART_STATE_READY) {
        if (HAL_UART_Receive_IT(rx->uart, &rx->byte, 1U) != HAL_OK) rx->broken = 1U;
    }
    __set_PRIMASK(mask);
    return broken;
}
uint8_t Serial_Pop(SerialRx *rx, uint8_t *byte, uint32_t *tick)
{
    if (rx->tail == rx->head || rx->broken) return 0U;
    __DMB();
    *byte = rx->data[rx->tail];
    *tick = rx->tick[rx->tail];
    __DMB();
    rx->tail = (uint16_t)((rx->tail + 1U) % SERIAL_RX_SIZE);
    return 1U;
}
uint8_t Serial_Peek(const SerialRx *rx, uint8_t *byte)
{
    if (rx->tail == rx->head || rx->broken) return 0U;
    __DMB();
    *byte = rx->data[rx->tail];
    return 1U;
}
void Debug_Log(const char *text)
{
    uint8_t next = (uint8_t)((log_head + 1U) % 16U);
    if (next == log_tail) return; /* Logs never hold up safety processing. */
    (void)strncpy(logs[log_head], text, sizeof(logs[0]) - 1U);
    logs[log_head][sizeof(logs[0]) - 1U] = '\0';
    log_head = next;
}
uint8_t Debug_CanLog(uint8_t count)
{
    return (uint8_t)(((log_tail + 16U - log_head - 1U) % 16U) >= count);
}
void Debug_TxCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart1) log_active = 0U;
}
void Debug_Process(void)
{
    static char tx[80];
    if (log_active && (uint32_t)(HAL_GetTick() - log_tick) > 200U) {
        (void)HAL_UART_AbortTransmit(&huart1);
        log_active = 0U;
    }
    if (log_active || log_head == log_tail) return;
    memcpy(tx, logs[log_tail], sizeof(tx));
    log_active = 1U;
    log_tick = HAL_GetTick();
    if (HAL_UART_Transmit_IT(&huart1, (uint8_t *)tx, (uint16_t)strlen(tx)) == HAL_OK)
        log_tail = (uint8_t)((log_tail + 1U) % 16U);
    else log_active = 0U;
}
