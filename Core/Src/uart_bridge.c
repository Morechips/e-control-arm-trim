#include "uart_bridge.h"
#include "bluetooth_driver.h"
#include "motor_driver.h"
#include "serial_io.h"
#include "pid_tuner.h"
#include "maxicam.h"

#define BRIDGE_QUEUE_SIZE 1024U

typedef struct {
    UART_HandleTypeDef *uart;
    uint8_t rx;
    uint8_t tx[BRIDGE_QUEUE_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint8_t active;
} BridgePort;

static BridgePort ports[2];
static uint8_t enabled;
static uint8_t dma_rx;
volatile uint32_t usart1_to_usart2_bytes;
volatile uint32_t usart2_to_usart1_bytes;
volatile uint32_t usart1_to_uart5_bytes;
volatile uint32_t uart5_to_usart1_bytes;
volatile uint32_t bridge_dropped_bytes;
volatile uint32_t bridge_uart_errors;

static uint16_t Next(uint16_t index)
{
    return (uint16_t)((index + 1U) % BRIDGE_QUEUE_SIZE);
}

static HAL_StatusTypeDef Init(UART_HandleTypeDef *u1, UART_HandleTypeDef *motor, uint8_t use_dma)
{
    HAL_StatusTypeDef result;
    if (enabled || u1 == NULL || motor == NULL || u1 == motor) {
        return HAL_ERROR;
    }
    ports[0].uart = u1;
    ports[1].uart = motor;
    dma_rx = use_dma;
    enabled = 1U;
    result = HAL_UART_Receive_IT(u1, &ports[0].rx, 1U);
    if (result != HAL_OK) {
        enabled = 0U;
        return result;
    }
    if (dma_rx) return HAL_OK;
    result = HAL_UART_Receive_IT(motor, &ports[1].rx, 1U);
    if (result != HAL_OK) {
        enabled = 0U;
        (void)HAL_UART_AbortReceive(u1);
    }
    return result;
}

HAL_StatusTypeDef UART_Bridge_Init(UART_HandleTypeDef *u1, UART_HandleTypeDef *motor)
{
    return Init(u1, motor, 0U);
}

HAL_StatusTypeDef UART_Bridge_InitDMA(UART_HandleTypeDef *u1, UART_HandleTypeDef *u2)
{
    return Init(u1, u2, 1U);
}

void UART_Bridge_FeedDMA(const uint8_t *data, uint16_t size)
{
    uint16_t i;
    if (!enabled || !dma_rx || data == NULL) return;
    /* Foreground is the only producer for U1 TX; IRQ only advances tail. */
    for (i = 0U; i < size; i++) {
        uint16_t next = Next(ports[0].head);
        if (next == ports[0].tail) {
            uint32_t mask = __get_PRIMASK();
            __disable_irq();
            bridge_dropped_bytes++;
            __set_PRIMASK(mask);
            continue;
        }
        ports[0].tx[ports[0].head] = data[i];
        __DMB();
        ports[0].head = next;
        usart2_to_usart1_bytes++;
    }
}

void UART_Bridge_Process(void)
{
    uint32_t i;
    if (!enabled) {
        return;
    }
    for (i = 0U; i < 2U; i++) {
        BridgePort *port = &ports[i];
        uint32_t mask = __get_PRIMASK();
        /* IRQ callbacks publish RX bytes and retire TX bytes. Protect the
         * foreground start operation; never wait for UART data here. */
        __disable_irq();
        if (!(dma_rx && i == 1U) && port->uart->RxState == HAL_UART_STATE_READY) {
            if (HAL_UART_Receive_IT(port->uart, &port->rx, 1U) != HAL_OK) {
                bridge_uart_errors++;
            }
        }
        if (!port->active && port->head != port->tail) {
            port->active = 1U;
            if (HAL_UART_Transmit_IT(port->uart, &port->tx[port->tail], 1U) != HAL_OK) {
                port->active = 0U;
                bridge_uart_errors++;
            }
        }
        __set_PRIMASK(mask);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    uint32_t i;
    if (!enabled || (uart != ports[0].uart && uart != ports[1].uart)) {
        Bluetooth_RxCallback(uart); Motor_RxCallback(uart); MaxiCam_RxCallback(uart);
        return;
    }
    if (!enabled) {
        return;
    }
    for (i = 0U; i < 2U; i++) {
        if (uart == ports[i].uart) {
            if (dma_rx && i == 1U) return;
            BridgePort *destination = &ports[1U - i];
            uint8_t byte = ports[i].rx;
            uint16_t next = Next(destination->head);
            if (HAL_UART_Receive_IT(uart, &ports[i].rx, 1U) != HAL_OK) {
                bridge_uart_errors++;
            }
            if (next == destination->tail) {
                /* Finite buffering: retain existing bytes, drop newest. */
                bridge_dropped_bytes++;
            } else {
                destination->tx[destination->head] = byte;
                destination->head = next;
                if (i == 0U) {
                    if (dma_rx) usart1_to_usart2_bytes++;
                    else usart1_to_uart5_bytes++;
                } else {
                    uart5_to_usart1_bytes++;
                }
            }
            return;
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    uint32_t i;
    if (!enabled || (uart != ports[0].uart && uart != ports[1].uart)) {
        Motor_TxCallback(uart); Debug_TxCallback(uart); PID_Tuner_TxCallback(uart);
        return;
    }
    if (!enabled) {
        return;
    }
    for (i = 0U; i < 2U; i++) {
        if (uart == ports[i].uart && ports[i].active) {
            ports[i].tail = Next(ports[i].tail);
            ports[i].active = 0U;
            return;
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (!enabled || (uart != ports[0].uart && uart != ports[1].uart)) {
        Bluetooth_ErrorCallback(uart); Motor_ErrorCallback(uart); MaxiCam_ErrorCallback(uart);
        return;
    }
    if (enabled && (uart == ports[0].uart || uart == ports[1].uart)) {
        /* HAL ends blocking RX errors before this callback. Foreground
         * Process rearms READY receivers; TX is independent of RX errors. */
        bridge_uart_errors++;
    }
}
