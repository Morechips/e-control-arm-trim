#ifndef ZLIS2_TEST_HAL_H
#define ZLIS2_TEST_HAL_H
#include <stdint.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct {
    uint32_t BaudRate, WordLength, StopBits, Parity, Mode, HwFlowCtl;
} UART_InitTypeDef;
typedef struct {
    void *Instance;
    UART_InitTypeDef Init;
    uint32_t gState;
} UART_HandleTypeDef;
#define UART_WORDLENGTH_8B 0U
#define UART_WORDLENGTH_9B 0x1000U
#define UART_STOPBITS_1 0U
#define UART_STOPBITS_2 0x2000U
#define UART_PARITY_NONE 0U
#define UART_PARITY_EVEN 0x400U
#define UART_MODE_TX 8U
#define UART_MODE_RX 4U
#define UART_MODE_TX_RX (UART_MODE_TX | UART_MODE_RX)
#define UART_HWCONTROL_NONE 0U
#define UART_HWCONTROL_RTS 0x100U
#define HAL_UART_STATE_READY 0x20U
#define HAL_UART_STATE_RESET 0U
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, const uint8_t *, uint16_t, uint32_t);
#endif
