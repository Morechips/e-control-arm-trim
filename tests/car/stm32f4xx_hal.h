#ifndef TEST_CAR_HAL_H
#define TEST_CAR_HAL_H
#include <stdint.h>
#include <stddef.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
#include "../gpio_hal.h"
typedef struct { int unused; } I2C_HandleTypeDef;
#define HAL_UART_STATE_READY 0x20U
typedef struct {
    uint32_t BaudRate, WordLength, StopBits, Parity, Mode, HwFlowCtl;
} UART_InitTypeDef;
typedef struct {
    void *Instance;
    UART_InitTypeDef Init;
    uint32_t RxState, gState;
    uint8_t *rx;
    uint8_t pending;
} UART_HandleTypeDef;
#define UART_WORDLENGTH_8B 0U
#define UART_STOPBITS_1 0U
#define UART_PARITY_NONE 0U
#define UART_MODE_TX 8U
#define UART_MODE_RX 4U
#define UART_MODE_TX_RX (UART_MODE_TX | UART_MODE_RX)
#define UART_HWCONTROL_NONE 0U
#define __DMB() ((void)0)
#define __HAL_UART_CLEAR_OREFLAG(u) ((void)(u))
#define __get_PRIMASK() 0U
#define __disable_irq() ((void)0)
#define __set_PRIMASK(m) ((void)(m))
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *, const uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, const uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *);
#endif
