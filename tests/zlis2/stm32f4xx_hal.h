#ifndef SERVO_TEST_HAL_H
#define SERVO_TEST_HAL_H
#include <stdint.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct {
    uint32_t BaudRate, WordLength, StopBits, Parity, Mode, HwFlowCtl;
} UART_InitTypeDef;
typedef struct {
    void *Instance;
    UART_InitTypeDef Init;
    uint32_t gState, RxState;
    uint8_t *rx;
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
#define HAL_UART_STATE_BUSY_TX 0x21U
#define __get_PRIMASK() 0U
#define __disable_irq() ((void)0)
#define __set_PRIMASK(m) ((void)(m))
#define __DMB() ((void)0)
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *, const uint8_t *, uint16_t);
#define __HAL_UART_CLEAR_OREFLAG(u) ((void)(u))
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *);
#endif
