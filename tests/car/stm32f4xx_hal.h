#ifndef TEST_CAR_HAL_H
#define TEST_CAR_HAL_H
#include <stdint.h>
#include <stddef.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
typedef struct { uint32_t IDR; } GPIO_TypeDef;
typedef struct {
    uint32_t Pin, Mode, Pull, Speed;
} GPIO_InitTypeDef;
extern GPIO_TypeDef mock_gpioc;
extern GPIO_TypeDef mock_gpiob;
#define GPIOC (&mock_gpioc)
#define GPIOB (&mock_gpiob)
#define GPIO_PIN_3 0x0008U
#define GPIO_PIN_8 0x0100U
#define GPIO_MODE_INPUT 0U
#define GPIO_MODE_OUTPUT_OD 1U
#define GPIO_NOPULL 0U
#define GPIO_PULLUP 1U
#define GPIO_SPEED_FREQ_LOW 0U
#define __HAL_RCC_GPIOC_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
void HAL_GPIO_Init(GPIO_TypeDef *, GPIO_InitTypeDef *);
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint16_t, GPIO_PinState);
typedef struct { int unused; } I2C_HandleTypeDef;
#define HAL_UART_STATE_READY 0x20U
typedef struct {
    uint32_t RxState, gState;
    uint8_t *rx;
    uint8_t pending;
} UART_HandleTypeDef;
#define __DMB() ((void)0)
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
