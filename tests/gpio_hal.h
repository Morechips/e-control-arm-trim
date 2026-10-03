#ifndef TEST_GPIO_HAL_H
#define TEST_GPIO_HAL_H
#include <stdint.h>
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
typedef struct { uint32_t IDR; } GPIO_TypeDef;
typedef struct { uint32_t Pin, Mode, Pull, Speed; } GPIO_InitTypeDef;
extern GPIO_TypeDef mock_gpiob, mock_gpioc, mock_gpiod, mock_gpioe;
#define GPIOB (&mock_gpiob)
#define GPIOC (&mock_gpioc)
#define GPIOD (&mock_gpiod)
#define GPIOE (&mock_gpioe)
#define GPIO_PIN_0 0x0001U
#define GPIO_PIN_1 0x0002U
#define GPIO_PIN_3 0x0008U
#define GPIO_PIN_4 0x0010U
#define GPIO_PIN_8 0x0100U
#define GPIO_PIN_10 0x0400U
#define GPIO_PIN_11 0x0800U
#define GPIO_PIN_14 0x4000U
#define GPIO_PIN_15 0x8000U
#define GPIO_MODE_INPUT 0U
#define GPIO_MODE_OUTPUT_OD 1U
#define GPIO_NOPULL 0U
#define GPIO_PULLUP 1U
#define GPIO_SPEED_FREQ_LOW 0U
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOC_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOD_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOE_CLK_ENABLE() ((void)0)
void HAL_GPIO_Init(GPIO_TypeDef *, GPIO_InitTypeDef *);
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint16_t, GPIO_PinState);
#endif
