#include "main.h"

void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef gpio = {0};
    if (hi2c->Instance == I2C1)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        gpio.Mode = GPIO_MODE_AF_OD;
        /* External 4.7 kohm pull-ups to 3.3 V on SCL and SDA. */
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF4_I2C1;
        HAL_GPIO_Init(GPIOB, &gpio);
        __HAL_RCC_I2C1_CLK_ENABLE();
        HAL_NVIC_SetPriority(I2C1_EV_IRQn, 3U, 0U);
        HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
        HAL_NVIC_SetPriority(I2C1_ER_IRQn, 3U, 0U);
        HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
    }
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gpio = {0};
    if (huart->Instance == USART1)
    {
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        gpio.Pin = GPIO_PIN_9 | GPIO_PIN_10;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOA, &gpio);
        HAL_NVIC_SetPriority(USART1_IRQn, 1U, 0U);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    }
    else if (huart->Instance == USART3)
    {
        __HAL_RCC_USART3_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        /* Dedicated ZL-IS2 port: PB10 TX, PB11 RX, interrupt TX. */
        gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF7_USART3;
        HAL_GPIO_Init(GPIOB, &gpio);
        HAL_NVIC_SetPriority(USART3_IRQn, 3U, 0U);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
    }
    else if (huart->Instance == UART4)
    {
        __HAL_RCC_UART4_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        /* MaxiCam vision port: PC10 TX, PC11 RX. */
        gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF8_UART4;
        HAL_GPIO_Init(GPIOC, &gpio);
        HAL_NVIC_SetPriority(UART4_IRQn, 3U, 0U);
        HAL_NVIC_EnableIRQ(UART4_IRQn);
    }
    else if (huart->Instance == UART5)
    {
        __HAL_RCC_UART5_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();

        gpio.Pin = GPIO_PIN_12;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF8_UART5;
        HAL_GPIO_Init(GPIOC, &gpio);

        gpio.Pin = GPIO_PIN_2;
        HAL_GPIO_Init(GPIOD, &gpio);
        HAL_NVIC_SetPriority(UART5_IRQn, 1U, 0U);
        HAL_NVIC_EnableIRQ(UART5_IRQn);
    }
    else if (huart->Instance == USART6)
    {
        __HAL_RCC_USART6_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF8_USART6;
        HAL_GPIO_Init(GPIOC, &gpio);
        HAL_NVIC_SetPriority(USART6_IRQn, 1U, 0U);
        HAL_NVIC_EnableIRQ(USART6_IRQn);
    }
}
