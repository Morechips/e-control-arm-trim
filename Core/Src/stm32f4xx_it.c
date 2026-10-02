#include "main.h"
#include "usart2_dma.h"

void TIM6_DAC_IRQHandler(void) { USART2_DMA_TimerIRQ(); }
void USART2_IRQHandler(void) { HAL_UART_IRQHandler(&huart2); }

void SysTick_Handler(void)
{
    HAL_IncTick();
}

void HardFault_Handler(void)
{
    Error_Handler();
}

void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}

void UART4_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart4);
}

void UART5_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart5);
}

void USART6_IRQHandler(void) { HAL_UART_IRQHandler(&huart6); }
void I2C1_EV_IRQHandler(void) { HAL_I2C_EV_IRQHandler(&hi2c1); }
void I2C1_ER_IRQHandler(void) { HAL_I2C_ER_IRQHandler(&hi2c1); }
