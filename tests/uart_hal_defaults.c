#include "stm32f4xx_hal.h"
#include <stddef.h>
/* The compile command selects the fixture-owned HAL boundaries. */
#ifndef UART_TEST_HAS_HAL_GetTick
uint32_t HAL_GetTick(void) { return 0U; }
#endif
#ifndef UART_TEST_HAS_HAL_UART_Transmit
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u,
    const uint8_t *p, uint16_t n, uint32_t t)
{ (void)u; (void)p; (void)n; (void)t; return HAL_ERROR; }
#endif
#ifndef UART_TEST_HAS_HAL_UART_Transmit_IT
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u,
    const uint8_t *p, uint16_t n)
{ (void)u; (void)p; (void)n; return HAL_ERROR; }
#endif
#ifndef UART_TEST_HAS_HAL_UART_Receive_IT
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u,
    uint8_t *p, uint16_t n)
{
    if (u == NULL || p == NULL || n != 1U) return HAL_ERROR;
    u->rx = p; u->RxState = 1U; return HAL_OK;
}
#endif
#ifndef UART_TEST_HAS_HAL_UART_AbortTransmit
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *u)
{ u->gState = HAL_UART_STATE_READY; return HAL_OK; }
#endif
#ifndef UART_TEST_HAS_HAL_UART_AbortReceive
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *u)
{ u->RxState = HAL_UART_STATE_READY; return HAL_OK; }
#endif
