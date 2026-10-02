#ifndef X42_H
#define X42_H

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    X42_OK = 0,
    X42_ERROR,
    X42_TIMEOUT,
    X42_BAD_RESPONSE,
    X42_NOT_SUPPORTED,
    X42_BUFFER_TOO_SMALL
} X42_Result;

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t address;
} X42_HandleTypeDef;

/* Foreground blocking driver. Caller must exclusively own this UART:
 * no concurrent IRQ/DMA receiver, bridge, or other caller. Tick must run.
 * Init binds the handle only; UART initialization remains the caller's job. */
void X42_Init(X42_HandleTypeDef *motor, UART_HandleTypeDef *huart, uint8_t address);
X42_Result X42_Enable(X42_HandleTypeDef *motor);
X42_Result X42_Run(X42_HandleTypeDef *motor);
/* UNCONFIRMED: no verified Stop request; sends nothing. */
X42_Result X42_Stop(X42_HandleTypeDef *motor);

/* Raw HEX capture only: OK means bytes captured until an idle gap, NOT
 * validated speed/status. UNCONFIRMED response length/fields/units.
 * rx_len is required and reset on entry; partial data is retained on errors.
 * BUFFER_TOO_SMALL means stored prefix only; remaining UART bytes may exist.
 * A 3 ms gap is a transport heuristic inherited from main.c, not a verified
 * protocol boundary. TODO: replace with confirmed framing when available. */
X42_Result X42_QuerySpeed(X42_HandleTypeDef *motor, uint8_t *rx,
                         uint16_t rx_size, uint16_t *rx_len);
X42_Result X42_QueryStatus(X42_HandleTypeDef *motor, uint8_t *rx,
                          uint16_t rx_size, uint16_t *rx_len);

#ifdef __cplusplus
}
#endif
#endif
