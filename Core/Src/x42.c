#include "x42.h"
#include <string.h>

#define X42_CMD_ENABLE       0xF3U
#define X42_CMD_RUN          0xF6U
#define X42_CMD_QUERY_SPEED  0x35U
#define X42_CMD_STATUS       0x3AU
#define X42_ACK_VALUE        0x02U
#define X42_FRAME_END        0x6BU
#define X42_FIXED_SIZE       4U
#define X42_TX_TIMEOUT_MS    100U
#define X42_RX_TIMEOUT_MS    50U
#define X42_RX_GAP_MS        3U

static X42_Result X42_MapHAL(HAL_StatusTypeDef status)
{
    return status == HAL_OK ? X42_OK :
           (status == HAL_TIMEOUT ? X42_TIMEOUT : X42_ERROR);
}

static X42_Result X42_Exchange(X42_HandleTypeDef *motor, const uint8_t *tx,
                              uint16_t tx_len, uint8_t *rx,
                              uint16_t rx_size, uint16_t *rx_len)
{
    HAL_StatusTypeDef status;
    uint32_t start;
    if (rx_len == NULL) {
        return X42_ERROR;
    }
    *rx_len = 0U;
    if (motor == NULL || motor->huart == NULL || rx == NULL || rx_size == 0U) {
        return X42_ERROR;
    }
    if (motor->huart->RxState != HAL_UART_STATE_READY ||
        motor->huart->gState != HAL_UART_STATE_READY) {
        return X42_ERROR;
    }
    /* Clear stale RX/error before TX only. Late replies cannot be correlated
     * without confirmed protocol transaction IDs; caller must serialize. */
    __HAL_UART_CLEAR_OREFLAG(motor->huart);
    status = HAL_UART_Transmit(motor->huart, tx, tx_len, X42_TX_TIMEOUT_MS);
    if (status != HAL_OK) {
        return X42_MapHAL(status);
    }
    start = HAL_GetTick();
    for (;;) {
        uint8_t byte;
        uint32_t elapsed = (uint32_t)(HAL_GetTick() - start);
        uint32_t remaining;
        uint32_t timeout;
        if (elapsed >= X42_RX_TIMEOUT_MS) {
            return X42_TIMEOUT;
        }
        remaining = X42_RX_TIMEOUT_MS - elapsed;
        timeout = (*rx_len > 0U && remaining > X42_RX_GAP_MS) ?
                  X42_RX_GAP_MS : remaining;
        status = HAL_UART_Receive(motor->huart, &byte, 1U, timeout);
        if (status == HAL_TIMEOUT) {
            return (*rx_len > 0U && remaining > X42_RX_GAP_MS) ?
                   X42_OK : X42_TIMEOUT;
        }
        if (status != HAL_OK) {
            return X42_MapHAL(status);
        }
        /* Probe one extra byte to distinguish exact capacity from overflow. */
        if (*rx_len >= rx_size) {
            return X42_BUFFER_TOO_SMALL;
        }
        rx[*rx_len] = byte;
        (*rx_len)++;
    }
}

void X42_Init(X42_HandleTypeDef *motor, UART_HandleTypeDef *huart, uint8_t address)
{
    if (motor != NULL) {
        motor->huart = huart;
        motor->address = address;
    }
}

static X42_Result X42_FixedCommand(X42_HandleTypeDef *motor, uint8_t command)
{
    uint8_t tx[X42_FIXED_SIZE];
    uint8_t rx[X42_FIXED_SIZE];
    uint16_t length;
    X42_Result result;
    if (motor == NULL) {
        return X42_ERROR;
    }
    /* CONFIRMED by current task: request and expected reply are identical.
     * Historical main.c used different requests. This is the user's selected
     * protocol profile; TODO: validate on the actual X42 firmware. */
    tx[0] = motor->address;
    tx[1] = command;
    tx[2] = X42_ACK_VALUE;
    tx[3] = X42_FRAME_END;
    result = X42_Exchange(motor, tx, sizeof(tx), rx, sizeof(rx), &length);
    if (result == X42_BUFFER_TOO_SMALL) {
        return X42_BAD_RESPONSE;
    }
    if (result != X42_OK) {
        return result;
    }
    return (length == sizeof(tx) && memcmp(tx, rx, sizeof(tx)) == 0) ?
           X42_OK : X42_BAD_RESPONSE;
}

X42_Result X42_Enable(X42_HandleTypeDef *motor)
{
    return X42_FixedCommand(motor, X42_CMD_ENABLE);
}

X42_Result X42_Run(X42_HandleTypeDef *motor)
{
    return X42_FixedCommand(motor, X42_CMD_RUN);
}

X42_Result X42_Stop(X42_HandleTypeDef *motor)
{
    (void)motor;
    /* UNCONFIRMED: historical FE frame has no verified device response.
     * TODO: add Stop only after the actual firmware protocol is confirmed. */
    return X42_NOT_SUPPORTED;
}

static X42_Result X42_Query(X42_HandleTypeDef *motor, uint8_t command,
                            uint8_t *rx, uint16_t rx_size, uint16_t *rx_len)
{
    uint8_t tx[3];
    if (rx_len != NULL) {
        *rx_len = 0U;
    }
    if (motor == NULL) {
        return X42_ERROR;
    }
    /* CONFIRMED command IDs. Existing project's request layout is retained.
     * UNCONFIRMED on hardware: address, command, 6B query frame and all reply
     * fields. TODO: verify firmware protocol; do not decode or infer units. */
    tx[0] = motor->address;
    tx[1] = command;
    tx[2] = X42_FRAME_END;
    return X42_Exchange(motor, tx, sizeof(tx), rx, rx_size, rx_len);
}

X42_Result X42_QuerySpeed(X42_HandleTypeDef *motor, uint8_t *rx,
                         uint16_t rx_size, uint16_t *rx_len)
{
    return X42_Query(motor, X42_CMD_QUERY_SPEED, rx, rx_size, rx_len);
}

X42_Result X42_QueryStatus(X42_HandleTypeDef *motor, uint8_t *rx,
                          uint16_t rx_size, uint16_t *rx_len)
{
    return X42_Query(motor, X42_CMD_STATUS, rx, rx_size, rx_len);
}
