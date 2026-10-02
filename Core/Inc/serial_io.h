#ifndef SERIAL_IO_H
#define SERIAL_IO_H
#include "main.h"
/* Single producer (ISR), single consumer (foreground). Drop invalid streams. */
#define SERIAL_RX_SIZE 256U
typedef struct {
    UART_HandleTypeDef *uart;
    uint8_t byte;
    uint8_t data[SERIAL_RX_SIZE];
    uint32_t tick[SERIAL_RX_SIZE];
    volatile uint16_t head, tail;
    volatile uint8_t broken;
} SerialRx;
void Serial_Init(SerialRx *rx, UART_HandleTypeDef *uart);
void Serial_RxCallback(SerialRx *rx, UART_HandleTypeDef *uart);
void Serial_ErrorCallback(SerialRx *rx, UART_HandleTypeDef *uart);
uint8_t Serial_Recover(SerialRx *rx);
uint8_t Serial_Pop(SerialRx *rx, uint8_t *byte, uint32_t *tick);
/* Read the next queued byte without consuming it, for frame boundaries. */
uint8_t Serial_Peek(const SerialRx *rx, uint8_t *byte);
void Debug_Log(const char *text);
uint8_t Debug_CanLog(uint8_t count);
void Debug_Process(void);
void Debug_TxCallback(UART_HandleTypeDef *uart);
#endif
