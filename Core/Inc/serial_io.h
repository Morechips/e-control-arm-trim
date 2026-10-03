#ifndef SERIAL_IO_H
#define SERIAL_IO_H
#include "main.h"
#include "uart_driver.h"
void Debug_Log(const char *text);
uint8_t Debug_CanLog(uint8_t count);
void Debug_Process(void);
void Debug_TxCallback(UART_HandleTypeDef *uart);
#endif
