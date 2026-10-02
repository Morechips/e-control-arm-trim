#include "zlis2_driver.h"
#include <stdarg.h>
#include <stdio.h>

static UART_HandleTypeDef *zlis2_uart;

static ZLIS2_Status zlis2_send(const char *data, size_t length)
{
    if (zlis2_uart == NULL) return ZLIS2_NOT_INITIALIZED;
    if (data == NULL || length == 0U) return ZLIS2_INVALID_PARAM;
    if (length > ZLIS2_MAX_TX_LENGTH) return ZLIS2_BUFFER_OVERFLOW;
    /* 362 bytes take about 31.5 ms at 115200 8N1; leave bounded margin.
     * No interrupt masking, DMA, logs, delays, RX parsing or retries here. */
    return HAL_UART_Transmit(zlis2_uart, (const uint8_t *)data,
                             (uint16_t)length, ZLIS2_TX_TIMEOUT_MS) == HAL_OK
           ? ZLIS2_OK : ZLIS2_UART_ERROR;
}

static ZLIS2_Status zlis2_format_send(const char *format, ...)
{
    char buffer[32];
    int length;
    va_list args;
    if (zlis2_uart == NULL) return ZLIS2_NOT_INITIALIZED;
    va_start(args, format);
    length = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    if (length < 0) return ZLIS2_ERROR;
    if ((size_t)length >= sizeof(buffer)) return ZLIS2_BUFFER_OVERFLOW;
    return zlis2_send(buffer, (size_t)length);
}

static int zlis2_servo_valid(uint16_t id, uint16_t pwm, uint16_t time_ms)
{
    return id <= 254U && pwm >= 500U && pwm <= 2500U && time_ms <= 9999U;
}

ZLIS2_Status ZLIS2_Init(UART_HandleTypeDef *huart)
{
    zlis2_uart = NULL;
    if (huart == NULL || huart->Instance == NULL) return ZLIS2_INVALID_PARAM;
    if (huart->Init.BaudRate != ZLIS2_BAUD_RATE ||
        huart->Init.WordLength != UART_WORDLENGTH_8B ||
        huart->Init.StopBits != UART_STOPBITS_1 ||
        huart->Init.Parity != UART_PARITY_NONE ||
        huart->Init.HwFlowCtl != UART_HWCONTROL_NONE ||
        (huart->Init.Mode & UART_MODE_TX) == 0U) return ZLIS2_INVALID_PARAM;
    if (huart->gState != HAL_UART_STATE_READY) return ZLIS2_UART_ERROR;
    zlis2_uart = huart;
    return ZLIS2_OK;
}

ZLIS2_Status ZLIS2_SetServo(uint16_t id, uint16_t pwm, uint16_t time_ms)
{
    if (!zlis2_servo_valid(id, pwm, time_ms)) return ZLIS2_INVALID_PARAM;
    return zlis2_format_send("#%03uP%04uT%04u!", (unsigned int)id,
                             (unsigned int)pwm, (unsigned int)time_ms);
}

ZLIS2_Status ZLIS2_SetServos(const ZLIS2_ServoCommand *commands, size_t count)
{
    char buffer[ZLIS2_MAX_TX_LENGTH + 1U];
    size_t i, used = 1U;
    if (zlis2_uart == NULL) return ZLIS2_NOT_INITIALIZED;
    if (commands == NULL || count == 0U) return ZLIS2_INVALID_PARAM;
    /* Check count before multiplication, traversal or writing to the buffer. */
    if (count > ZLIS2_MAX_SERVO_COMMANDS) return ZLIS2_BUFFER_OVERFLOW;
    for (i = 0U; i < count; ++i) {
        if (!zlis2_servo_valid(commands[i].id, commands[i].pwm, commands[i].time_ms))
            return ZLIS2_INVALID_PARAM;
    }
    buffer[0] = '{';
    for (i = 0U; i < count; ++i) {
        int length = snprintf(buffer + used, sizeof(buffer) - used,
                              "#%03uP%04uT%04u!", (unsigned int)commands[i].id,
                              (unsigned int)commands[i].pwm,
                              (unsigned int)commands[i].time_ms);
        if (length < 0) return ZLIS2_ERROR;
        if ((size_t)length >= sizeof(buffer) - used) return ZLIS2_BUFFER_OVERFLOW;
        used += (size_t)length;
    }
    if (sizeof(buffer) - used < 2U) return ZLIS2_BUFFER_OVERFLOW;
    buffer[used++] = '}';
    buffer[used] = '\0';
    return zlis2_send(buffer, used);
}

ZLIS2_Status ZLIS2_RunAction(uint16_t action)
{
    return zlis2_format_send("$DGS:%u!", (unsigned int)action);
}

ZLIS2_Status ZLIS2_RunActionRange(uint16_t start_action, uint16_t end_action, uint16_t repeat)
{
    if (start_action > end_action) return ZLIS2_INVALID_PARAM;
    return zlis2_format_send("$DGT:%u-%u,%u!", (unsigned int)start_action,
                             (unsigned int)end_action, (unsigned int)repeat);
}

ZLIS2_Status ZLIS2_StopAll(void) { return zlis2_send("$DST!", sizeof("$DST!") - 1U); }

ZLIS2_Status ZLIS2_StopServo(uint16_t id)
{
    if (id > 254U) return ZLIS2_INVALID_PARAM;
    return zlis2_format_send("$DST:%u!", (unsigned int)id);
}

ZLIS2_Status ZLIS2_SetServoBias(uint16_t id, int16_t bias)
{
    unsigned int magnitude;
    if (id > 254U || bias < -500 || bias > 500) return ZLIS2_INVALID_PARAM;
    magnitude = (unsigned int)(bias < 0 ? -(int)bias : (int)bias);
    return zlis2_format_send("#%03uPSCK%c%03u!", (unsigned int)id,
                             bias < 0 ? '-' : '+', magnitude);
}

ZLIS2_Status ZLIS2_RunCombinedAction(uint16_t group, uint16_t repeat)
{
    return zlis2_format_send("$DKT:%u,%u!", (unsigned int)group, (unsigned int)repeat);
}

ZLIS2_Status ZLIS2_RecordPose(void)
{
    return zlis2_send("$DJ_RECORD!", sizeof("$DJ_RECORD!") - 1U);
}

ZLIS2_Status ZLIS2_RunRecorded(uint16_t repeat)
{
    return zlis2_format_send("$DJ_RECORD_DO:%u!", (unsigned int)repeat);
}

ZLIS2_Status ZLIS2_ClearRecorded(void)
{
    return zlis2_send("$DJ_RECORD_CLEAR!", sizeof("$DJ_RECORD_CLEAR!") - 1U);
}

ZLIS2_Status ZLIS2_SetRecordPeriod(uint16_t period_ms)
{
    return zlis2_format_send("$DJ_RECORD_TIME:%u!", (unsigned int)period_ms);
}

ZLIS2_Status ZLIS2_Reset(void) { return zlis2_send("$RST!", sizeof("$RST!") - 1U); }

ZLIS2_Status ZLIS2_SendRaw(const char *command)
{
    size_t length = 0U;
    if (zlis2_uart == NULL) return ZLIS2_NOT_INITIALIZED;
    if (command == NULL || command[0] == '\0') return ZLIS2_INVALID_PARAM;
    while (length <= ZLIS2_MAX_TX_LENGTH && command[length] != '\0') ++length;
    if (length > ZLIS2_MAX_TX_LENGTH) return ZLIS2_BUFFER_OVERFLOW;
    return zlis2_send(command, length);
}
