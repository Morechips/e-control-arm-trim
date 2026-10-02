#include "emm42_driver.h"
#include "serial_io.h"
#include <stdio.h>
#include <string.h>
#define CMD_ENABLE 0xF3U
#define CMD_SPEED 0xF6U
#define CMD_STOP 0xFEU
#define CMD_SYNC 0xFFU
#define FRAME_END 0x6BU
#define ACK_OK 0x02U
#define ACK_CONDITION 0xE2U
#define ACK_ERROR 0xEEU
#define TX_QUEUE_SIZE 16U

typedef struct { uint8_t bytes[8]; uint16_t length; } MotorFrame;
static MotorFrame queue[TX_QUEUE_SIZE], transmitting;
static uint8_t head, tail;
static volatile uint8_t active, fault;
static volatile uint32_t completed_tick;
static uint32_t started_tick;
static SerialRx rx;
static Emm42Status_t statuses[5];
static uint8_t reply[4], reply_used;
static volatile uint32_t stop_requests, uart5_errors, tx_timeouts, tx_start_errors;
static volatile uint8_t stop_start_mask, stop_done_mask, stop_ack_mask;
static uint32_t stop_diag_ms, transmitting_stop_request;

static void LogStopDiagnostics(void)
{
    char line[80];
    uint32_t now = HAL_GetTick();
    if (stop_requests == 0U || (uint32_t)(now - stop_diag_ms) < 1000U ||
        !Debug_CanLog(2U)) return;
    stop_diag_ms = now;
    (void)snprintf(line, sizeof(line),
                   "[MSTOP] req=%lu tx=%X done=%X ack=%X\r\n",
                   (unsigned long)stop_requests, (unsigned)stop_start_mask,
                   (unsigned)stop_done_mask, (unsigned)stop_ack_mask);
    Debug_Log(line);
    (void)snprintf(line, sizeof(line),
                   "[MSTOP] uerr=%lu timeout=%lu txerr=%lu fault=%u\r\n",
                   (unsigned long)uart5_errors, (unsigned long)tx_timeouts,
                   (unsigned long)tx_start_errors, (unsigned)fault);
    Debug_Log(line);
}
static uint8_t AddressValid(uint8_t addr) { return (uint8_t)(addr >= 1U && addr <= 4U); }
static HAL_StatusTypeDef Enqueue(const uint8_t *bytes, uint16_t length)
{
    uint8_t next = (uint8_t)((head + 1U) % TX_QUEUE_SIZE);
    if (next == tail) return HAL_BUSY;
    memcpy(queue[head].bytes, bytes, length);
    queue[head].length = length;
    head = next;
    return HAL_OK;
}
static HAL_StatusTypeDef EnableFrame(uint8_t addr, uint8_t en)
{
    uint8_t bytes[] = {addr, CMD_ENABLE, 0xABU, en, 0U, FRAME_END};
    if (!AddressValid(addr)) return HAL_ERROR;
    return Enqueue(bytes, sizeof(bytes));
}
static HAL_StatusTypeDef SpeedFrame(uint8_t addr, int16_t rpm, uint8_t sync)
{
    int32_t value = rpm;
    uint16_t magnitude;
    uint8_t bytes[] = {addr, CMD_SPEED, 0U, 0U, 0U, MOTOR_ACC, sync, FRAME_END};
    if (!AddressValid(addr)) return HAL_ERROR;



    bytes[2] = (uint8_t)(value < 0 ? 1U : 0U);
    magnitude = (uint16_t)(value < 0 ? -value : value);
    bytes[3] = (uint8_t)(magnitude >> 8);
    bytes[4] = (uint8_t)magnitude;
    return Enqueue(bytes, sizeof(bytes));
}
static HAL_StatusTypeDef StopFrame(uint8_t addr)
{
    uint8_t bytes[] = {addr, CMD_STOP, 0x98U, 0U, FRAME_END};
    return Enqueue(bytes, sizeof(bytes));
}
void Emm42_Init(void)
{
    head = tail = active = fault = reply_used = 0U;
    stop_requests = uart5_errors = tx_timeouts = tx_start_errors = 0U;
    stop_start_mask = stop_done_mask = stop_ack_mask = 0U;
    stop_diag_ms = HAL_GetTick();
    transmitting_stop_request = 0U;
    memset(statuses, 0, sizeof(statuses));
    completed_tick = HAL_GetTick() - MOTOR_FRAME_GAP_MS;
    Serial_Init(&rx, &huart5);
    (void)Emm42_EStopAll();
    (void)Emm42_DisableAll(); /* MCU reset must not retain an old motor target. */
}
uint8_t Emm42_IsIdle(void) { return (uint8_t)(!active && head == tail); }
HAL_StatusTypeDef Emm42_Enable(uint8_t addr)
{
    if (!AddressValid(addr) || fault) return HAL_ERROR;
    if (!Emm42_IsIdle()) return HAL_BUSY;
    return EnableFrame(addr, 1U);
}
HAL_StatusTypeDef Emm42_Disable(uint8_t addr) { return EnableFrame(addr, 0U); }
HAL_StatusTypeDef Emm42_EnableAll(void)
{
    uint8_t addr;
    if (fault) return HAL_ERROR;
    if (!Emm42_IsIdle()) return HAL_BUSY;
    for (addr = 1U; addr <= 4U; ++addr) (void)EnableFrame(addr, 1U);
    return HAL_OK;
}
HAL_StatusTypeDef Emm42_DisableAll(void)
{
    uint8_t addr;
    uint8_t free_slots = (uint8_t)((tail + TX_QUEUE_SIZE - head - 1U) % TX_QUEUE_SIZE);
    if (free_slots < 4U) return HAL_BUSY;
    for (addr = 1U; addr <= 4U; ++addr) (void)EnableFrame(addr, 0U);
    return HAL_OK;
}
HAL_StatusTypeDef Emm42_SetSpeed(uint8_t addr, int16_t rpm)
{
    if (!AddressValid(addr) || fault) return HAL_ERROR;
    if (!Emm42_IsIdle()) return HAL_BUSY;
    return SpeedFrame(addr, rpm, 0U);
}
HAL_StatusTypeDef Emm42_SetSpeedSync4(int16_t a, int16_t b, int16_t c, int16_t d)
{
    const uint8_t sync[] = {0U, CMD_SYNC, 0x66U, FRAME_END};
    if (fault) return HAL_ERROR;
    if (!Emm42_IsIdle()) return HAL_BUSY;
    (void)SpeedFrame(1U, a, 1U);
    (void)SpeedFrame(2U, b, 1U);
    (void)SpeedFrame(3U, c, 1U);
    (void)SpeedFrame(4U, d, 1U);
    return Enqueue(sync, sizeof(sync));
}
HAL_StatusTypeDef Emm42_EStop(uint8_t addr)
{
    if (!AddressValid(addr)) return HAL_ERROR;
    head = tail = 0U; /* Cancel unsent speed frames and the sync trigger. */
    return StopFrame(addr);
}
HAL_StatusTypeDef Emm42_EStopAll(void)
{
    uint8_t addr;
    stop_requests++;
    stop_start_mask = stop_done_mask = stop_ack_mask = 0U;
    head = tail = 0U;
    /* Finish at most ONE in-flight frame to avoid corrupting the wire protocol.
     * No other normal frame is launched by an ISR. Stops take the next slots. */
    for (addr = 1U; addr <= 4U; ++addr) (void)StopFrame(addr);
    completed_tick = HAL_GetTick() - MOTOR_FRAME_GAP_MS;
    return HAL_OK;
}
void Emm42_RxCallback(UART_HandleTypeDef *uart) { Serial_RxCallback(&rx, uart); }
void Emm42_ErrorCallback(UART_HandleTypeDef *uart)
{
    Serial_ErrorCallback(&rx, uart);
    if (uart == &huart5) { uart5_errors++; fault = 1U; }
}
void Emm42_TxCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart5) {
        if (transmitting.bytes[1] == CMD_STOP &&
            transmitting_stop_request == stop_requests &&
            AddressValid(transmitting.bytes[0]))
            stop_done_mask |= (uint8_t)(1U << (transmitting.bytes[0] - 1U));
        completed_tick = HAL_GetTick(); active = 0U;
    }
}
void Emm42_ProcessRx(const uint8_t *data, uint16_t length)
{
    uint16_t n;
    if (data == NULL) return;
    for (n = 0U; n < length; ++n) {
        uint8_t addr, command, status;
        reply[reply_used++] = data[n];
        if (reply_used < sizeof(reply)) continue;
        addr = reply[0]; command = reply[1]; status = reply[2];
        if (AddressValid(addr) && reply[3] == FRAME_END &&
            (command == CMD_ENABLE || command == CMD_SPEED || command == CMD_STOP || command == CMD_SYNC) &&
            (status == ACK_OK || status == ACK_CONDITION || status == ACK_ERROR)) {
            Emm42Status_t *s = &statuses[addr];
            if (status != ACK_OK || command != CMD_SPEED || !s->valid || s->response_status != ACK_OK) {
                char log[64];
                (void)snprintf(log, sizeof(log), "[M%u] %02X %s %02X\r\n",
                    (unsigned)addr, (unsigned)command, status == ACK_OK ? "OK" : "ERROR", (unsigned)status);
                Debug_Log(log);
            }
            s->command = command; s->response_status = status;
            s->timestamp = HAL_GetTick(); s->valid = 1U;
            if (command == CMD_STOP && status == ACK_OK &&
                (stop_start_mask & (uint8_t)(1U << (addr - 1U))) != 0U)
                stop_ack_mask |= (uint8_t)(1U << (addr - 1U));
            if (status != ACK_OK) fault = 1U;
            reply_used = 0U;
        } else {
            memmove(reply, &reply[1], sizeof(reply) - 1U);
            reply_used = (uint8_t)(sizeof(reply) - 1U);
        }
    }
}
void Emm42_Process(void)
{
    uint8_t byte;
    uint32_t tick;
    if (Serial_Recover(&rx)) { fault = 1U; reply_used = 0U; }
    while (Serial_Pop(&rx, &byte, &tick)) Emm42_ProcessRx(&byte, 1U);
    LogStopDiagnostics();
    if (active && (uint32_t)(HAL_GetTick() - started_tick) > MOTOR_TX_TIMEOUT_MS) {
        tx_timeouts++;
        (void)HAL_UART_AbortTransmit(&huart5);
        active = 0U; fault = 1U;
        (void)Emm42_EStopAll();
    }
    if (active || head == tail ||
        (uint32_t)(HAL_GetTick() - completed_tick) < MOTOR_FRAME_GAP_MS) return;
    /* A fault must not launch the remaining part of a normal transaction. */
    if (fault && queue[tail].bytes[1] != CMD_STOP &&
        !(queue[tail].bytes[1] == CMD_ENABLE && queue[tail].bytes[3] == 0U)) {
        (void)Emm42_EStopAll();
    }
    transmitting = queue[tail];
    transmitting_stop_request = stop_requests;
    active = 1U;
    started_tick = HAL_GetTick();
    if (HAL_UART_Transmit_IT(&huart5, transmitting.bytes, transmitting.length) == HAL_OK) {
        if (transmitting.bytes[1] == CMD_STOP && AddressValid(transmitting.bytes[0]))
            stop_start_mask |= (uint8_t)(1U << (transmitting.bytes[0] - 1U));
        tail = (uint8_t)((tail + 1U) % TX_QUEUE_SIZE);
        if (CAR_MECANUM_TEST_MODE && transmitting.bytes[1] == CMD_SPEED) {
            char text[80];
            int32_t physical = (int32_t)((uint16_t)transmitting.bytes[3] * 256U + transmitting.bytes[4]);
            if (transmitting.bytes[2] != 0U) physical = -physical;
            (void)snprintf(text, sizeof(text), "[UART5 TX_START] addr=%u physical=%ld RPM\r\n",
                           (unsigned)transmitting.bytes[0], (long)physical);
            Debug_Log(text); /* Frame submitted to UART, not motor feedback. */
        }
    }
    else { active = 0U; fault = 1U; tx_start_errors++; }
}
const Emm42Status_t *Emm42_GetLastStatus(uint8_t addr)
{
    return AddressValid(addr) ? &statuses[addr] : NULL;
}
uint8_t Emm42_HasFault(void) { return fault; }
void Emm42_ClearFault(void) { fault = 0U; }
Emm42TelemetryCapability_t Emm42_GetTelemetryCapability(void)
{
    return EMM42_TELEMETRY_UNSUPPORTED;
}
