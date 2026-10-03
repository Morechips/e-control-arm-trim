#include "uart_tx_queue.h"
#include "uart_driver.h"
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

static uint8_t queue_storage[15][8], current[8];
static uint16_t lengths[15];
static uint32_t tags[15];
static UartTxQueue_t tx;
static volatile uint8_t fault;
static UartRx_t rx;
static Emm42Status_t statuses[5];
static uint8_t reply[4], reply_used;
static volatile uint32_t stop_requests, uart5_errors, tx_timeouts, tx_start_errors;
static volatile uint8_t stop_start_mask, stop_done_mask, stop_ack_mask;
static uint32_t stop_diag_ms;
static uint8_t log_pending, log_address;
static int32_t log_physical;

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
static void RxError(UART_HandleTypeDef *uart)
{
    (void)uart; ++uart5_errors; fault = 1U;
}
static void TxEvent(UartTxQueue_t *q, UartQueueEvent_t event, const uint8_t *bytes,
                    uint16_t length, uint32_t tag, HAL_StatusTypeDef result)
{
    (void)q; (void)length; (void)result;
    if (event == UART_QUEUE_TIMED_OUT) {
        ++tx_timeouts; fault = 1U; (void)Emm42_EStopAll();
    } else if (event == UART_QUEUE_START_FAILED) { ++tx_start_errors; fault = 1U; }
    else if (event == UART_QUEUE_STARTED) {
        if (bytes[1] == CMD_STOP && AddressValid(bytes[0]))
            stop_start_mask |= (uint8_t)(1U << (bytes[0] - 1U));
        if (CAR_MECANUM_TEST_MODE && bytes[1] == CMD_SPEED) {
            log_address = bytes[0];
            log_physical = (int32_t)((uint16_t)bytes[3] * 256U + bytes[4]);
            if (bytes[2] != 0U) log_physical = -log_physical;
            log_pending = 1U;
        }
    } else if (event == UART_QUEUE_COMPLETED && bytes[1] == CMD_STOP &&
               tag == stop_requests && AddressValid(bytes[0]))
        stop_done_mask |= (uint8_t)(1U << (bytes[0] - 1U));
}
static HAL_StatusTypeDef Enqueue(const uint8_t *bytes, uint16_t length)
{
    return UART_TxQueue_Submit(&tx, bytes, length, bytes[1] == CMD_STOP ? stop_requests : 0U);
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
    const UartTxQueueConfig_t config = {
        .policy = UART_QUEUE_FIFO, .capacity = 15U, .frame_size = 8U,
        .timeout_ms = MOTOR_TX_TIMEOUT_MS, .gap_ms = MOTOR_FRAME_GAP_MS,
        .retain_failed = 1U, .notify = TxEvent
    };
    (void)UART_TxQueue_Init(&tx, &huart5, &config, &queue_storage[0][0], lengths, tags, current);
    fault = reply_used = log_pending = 0U;
    stop_requests = uart5_errors = tx_timeouts = tx_start_errors = 0U;
    stop_start_mask = stop_done_mask = stop_ack_mask = 0U;
    stop_diag_ms = HAL_GetTick();
    memset(statuses, 0, sizeof(statuses));
    (void)UART_BindRx(&rx, &huart5, NULL, RxError);
    (void)Emm42_EStopAll();
    (void)Emm42_DisableAll(); /* MCU reset must not retain an old motor target. */
}
uint8_t Emm42_IsIdle(void) { return UART_TxQueue_IsIdle(&tx); }
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
    uint16_t free_slots = UART_TxQueue_Free(&tx);
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
    UART_TxQueue_CancelPending(&tx, 0U); /* Keep the in-flight frame. */
    return StopFrame(addr);
}
HAL_StatusTypeDef Emm42_EStopAll(void)
{
    uint8_t addr;
    stop_requests++;
    stop_start_mask = stop_done_mask = stop_ack_mask = 0U;
    UART_TxQueue_CancelPending(&tx, 1U);
    /* Finish at most ONE in-flight frame to avoid corrupting the wire protocol.
     * No other normal frame is launched by an ISR. Stops take the next slots. */
    for (addr = 1U; addr <= 4U; ++addr) (void)StopFrame(addr);
    return HAL_OK;
}
void Emm42_RxCallback(UART_HandleTypeDef *uart) { UART_RxCallback(uart); }
void Emm42_ErrorCallback(UART_HandleTypeDef *uart)
{
    UART_ErrorCallback(uart);
}
void Emm42_TxCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart5) UART_TxQueue_Complete(&tx);
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
    size_t received;
    const uint8_t *pending;
    if (UART_Recover(&huart5)) { fault = 1U; reply_used = 0U; }
    while (UART_RECV(&byte, 1U, &huart5, &received, &tick) == HAL_OK && received != 0U)
        Emm42_ProcessRx(&byte, 1U);
    LogStopDiagnostics();
    pending = UART_TxQueue_Peek(&tx);
    if (fault && !UART_TxQueue_IsSending(&tx) && pending != NULL && pending[1] != CMD_STOP &&
        !(pending[1] == CMD_ENABLE && pending[3] == 0U)) (void)Emm42_EStopAll();
    (void)UART_TxQueue_Process(&tx);
    if (log_pending) {
        char text[80];
        log_pending = 0U;
        (void)snprintf(text, sizeof(text), "[UART5 TX_START] addr=%u physical=%ld RPM\r\n",
                       (unsigned)log_address, (long)log_physical);
        Debug_Log(text); /* Formatting stays outside the queue's short IRQ guard. */
    }
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
