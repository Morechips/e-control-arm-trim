#include "uart_tx_queue.h"
#include "servo.h"
#include <string.h>

static UART_HandleTypeDef *servo_uart;
static uint8_t pending_frame[SERVO_MAX_TX_LENGTH], current_frame[SERVO_MAX_TX_LENGTH];
static uint16_t pending_lengths[1];
static uint32_t pending_tags[1];
static UartTxQueue_t tx;
static const void *motion_owner;
static volatile ServoTransferStatus_t transfer;
static uint32_t next_tag, transfer_tag;

static char *BeginFrame(void)
{
    return motion_owner != NULL ? NULL : (char *)UART_TxQueue_Reserve(&tx);
}
static ServoStatus_t Result(HAL_StatusTypeDef status)
{
    return status == HAL_OK ? SERVO_OK : status == HAL_BUSY ? SERVO_BUSY : SERVO_UART_ERROR;
}
static ServoStatus_t CommitFrame(char *frame, size_t length)
{
    if (frame == NULL) return SERVO_BUSY;
    return Result(UART_TxQueue_Commit(&tx, length, 0U));
}
static ServoStatus_t servo_send(const char *data, size_t length)
{
    if (servo_uart == NULL) return SERVO_NOT_INITIALIZED;
    if (motion_owner != NULL) return SERVO_BUSY;
    if (data == NULL || length == 0U) return SERVO_INVALID_PARAM;
    if (length > SERVO_MAX_TX_LENGTH) return SERVO_BUFFER_OVERFLOW;
    return Result(UART_TxQueue_Submit(&tx, (const uint8_t *)data, length, 0U));
}

static void TransferEvent(UartTxQueue_t *queue, UartQueueEvent_t event,
                          const uint8_t *data, uint16_t length,
                          uint32_t tag, HAL_StatusTypeDef status)
{
    (void)data; (void)length;
    if (motion_owner == NULL || tag == 0U || tag != transfer_tag ||
        transfer.state != SERVO_TRANSFER_PENDING) return;
    if (event == UART_QUEUE_STARTED) {
        transfer.started = true;
        transfer.started_tick = queue->started_tick;
    } else if (event == UART_QUEUE_COMPLETED) {
        transfer.completed_tick = queue->completed_tick;
        /* IRQ completion can precede the next foreground deadline service. */
        if (!transfer.started || (tx.config.timeout_ms != 0U &&
            (uint32_t)(queue->completed_tick - transfer.started_tick) >= tx.config.timeout_ms)) {
            transfer.result = SERVO_UART_ERROR;
            transfer.state = SERVO_TRANSFER_FAILED;
        } else {
            transfer.result = SERVO_OK;
            transfer.state = SERVO_TRANSFER_COMPLETE;
        }
    } else {
        transfer.completed_tick = HAL_GetTick();
        transfer.result = Result(status);
        transfer.state = SERVO_TRANSFER_FAILED;
    }
}

static ServoStatus_t OwnerReady(const void *owner)
{
    if (servo_uart == NULL) return SERVO_NOT_INITIALIZED;
    if (owner == NULL) return SERVO_INVALID_PARAM;
    if (motion_owner != owner || transfer.state == SERVO_TRANSFER_PENDING) return SERVO_BUSY;
    return SERVO_OK;
}

static ServoStatus_t CommitOwned(char *frame, size_t length)
{
    HAL_StatusTypeDef result;
    uint32_t mask = __get_PRIMASK();
    if (frame == NULL) return SERVO_BUSY;
    __disable_irq();
    ++next_tag;
    if (next_tag == 0U) ++next_tag;
    transfer_tag = next_tag;
    transfer.started = false;
    transfer.started_tick = transfer.completed_tick = 0U;
    transfer.result = SERVO_OK;
    transfer.state = SERVO_TRANSFER_PENDING;
    result = UART_TxQueue_Commit(&tx, length, transfer_tag);
    __set_PRIMASK(mask);
    return Result(result);
}

ServoStatus_t Servo_Acquire(const void *owner)
{
    uint32_t mask;
    if (servo_uart == NULL) return SERVO_NOT_INITIALIZED;
    if (owner == NULL) return SERVO_INVALID_PARAM;
    mask = __get_PRIMASK(); __disable_irq();
    if (motion_owner == owner) { __set_PRIMASK(mask); return SERVO_OK; }
    if (motion_owner != NULL || !Servo_IsIdle()) { __set_PRIMASK(mask); return SERVO_BUSY; }
    motion_owner = owner;
    transfer_tag = 0U;
    transfer.state = SERVO_TRANSFER_NONE;
    transfer.result = SERVO_OK;
    transfer.started = false;
    transfer.started_tick = transfer.completed_tick = 0U;
    tx.config.timeout_ms = SERVO_TX_TIMEOUT_MS;
    tx.config.timeout_inclusive = tx.config.abort_failure_keeps_active = 1U;
    __set_PRIMASK(mask);
    return SERVO_OK;
}

ServoStatus_t Servo_Release(const void *owner)
{
    ServoStatus_t result = OwnerReady(owner);
    uint32_t mask;
    if (result != SERVO_OK) return result;
    mask = __get_PRIMASK(); __disable_irq();
    if (!Servo_IsIdle()) { __set_PRIMASK(mask); return SERVO_BUSY; }
    motion_owner = NULL; transfer_tag = 0U;
    transfer.state = SERVO_TRANSFER_NONE;
    tx.config.timeout_ms = 0U;
    tx.config.timeout_inclusive = tx.config.abort_failure_keeps_active = 0U;
    __set_PRIMASK(mask);
    return SERVO_OK;
}

ServoStatus_t Servo_CancelPendingOwned(const void *owner)
{
    uint32_t mask;
    if (servo_uart == NULL) return SERVO_NOT_INITIALIZED;
    if (owner == NULL) return SERVO_INVALID_PARAM;
    if (motion_owner != owner) return SERVO_BUSY;
    mask = __get_PRIMASK(); __disable_irq();
    UART_TxQueue_CancelPending(&tx, 1U);
    transfer_tag = 0U;
    transfer.state = SERVO_TRANSFER_NONE;
    transfer.result = SERVO_OK;
    transfer.started = false;
    transfer.started_tick = transfer.completed_tick = 0U;
    __set_PRIMASK(mask);
    return SERVO_OK;
}

ServoTransferStatus_t Servo_GetTransferStatus(const void *owner)
{
    ServoTransferStatus_t result;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (owner != NULL && motion_owner == owner) result = transfer;
    else {
        result = (ServoTransferStatus_t){SERVO_TRANSFER_FAILED,
            servo_uart == NULL ? SERVO_NOT_INITIALIZED : SERVO_BUSY, false, 0U, 0U};
    }
    __set_PRIMASK(mask);
    return result;
}

void Servo_Process(void)
{
    HAL_StatusTypeDef result;
    uint32_t mask;
    if (servo_uart == NULL || motion_owner == NULL) return;
    result = UART_TxQueue_Process(&tx);
    /* An abort refusal leaves the active buffer protected. Surface failure
     * without treating a partially transmitted frame as safe to repeat. */
    if (result != HAL_OK) {
        mask = __get_PRIMASK(); __disable_irq();
        if (transfer.state == SERVO_TRANSFER_PENDING) {
            UART_TxQueue_CancelPending(&tx, 1U);
            transfer.result = Result(result);
            transfer.completed_tick = HAL_GetTick();
            transfer.state = SERVO_TRANSFER_FAILED;
        }
        __set_PRIMASK(mask);
    }
}
void Servo_TxCallback(UART_HandleTypeDef *uart)
{
    if (uart != NULL && uart == servo_uart) UART_TxQueue_Complete(&tx);
}
uint8_t Servo_IsIdle(void)
{
    return (uint8_t)(servo_uart != NULL && UART_TxQueue_IsIdle(&tx) &&
                     servo_uart->gState == HAL_UART_STATE_READY);
}

static int servo_servo_valid(uint16_t id, uint16_t pwm, uint16_t time_ms)
{
    return id <= 254U && pwm >= 500U && pwm <= 2500U && time_ms <= 9999U;
}

/* Minimum decimal width, matching %03u/%04u and unpadded %u. */
static size_t servo_append_u(char *dst, size_t cap, size_t used, unsigned value, unsigned digits)
{
    char reversed[10];
    unsigned count = 0U;
    do { reversed[count++] = (char)('0' + value % 10U); value /= 10U; } while (value != 0U);
    while (count < digits) reversed[count++] = '0';
    if (used > cap || count > cap - used) return cap + 1U;
    while (count != 0U) dst[used++] = reversed[--count];
    return used;
}

static size_t AppendServo(char *frame, size_t used, uint16_t id, uint16_t pwm, uint16_t time_ms)
{
    frame[used++] = '#';
    used = servo_append_u(frame, SERVO_MAX_TX_LENGTH, used, id, 3U);
    frame[used++] = 'P';
    used = servo_append_u(frame, SERVO_MAX_TX_LENGTH, used, pwm, 4U);
    frame[used++] = 'T';
    used = servo_append_u(frame, SERVO_MAX_TX_LENGTH, used, time_ms, 4U);
    frame[used++] = '!';
    return used;
}

ServoStatus_t Servo_Init(UART_HandleTypeDef *huart)
{
    const UartTxQueueConfig_t config = {
        .policy = UART_QUEUE_LATEST, .capacity = 1U, .frame_size = SERVO_MAX_TX_LENGTH,
        .immediate = 1U, .continue_in_irq = 1U, .notify = TransferEvent
    };
    if (motion_owner != NULL) return SERVO_BUSY;
    if (servo_uart != NULL && !UART_TxQueue_IsIdle(&tx)) return SERVO_BUSY;
    if (servo_uart != NULL) UART_UnbindTx(servo_uart, &tx);
    servo_uart = NULL;
    if (huart == NULL || huart->Instance == NULL) return SERVO_INVALID_PARAM;
    if (huart->Init.BaudRate != SERVO_BAUD_RATE ||
        huart->Init.WordLength != UART_WORDLENGTH_8B ||
        huart->Init.StopBits != UART_STOPBITS_1 ||
        huart->Init.Parity != UART_PARITY_NONE ||
        huart->Init.HwFlowCtl != UART_HWCONTROL_NONE ||
        (huart->Init.Mode & UART_MODE_TX) == 0U) return SERVO_INVALID_PARAM;
    if (huart->gState != HAL_UART_STATE_READY) return SERVO_UART_ERROR;
    if (UART_TxQueue_Init(&tx, huart, &config, pending_frame, pending_lengths, pending_tags,
                          current_frame) != HAL_OK) return SERVO_BUSY;
    servo_uart = huart;
    return SERVO_OK;
}

ServoStatus_t Servo_SetChannel(uint16_t id, uint16_t pwm, uint16_t time_ms)
{
    char frame[SERVO_COMMAND_LENGTH];
    if (!servo_servo_valid(id, pwm, time_ms)) return SERVO_INVALID_PARAM;
    return servo_send(frame, AppendServo(frame, 0U, id, pwm, time_ms));
}

ServoStatus_t Servo_SetValues(const uint16_t *pwm, size_t count, uint16_t time_ms)
{
    char *frame;
    size_t i, used = 0U;
    if (servo_uart == NULL) return SERVO_NOT_INITIALIZED;
    if (pwm == NULL || count == 0U) return SERVO_INVALID_PARAM;
    if (count > SERVO_MAX_POSE_COMMANDS) return SERVO_BUFFER_OVERFLOW;
    for (i = 0U; i < count; ++i)
        if (!servo_servo_valid((uint16_t)i, pwm[i], time_ms)) return SERVO_INVALID_PARAM;
    frame = BeginFrame();
    if (frame == NULL) return SERVO_BUSY;
    frame[used++] = '{';
    for (i = 0U; i < count; ++i) used = AppendServo(frame, used, (uint16_t)i, pwm[i], time_ms);
    frame[used++] = '}';
    return CommitFrame(frame, used);
}

ServoStatus_t Servo_SetCommands(const ServoCommand_t *commands, size_t count)
{
    char *frame;
    size_t i, used = 0U;
    if (servo_uart == NULL) return SERVO_NOT_INITIALIZED;
    if (commands == NULL || count == 0U) return SERVO_INVALID_PARAM;
    if (count > SERVO_MAX_COMMANDS) return SERVO_BUFFER_OVERFLOW;
    for (i = 0U; i < count; ++i)
        if (!servo_servo_valid(commands[i].id, commands[i].pwm, commands[i].time_ms))
            return SERVO_INVALID_PARAM;
    frame = BeginFrame();
    if (frame == NULL) return SERVO_BUSY;
    frame[used++] = '{';
    for (i = 0U; i < count; ++i)
        used = AppendServo(frame, used, commands[i].id, commands[i].pwm, commands[i].time_ms);
    frame[used++] = '}';
    return CommitFrame(frame, used);
}

ServoStatus_t Servo_SetValuesOwned(const void *owner, const uint16_t *pwm,
                                  size_t count, uint16_t time_ms)
{
    ServoStatus_t result = OwnerReady(owner);
    char *frame;
    size_t i, used = 0U;
    if (result != SERVO_OK) return result;
    if (!Servo_IsIdle()) return SERVO_BUSY;
    if (pwm == NULL || count == 0U) return SERVO_INVALID_PARAM;
    if (count > SERVO_MAX_POSE_COMMANDS) return SERVO_BUFFER_OVERFLOW;
    for (i = 0U; i < count; ++i)
        if (!servo_servo_valid((uint16_t)i, pwm[i], time_ms)) return SERVO_INVALID_PARAM;
    frame = (char *)UART_TxQueue_Reserve(&tx);
    if (frame == NULL) return SERVO_BUSY;
    frame[used++] = '{';
    for (i = 0U; i < count; ++i) used = AppendServo(frame, used, (uint16_t)i, pwm[i], time_ms);
    frame[used++] = '}';
    return CommitOwned(frame, used);
}

ServoStatus_t Servo_SetCommandsOwned(const void *owner, const ServoCommand_t *commands,
                                    size_t count)
{
    ServoStatus_t result = OwnerReady(owner);
    char *frame;
    size_t i, used = 0U;
    if (result != SERVO_OK) return result;
    if (!Servo_IsIdle()) return SERVO_BUSY;
    if (commands == NULL || count == 0U) return SERVO_INVALID_PARAM;
    if (count > SERVO_MAX_COMMANDS) return SERVO_BUFFER_OVERFLOW;
    for (i = 0U; i < count; ++i)
        if (!servo_servo_valid(commands[i].id, commands[i].pwm, commands[i].time_ms))
            return SERVO_INVALID_PARAM;
    frame = (char *)UART_TxQueue_Reserve(&tx);
    if (frame == NULL) return SERVO_BUSY;
    frame[used++] = '{';
    for (i = 0U; i < count; ++i)
        used = AppendServo(frame, used, commands[i].id, commands[i].pwm, commands[i].time_ms);
    frame[used++] = '}';
    return CommitOwned(frame, used);
}

ServoStatus_t Servo_StopServoOwned(const void *owner, uint16_t id)
{
    ServoStatus_t result = OwnerReady(owner);
    char *frame;
    size_t used;
    if (result != SERVO_OK) return result;
    if (id > 254U) return SERVO_INVALID_PARAM;
    frame = (char *)UART_TxQueue_Reserve(&tx);
    if (frame == NULL) return SERVO_BUSY;
    memcpy(frame, "$DST:", 5U);
    used = servo_append_u(frame, SERVO_MAX_TX_LENGTH, 5U, id, 0U);
    frame[used++] = '!';
    return CommitOwned(frame, used);
}

ServoStatus_t Servo_FormatCommands(const ServoCommand_t *commands, size_t count,
                                char *buffer, size_t capacity, size_t *length)
{
    size_t used = 0U, i;
    if (length == NULL) return SERVO_INVALID_PARAM;
    *length = 0U;
    if (buffer != NULL && capacity != 0U) buffer[0] = '\0';
    if (commands == NULL || buffer == NULL || count == 0U) return SERVO_INVALID_PARAM;
    if (count > SERVO_MAX_COMMANDS || capacity < 3U + count * 15U)
        return SERVO_BUFFER_OVERFLOW;
    for (i = 0U; i < count; ++i)
        if (!servo_servo_valid(commands[i].id, commands[i].pwm, commands[i].time_ms))
            return SERVO_INVALID_PARAM;
    buffer[used++] = '{';
    for (i = 0U; i < count; ++i)
        used = AppendServo(buffer, used, commands[i].id, commands[i].pwm, commands[i].time_ms);
    buffer[used++] = '}'; buffer[used] = '\0'; *length = used;
    return SERVO_OK;
}

/* Extension commands use a small local builder, also without libc formatting. */
static ServoStatus_t ActionCommand(const char *prefix, uint16_t a, const char *separator,
                                  uint16_t b, const char *separator2, uint16_t c)
{
    char frame[32];
    size_t used = strlen(prefix);
    memcpy(frame, prefix, used);
    used = servo_append_u(frame, sizeof(frame), used, a, 0U);
    if (separator != NULL) {
        size_t n = strlen(separator); memcpy(frame + used, separator, n); used += n;
        used = servo_append_u(frame, sizeof(frame), used, b, 0U);
    }
    if (separator2 != NULL) {
        size_t n = strlen(separator2); memcpy(frame + used, separator2, n); used += n;
        used = servo_append_u(frame, sizeof(frame), used, c, 0U);
    }
    frame[used++] = '!';
    return servo_send(frame, used);
}

ServoStatus_t Servo_RunAction(uint16_t action) { return ActionCommand("$DGS:", action, NULL, 0U, NULL, 0U); }
ServoStatus_t Servo_RunActionRange(uint16_t start, uint16_t end, uint16_t repeat)
{
    if (start > end) return SERVO_INVALID_PARAM;
    return ActionCommand("$DGT:", start, "-", end, ",", repeat);
}
ServoStatus_t Servo_StopAll(void) { return servo_send("$DST!", 5U); }
ServoStatus_t Servo_StopServo(uint16_t id)
{
    if (id > 254U) return SERVO_INVALID_PARAM;
    return ActionCommand("$DST:", id, NULL, 0U, NULL, 0U);
}
ServoStatus_t Servo_SetBias(uint16_t id, int16_t bias)
{
    char frame[13];
    size_t used = 0U;
    if (id > 254U || bias < -500 || bias > 500) return SERVO_INVALID_PARAM;
    frame[used++] = '#';
    used = servo_append_u(frame, sizeof(frame), used, id, 3U);
    memcpy(frame + used, "PSCK", 4U); used += 4U;
    frame[used++] = bias < 0 ? '-' : '+';
    used = servo_append_u(frame, sizeof(frame), used, (unsigned)(bias < 0 ? -bias : bias), 3U);
    frame[used++] = '!';
    return servo_send(frame, used);
}
ServoStatus_t Servo_RunCombinedAction(uint16_t group, uint16_t repeat) { return ActionCommand("$DKT:", group, ",", repeat, NULL, 0U); }
ServoStatus_t Servo_RecordPose(void) { return servo_send("$DJ_RECORD!", 11U); }
ServoStatus_t Servo_RunRecorded(uint16_t repeat) { return ActionCommand("$DJ_RECORD_DO:", repeat, NULL, 0U, NULL, 0U); }
ServoStatus_t Servo_ClearRecorded(void) { return servo_send("$DJ_RECORD_CLEAR!", 17U); }
ServoStatus_t Servo_SetRecordPeriod(uint16_t period_ms) { return ActionCommand("$DJ_RECORD_TIME:", period_ms, NULL, 0U, NULL, 0U); }
ServoStatus_t Servo_Reset(void) { return servo_send("$RST!", 5U); }
ServoStatus_t Servo_SendRaw(const char *command)
{
    size_t length = 0U;
    if (servo_uart == NULL) return SERVO_NOT_INITIALIZED;
    if (command == NULL || command[0] == '\0') return SERVO_INVALID_PARAM;
    while (length <= SERVO_MAX_TX_LENGTH && command[length] != '\0') ++length;
    return servo_send(command, length);
}

/* Single preset source: one row per action; unused channels are not sent. */
const ServoCommand_t codes[ServoCode_MAX][SERVO_PER_CODE_COUNT] = {
    [TakeBall_Before] = {{0U, 1717U, 2000U}, {1U, 2297U, 2000U}, {2U, 884U, 2000U}, {3U, 500U, 2000U}}, /* 放球前 TB_B */
    [TakeBall_Mid] = {{0U, 1356U, 2000U}, {1U, 1850U, 2000U}, {2U, 698U, 2000U}, {3U, 1480U, 2000U}}, /* 抓球中 TB_M */
    [TakeBall_Gap] = {{0U, 1356U, 2000U}, {1U, 1850U, 2000U}, {2U, 698U, 2000U}, {3U, 500U, 2000U}}, /* 夹取球 TB_G */
    [BarrelDown_Up] = {{0U, 1566U, 2000U}, {1U, 1896U, 2000U}, {2U, 673U, 2000U}, {3U, 500U, 2000U}}, /* 防爆桶上方 BD_U */
    [BarrelDown_Down] = {{0U, 1566U, 2000U}, {1U, 1896U, 2000U}, {2U, 673U, 2000U}, {3U, 1200U, 2000U}}, /* 放球 BD_D */
    [TakeHostage_Up] = {{0U, 1800U, 2000U}, {1U, 1855U, 2000U}, {2U, 528U, 2000U}, {3U, 500U, 2000U}}, /* 抓起人质 TH_U */
    [TakeHostage_Catch] = {{0U, 1667U, 2000U}, {1U, 1882U, 2000U}, {2U, 651U, 2000U}, {3U, 1483U, 2000U}}, /* 抓人质前 TH_C */
    [TakeHostage_Gap] = {{0U, 1667U, 2000U}, {1U, 1882U, 2000U}, {2U, 651U, 2000U}, {3U, 500U, 2000U}}, /* 夹住人质 TH_G */
    [TakeHostage_Leave] = {{0U, 1673U, 2000U}, {1U, 1394U, 2000U}, {2U, 732U, 2000U}, {3U, 500U, 2000U}}, /* 放低人质 TH_L */
    [Servo_REFERENCE] = {{0U, 1532U, 2000U}, {1U, 2219U, 2000U}, {2U, 1202U, 2000U}, {3U, 0U, 2000U}}, /* 参考姿态 REFERENCE */
    [Servo_RST] = {{0U, 1524U, 2000U}, {1U, 1163U, 2000U}, {2U, 1693U, 2000U}, {3U, 1486U, 2000U}}, /* 机械臂复位 RST */
    [Servo_AIM] = {{0U, 1058U, 2000U}, {1U, 821U, 2000U}, {2U, 554U, 2000U}, {3U, 1499U, 2000U}}, /* 瞄准 AIM */
    [TakeHostage_PreGrab] = {{0U, 1684U, 2000U}, {1U, 2136U, 2000U}, {2U, 785U, 2000U}, {3U, 1483U, 2000U}}, /* 人质预抓取 TH_PRE */
};
const uint8_t servo_code_counts[ServoCode_MAX] = {
    [TakeBall_Before] = 4U,
    [TakeBall_Mid] = 4U,
    [TakeBall_Gap] = 4U,
    [BarrelDown_Up] = 4U,
    [BarrelDown_Down] = 4U,
    [TakeHostage_Up] = 4U,
    [TakeHostage_Catch] = 4U,
    [TakeHostage_Gap] = 4U,
    [TakeHostage_Leave] = 4U,
    [Servo_REFERENCE] = 3U,
    [Servo_RST] = 4U,
    [Servo_AIM] = 4U,
    [TakeHostage_PreGrab] = 4U,
};
static const char *const names[ServoCode_MAX] = {
    [TakeBall_Before] = "放球前 TB_B",
    [TakeBall_Mid] = "抓球中 TB_M",
    [TakeBall_Gap] = "夹取球 TB_G",
    [BarrelDown_Up] = "防爆桶上方 BD_U",
    [BarrelDown_Down] = "放球 BD_D",
    [TakeHostage_Up] = "抓起人质 TH_U",
    [TakeHostage_Catch] = "抓人质前 TH_C",
    [TakeHostage_Gap] = "夹住人质 TH_G",
    [TakeHostage_Leave] = "放低人质 TH_L",
    [Servo_REFERENCE] = "参考姿态 REFERENCE",
    [Servo_RST] = "机械臂复位 RST",
    [Servo_AIM] = "瞄准 AIM",
    [TakeHostage_PreGrab] = "人质预抓取 TH_PRE",
};
const uint16_t g_servo_gap_time_ms = 2000U;
const uint16_t g_servo_legacy_reset_guard_ms = 2000U;
const uint16_t g_servo_legacy_aim_guard_ms = 2000U;
const char *Servo_GetName(ServoCode code)
{
    return (unsigned)code < (unsigned)ServoCode_MAX ? names[code] : "NONE";
}
bool GetFullCommand(ServoCode code, char *buffer, size_t capacity, size_t *length)
{
    if ((unsigned)code >= (unsigned)ServoCode_MAX) {
        if (length != NULL) *length = 0U;
        if (buffer != NULL && capacity != 0U) buffer[0] = '\0';
        return false;
    }
    return Servo_FormatCommands(codes[code], servo_code_counts[code], buffer, capacity, length) == SERVO_OK;
}
ServoStatus_t Servo_SendPreset(ServoCode code)
{
    if ((unsigned)code >= (unsigned)ServoCode_MAX) return SERVO_INVALID_PARAM;
    return Servo_SetCommands(codes[code], servo_code_counts[code]);
}
ServoStatus_t Servo_SendGap(uint16_t pwm)
{
    const ServoCommand_t command = {3U, pwm, g_servo_gap_time_ms};
    return Servo_SetCommands(&command, 1U);
}
