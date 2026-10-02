#include "maxicam.h"
#include "vision_config.h"
#include "serial_io.h"
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart4;
volatile bool qr_success;

static SerialRx *receiver;
static unsigned set_true_count;
static unsigned log_count;
static char last_log[32];
static uint8_t transmitted[4];
static size_t transmitted_count;
static HAL_StatusTypeDef transmit_result = HAL_OK;

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, const uint8_t *data,
                                    uint16_t size, uint32_t timeout)
{
    assert(uart == &huart4 && size == 1U && timeout == MAXICAM_MODE_TX_TIMEOUT_MS);
    if (transmit_result == HAL_OK) transmitted[transmitted_count++] = data[0];
    return transmit_result;
}

void Serial_Init(SerialRx *rx, UART_HandleTypeDef *uart)
{
    memset(rx, 0, sizeof(*rx));
    rx->uart = uart;
    receiver = rx;
}

void Serial_RxCallback(SerialRx *rx, UART_HandleTypeDef *uart)
{
    uint16_t next;
    if (rx->uart != uart) return;
    next = (uint16_t)((rx->head + 1U) % SERIAL_RX_SIZE);
    assert(next != rx->tail);
    rx->data[rx->head] = rx->byte;
    rx->head = next;
}

void Serial_ErrorCallback(SerialRx *rx, UART_HandleTypeDef *uart)
{
    if (rx->uart == uart) rx->broken = 1U;
}

uint8_t Serial_Recover(SerialRx *rx)
{
    uint8_t broken = rx->broken;
    rx->broken = 0U;
    return broken;
}

uint8_t Serial_Pop(SerialRx *rx, uint8_t *byte, uint32_t *tick)
{
    if (rx->tail == rx->head) return 0U;
    *byte = rx->data[rx->tail];
    *tick = 0U;
    rx->tail = (uint16_t)((rx->tail + 1U) % SERIAL_RX_SIZE);
    return 1U;
}

void Debug_Log(const char *text)
{
    ++log_count;
    (void)snprintf(last_log, sizeof(last_log), "%s", text);
}

void ActionFSM_SetQRSuccess(bool success)
{
    qr_success = success;
    if (success) ++set_true_count;
}

static void Inject(uint8_t byte)
{
    receiver->byte = byte;
    MaxiCam_RxCallback(&huart4);
    MaxiCam_Process();
}

static void InjectTarget(uint8_t type, int16_t offset_x, int16_t offset_y)
{
    const uint8_t bytes[] = {
        type,
        (uint8_t)((uint16_t)offset_x & 0xFFU),
        (uint8_t)((uint16_t)offset_x >> 8),
        (uint8_t)((uint16_t)offset_y & 0xFFU),
        (uint8_t)((uint16_t)offset_y >> 8)
    };
    size_t i;
    for (i = 0U; i < sizeof(bytes); ++i) Inject(bytes[i]);
}

int main(void)
{
    UART_HandleTypeDef other;
    MaxiCamTargetData_t target;
    uint32_t sequence;

    MaxiCam_Init();
    assert(receiver != NULL && receiver->uart == &huart4);
    Inject('8');
    Inject('0');
    Inject(0x7FU);
    Inject(0x81U);
    assert(!qr_success && set_true_count == 0U && log_count == 0U);

    receiver->byte = QR_SUCCESS_CODE;
    MaxiCam_RxCallback(&other);
    MaxiCam_Process();
    assert(!qr_success && set_true_count == 0U);

    Inject(QR_SUCCESS_CODE);
    assert(qr_success && set_true_count == 1U && log_count == 1U);
    assert(strcmp(last_log, "[QR] RX: 0x80\r\n") == 0);

    /* The standalone first packet is QR; later 0x80 payload bytes belong to
     * the active 5-byte position packet and must not retrigger QR. */
    InjectTarget(DETECT_REDBALL, 0x0180, -128);
    MaxiCam_GetTargetData(&target, &sequence);
    assert(sequence == 1U && target.target_valid && target.type == DETECT_REDBALL);
    assert(target.offset_x == 0x0180 && target.offset_y == -128);
    assert(set_true_count == 1U && log_count == 1U);

    InjectTarget(DETECT_HOSTAGE_3, -10, 321);
    MaxiCam_GetTargetData(&target, &sequence);
    assert(sequence == 2U && target.target_valid && target.type == DETECT_HOSTAGE_3);
    assert(target.offset_x == -10 && target.offset_y == 321);

    InjectTarget(DETECT_TARGET, -5, 6);
    MaxiCam_GetTargetData(&target, &sequence);
    assert(sequence == 3U && target.target_valid && target.type == DETECT_TARGET);
    assert(target.offset_x == -5 && target.offset_y == 6);

    InjectTarget(DETECT_UNKNOWN, 77, -66);
    MaxiCam_GetTargetData(&target, &sequence);
    assert(sequence == 4U && !target.target_valid && target.type == DETECT_UNKNOWN);
    assert(target.offset_x == 77 && target.offset_y == -66);

    /* UART failure invalidates the target and abandons a partial packet. */
    Inject(DETECT_BLUEBALL);
    receiver->byte = 0x34U;
    MaxiCam_RxCallback(&huart4);
    MaxiCam_ErrorCallback(&huart4);
    MaxiCam_Process();
    MaxiCam_GetTargetData(&target, &sequence);
    assert(sequence == 5U && !target.target_valid && target.type == UINT8_MAX);

    Inject(QR_SUCCESS_CODE);
    assert(set_true_count == 1U && log_count == 1U);

    InjectTarget(DETECT_REDBALL, 12, 0);
    Inject(DETECT_BLUEBALL); /* Incomplete packet from the old mode. */
    Inject(0x42U);
    assert(MaxiCam_SendMode(MODE_CMD_AIM) == HAL_OK);
    MaxiCam_GetTargetData(&target, &sequence);
    assert(!target.target_valid && target.type == UINT8_MAX);
    InjectTarget(DETECT_TARGET, 1, -2);
    MaxiCam_GetTargetData(&target, &sequence);
    assert(target.target_valid && target.type == DETECT_TARGET &&
           target.offset_x == 1 && target.offset_y == -2);
    assert(MaxiCam_SendMode(MODE_CMD_OBJECT) == HAL_OK);
    assert(transmitted_count == 2U && transmitted[0] == 0x00U &&
           transmitted[1] == 0x01U);
    assert(MaxiCam_SendMode(0x80U) == HAL_ERROR && transmitted_count == 2U);
    transmit_result = HAL_BUSY;
    assert(MaxiCam_SendMode(MODE_CMD_AIM) == HAL_BUSY && transmitted_count == 2U);

    puts("PASS MaxiCam: standalone first-packet 0x80 remains distinct from position payload");
    puts("PASS MaxiCam target: signed little-endian x/y, validity, sequence and UART recovery");
    return 0;
}
