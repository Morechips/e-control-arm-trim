#include "uart_driver.h"
#include "maxicam.h"
#include "vision_config.h"
#include "serial_io.h"
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart4;
volatile bool qr_success;

static unsigned set_true_count;
static unsigned log_count;
static char last_log[32];
static uint8_t transmitted[8];
static size_t transmitted_count;
static unsigned transmit_attempts;
static HAL_StatusTypeDef transmit_result = HAL_OK;
static uint32_t tick;

uint32_t HAL_GetTick(void) { return tick; }
uint8_t Debug_CanLog(uint8_t count) { (void)count; return 1U; }

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, const uint8_t *data,
                                    uint16_t size, uint32_t timeout)
{
    assert(uart == &huart4 && size == 1U && timeout == MAXICAM_MODE_TX_TIMEOUT_MS);
    ++transmit_attempts;
    if (transmit_result == HAL_OK) {
        assert(transmitted_count < sizeof(transmitted));
        transmitted[transmitted_count++] = data[0];
    }
    return transmit_result;
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
    *huart4.rx = byte;
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
    assert(huart4.rx != NULL);
    assert(!MaxiCam_QrNotified() && MaxiCam_GetQrNoticeCount() == 0U);
    Inject('8');
    Inject('0');
    Inject(0x7FU);
    Inject(0x81U);
    assert(!qr_success && set_true_count == 0U && log_count == 0U);

    *huart4.rx = QR_SUCCESS_CODE;
    MaxiCam_RxCallback(&other);
    MaxiCam_Process();
    assert(!qr_success && set_true_count == 0U);

    Inject(QR_SUCCESS_CODE);
    assert(qr_success && set_true_count == 1U && log_count == 1U);
    assert(strcmp(last_log, "[QR] RX: 0x80\r\n") == 0);
    assert(MaxiCam_QrNotified() && MaxiCam_GetQrNoticeCount() == 1U);

    /* The standalone first packet is QR; later 0x80 payload bytes belong to
     * the active 5-byte position packet and must not retrigger QR. */
    InjectTarget(DETECT_REDBALL, 0x0180, -128);
    MaxiCam_GetTargetData(&target, &sequence);
    assert(sequence == 1U && target.target_valid && target.type == DETECT_REDBALL);
    assert(target.offset_x == 0x0180 && target.offset_y == -128);
    assert(set_true_count == 1U && log_count == 1U && MaxiCam_GetQrNoticeCount() == 1U);

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

    /* Every notice is published: the old build suppressed all but the first. */
    tick += 1200U;
    Inject(QR_SUCCESS_CODE);
    assert(MaxiCam_GetQrNoticeCount() == 2U && set_true_count == 2U);

    /* A second notice while a request is already current must not resend. */
    MaxiCam_RequestMode(MAXICAM_MODE_AIM);
    MaxiCam_Process();
    assert(transmitted_count == 1U && transmitted[0] == MODE_CMD_AIM);
    assert(MaxiCam_GetMode() == MAXICAM_MODE_AIM && !MaxiCam_ModePending());
    Inject(QR_SUCCESS_CODE);
    assert(transmitted_count == 1U);

    /* Switching back sends the other byte exactly once. */
    MaxiCam_RequestMode(MAXICAM_MODE_OBJECT);
    assert(MaxiCam_ModePending());
    tick += MAXICAM_MODE_FLUSH_MS + 1U; /* skip the post-mode flush window */
    MaxiCam_Process();
    assert(transmitted_count == 2U && transmitted[1] == MODE_CMD_OBJECT);
    assert(MaxiCam_GetMode() == MAXICAM_MODE_OBJECT && !MaxiCam_ModePending());

    /* UART failure cannot fail the request: it stays pending and retries. */
    transmit_result = HAL_BUSY;
    MaxiCam_RequestMode(MAXICAM_MODE_AIM);
    tick += MAXICAM_MODE_FLUSH_MS + 1U; /* leave the previous mode's flush window */
    MaxiCam_Process();                  /* first attempt fails */
    assert(MaxiCam_ModePending() && MaxiCam_GetMode() == MAXICAM_MODE_OBJECT);
    assert(transmitted_count == 2U);
    transmit_result = HAL_OK;
    tick += MAXICAM_MODE_RETRY_GAP_MS;
    MaxiCam_Process();                  /* retry succeeds */
    assert(transmitted_count == 3U && transmitted[2] == MODE_CMD_AIM);
    assert(MaxiCam_GetMode() == MAXICAM_MODE_AIM && !MaxiCam_ModePending());

    /* An invalid mode value is ignored. */
    MaxiCam_RequestMode((MaxiCamMode_t)99);
    tick += MAXICAM_MODE_FLUSH_MS + 1U;
    MaxiCam_Process();
    assert(transmitted_count == 3U && MaxiCam_GetMode() == MAXICAM_MODE_AIM);

    /* UART failure invalidates the target and abandons a partial packet. */
    MaxiCam_GetTargetData(&target, &sequence);
    uint32_t before_error = sequence;
    Inject(DETECT_BLUEBALL);
    *huart4.rx = 0x34U;
    MaxiCam_RxCallback(&huart4);
    MaxiCam_ErrorCallback(&huart4);
    MaxiCam_Process();
    MaxiCam_GetTargetData(&target, &sequence);
    assert(sequence == before_error + 1U && !target.target_valid && target.type == UINT8_MAX);

    /* A silent partial packet must not shift every later frame: after the
     * frame timeout the next byte starts a fresh packet. */
    Inject(DETECT_REDBALL);
    Inject(DETECT_BLUEBALL);
    tick += VISION_FRAME_TIMEOUT_MS + 1U;
    InjectTarget(DETECT_TARGET, 1, -2);
    MaxiCam_GetTargetData(&target, &sequence);
    assert(target.target_valid && target.type == DETECT_TARGET &&
           target.offset_x == 1 && target.offset_y == -2);

    /* A silent tail publishes loss without requiring a new byte. */
    Inject(DETECT_REDBALL);
    MaxiCam_GetTargetData(&target, &sequence);
    before_error = sequence;
    tick += VISION_FRAME_TIMEOUT_MS + 1U;
    MaxiCam_Process();
    MaxiCam_GetTargetData(&target, &sequence);
    assert(sequence == before_error + 1U && !target.target_valid && target.type == UINT8_MAX);
    Inject(QR_SUCCESS_CODE);
    assert(MaxiCam_QrNotified());

    /* Deferred request is silent after its one diagnostic, and notifications
     * are independent of the action FSM's cleared flag. */
    tick = 0U; MaxiCam_Init(); transmitted_count = transmit_attempts = log_count = 0U;
    MaxiCam_RequestMode(MAXICAM_MODE_AIM); MaxiCam_Process();
    assert(MaxiCam_ModePending() && transmit_attempts == 0U && log_count == 1U);
    MaxiCam_RequestMode(MAXICAM_MODE_AIM); MaxiCam_Process();
    assert(transmit_attempts == 0U && log_count == 1U);
    Inject(QR_SUCCESS_CODE);
    assert(transmit_attempts == 1U && transmitted_count == 1U);
    tick = 20U;
    *huart4.rx = DETECT_BLUEBALL; MaxiCam_RxCallback(&huart4);
    /* Drain only after the 50 ms window: arrival time still rejects its tail. */
    tick = 60U; MaxiCam_Process();
    InjectTarget(DETECT_TARGET, 128, -128);
    MaxiCam_GetTargetData(&target, NULL);
    assert(target.target_valid && target.offset_x == 128 && target.offset_y == -128);
    MaxiCam_ResetQrNotice();
    assert(!MaxiCam_QrNotified() && MaxiCam_GetQrNoticeCount() == 1U);
    qr_success = false; Inject(QR_SUCCESS_CODE);
    assert(qr_success && MaxiCam_GetQrNoticeCount() == 2U && transmit_attempts == 1U);

    /* Fast attempt count is bounded even at tick zero, then slow retries
     * remain recoverable. A repeated request cannot bypass the retry gap. */
    tick = 0U; MaxiCam_Init(); transmitted_count = transmit_attempts = 0U;
    transmit_result = HAL_ERROR;
    MaxiCam_RequestMode(MAXICAM_MODE_AIM); Inject(QR_SUCCESS_CODE);
    assert(transmit_attempts == 1U);
    for (unsigned i = 1U; i < MAXICAM_MODE_RETRY_MAX; ++i) {
        tick += MAXICAM_MODE_RETRY_GAP_MS - 1U;
        MaxiCam_RequestMode(MAXICAM_MODE_AIM); MaxiCam_Process();
        assert(transmit_attempts == i);
        ++tick; MaxiCam_Process(); assert(transmit_attempts == i + 1U);
    }
    assert(MaxiCam_ModePending() && MaxiCam_GetMode() == MAXICAM_MODE_NONE);
    tick += MAXICAM_MODE_RETRY_SLOW_MS - 1U; MaxiCam_Process();
    assert(transmit_attempts == MAXICAM_MODE_RETRY_MAX);
    ++tick; MaxiCam_Process(); assert(transmit_attempts == MAXICAM_MODE_RETRY_MAX + 1U);
    transmit_result = HAL_OK; tick += MAXICAM_MODE_RETRY_SLOW_MS; MaxiCam_Process();
    assert(!MaxiCam_ModePending() && transmitted_count == 1U);

    /* Buffered valid packets survive a slow foreground call: bytes are judged
     * by arrival gaps. A gap across tick wrap still abandons an old tail. */
    MaxiCam_Init(); tick = 10U;
    const uint8_t buffered[] = {DETECT_TARGET, 0x80U, 0U, 0x80U, 0xFFU};
    for (size_t i = 0U; i < sizeof(buffered); ++i) {
        *huart4.rx = buffered[i]; MaxiCam_RxCallback(&huart4); ++tick;
    }
    tick += VISION_FRAME_TIMEOUT_MS + 1U; MaxiCam_Process();
    MaxiCam_GetTargetData(&target, NULL);
    assert(target.target_valid && target.offset_x == 128 && target.offset_y == -128);
    assert(MaxiCam_GetQrNoticeCount() == 0U);
    tick = UINT32_MAX - 100U; Inject(DETECT_REDBALL);
    tick = 101U; Inject(QR_SUCCESS_CODE);
    assert(MaxiCam_QrNotified() && MaxiCam_GetQrNoticeCount() == 1U);

    puts("PASS MaxiCam: standalone first-packet 0x80 remains distinct from position payload");
    puts("PASS MaxiCam target: signed little-endian x/y, validity, sequence and UART recovery");
    puts("PASS MaxiCam mode: deferred until the QR notice, sent once, retried on failure");
    puts("PASS MaxiCam resync: a silent partial packet is dropped after the frame timeout");
    return 0;
}
