#include "uart_driver.h"
#include "maxicam.h"
#include "action_fsm.h"
#include "serial_io.h"
#include "vision_config.h"
#include <stdio.h>
#include <string.h>

static UartRx_t maxicam_rx;
static MaxiCamTargetData_t target_data;
static uint32_t target_frame_sequence;
static uint8_t target_frame[sizeof(DetectData)];
static uint8_t target_frame_length;
static uint32_t target_frame_tick; /* arrival of the last byte of that frame */
static uint32_t qr_notice_count, qr_log_tick;
static uint32_t recoveries, received_frames, diagnostic_tick;
static uint8_t qr_notified, mode_attempted, mode_flush_reset_pending;
static MaxiCamMode_t requested_mode, current_mode;
static uint32_t mode_retry_tick, mode_flush_tick;
static uint8_t mode_retries, mode_deferred_logged, mode_flush_active;

static int16_t DecodeInt16LE(const uint8_t *bytes)
{
    return (int16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8));
}

static void DropPartialFrame(void)
{
    /* A UART error or a silent tail can split a 5-byte position packet. Discard
     * the partial packet and publish target loss; never reinterpret its tail as
     * QR and never let it shift the alignment of later packets. */
    target_frame_length = 0U;
    target_frame_tick = HAL_GetTick();
    target_data.target_valid = false;
    target_data.type = UINT8_MAX; /* Not an explicit UNKNOWN frame. */
    ++target_frame_sequence;
    if (target_frame_sequence == 0U) ++target_frame_sequence;
}

static void ParseTargetFrame(void)
{
    target_data.type = target_frame[0];
    target_data.offset_x = DecodeInt16LE(&target_frame[1]);
    target_data.offset_y = DecodeInt16LE(&target_frame[3]);
    target_data.target_valid = target_frame[0] < (uint8_t)DETECT_UNKNOWN;
    ++target_frame_sequence;
    ++received_frames;
    if (target_frame_sequence == 0U) ++target_frame_sequence;
    /* offset_y is decoded but is not used for vehicle alignment. */
}

void MaxiCam_Init(void)
{
    memset(&target_data, 0, sizeof(target_data));
    target_frame_sequence = 0U;
    target_frame_length = 0U;
    qr_notice_count = 0U;
    qr_notified = mode_attempted = mode_flush_reset_pending = 0U;
    received_frames = 0U;
    /* Let the very first notice log even before the first second has elapsed. */
    qr_log_tick = HAL_GetTick() - MAXICAM_QR_LOG_PERIOD_MS;
    recoveries = 0U;
    diagnostic_tick = HAL_GetTick();
    requested_mode = current_mode = MAXICAM_MODE_NONE;
    mode_retry_tick = mode_flush_tick = 0U;
    mode_retries = mode_deferred_logged = mode_flush_active = 0U;
    (void)UART_BindRx(&maxicam_rx, &huart4, NULL, NULL);
}

void MaxiCam_RxCallback(UART_HandleTypeDef *uart)
{
    UART_RxCallback(uart);
}

void MaxiCam_ErrorCallback(UART_HandleTypeDef *uart)
{
    /* Clear the overrun flag explicitly: the HAL only clears it together with
     * the RXNE read, so a wedged ORE would re-enter the error path forever. */
    UART_ErrorCallback(uart);
}

/* Bytes arriving right after a mode change belong to the previous mode's frame
 * stream. The test must be time based at the point of use: a flag that is only
 * cleared once per process call would swallow the first byte of the new mode
 * and shift every later packet. */
static uint8_t InModeFlush(uint32_t now)
{
    return (uint8_t)(mode_flush_active != 0U &&
                     (uint32_t)(now - mode_flush_tick) < MAXICAM_MODE_FLUSH_MS);
}

/* Camera mode is requested by the control layer and released by the first QR
 * notification; a failed byte is retried, never latched as an error. */
static void UpdateMode(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t gap;
    uint8_t command;
    if (InModeFlush(now)) return;
    if (requested_mode == MAXICAM_MODE_NONE || requested_mode == current_mode) return;
    if (!MaxiCam_QrNotified()) {
        if (!mode_deferred_logged) {
            mode_deferred_logged = 1U;
            Debug_Log("[MAXICAM] mode deferred: waiting QR notice\r\n");
        }
        return;
    }
    gap = (mode_retries >= MAXICAM_MODE_RETRY_MAX) ? MAXICAM_MODE_RETRY_SLOW_MS
                                                   : MAXICAM_MODE_RETRY_GAP_MS;
    if (mode_attempted && (uint32_t)(now - mode_retry_tick) < gap) return;
    mode_attempted = 1U;
    command = (requested_mode == MAXICAM_MODE_AIM) ? MODE_CMD_AIM : MODE_CMD_OBJECT;
    if (UART_SEND(&command, 1U, &huart4, UART_TX_BLOCKING, MAXICAM_MODE_TX_TIMEOUT_MS) == HAL_OK) {
        current_mode = requested_mode;
        mode_retries = 0U;
        mode_retry_tick = now;
        /* The camera switches mid-stream; its old-mode tail is not a packet. */
        mode_flush_active = 1U;
        mode_flush_tick = HAL_GetTick();
        mode_flush_reset_pending = 1U;
        Debug_Log(requested_mode == MAXICAM_MODE_AIM ? "[MAXICAM] mode TX ok: aim\r\n"
                                                     : "[MAXICAM] mode TX ok: object\r\n");
        return;
    }
    if (mode_retries < MAXICAM_MODE_RETRY_MAX) ++mode_retries;
    mode_retry_tick = now;
    Debug_Log(mode_retries >= MAXICAM_MODE_RETRY_MAX ? "[MAXICAM] mode TX failed: slow retry\r\n"
                                                           : "[MAXICAM] mode TX retry\r\n");
}

static void TraceStatus(void)
{
    char line[80];
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - diagnostic_tick) < 1000U || !Debug_CanLog(1U)) return;
    diagnostic_tick = now;
    (void)snprintf(line, sizeof(line),
                   "[MAXICAM] rx=%lu err=%lu mode=%u/%u qr=%lu\r\n",
                   (unsigned long)received_frames, (unsigned long)recoveries,
                   (unsigned)current_mode, (unsigned)requested_mode,
                   (unsigned long)qr_notice_count);
    Debug_Log(line);
}

void MaxiCam_Process(void)
{
    uint8_t rx_byte;
    size_t received;
    uint32_t rx_tick, now = HAL_GetTick();

    if (mode_flush_reset_pending) {
        DropPartialFrame();
        mode_flush_reset_pending = 0U;
    }
    if (UART_Recover(&huart4))
    {
        ++recoveries;
        DropPartialFrame();
    }
    while (UART_RECV(&rx_byte, 1U, &huart4, &received, &rx_tick) == HAL_OK && received != 0U)
    {
        if (InModeFlush(rx_tick)) continue; /* old-mode tail after a mode change */
        if (target_frame_length != 0U)
        {
            if ((uint32_t)(rx_tick - target_frame_tick) > VISION_FRAME_TIMEOUT_MS)
            {
                /* The tail never arrived: resynchronise instead of consuming
                 * this byte as part of a phantom packet. */
                DropPartialFrame();
            }
            else
            {
                /* Already inside a position packet: 0x80 is payload here. */
                target_frame[target_frame_length++] = rx_byte;
                target_frame_tick = rx_tick;
                if (target_frame_length == sizeof(target_frame))
                {
                    ParseTargetFrame();
                    target_frame_length = 0U;
                }
                continue;
            }
        }
        if (rx_byte == QR_SUCCESS_CODE)
        {
            /* QR success is a standalone one-byte packet and is recognized
             * only while waiting for the first byte of a new packet. Every
             * notice is published: the action FSM clears its own flag when it
             * starts waiting for one. */
            ++qr_notice_count;
            if (qr_notice_count == 0U) ++qr_notice_count;
            qr_notified = 1U;
            ActionFSM_SetQRSuccess(true);
            if ((uint32_t)(now - qr_log_tick) >= MAXICAM_QR_LOG_PERIOD_MS)
            {
                qr_log_tick = now;
                Debug_Log("[QR] RX: 0x80\r\n");
            }
        }
        else if (rx_byte <= (uint8_t)DETECT_UNKNOWN)
        {
            target_frame[0] = rx_byte;
            target_frame_length = 1U;
            target_frame_tick = rx_tick;
        }
    }
    /* Expire a silent partial packet even when no new byte arrives. Arrival
     * gaps, rather than foreground age, preserve complete buffered packets. */
    if (target_frame_length != 0U &&
        (uint32_t)(now - target_frame_tick) > VISION_FRAME_TIMEOUT_MS) DropPartialFrame();
    if (mode_flush_active && !InModeFlush(now)) mode_flush_active = 0U;
    UpdateMode();
    TraceStatus();
}

void MaxiCam_GetTargetData(MaxiCamTargetData_t *data, uint32_t *frame_sequence)
{
    if (data != NULL) *data = target_data;
    if (frame_sequence != NULL) *frame_sequence = target_frame_sequence;
}

void MaxiCam_RequestMode(MaxiCamMode_t mode)
{
    if (mode != MAXICAM_MODE_NONE && mode != MAXICAM_MODE_OBJECT && mode != MAXICAM_MODE_AIM)
        return;
    if (requested_mode == mode) return;
    requested_mode = mode;
    mode_deferred_logged = 0U;
    /* No wait for the retry gap: a new request is attempted on the next call. */
    mode_retry_tick = 0U;
    mode_retries = mode_attempted = 0U;
}

bool MaxiCam_ModePending(void)
{
    return (requested_mode != MAXICAM_MODE_NONE && requested_mode != current_mode);
}

MaxiCamMode_t MaxiCam_GetMode(void) { return current_mode; }

uint32_t MaxiCam_GetQrNoticeCount(void) { return qr_notice_count; }

bool MaxiCam_QrNotified(void) { return qr_notified != 0U; }

void MaxiCam_ResetQrNotice(void) { qr_notified = 0U; mode_deferred_logged = 0U; }
