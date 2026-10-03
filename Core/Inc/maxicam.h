#ifndef MAXICAM_H
#define MAXICAM_H

#include "main.h"
#include "car_config.h"
#include "detect_data.h"
#include <stdbool.h>

/* Confirmed MaxiCam QR success response: one standalone binary byte, not
 * ASCII "80". A 0x80 inside a 5-byte position packet remains payload. */
#define QR_SUCCESS_CODE QRCODE_NOTIFY_PACK

/* General alignment uses a right-shifted image target; bounds are inclusive. */
#define MAXICAM_ALIGN_TARGET_X 10
#define MAXICAM_ALIGN_DEADZONE_X 10
#define MAXICAM_ALIGN_MIN_X (MAXICAM_ALIGN_TARGET_X - MAXICAM_ALIGN_DEADZONE_X)
#define MAXICAM_ALIGN_MAX_X (MAXICAM_ALIGN_TARGET_X + MAXICAM_ALIGN_DEADZONE_X)
/* Shooting alone centers at zero while retaining the same deadzone width. */
#define MAXICAM_SHOT_TARGET_X 0
#define MAXICAM_SHOT_MIN_X (MAXICAM_SHOT_TARGET_X - MAXICAM_ALIGN_DEADZONE_X)
#define MAXICAM_SHOT_MAX_X (MAXICAM_SHOT_TARGET_X + MAXICAM_ALIGN_DEADZONE_X)

typedef struct
{
    uint8_t type;
    int16_t offset_x;
    int16_t offset_y;
    bool target_valid;
} MaxiCamTargetData_t;

/* Camera recognition mode. The controller byte is only effective after the
 * camera has sent a QR notification (detect_data.h), and the protocol has no
 * acknowledge, so requests are deferred and retried instead of failing. */
typedef enum
{
    MAXICAM_MODE_NONE = 0,  /* leave the camera in its power-up mode */
    MAXICAM_MODE_OBJECT,    /* MODE_CMD_OBJECT */
    MAXICAM_MODE_AIM        /* MODE_CMD_AIM */
} MaxiCamMode_t;

void MaxiCam_Init(void);
void MaxiCam_Process(void);
void MaxiCam_RxCallback(UART_HandleTypeDef *uart);
void MaxiCam_ErrorCallback(UART_HandleTypeDef *uart);
void MaxiCam_GetTargetData(MaxiCamTargetData_t *data, uint32_t *frame_sequence);
/* Records the wanted mode. Never blocks and never fails: the byte is sent from
 * MaxiCam_Process() once a QR notification has been seen, and retried until it
 * gets through. */
void MaxiCam_RequestMode(MaxiCamMode_t mode);
/* 1 while the requested mode has not been transmitted yet. */
bool MaxiCam_ModePending(void);
/* Last mode the camera was told about (MAXICAM_MODE_NONE before the first). */
MaxiCamMode_t MaxiCam_GetMode(void);
/* Monotonic count of independent 0x80 QR notifications since power-up. */
uint32_t MaxiCam_GetQrNoticeCount(void);
bool MaxiCam_QrNotified(void);
/* Clear only the notification latch; the diagnostic count stays monotonic. */
void MaxiCam_ResetQrNotice(void);

#endif
