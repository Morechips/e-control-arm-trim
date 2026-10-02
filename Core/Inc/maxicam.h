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

void MaxiCam_Init(void);
void MaxiCam_Process(void);
void MaxiCam_RxCallback(UART_HandleTypeDef *uart);
void MaxiCam_ErrorCallback(UART_HandleTypeDef *uart);
void MaxiCam_GetTargetData(MaxiCamTargetData_t *data, uint32_t *frame_sequence);
/* Sends exactly one protocol byte on UART4; HAL_OK does not acknowledge mode. */
HAL_StatusTypeDef MaxiCam_SendMode(uint8_t command);

#endif
