#ifndef TEST_CAR_MOCK_MAXICAM_H
#define TEST_CAR_MOCK_MAXICAM_H

#include "maxicam.h"

/* Host stand-in for Core/Src/maxicam.c. Target frames are published directly;
 * the camera mode byte follows the real contract: MaxiCam_RequestMode() only
 * records the wish, the byte is released by the first QR notice (a standalone
 * 0x80), and a failed byte stays pending and is retried instead of latching an
 * error. */
void TestMaxiCam_Reset(void);
void TestMaxiCam_Publish(uint8_t type, int16_t offset_x, int16_t offset_y,
                         uint8_t valid);
/* Emulates the camera sending its standalone 0x80 notice, then runs the mode
 * step so a pending request is transmitted exactly like the real module does. */
void TestMaxiCam_PublishQrNotice(void);
/* Emulates the mode part of MaxiCam_Process(): defers without a notice,
 * transmits once released, and retries while the transmit keeps failing. */
void TestMaxiCam_ModeStep(void);
/* Result of every transmit attempt until it is changed again. */
void TestMaxiCam_SetSendResult(HAL_StatusTypeDef result);

/* Last byte actually transmitted, UINT8_MAX before the first one. */
uint8_t TestMaxiCam_LastMode(void);
/* Successful transmissions; failures are counted by TestMaxiCam_ModeAttempts. */
uint32_t TestMaxiCam_ModeCount(void);
uint32_t TestMaxiCam_ModeAttempts(void);
MaxiCamMode_t TestMaxiCam_RequestedMode(void);
/* Accepted MaxiCam_RequestMode() calls. */
uint32_t TestMaxiCam_RequestCount(void);
bool TestMaxiCam_ModePending(void);

#endif
