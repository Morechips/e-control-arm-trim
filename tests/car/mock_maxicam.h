#ifndef TEST_CAR_MOCK_MAXICAM_H
#define TEST_CAR_MOCK_MAXICAM_H

#include "maxicam.h"

void TestMaxiCam_Reset(void);
void TestMaxiCam_Publish(uint8_t type, int16_t offset_x, int16_t offset_y,
                         uint8_t valid);
void TestMaxiCam_SetSendResult(HAL_StatusTypeDef result);
uint32_t TestMaxiCam_ModeCount(void);
uint32_t TestMaxiCam_ModeAttempts(void);
uint8_t TestMaxiCam_LastMode(void);

#endif
