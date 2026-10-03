#ifndef BOARD_APP_H
#define BOARD_APP_H
#include "main.h"
#include "car_config.h"
/* Hardware bring-up without any blocking delay; safe to call before the
 * Bluetooth/JY61/MaxiCam receivers are armed. */
void Board_Init(void);
/* Cosmetic OLED bring-up, deliberately separated so that a slow or absent
 * display can never delay the control link. Returns HAL_OK when the panel
 * answered. */
HAL_StatusTypeDef Board_InitDisplay(void);
void Board_Process(void);
#endif
