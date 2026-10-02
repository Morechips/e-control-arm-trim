#ifndef BOARD_APP_H
#define BOARD_APP_H
#include "main.h"
void Board_Init(void);
void Board_Process(void);
uint8_t Board_PD10IsLow(void);
uint8_t Board_VisionButtonIsLow(void);
uint8_t Board_ShotButtonIsLow(void);
uint8_t Board_ServoButtonIsLow(void);
#endif
