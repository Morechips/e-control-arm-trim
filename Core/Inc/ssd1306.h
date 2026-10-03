#ifndef SSD1306_H
#define SSD1306_H

#include "stm32f4xx_hal.h"

#define SSD1306_WIDTH 128U
#define SSD1306_HEIGHT 64U
#ifndef SSD1306_INIT_TIMEOUT_MS
#define SSD1306_INIT_TIMEOUT_MS 50U
#endif
#ifndef SSD1306_I2C_ADDRESS
#define SSD1306_I2C_ADDRESS 0x3CU /* Seven-bit address; shifted only in the driver. */
#endif

HAL_StatusTypeDef SSD1306_Init(I2C_HandleTypeDef *i2c);
/* Clear and WriteString edit RAM; UpdateScreen transfers it to the OLED. */
void SSD1306_Clear(void);
/* x: pixel column, page: 0..7 (8 pixels high); 5x7 font with 1-column spacing.
 * The minimal font covers the displayed status strings; others display '?'. */
void SSD1306_WriteString(uint8_t x, uint8_t page, const char *text);
HAL_StatusTypeDef SSD1306_UpdateScreen(void);
HAL_StatusTypeDef SSD1306_UpdateScreenAsync(void);
void SSD1306_Process(void);
uint8_t SSD1306_IsBusy(void);

#endif
