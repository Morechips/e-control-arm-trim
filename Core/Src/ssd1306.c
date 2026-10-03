#include "ssd1306.h"
#include <stddef.h>
#include <string.h>

#define SSD1306_HAL_ADDRESS ((uint16_t)(SSD1306_I2C_ADDRESS << 1U))
#define SSD1306_TIMEOUT_MS 250U
#define SSD1306_BUFFER_SIZE (SSD1306_WIDTH * SSD1306_HEIGHT / 8U)

static I2C_HandleTypeDef *oled_i2c;
/* Byte zero is the I2C data control byte, followed by 8 pages of pixels. */
static uint8_t frame[1U + SSD1306_BUFFER_SIZE];
static uint8_t async_frame[1U + SSD1306_BUFFER_SIZE];
static uint8_t async_phase;
static uint32_t async_tick;

uint8_t SSD1306_IsBusy(void) { return async_phase; }
HAL_StatusTypeDef SSD1306_UpdateScreenAsync(void)
{
    static uint8_t window[] = {0x00U, 0x21U, 0U, 127U, 0x22U, 0U, 7U};
    HAL_StatusTypeDef status;
    if (oled_i2c == NULL) return HAL_ERROR;
    if (async_phase) return HAL_BUSY;
    /* HAL's IT entry still polls BUSY; avoid that poll on a stuck bus. */
    if (__HAL_I2C_GET_FLAG(oled_i2c, I2C_FLAG_BUSY) != RESET) return HAL_BUSY;
    memcpy(async_frame, frame, sizeof(frame));
    status = HAL_I2C_Master_Transmit_IT(oled_i2c, SSD1306_HAL_ADDRESS, window, sizeof(window));
    if (status == HAL_OK) { async_phase = 1U; async_tick = HAL_GetTick(); }
    return status;
}
void SSD1306_Process(void)
{
    if (!async_phase || oled_i2c == NULL) return;
    if ((uint32_t)(HAL_GetTick() - async_tick) > SSD1306_TIMEOUT_MS) {
        /* A missing display must not block vehicle control. Reset the I2C
         * peripheral in foreground; GPIO configuration remains unchanged. */
        (void)HAL_I2C_DeInit(oled_i2c);
        (void)HAL_I2C_Init(oled_i2c);
        async_phase = 0U;
        return;
    }
    if (HAL_I2C_GetState(oled_i2c) != HAL_I2C_STATE_READY) return;
    if (HAL_I2C_GetError(oled_i2c) != HAL_I2C_ERROR_NONE || async_phase == 2U) {
        async_phase = 0U;
        return;
    }
    if (__HAL_I2C_GET_FLAG(oled_i2c, I2C_FLAG_BUSY) != RESET) return;
    if (HAL_I2C_Master_Transmit_IT(oled_i2c, SSD1306_HAL_ADDRESS, async_frame, sizeof(async_frame)) == HAL_OK)
        async_phase = 2U;
    else async_phase = 0U;
}

typedef struct
{
    char character;
    uint8_t columns[5];
} Glyph;

/* Bit zero is the top pixel. Only the test text's glyphs are needed. */
static const Glyph font[] = {
    {' ', {0x00, 0x00, 0x00, 0x00, 0x00}},
    {'0', {0x3E, 0x51, 0x49, 0x45, 0x3E}},
    {'2', {0x42, 0x61, 0x51, 0x49, 0x46}},
    {'3', {0x21, 0x41, 0x45, 0x4B, 0x31}},
    {'4', {0x18, 0x14, 0x12, 0x7F, 0x10}},
    {'6', {0x3C, 0x4A, 0x49, 0x49, 0x30}},
    {'7', {0x01, 0x71, 0x09, 0x05, 0x03}},
    {':', {0x00, 0x36, 0x36, 0x00, 0x00}},
    {'?', {0x02, 0x01, 0x51, 0x09, 0x06}},
    {'A', {0x7E, 0x11, 0x11, 0x11, 0x7E}},
    {'B', {0x7F, 0x49, 0x49, 0x49, 0x36}},
    {'C', {0x3E, 0x41, 0x41, 0x41, 0x22}},
    {'F', {0x7F, 0x09, 0x09, 0x09, 0x01}},
    {'H', {0x7F, 0x08, 0x08, 0x08, 0x7F}},
    {'I', {0x00, 0x41, 0x7F, 0x41, 0x00}},
    {'M', {0x7F, 0x02, 0x0C, 0x02, 0x7F}},
    {'O', {0x3E, 0x41, 0x41, 0x41, 0x3E}},
    {'R', {0x7F, 0x09, 0x19, 0x29, 0x46}},
    {'S', {0x46, 0x49, 0x49, 0x49, 0x31}},
    {'T', {0x01, 0x01, 0x7F, 0x01, 0x01}},
    {'U', {0x3F, 0x40, 0x40, 0x40, 0x3F}},
    {'W', {0x3F, 0x40, 0x38, 0x40, 0x3F}},
    {'d', {0x38, 0x44, 0x44, 0x48, 0x7F}},
    {'e', {0x38, 0x54, 0x54, 0x54, 0x18}},
    {'1', {0x00, 0x42, 0x7F, 0x40, 0x00}},
    {'P', {0x7F, 0x09, 0x09, 0x09, 0x06}},
    {'h', {0x7F, 0x08, 0x04, 0x04, 0x78}},
    {'l', {0x00, 0x41, 0x7F, 0x40, 0x00}},
    {'o', {0x38, 0x44, 0x44, 0x44, 0x38}},
    {'r', {0x7C, 0x08, 0x04, 0x04, 0x08}},
    {'s', {0x48, 0x54, 0x54, 0x54, 0x20}},
    {'u', {0x3C, 0x40, 0x40, 0x20, 0x7C}}
};

static HAL_StatusTypeDef WriteCommand(uint8_t command)
{
    uint8_t packet[] = {0x00U, command};
    return HAL_I2C_Master_Transmit(oled_i2c, SSD1306_HAL_ADDRESS,
                                   packet, sizeof(packet), SSD1306_INIT_TIMEOUT_MS);
}

HAL_StatusTypeDef SSD1306_Init(I2C_HandleTypeDef *i2c)
{
    /* Horizontal addressing, 64 COM lines, internal charge pump. */
    static const uint8_t commands[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
        0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x12,
        0x81, 0x7F, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0x2E
    };
    HAL_StatusTypeDef status;

    oled_i2c = i2c;
    if (oled_i2c == NULL)
    {
        return HAL_ERROR;
    }
    status = HAL_I2C_IsDeviceReady(oled_i2c, SSD1306_HAL_ADDRESS,
                                 1U, SSD1306_INIT_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        return status;
    }
    for (size_t index = 0U; index < sizeof(commands); ++index)
    {
        status = WriteCommand(commands[index]);
        if (status != HAL_OK)
        {
            return status;
        }
    }
    /* The full 1025-byte screen takes about 93 ms at 100 kHz. Keep it out of
     * boot's blocking path; Board_InitDisplay submits the first screen via IT. */
    SSD1306_Clear();
    return WriteCommand(0xAFU);
}

void SSD1306_Clear(void)
{
    frame[0] = 0x40U;
    memset(&frame[1], 0, SSD1306_BUFFER_SIZE);
}

void SSD1306_WriteString(uint8_t x, uint8_t page, const char *text)
{
    if ((text == NULL) || (page >= SSD1306_HEIGHT / 8U))
    {
        return;
    }
    while ((*text != '\0') && ((uint16_t)x + 6U <= SSD1306_WIDTH))
    {
        const uint8_t *columns = font[8].columns; /* '?' fallback */
        for (size_t index = 0U; index < sizeof(font) / sizeof(font[0]); ++index)
        {
            if (font[index].character == *text)
            {
                columns = font[index].columns;
                break;
            }
        }
        size_t offset = 1U + (size_t)page * SSD1306_WIDTH + x;
        memcpy(&frame[offset], columns, 5U);
        frame[offset + 5U] = 0U;
        x += 6U;
        ++text;
    }
}

HAL_StatusTypeDef SSD1306_UpdateScreen(void)
{
    /* Reset the full column/page range before every framebuffer transfer. */
    uint8_t address_window[] = {0x00U, 0x21U, 0U, 127U, 0x22U, 0U, 7U};
    HAL_StatusTypeDef status;
    if (oled_i2c == NULL)
    {
        return HAL_ERROR;
    }
    status = HAL_I2C_Master_Transmit(oled_i2c, SSD1306_HAL_ADDRESS,
                                    address_window, sizeof(address_window),
                                    SSD1306_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        return status;
    }
    frame[0] = 0x40U;
    return HAL_I2C_Master_Transmit(oled_i2c, SSD1306_HAL_ADDRESS,
                                   frame, sizeof(frame), SSD1306_TIMEOUT_MS);
}
