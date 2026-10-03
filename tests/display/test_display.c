#include "ssd1306.h"
#include <assert.h>
#include <stdio.h>
static I2C_HandleTypeDef i2c;
static HAL_StatusTypeDef ready_result, command_result;
static unsigned probes, commands, async_calls, resets;
static uint32_t tick;
uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *bus, uint16_t addr, uint32_t trials, uint32_t timeout)
{
    assert(bus == &i2c && addr == 0x78U && trials == 1U && timeout == 50U);
    ++probes; return ready_result;
}
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *bus, uint16_t addr, uint8_t *data, uint16_t size, uint32_t timeout)
{
    assert(bus == &i2c && addr == 0x78U && data[0] == 0U);
    assert(size == 2U && timeout == 50U); /* No synchronous framebuffer on boot. */
    ++commands; return command_result;
}
HAL_StatusTypeDef HAL_I2C_Master_Transmit_IT(I2C_HandleTypeDef *bus, uint16_t addr, uint8_t *data, uint16_t size)
{
    assert(bus == &i2c && addr == 0x78U && data != NULL);
    assert(size == 7U || size == 1025U); ++async_calls; return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *bus) { assert(bus == &i2c); ++resets; return HAL_OK; }
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *bus) { assert(bus == &i2c); return HAL_OK; }
uint32_t HAL_I2C_GetState(I2C_HandleTypeDef *bus) { return bus->state; }
uint32_t HAL_I2C_GetError(I2C_HandleTypeDef *bus) { return bus->error; }
int main(void)
{
    assert(SSD1306_Init(NULL) == HAL_ERROR);
    ready_result = HAL_TIMEOUT;
    assert(SSD1306_Init(&i2c) == HAL_TIMEOUT && probes == 1U && commands == 0U);
    ready_result = HAL_OK; command_result = HAL_ERROR;
    assert(SSD1306_Init(&i2c) == HAL_ERROR && commands == 1U);
    command_result = HAL_OK;
    assert(SSD1306_Init(&i2c) == HAL_OK && !SSD1306_IsBusy() && async_calls == 0U);
    i2c.busy = 1U;
    assert(SSD1306_UpdateScreenAsync() == HAL_BUSY && async_calls == 0U);
    i2c.busy = 0U;
    assert(SSD1306_UpdateScreenAsync() == HAL_OK && async_calls == 1U);
    assert(SSD1306_UpdateScreenAsync() == HAL_BUSY);
    SSD1306_Process(); assert(async_calls == 2U && SSD1306_IsBusy());
    SSD1306_Process(); assert(!SSD1306_IsBusy());
    assert(SSD1306_UpdateScreenAsync() == HAL_OK);
    tick = 251U; SSD1306_Process();
    assert(resets == 1U && !SSD1306_IsBusy());
    puts("PASS OLED: optional failure, one 50ms probe, no boot delay/framebuffer wait, async/stuck-bus recovery");
    return 0;
}
