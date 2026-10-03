#include "start_button.h"
#include "start_button_config.h"

static uint8_t raw, stable, press_pending;
static uint32_t change_tick;

static uint8_t Sample(void)
{
#if START_BUTTON_ACTIVE_LOW
    return (uint8_t)((START_BUTTON_GPIO_PORT->IDR & START_BUTTON_GPIO_PIN) == 0U);
#else
    return (uint8_t)((START_BUTTON_GPIO_PORT->IDR & START_BUTTON_GPIO_PIN) != 0U);
#endif
}

void StartButton_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    START_BUTTON_GPIO_CLK_ENABLE();
    gpio.Pin = START_BUTTON_GPIO_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(START_BUTTON_GPIO_PORT, &gpio);
    raw = stable = Sample();
    press_pending = 0U;
    change_tick = HAL_GetTick();
}

void StartButton_Process(void)
{
    uint8_t sampled = Sample();
    uint32_t now = HAL_GetTick();
    if (sampled != raw) {
        raw = sampled;
        change_tick = now;
    }
    if (raw == stable || (uint32_t)(now - change_tick) < START_BUTTON_DEBOUNCE_MS) return;
    stable = raw;
    /* Only the press edge is published; the release just re-arms the key. */
    if (stable != 0U) press_pending = 1U;
}

uint8_t StartButton_TakePressEvent(void)
{
    uint8_t pending = press_pending;
    press_pending = 0U;
    return pending;
}

uint8_t StartButton_IsHeld(void) { return stable; }
