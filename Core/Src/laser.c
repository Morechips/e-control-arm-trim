#include "laser.h"
#include "laser_config.h"

static bool laser_initialized;
static bool automatic_requested;
static bool manual_requested;
static bool output_active;
static bool button_press_pending;
static uint32_t button_press_tick;

static void SetPinFloating(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = LASER_TRIGGER_GPIO_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(LASER_TRIGGER_GPIO_PORT, &gpio);
}

static void ApplyOutput(void)
{
    bool requested = automatic_requested || manual_requested;
    GPIO_InitTypeDef gpio = {0};
    if (!laser_initialized || requested == output_active) return;
    if (!requested)
    {
        SetPinFloating();
        output_active = false;
        return;
    }
    HAL_GPIO_WritePin(LASER_TRIGGER_GPIO_PORT, LASER_TRIGGER_GPIO_PIN, GPIO_PIN_RESET);
    gpio.Pin = LASER_TRIGGER_GPIO_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LASER_TRIGGER_GPIO_PORT, &gpio);
    output_active = true;
}

void Laser_Init(void)
{
    GPIO_InitTypeDef button = {0};
    LASER_TRIGGER_GPIO_CLK_ENABLE();
    LASER_BUTTON_GPIO_CLK_ENABLE();
    SetPinFloating();
    button.Pin = LASER_BUTTON_GPIO_PIN;
    button.Mode = GPIO_MODE_INPUT;
    button.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(LASER_BUTTON_GPIO_PORT, &button);
    automatic_requested = false;
    manual_requested = false;
    output_active = false;
    button_press_pending = false;
    button_press_tick = 0U;
    laser_initialized = true;
}

void Laser_ProcessButton(void)
{
    bool pressed;
    if (!laser_initialized) return;
    pressed = (LASER_BUTTON_GPIO_PORT->IDR & LASER_BUTTON_GPIO_PIN) == 0U;
    if (!pressed)
    {
        button_press_pending = false;
        if (manual_requested)
        {
            manual_requested = false;
            ApplyOutput();
        }
        return;
    }
    if (manual_requested) return;
    if (!button_press_pending)
    {
        button_press_pending = true;
        button_press_tick = HAL_GetTick();
        return;
    }
    if ((uint32_t)(HAL_GetTick() - button_press_tick) >= LASER_BUTTON_DEBOUNCE_MS)
    {
        manual_requested = true;
        ApplyOutput();
    }
}

void Laser_Enable(void)
{
    if (!laser_initialized) return;
    automatic_requested = true;
    ApplyOutput();
}

void Laser_Disable(void)
{
    automatic_requested = false;
    ApplyOutput();
}

bool Laser_IsEnabled(void)
{
    return output_active;
}
