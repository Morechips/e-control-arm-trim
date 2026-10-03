#include "laser.h"
#include "laser_config.h"

static bool laser_initialized;
static bool automatic_requested;
static bool output_active;
static bool manual_requested;

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
    LASER_TRIGGER_GPIO_CLK_ENABLE();
    SetPinFloating();
    manual_requested = false;
    automatic_requested = false;
    output_active = false;
    laser_initialized = true;
}

void Laser_SetManualRequest(bool requested)
{
    manual_requested = requested;
    ApplyOutput();
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
