#include "board_inputs.h"
#include "board_input_config.h"
#include "car_control.h"
#include "laser.h"
#include "laser_config.h"
#include "car_config.h"
#include <assert.h>
#include <stdio.h>

GPIO_TypeDef mock_gpiob, mock_gpioc, mock_gpiod, mock_gpioe;
static uint32_t tick;
static unsigned button_inits, low_writes;
void Debug_Log(const char *text) { (void)text; }
uint8_t Debug_CanLog(uint8_t count) { (void)count; return 1U; }
void Car_Control_SubmitLocalInput(const CarLocalInput_t *input) { (void)input; }
uint32_t HAL_GetTick(void) { return tick; }
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init)
{
    if (port == GPIOB) {
        assert(init->Pin == GPIO_PIN_8 && init->Mode == GPIO_MODE_INPUT && init->Pull == GPIO_PULLUP);
        ++button_inits;
    } else if (port == GPIOC && init->Pin == GPIO_PIN_3) assert(init->Pull == GPIO_NOPULL);
    else assert(init->Mode == GPIO_MODE_INPUT && init->Pull == GPIO_PULLUP);
}
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    assert(port == GPIOC && pin == GPIO_PIN_3 && state == GPIO_PIN_RESET);
    ++low_writes;
}
int main(void)
{
    mock_gpiob.IDR = GPIO_PIN_8;
    mock_gpioc.IDR = mock_gpiod.IDR = mock_gpioe.IDR = UINT16_MAX;
    Laser_Init(); BoardInputs_Init();
    assert(button_inits == (unsigned)CAR_TEST_INPUTS_ENABLE && !Laser_IsEnabled());
    mock_gpiob.IDR = 0U; BoardInputs_ProcessAux();
    tick = LASER_BUTTON_DEBOUNCE_MS; BoardInputs_ProcessAux();
    assert(Laser_IsEnabled() == (CAR_TEST_INPUTS_ENABLE != 0));
    mock_gpiob.IDR = GPIO_PIN_8; BoardInputs_ProcessAux();
    assert(!Laser_IsEnabled());
    Laser_Enable(); assert(Laser_IsEnabled());
    Laser_Disable(); assert(!Laser_IsEnabled());
    assert(low_writes == 1U + (unsigned)CAR_TEST_INPUTS_ENABLE);
    printf("PASS laser inputs: production/bench=%u GPIO ownership, manual gating, automatic trigger\n",
           (unsigned)CAR_TEST_INPUTS_ENABLE);
    return 0;
}
