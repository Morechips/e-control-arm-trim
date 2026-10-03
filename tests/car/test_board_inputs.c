#include "board_inputs.h"
#include "board_input_config.h"
#include "car_control.h"
#include "laser.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
GPIO_TypeDef mock_gpiob, mock_gpioc, mock_gpiod, mock_gpioe;
static uint32_t tick;
static CarLocalInput_t last;
static unsigned samples, starts, inits;
uint32_t HAL_GetTick(void) { return tick; }
void Car_Control_SubmitLocalInput(const CarLocalInput_t *input) { last = *input; ++samples; }
void Debug_Log(const char *text) { if (strcmp(text,"[START] press\r\n")==0) ++starts; }
uint8_t Debug_CanLog(uint8_t n) { (void)n; return 1U; }
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *gpio)
{
    if (port == GPIOC && gpio->Pin == GPIO_PIN_3) return;
    assert(gpio->Mode == GPIO_MODE_INPUT && gpio->Pull == GPIO_PULLUP); ++inits;
}
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState value)
{ assert(port == GPIOC && pin == GPIO_PIN_3 && value == GPIO_PIN_RESET); }
static void Released(void)
{ mock_gpiob.IDR = mock_gpioc.IDR = mock_gpiod.IDR = mock_gpioe.IDR = UINT16_MAX; }
int main(void)
{
    Released(); Laser_Init(); BoardInputs_Init();
    assert(inits == (CAR_TEST_INPUTS_ENABLE ? 6U : 1U));
    tick = UINT32_MAX - 10U;
    mock_gpiod.IDR &= ~GPIO_PIN_10; mock_gpioe.IDR &= ~GPIO_PIN_0; mock_gpioc.IDR &= ~GPIO_PIN_1;
    BoardInputs_ProcessControl(); tick += 19U; BoardInputs_ProcessControl();
    assert(starts == 0U);
#if CAR_TEST_INPUTS_ENABLE
    assert(!last.vision_press && !last.shot_press && !last.pd10_ready && last.pd10_low);
#endif
    ++tick; BoardInputs_ProcessControl(); assert(starts == 1U);
#if CAR_TEST_INPUTS_ENABLE
    assert(last.vision_press && last.shot_press && last.pd10_ready);
    BoardInputs_ProcessControl(); assert(!last.vision_press && !last.shot_press);
    tick += 100U; BoardInputs_ProcessControl(); assert(starts == 1U);
    Released(); BoardInputs_ProcessControl(); assert(!last.pd10_low && !last.pd10_ready);
    tick += 20U; BoardInputs_ProcessControl(); assert(!last.vision_held && !last.shot_held);
    mock_gpioe.IDR &= ~GPIO_PIN_4;
    assert(!BoardInputs_TakeServoAimPress()); tick += 19U; assert(!BoardInputs_TakeServoAimPress());
    ++tick; assert(BoardInputs_TakeServoAimPress());
    assert(!BoardInputs_TakeServoAimPress());
    Released(); BoardInputs_TakeServoAimPress(); tick += 20U; BoardInputs_TakeServoAimPress();
    mock_gpioe.IDR &= ~GPIO_PIN_4; BoardInputs_TakeServoAimPress(); tick += 20U;
    assert(BoardInputs_TakeServoAimPress());
    /* Initially held PE0/PC1/PE4/start must be released before a press event. */
    mock_gpioe.IDR &= ~GPIO_PIN_0; mock_gpioc.IDR &= ~GPIO_PIN_1; mock_gpiod.IDR &= ~GPIO_PIN_10;
    unsigned before = starts; BoardInputs_Init(); tick += 50U; BoardInputs_ProcessControl();
    assert(!last.vision_press && !last.shot_press && !BoardInputs_TakeServoAimPress() && starts == before);
    /* PB8 starts debounce on the first auxiliary poll and releases immediately. */
    Released(); BoardInputs_ProcessAux(); mock_gpiob.IDR &= ~GPIO_PIN_8;
    BoardInputs_ProcessAux(); tick += 19U; BoardInputs_ProcessAux(); assert(!Laser_IsEnabled());
    ++tick; BoardInputs_ProcessAux(); assert(Laser_IsEnabled());
    Laser_Enable(); Released(); BoardInputs_ProcessAux(); assert(Laser_IsEnabled());
    Laser_Disable(); assert(!Laser_IsEnabled());
#else
    assert(samples == 0U && !BoardInputs_TakeServoAimPress());
    mock_gpiob.IDR = mock_gpioe.IDR = 0U;
    BoardInputs_ProcessAux(); tick += 100U; BoardInputs_ProcessAux();
    assert(!Laser_IsEnabled() && !BoardInputs_DisplayPressed());
#endif
    printf("PASS board inputs: mode=%u debounce/wrap, one-shot, boot hold, PD10 release, PE4 arbitration and PB8 overlap\n",(unsigned)CAR_TEST_INPUTS_ENABLE);
    return 0;
}
