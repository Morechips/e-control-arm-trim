/* Start-key regression: debounce, one-shot press events, and the rule that a
 * key already held at power-up must be released before it can fire. */
#include "start_button.h"
#include "start_button_config.h"
#include <assert.h>
#include <stdio.h>

GPIO_TypeDef mock_gpiod;
static uint32_t now;
static unsigned init_count;
static unsigned last_pull;

uint32_t HAL_GetTick(void) { return now; }
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init)
{
    assert(port == GPIOD);
    assert(init->Pin == START_BUTTON_GPIO_PIN);
    assert(init->Mode == GPIO_MODE_INPUT);
    last_pull = init->Pull;
    ++init_count;
}
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    (void)port; (void)pin; (void)state;
}

/* Released = the internal pull-up holds the pin high. */
static void Release(void) { mock_gpiod.IDR = START_BUTTON_GPIO_PIN; }
static void Press(void) { mock_gpiod.IDR = 0U; }
/* One sample followed by the settling interval this module debounces over. */
static void Settle(void)
{
    StartButton_Process();
    now += START_BUTTON_DEBOUNCE_MS;
    StartButton_Process();
}

int main(void)
{
    unsigned i;
    Release();
    StartButton_Init();
    assert(init_count == 1U && last_pull == GPIO_PULLUP);
    assert(StartButton_IsHeld() == 0U);
    assert(StartButton_TakePressEvent() == 0U);

    /* A press is only reported after the debounce interval elapses. */
    Press();
    StartButton_Process();
    assert(StartButton_IsHeld() == 0U && StartButton_TakePressEvent() == 0U);
    now += START_BUTTON_DEBOUNCE_MS - 1U;
    StartButton_Process();
    assert(StartButton_IsHeld() == 0U && StartButton_TakePressEvent() == 0U);
    now += 1U;
    StartButton_Process();
    assert(StartButton_IsHeld() == 1U);
    assert(StartButton_TakePressEvent() == 1U);
    /* The event is one-shot: holding the key must not repeat it. */
    for (i = 0U; i < 50U; ++i) { now += 10U; StartButton_Process(); }
    assert(StartButton_IsHeld() == 1U && StartButton_TakePressEvent() == 0U);

    /* Release, then a new press produces exactly one new event. */
    Release();
    Settle();
    assert(StartButton_IsHeld() == 0U && StartButton_TakePressEvent() == 0U);
    Press();
    Settle();
    assert(StartButton_IsHeld() == 1U);
    assert(StartButton_TakePressEvent() == 1U);
    assert(StartButton_TakePressEvent() == 0U);

    /* A contact bounce shorter than the debounce window is ignored. */
    Release();
    Settle();
    assert(StartButton_IsHeld() == 0U);
    Press();
    StartButton_Process();
    now += START_BUTTON_DEBOUNCE_MS / 2U;
    StartButton_Process();
    Release();
    Settle();
    assert(StartButton_IsHeld() == 0U && StartButton_TakePressEvent() == 0U);

    /* Held through power-up: inert until released and pressed again. */
    Press();
    now += 1000U;
    StartButton_Init();
    assert(StartButton_IsHeld() == 1U);
    for (i = 0U; i < 20U; ++i) { now += 10U; StartButton_Process(); }
    assert(StartButton_TakePressEvent() == 0U);
    Release();
    Settle();
    assert(StartButton_IsHeld() == 0U);
    Press();
    Settle();
    assert(StartButton_TakePressEvent() == 1U);

    printf("start button PASS: debounce, one-shot events, bounce rejection, power-up hold\n");
    return 0;
}
