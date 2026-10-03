#include "board_inputs.h"
#include "board_input_config.h"
#include "car_control.h"
#include "laser.h"
#include "serial_io.h"
#include "start_button.h"
#include <stdio.h>

#if CAR_TEST_INPUTS_ENABLE
typedef struct { uint8_t raw, stable; uint32_t tick; } Button_t;
static Button_t vision, shot, servo, display;
static uint8_t pd10_seen, pd10_raw, laser_pending, laser_manual;
static uint32_t pd10_tick, laser_tick, servo_presses, diagnostic_tick;
static uint8_t Low(GPIO_TypeDef *port, uint16_t pin) { return (uint8_t)((port->IDR & pin) == 0U); }
static void InitButton(Button_t *b, uint8_t low)
{ b->raw = b->stable = low; b->tick = HAL_GetTick(); }
static uint8_t Press(Button_t *b, uint8_t low, uint32_t debounce)
{
    uint32_t now = HAL_GetTick();
    if (low != b->raw) { b->raw = low; b->tick = now; }
    if (low == b->stable || (uint32_t)(now - b->tick) < debounce) return 0U;
    b->stable = low; return low;
}
#endif
void BoardInputs_Init(void)
{
    StartButton_Init();
#if CAR_TEST_INPUTS_ENABLE
    GPIO_InitTypeDef gpio = {0};
    BOARD_DISPLAY_GPIO_CLK_ENABLE(); BOARD_PD10_GPIO_CLK_ENABLE();
    VISION_BUTTON_GPIO_CLK_ENABLE(); SHOT_BUTTON_GPIO_CLK_ENABLE();
    SERVO_BUTTON_GPIO_CLK_ENABLE(); LASER_BUTTON_GPIO_CLK_ENABLE();
    gpio.Mode = GPIO_MODE_INPUT; gpio.Pull = GPIO_PULLUP;
    gpio.Pin = BOARD_DISPLAY_GPIO_PINS; HAL_GPIO_Init(BOARD_DISPLAY_GPIO_PORT, &gpio);
    gpio.Pin = VISION_BUTTON_GPIO_PIN; HAL_GPIO_Init(VISION_BUTTON_GPIO_PORT, &gpio);
    gpio.Pin = SHOT_BUTTON_GPIO_PIN; HAL_GPIO_Init(SHOT_BUTTON_GPIO_PORT, &gpio);
    gpio.Pin = SERVO_BUTTON_GPIO_PIN; HAL_GPIO_Init(SERVO_BUTTON_GPIO_PORT, &gpio);
    gpio.Pin = LASER_BUTTON_GPIO_PIN; HAL_GPIO_Init(LASER_BUTTON_GPIO_PORT, &gpio);
    InitButton(&vision, Low(VISION_BUTTON_GPIO_PORT, VISION_BUTTON_GPIO_PIN));
    InitButton(&shot, Low(SHOT_BUTTON_GPIO_PORT, SHOT_BUTTON_GPIO_PIN));
    InitButton(&servo, Low(SERVO_BUTTON_GPIO_PORT, SERVO_BUTTON_GPIO_PIN));
    InitButton(&display, 0U);
    pd10_seen = laser_pending = laser_manual = 0U;
    servo_presses = 0U; diagnostic_tick = HAL_GetTick();
#endif
}
void BoardInputs_ProcessControl(void)
{
    StartButton_Process();
    if (StartButton_TakePressEvent()) Debug_Log("[START] press\r\n");
#if CAR_TEST_INPUTS_ENABLE
    CarLocalInput_t input = {0};
    uint32_t now = HAL_GetTick();
    uint8_t low = Low(BOARD_PD10_GPIO_PORT, BOARD_PD10_GPIO_PIN);
    input.vision_press = Press(&vision, Low(VISION_BUTTON_GPIO_PORT, VISION_BUTTON_GPIO_PIN), VISION_BUTTON_DEBOUNCE_MS);
    input.shot_press = Press(&shot, Low(SHOT_BUTTON_GPIO_PORT, SHOT_BUTTON_GPIO_PIN), SHOT_BUTTON_DEBOUNCE_MS);
    input.vision_held = vision.stable; input.shot_held = shot.stable;
    if (!pd10_seen || low != pd10_raw) { pd10_seen = 1U; pd10_raw = low; pd10_tick = now; }
    input.pd10_low = low;
    input.pd10_ready = (uint8_t)(low && (uint32_t)(now - pd10_tick) >= BOARD_PD10_DEBOUNCE_MS);
    Car_Control_SubmitLocalInput(&input);
#endif
}
uint8_t BoardInputs_TakeServoAimPress(void)
{
#if CAR_TEST_INPUTS_ENABLE
    uint8_t pressed = Press(&servo, Low(SERVO_BUTTON_GPIO_PORT, SERVO_BUTTON_GPIO_PIN), SERVO_BUTTON_DEBOUNCE_MS);
    if (pressed) ++servo_presses;
    return pressed;
#else
    return 0U;
#endif
}
void BoardInputs_ProcessAux(void)
{
#if CAR_TEST_INPUTS_ENABLE
    uint32_t now = HAL_GetTick();
    uint8_t low = Low(LASER_BUTTON_GPIO_PORT, LASER_BUTTON_GPIO_PIN);
    if (!low) { laser_pending = laser_manual = 0U; }
    else if (!laser_manual) {
        if (!laser_pending) { laser_pending = 1U; laser_tick = now; }
        else if ((uint32_t)(now - laser_tick) >= LASER_BUTTON_DEBOUNCE_MS) laser_manual = 1U;
    }
    Laser_SetManualRequest(laser_manual != 0U);
    (void)Press(&display, (uint8_t)((BOARD_DISPLAY_GPIO_PORT->IDR & BOARD_DISPLAY_GPIO_PINS) != BOARD_DISPLAY_GPIO_PINS), BOARD_DISPLAY_DEBOUNCE_MS);
#endif
}
uint8_t BoardInputs_DisplayPressed(void)
{
#if CAR_TEST_INPUTS_ENABLE
    return display.stable;
#else
    return 0U;
#endif
}
void BoardInputs_TraceServo(uint32_t transmissions, unsigned last_result)
{
#if CAR_TEST_INPUTS_ENABLE
    char line[80]; uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - diagnostic_tick) < 1000U || !Debug_CanLog(1U)) return;
    diagnostic_tick = now;
    (void)snprintf(line, sizeof(line), "[PE4] pin=%u presses=%lu tx=%lu last=%u\r\n",
        (unsigned)!servo.raw, (unsigned long)servo_presses, (unsigned long)transmissions, last_result);
    Debug_Log(line);
#else
    (void)transmissions; (void)last_result;
#endif
}
