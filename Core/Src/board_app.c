#include "board_app.h"
#include "ssd1306.h"
#include "car_config.h"
#include "zlis2_driver.h"
#include "vision_config.h"
#include "servo_remote_config.h"
#include "laser.h"
#include "usart6_config.h"
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1, huart3, huart4, huart5, huart6;
volatile HAL_StatusTypeDef oled_status = HAL_ERROR;
volatile HAL_StatusTypeDef bluetooth_at_status = HAL_ERROR;
volatile uint16_t bluetooth_at_response_length;
uint8_t bluetooth_at_response[32];
volatile GPIO_PinState button1_state = GPIO_PIN_SET;
static GPIO_PinState button_raw = GPIO_PIN_SET;
static uint32_t button_tick;
static uint8_t display_dirty;
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_USART6_UART_Init(void);
static void Bluetooth_SetAdvertisingName(void);
static void DrawButton(void)
{
    SSD1306_Clear();
    SSD1306_WriteString(0U, 0U, "Hello World");
    SSD1306_WriteString(0U, 2U, "STM32F407");
    SSD1306_WriteString(0U, 4U, "BT: " BT_DEVICE_NAME);
    SSD1306_WriteString(0U, 6U, button1_state == GPIO_PIN_RESET ? "BT1 Push" : "BT1");
}
void Board_Init(void)
{
    HAL_Init();
    SystemClock_Config();
    Laser_Init();
    MX_USART1_UART_Init();
    MX_UART5_Init();
    MX_USART3_UART_Init();
    MX_UART4_Init();
    /* Binding only: no ZL-IS2 command is sent on boot. */
    if (ZLIS2_Init(&huart3) != ZLIS2_OK) Error_Handler();
    if (CAR_UART_BRIDGE_TEST) return;
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART6_UART_Init();
    oled_status = SSD1306_Init(&hi2c1);
    if (oled_status == HAL_OK) { DrawButton(); oled_status = SSD1306_UpdateScreen(); }
    /* Startup-only AT exchange, before accepting ANY control packets. */
    Bluetooth_SetAdvertisingName();
    button_tick = HAL_GetTick();
}
uint8_t Board_PD10IsLow(void)
{
    /* Only PD10 drives the local test; the other display buttons do not. */
    return (uint8_t)((GPIOD->IDR & GPIO_PIN_10) == 0U);
}
uint8_t Board_VisionButtonIsLow(void)
{
    return (uint8_t)((VISION_BUTTON_GPIO_PORT->IDR & VISION_BUTTON_GPIO_PIN) == 0U);
}
uint8_t Board_ShotButtonIsLow(void)
{
    return (uint8_t)((SHOT_BUTTON_GPIO_PORT->IDR & SHOT_BUTTON_GPIO_PIN) == 0U);
}
uint8_t Board_ServoButtonIsLow(void)
{
    return (uint8_t)((SERVO_BUTTON_GPIO_PORT->IDR & SERVO_BUTTON_GPIO_PIN) == 0U);
}
void Board_Process(void)
{
    GPIO_PinState sampled = ((BUTTON1_GPIO_PORT->IDR & BUTTON1_GPIO_PINS) == BUTTON1_GPIO_PINS) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    uint32_t now = HAL_GetTick();
    Laser_ProcessButton();
    if (sampled != button_raw) { button_raw = sampled; button_tick = now; }
    if (button_raw != button1_state && (uint32_t)(now - button_tick) >= BUTTON1_DEBOUNCE_MS) {
        button1_state = button_raw;
        display_dirty = 1U;
    }
    SSD1306_Process();
    if (oled_status == HAL_OK && display_dirty && !SSD1306_IsBusy()) {
        HAL_StatusTypeDef status;
        DrawButton();
        status = SSD1306_UpdateScreenAsync();
        if (status != HAL_BUSY) { oled_status = status; display_dirty = 0U; }
    }
}
static void Bluetooth_SetAdvertisingName(void)
{
    static const uint8_t command[] = BT_AT_NAME_COMMAND;
    HAL_StatusTypeDef receive_status;
    uint32_t timeout = BT_AT_RESPONSE_TIMEOUT_MS;

    bluetooth_at_response_length = 0U;
    bluetooth_at_response[0] = '\0';

    /* Give the module time to enter AT mode after power-up. */
    HAL_Delay(BT_AT_STARTUP_DELAY_MS);
    bluetooth_at_status = HAL_UART_Transmit(&huart6, command,
                                            (uint16_t)(sizeof(command) - 1U),
                                            1000U);
    if (bluetooth_at_status != HAL_OK)
    {
        return;
    }

    /* Capture an optional OK/error reply without making it mandatory. */
    while (bluetooth_at_response_length < (sizeof(bluetooth_at_response) - 1U))
    {
        receive_status = HAL_UART_Receive(
            &huart6,
            &bluetooth_at_response[bluetooth_at_response_length],
            1U,
            timeout);
        if (receive_status != HAL_OK)
        {
            break;
        }

        bluetooth_at_response_length++;
        timeout = BT_AT_RESPONSE_INTERBYTE_TIMEOUT_MS;
    }

    bluetooth_at_response[bluetooth_at_response_length] = '\0';

}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef oscillator = {0};
    RCC_ClkInitTypeDef clocks = {0};

    /* Internal HSI: SYSCLK, HCLK, PCLK1 and PCLK2 are all 16 MHz. */
    oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    oscillator.HSIState = RCC_HSI_ON;
    oscillator.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    oscillator.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&oscillator) != HAL_OK)
    {
        Error_Handler();
    }

    clocks.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clocks.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    clocks.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clocks.APB1CLKDivider = RCC_HCLK_DIV1;
    clocks.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clocks, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    VISION_BUTTON_GPIO_CLK_ENABLE();
    SERVO_BUTTON_GPIO_CLK_ENABLE();

    gpio.Pin = BUTTON1_GPIO_PINS;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BUTTON1_GPIO_PORT, &gpio);

    gpio.Pin = VISION_BUTTON_GPIO_PIN;
    HAL_GPIO_Init(VISION_BUTTON_GPIO_PORT, &gpio);

    gpio.Pin = SHOT_BUTTON_GPIO_PIN;
    HAL_GPIO_Init(SHOT_BUTTON_GPIO_PORT, &gpio);

    gpio.Pin = SERVO_BUTTON_GPIO_PIN;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(SERVO_BUTTON_GPIO_PORT, &gpio);

    /* Alternate functions are configured by the HAL MSP callbacks. */
}

static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000U;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0U;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0U;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = UART1_BAUD_RATE;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_USART3_UART_Init(void)
{
    huart3.Instance = USART3;
    huart3.Init.BaudRate = ZLIS2_BAUD_RATE;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK) Error_Handler();
}

static void MX_UART4_Init(void)
{
    huart4.Instance = UART4;
    huart4.Init.BaudRate = MAXICAM_UART_BAUD_RATE;
    huart4.Init.WordLength = UART_WORDLENGTH_8B;
    huart4.Init.StopBits = UART_STOPBITS_1;
    huart4.Init.Parity = UART_PARITY_NONE;
    huart4.Init.Mode = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart4) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_UART5_Init(void)
{
    huart5.Instance = UART5;
    huart5.Init.BaudRate = MOTOR1_UART_BAUD_RATE;
    huart5.Init.WordLength = UART_WORDLENGTH_8B;
    huart5.Init.StopBits = UART_STOPBITS_1;
    huart5.Init.Parity = UART_PARITY_NONE;
    huart5.Init.Mode = UART_MODE_TX_RX;
    huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart5.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart5) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_USART6_UART_Init(void)
{
    huart6.Instance = USART6;
    huart6.Init.BaudRate = BT_UART_BAUD_RATE;
    huart6.Init.WordLength = UART_WORDLENGTH_8B;
    huart6.Init.StopBits = UART_STOPBITS_1;
    huart6.Init.Parity = UART_PARITY_NONE;
    huart6.Init.Mode = UART_MODE_TX_RX;
    huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart6) != HAL_OK)
    {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}
