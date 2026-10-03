#include "board_app.h"
#include "ssd1306.h"
#include "car_config.h"
#include "vision_config.h"
#include "servo.h"
#include "board_inputs.h"
#include "laser.h"
#include "usart6_config.h"
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1, huart3, huart4, huart5, huart6;
volatile HAL_StatusTypeDef oled_status = HAL_ERROR;
#if CAR_TEST_INPUTS_ENABLE
static uint8_t button1_pressed;
static uint8_t display_dirty;
#endif
void SystemClock_Config(void);
static HAL_StatusTypeDef MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_USART6_UART_Init(void);
static void DrawButton(void)
{
    SSD1306_Clear();
    SSD1306_WriteString(0U, 0U, "Hello World");
    SSD1306_WriteString(0U, 2U, "STM32F407");
    SSD1306_WriteString(0U, 4U, "BT: " BT_DEVICE_NAME);
#if CAR_TEST_INPUTS_ENABLE
    SSD1306_WriteString(0U, 6U, button1_pressed ? "BT1 Push" : "BT1");
#endif
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
    MX_USART6_UART_Init();
    /* Binding only: no ZL-IS2 command is sent on boot. */
    if (Servo_Init(&huart3) != SERVO_OK) Error_Handler();
    if (CAR_UART_BRIDGE_TEST) return;
    BoardInputs_Init();
    oled_status = MX_I2C1_Init();
}

HAL_StatusTypeDef Board_InitDisplay(void)
{
    /* I2C1 itself is optional: a missing or faulty panel must never stop the
     * control link, so a failed bus init only disables the display. */
    if (oled_status != HAL_OK) return oled_status;
    oled_status = SSD1306_Init(&hi2c1);
    if (oled_status == HAL_OK) {
        DrawButton();
        oled_status = SSD1306_UpdateScreenAsync();
    }
    return oled_status;
}
void Board_Process(void)
{
    BoardInputs_ProcessAux();
#if CAR_TEST_INPUTS_ENABLE
    uint8_t pressed = BoardInputs_DisplayPressed();
    if (pressed != button1_pressed) { button1_pressed = pressed; display_dirty = 1U; }
#endif
    SSD1306_Process();
#if CAR_TEST_INPUTS_ENABLE
    if (oled_status == HAL_OK && display_dirty && !SSD1306_IsBusy()) {
        HAL_StatusTypeDef status;
        DrawButton();
        status = SSD1306_UpdateScreenAsync();
        if (status != HAL_BUSY) { oled_status = status; display_dirty = 0U; }
    }
#endif
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

static HAL_StatusTypeDef MX_I2C1_Init(void)
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
    /* The panel is cosmetic: report the failure instead of halting the board
     * with interrupts disabled before the control link is up. */
    return HAL_I2C_Init(&hi2c1);
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
    huart3.Init.BaudRate = SERVO_BAUD_RATE;
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
