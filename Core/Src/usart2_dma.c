#include "usart2_dma.h"

UART_HandleTypeDef huart2;
static DMA_HandleTypeDef rx_dma;
static volatile uint8_t dma_buffer[USART2_RX_BUFFER_SIZE];
static uint8_t received[USART2_RX_BUFFER_SIZE];
static volatile uint16_t received_size, received_offset;
static volatile uint32_t received_tick;
static uint16_t previous;
static volatile uint8_t restart_required;
volatile uint32_t usart2_rx_periods, usart2_rx_bytes;
volatile uint32_t usart2_rx_overflows, usart2_rx_errors;
volatile uint32_t usart2_rx_unread_batches;
volatile uint32_t usart2_rx_dma_errors, usart2_rx_uart_errors;
volatile uint32_t usart2_rx_ore_errors, usart2_rx_ne_errors;
volatile uint32_t usart2_rx_fe_errors, usart2_rx_pe_errors;
volatile uint32_t usart2_last_dma_flags, usart2_last_uart_flags;

static HAL_StatusTypeDef StartReception(void)
{
    HAL_StatusTypeDef status;
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    previous = 0U;
    /* RX is owned here, not by HAL_UART_Receive_*; no RXNE/IDLE/DMA IRQs. */
    status = HAL_DMA_Start(&rx_dma, (uint32_t)&USART2->DR,
                           (uint32_t)dma_buffer, USART2_RX_BUFFER_SIZE);
    if (status == HAL_OK) SET_BIT(USART2->CR3, USART_CR3_DMAR);
    return status;
}

void USART2_DMA_Init(void)
{
    uint32_t timer_clock;
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_5 | GPIO_PIN_6;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &gpio);
    huart2.Instance = USART2;
    huart2.Init.BaudRate = USART2_BAUD_RATE;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();

    rx_dma.Instance = DMA1_Stream5;
    rx_dma.Init.Channel = DMA_CHANNEL_4;
    rx_dma.Init.Direction = DMA_PERIPH_TO_MEMORY;
    rx_dma.Init.PeriphInc = DMA_PINC_DISABLE;
    rx_dma.Init.MemInc = DMA_MINC_ENABLE;
    rx_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    rx_dma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    rx_dma.Init.Mode = DMA_CIRCULAR;
    rx_dma.Init.Priority = DMA_PRIORITY_HIGH;
    rx_dma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&rx_dma) != HAL_OK) Error_Handler();
    __HAL_LINKDMA(&huart2, hdmarx, rx_dma);
    if (StartReception() != HAL_OK) Error_Handler();

    /* TIM6 services the 256-byte ring at the configured period; HAL tick stays on SysTick. */
    __HAL_RCC_TIM6_CLK_ENABLE();
    timer_clock = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U) timer_clock *= 2U;
    TIM6->CR1 = 0U;
    TIM6->PSC = timer_clock / 10000U - 1U;
    TIM6->ARR = USART2_RX_PERIOD_MS * 10U - 1U;
    TIM6->EGR = TIM_EGR_UG;
    TIM6->SR = 0U;
    TIM6->DIER = TIM_DIER_UIE;
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 2U, 0U);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    HAL_NVIC_SetPriority(USART2_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    TIM6->CR1 = TIM_CR1_CEN;
}

void USART2_DMA_TimerIRQ(void)
{
    uint32_t flags, uart_errors;
    uint16_t position, count, i;
    if ((TIM6->SR & TIM_SR_UIF) == 0U) return;
    TIM6->SR = 0U;
    usart2_rx_periods++;
    if (restart_required) return;
    flags = DMA1->HISR;
    if ((flags & (DMA_HISR_TEIF5 | DMA_HISR_DMEIF5 | DMA_HISR_FEIF5)) != 0U) {
        CLEAR_BIT(USART2->CR3, USART_CR3_DMAR);
        usart2_last_dma_flags = flags;
        usart2_rx_dma_errors++;
        usart2_rx_errors++;
        restart_required = 1U;
        return;
    }
    uart_errors = USART2->SR & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE);
    if (uart_errors != 0U) {
        usart2_last_uart_flags = uart_errors;
        if (uart_errors & USART_SR_ORE) usart2_rx_ore_errors++;
        if (uart_errors & USART_SR_NE) usart2_rx_ne_errors++;
        if (uart_errors & USART_SR_FE) usart2_rx_fe_errors++;
        if (uart_errors & USART_SR_PE) usart2_rx_pe_errors++;
        usart2_rx_uart_errors++;
        __HAL_UART_CLEAR_OREFLAG(&huart2);
        usart2_rx_errors++;
    }
    position = (uint16_t)((USART2_RX_BUFFER_SIZE - rx_dma.Instance->NDTR)
                         % USART2_RX_BUFFER_SIZE);
    /* If DMA wrapped during the sample, include that wrap and resample.
     * Only clear a wrap actually observed, so a later wrap remains pending. */
    if ((flags & DMA_HISR_TCIF5) == 0U &&
        (DMA1->HISR & DMA_HISR_TCIF5) != 0U) {
        flags |= DMA_HISR_TCIF5;
        position = (uint16_t)((USART2_RX_BUFFER_SIZE - rx_dma.Instance->NDTR)
                             % USART2_RX_BUFFER_SIZE);
    }
    if ((flags & DMA_HISR_TCIF5) != 0U) DMA1->HIFCR = DMA_HIFCR_CTCIF5;
    count = (uint16_t)((position + USART2_RX_BUFFER_SIZE - previous)
                      % USART2_RX_BUFFER_SIZE);
    if ((flags & DMA_HISR_TCIF5) != 0U && position >= previous) {
        /* At least one whole ring was received. Retain the latest ring.
         * At continuous 115200 baud even this snapshot can be overwritten:
         * overflow is reported to the parser as a stream discontinuity. */
        usart2_rx_overflows++;
        count = USART2_RX_BUFFER_SIZE;
        previous = position;
    }
    if (count != 0U) {
        if (received_size != received_offset) usart2_rx_unread_batches++;
        for (i = 0U; i < count; i++) {
            received[i] = dma_buffer[previous];
            previous = (uint16_t)((previous + 1U) % USART2_RX_BUFFER_SIZE);
        }
        received_offset = 0U;
        received_size = count;
        received_tick = HAL_GetTick();
        usart2_rx_bytes += count;
    }
    previous = position;
}

void USART2_DMA_Process(void)
{
    /* HAL DMA abort may wait on hardware; recover outside the timer IRQ. */
    if (restart_required) {
        if (rx_dma.State == HAL_DMA_STATE_BUSY && HAL_DMA_Abort(&rx_dma) != HAL_OK) return;
        if (rx_dma.State != HAL_DMA_STATE_READY && HAL_DMA_Init(&rx_dma) != HAL_OK) return;
        if (StartReception() == HAL_OK) restart_required = 0U;
    }
}

uint16_t USART2_DMA_Read(uint8_t *destination, uint16_t capacity)
{
    uint32_t tick;
    return USART2_DMA_ReadTimed(destination, capacity, &tick);
}

void USART2_DMA_DiscardPending(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    received_size = received_offset = 0U;
    DMA1->HIFCR = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5;
    previous = (uint16_t)((USART2_RX_BUFFER_SIZE - rx_dma.Instance->NDTR) % USART2_RX_BUFFER_SIZE);
    __set_PRIMASK(mask);
}

uint16_t USART2_DMA_ReadTimed(uint8_t *destination, uint16_t capacity, uint32_t *tick)
{
    uint16_t count, i;
    uint32_t mask;
    if (destination == NULL || capacity == 0U || tick == NULL) return 0U;
    mask = __get_PRIMASK();
    __disable_irq();
    *tick = received_tick;
    count = (uint16_t)(received_size - received_offset);
    if (count > capacity) count = capacity;
    for (i = 0U; i < count; i++) destination[i] = received[received_offset + i];
    received_offset = (uint16_t)(received_offset + count);
    __set_PRIMASK(mask);
    return count;
}

uint16_t USART2_DMA_Remaining(void)
{
    return (uint16_t)rx_dma.Instance->NDTR;
}
