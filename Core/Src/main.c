#include "board_app.h"
#include "bluetooth_driver.h"
#include "motor_driver.h"
#include "car_control.h"
#include "serial_io.h"
#include "uart_bridge.h"
#include "usart2_dma.h"
#include "jy61.h"
#include "pid_tuner.h"
#include "maxicam.h"
#include "board_inputs.h"
#include "uart_driver.h"
#include "servo.h"
#include "arm_trim_input.h"
int main(void)
{
    Board_Init();
    JY61_Init();
    if (CAR_UART_BRIDGE_TEST) {
        if (UART_Bridge_Init(&huart1, &huart5) != HAL_OK) Error_Handler();
        for (;;) { UART_Bridge_Process(); USART2_DMA_Process(); }
    }
    /* Arm USART6/USART4 reception before any cosmetic work, so no control or
     * vision byte can be lost during a slow OLED bring-up. */
    Bluetooth_Init();
    MaxiCam_Init();
    if (CAR_USART2_U1_BRIDGE && UART_Bridge_InitDMA(&huart1, &huart2) != HAL_OK) Error_Handler();
    Motor_Init();
    Car_Control_Init();
    PID_Tuner_Init();
#if ARM_TRIM_ENABLE
    ArmTrimInput_Init();
#endif
    /* Cosmetic only; a missing panel must never delay the control link. */
    (void)Board_InitDisplay();
    for (;;) {
        JY61_Process();
        if (CAR_USART2_U1_BRIDGE) {
            static uint8_t data[USART2_RX_BUFFER_SIZE];
            size_t size;
            (void)UART_RECV(data, sizeof(data), &huart2, &size, NULL);
            UART_Bridge_FeedDMA(data, (uint16_t)size);
            UART_Bridge_Process();
        }
        Bluetooth_Process();
        MaxiCam_Process();
        BoardInputs_ProcessControl();
        Car_Control_Process(); /* Safety cancels pending speed BEFORE TX dispatch. */
        Motor_Process();
        Car_Control_Process(); /* React to motor reply/transport faults this loop. */
        PID_Tuner_Process();
#if ARM_TRIM_ENABLE
        Bluetooth_DispatchServoActions(BoardInputs_TakeServoAimPress());
        /* Interlocks cancel pending arm motion before service TX dispatch. */
        ArmTrimInput_Process();
#else
        Servo_Process();
        Bluetooth_DispatchServoActions(BoardInputs_TakeServoAimPress());
#endif
        if (!CAR_USART2_U1_BRIDGE) Debug_Process();
        Board_Process();
    }
}
