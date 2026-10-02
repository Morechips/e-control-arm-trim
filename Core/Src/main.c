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
#include "servo_remote.h"
#include "arm_control.h"
#include "arm_tuner.h"
int main(void)
{
    Board_Init();
    JY61_Init();
    if (CAR_UART_BRIDGE_TEST) {
        if (UART_Bridge_Init(&huart1, &huart5) != HAL_OK) Error_Handler();
        for (;;) { UART_Bridge_Process(); USART2_DMA_Process(); }
    }
    Bluetooth_Init();
    MaxiCam_Init();
    if (CAR_USART2_U1_BRIDGE && UART_Bridge_InitDMA(&huart1, &huart2) != HAL_OK) Error_Handler();
    Motor_Init();
    Car_Control_Init();
    PID_Tuner_Init();
    Arm_Init();
    ArmTuner_Init();
    ServoRemote_Init();
    for (;;) {
        JY61_Process();
        if (CAR_USART2_U1_BRIDGE) {
            static uint8_t data[USART2_RX_BUFFER_SIZE];
            uint16_t size = USART2_DMA_Read(data, sizeof(data));
            UART_Bridge_FeedDMA(data, size);
            UART_Bridge_Process();
        }
        Bluetooth_Process();
        MaxiCam_Process();
        Car_Control_Process(); /* Safety cancels pending speed BEFORE TX dispatch. */
        Motor_Process();
        Car_Control_Process(); /* React to motor reply/transport faults this loop. */
        PID_Tuner_Process();
        ArmTuner_Process();
        ServoRemote_Process();
        Arm_Process();
        if (!CAR_USART2_U1_BRIDGE) Debug_Process();
        Board_Process();
    }
}
