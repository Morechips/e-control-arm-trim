#include "motor_driver.h"

const int8_t motor_sign[5] = {0, -1, +1, +1, -1};
const CarMotorMap_t car_motor_map[4] = {
    {MOTOR_RF, 1U, 'A'}, {MOTOR_LF, 2U, 'B'},
    {MOTOR_LR, 3U, 'A'}, {MOTOR_RR, 4U, 'B'}
};
int16_t Car_Motor_Limit(int16_t logical_rpm)
{
    int32_t value = logical_rpm;
    if (value > MOTOR_MAX_RPM) value = MOTOR_MAX_RPM;
    if (value < -MOTOR_MAX_RPM) value = -MOTOR_MAX_RPM;
    if (value >= -MOTOR_DEADZONE_RPM && value <= MOTOR_DEADZONE_RPM) value = 0;
    return (int16_t)value;
}
int16_t Car_Motor_Physical(uint8_t address, int16_t logical_rpm)
{
    if (address < 1U || address > 4U) return 0;
    return (int16_t)(Car_Motor_Limit(logical_rpm) * motor_sign[address]);
}
HAL_StatusTypeDef car_motor_set(CarMotor_t motor, int16_t logical_rpm)
{
    unsigned i;
    for (i = 0U; i < 4U; i++) {
        if (car_motor_map[i].motor == motor)
            return Motor_SetSpeed(car_motor_map[i].address, logical_rpm);
    }
    return HAL_ERROR;
}
/* Compatibility entry points are logical-wheel APIs. All queue, transport,
 * fault and stop preemption behavior remains in the existing protocol core. */
HAL_StatusTypeDef Motor_SetSpeed(uint8_t addr, int16_t rpm)
{
    if (addr < 1U || addr > 4U) return HAL_ERROR;
    return Emm42_SetSpeed(addr, Car_Motor_Physical(addr, rpm));
}
HAL_StatusTypeDef Motor_SetSpeedSync4(int16_t a, int16_t b, int16_t c, int16_t d)
{
    return Emm42_SetSpeedSync4(Car_Motor_Physical(1U, a), Car_Motor_Physical(2U, b),
                              Car_Motor_Physical(3U, c), Car_Motor_Physical(4U, d));
}
void Motor_Init(void) { Emm42_Init(); }
HAL_StatusTypeDef Motor_Enable(uint8_t addr) { return Emm42_Enable(addr); }
HAL_StatusTypeDef Motor_Disable(uint8_t addr) { return Emm42_Disable(addr); }
HAL_StatusTypeDef Motor_EnableAll(void) { return Emm42_EnableAll(); }
HAL_StatusTypeDef Motor_DisableAll(void) { return Emm42_DisableAll(); }
HAL_StatusTypeDef Motor_EStop(uint8_t addr) { return Emm42_EStop(addr); }
HAL_StatusTypeDef Motor_EStopAll(void) { return Emm42_EStopAll(); }
void Motor_Process(void) { Emm42_Process(); }
void Motor_ProcessRx(const uint8_t *data, uint16_t length) { Emm42_ProcessRx(data, length); }
const MotorStatus_t *Motor_GetLastStatus(uint8_t addr) { return Emm42_GetLastStatus(addr); }
uint8_t Motor_HasFault(void) { return Emm42_HasFault(); }
void Motor_ClearFault(void) { Emm42_ClearFault(); }
uint8_t Motor_IsIdle(void) { return Emm42_IsIdle(); }
void Motor_RxCallback(UART_HandleTypeDef *uart) { Emm42_RxCallback(uart); }
void Motor_TxCallback(UART_HandleTypeDef *uart) { Emm42_TxCallback(uart); }
void Motor_ErrorCallback(UART_HandleTypeDef *uart) { Emm42_ErrorCallback(uart); }
