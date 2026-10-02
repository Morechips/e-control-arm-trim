#include "mecanum.h"

void Mecanum_Calculate(int16_t forward, int16_t right,
                       int16_t heading_correction, MecanumWheels_t *out)
{
    int32_t raw[4], maximum = MOTOR_MAX_RPM;
    unsigned i;
    if (out == NULL) return;
    /* Logical wheel order: M1 RF, M2 LF, M3 LR, M4 RR. Positive right
     * uses the calibrated lateral polarity; motor installation signs remain
     * exclusively in motor_driver.c. */
    raw[0] = (int32_t)forward + right - heading_correction;
    raw[1] = (int32_t)forward - right + heading_correction;
    raw[2] = (int32_t)forward + right + heading_correction;
    raw[3] = (int32_t)forward - right - heading_correction;
    for (i = 0U; i < 4U; i++) {
        int32_t magnitude = raw[i] < 0 ? -raw[i] : raw[i];
        if (magnitude > maximum) maximum = magnitude;
    }
    /* One common scale preserves chassis direction under saturation.
     * int32_t covers all int16_t inputs and the product at max RPM=500. */
    for (i = 0U; i < 4U; i++) raw[i] = raw[i] * MOTOR_MAX_RPM / maximum;
    out->m1 = (int16_t)raw[0]; out->m2 = (int16_t)raw[1];
    out->m3 = (int16_t)raw[2]; out->m4 = (int16_t)raw[3];
}

HAL_StatusTypeDef mecanum_drive(int16_t forward, int16_t right,
                                int16_t heading_correction)
{
    MecanumWheels_t wheels;
    Mecanum_Calculate(forward, right, heading_correction, &wheels);
    return Motor_SetSpeedSync4(wheels.m1, wheels.m2, wheels.m3, wheels.m4);
}

HAL_StatusTypeDef up(int16_t rpm)
{
    if (rpm < 0) return HAL_ERROR;
    return mecanum_drive(rpm, 0, 0);
}

HAL_StatusTypeDef down(int16_t rpm)
{
    if (rpm < 0) return HAL_ERROR;
    return mecanum_drive((int16_t)-rpm, 0, 0);
}

HAL_StatusTypeDef left(int16_t rpm)
{
    if (rpm < 0) return HAL_ERROR;
    return mecanum_drive(0, (int16_t)-rpm, 0);
}

HAL_StatusTypeDef right(int16_t rpm)
{
    if (rpm < 0) return HAL_ERROR;
    return mecanum_drive(0, rpm, 0);
}

HAL_StatusTypeDef brake(void)
{
    return Motor_EStopAll();
}
