#include "heading_control.h"
#include "serial_io.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

static HeadingStatus h;
static HeadingPIDParameters pid;
static float integral_output;
static uint32_t control_ms, log_ms;
static bool allowed_last;

static float Abs(float value) { return value < 0.0f ? -value : value; }

static float Normalize(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static void LogValue(const char *name, float value)
{
    char line[80];
    int32_t hundredths=(int32_t)(value*100.0f);
    uint32_t magnitude=(uint32_t)(hundredths<0 ? -hundredths : hundredths);
    (void)snprintf(line,sizeof(line),"%s=%s%lu.%02lu\r\n",name,
        hundredths<0 ? "-" : "",(unsigned long)(magnitude/100U),(unsigned long)(magnitude%100U));
    Debug_Log(line);
}

static void ClearOutput(void)
{
    integral_output=0.0f;
    h.omega_correction=0.0f;
    h.omega_final=0;
}

static void CaptureReference(const JY61_Data *data)
{
    h.yaw_zero=data->yaw;
    h.target=0.0f;
    h.actual=0.0f;
    h.yaw_error=0.0f;
    h.reference_valid=true;
    h.heading_hold=true;
    h.angle_adjusting=false;
    h.state=HEADING_HOLD;
    h.fault=HEADING_NO_FAULT;
    ClearOutput();
}

void Heading_Init(void)
{
    memset(&h,0,sizeof(h));
    pid.kp=HEADING_KP;
    pid.ki=HEADING_KI;
    pid.kd=HEADING_KD;
    integral_output=0.0f;
    control_ms=log_ms=HAL_GetTick();
    allowed_last=false;
    h.state=HEADING_WAIT_REFERENCE;
    h.fault=HEADING_NO_REFERENCE;
}

void Heading_ClearReference(void)
{
    ClearOutput();
    h.reference_valid=false;
    h.heading_hold=false;
    h.angle_adjusting=false;
    h.yaw_zero=0.0f;
    h.target=0.0f;
    h.actual=0.0f;
    h.yaw_error=0.0f;
    h.state=HEADING_WAIT_REFERENCE;
    h.fault=HEADING_NO_REFERENCE;
    allowed_last=false;
}

const HeadingStatus *Heading_GetStatus(void) { return &h; }
const HeadingPIDParameters *Heading_GetPID(void) { return &pid; }
float Heading_GetIntegralOutput(void) { return integral_output; }

bool Heading_SetPID(float kp, float ki, float kd)
{
    if (!isfinite(kp) || !isfinite(ki) || !isfinite(kd) ||
        kp<0.0f || ki<0.0f || kd<0.0f) return false;
    if (ki!=pid.ki) integral_output=0.0f;
    pid.kp=kp;
    pid.ki=ki;
    pid.kd=kd;
    return true;
}

bool Heading_RequestReference(void)
{
    const JY61_Data *data=JY61_GetData();
    if (!allowed_last || !JY61_IsValid()) return false;
    CaptureReference(data);
    return true;
}

bool Heading_SetTarget(float target_degrees)
{
    if (!isfinite(target_degrees) || !allowed_last || !h.reference_valid || !JY61_IsValid())
        return false;
    h.target=Normalize(target_degrees);
    h.angle_adjusting=true;
    h.heading_hold=false;
    integral_output=0.0f;
    return true;
}

bool Heading_AdjustTarget(float delta_degrees)
{
    if (!isfinite(delta_degrees)) return false;
    return Heading_SetTarget(h.target+delta_degrees);
}

void Heading_Suspend(void)
{
    ClearOutput();
    allowed_last = false;
    control_ms = HAL_GetTick();
    h.heading_hold = false;
    h.fault = HEADING_INHIBITED;
}

bool Heading_SetReference(float raw_yaw, float target_degrees)
{
    if (!isfinite(raw_yaw) || !isfinite(target_degrees)) return false;
    Heading_Suspend();
    h.yaw_zero = Normalize(raw_yaw);
    h.target = Normalize(target_degrees);
    h.reference_valid = true;
    h.angle_adjusting = false;
    h.state = HEADING_HOLD;
    return true;
}

int16_t Heading_Update(bool motion_allowed)
{
    return Heading_UpdateWithLimits(motion_allowed, HEADING_START_DEG,
                                   HEADING_STOP_DEG, MAX_YAW_CORRECTION_RPM);
}

int16_t Heading_UpdateWithLimits(bool motion_allowed, float start_degrees,
                                float stop_degrees, float max_rpm)
{
    const JY61_Data *data=JY61_GetData();
    uint32_t now=HAL_GetTick();
    uint32_t elapsed=now-control_ms;
    float error_magnitude;
    if (!isfinite(start_degrees) || !isfinite(stop_degrees) || !isfinite(max_rpm) ||
        stop_degrees < 0.0f || start_degrees <= stop_degrees || max_rpm <= 0.0f) {
        Heading_Suspend();
        return 0;
    }
    control_ms=now;
    allowed_last=motion_allowed;
    h.omega_correction=0.0f;
    h.omega_final=0;

    if (!motion_allowed) {
        ClearOutput();
        if (h.reference_valid && data->valid) {
            h.actual=Normalize(data->yaw-h.yaw_zero);
            h.yaw_error=Normalize(h.target-h.actual);
        }
        h.heading_hold=false;
        h.fault=HEADING_INHIBITED;
        return 0;
    }
    if (!JY61_IsValid()) {
        integral_output=0.0f;
        h.heading_hold=false;
        h.state=h.reference_valid ? HEADING_FAULT : HEADING_WAIT_REFERENCE;
        h.fault=HEADING_IMU_LOST;
        return 0;
    }
    if (!h.reference_valid) CaptureReference(data);

    h.actual=Normalize(data->yaw-h.yaw_zero);
    h.yaw_error=Normalize(h.target-h.actual);
    error_magnitude=Abs(h.yaw_error);
    h.fault=HEADING_NO_FAULT;

    if (error_magnitude<=stop_degrees) {
        integral_output=0.0f;
        h.heading_hold=true;
        h.angle_adjusting=false;
        h.state=HEADING_HOLD;
        return 0;
    }
    if (!h.angle_adjusting && h.state==HEADING_HOLD && error_magnitude<start_degrees) {
        h.heading_hold=true;
        return 0;
    }

    h.heading_hold=false;
    h.state=h.angle_adjusting ? HEADING_ANGLE_ADJUST : HEADING_CORRECTING;
    {
        double correction;
        float pd_single=pid.kp*h.yaw_error-pid.kd*data->gz;
        double pd=isfinite(pd_single) ? (double)pd_single :
                  (double)pid.kp*h.yaw_error-(double)pid.kd*data->gz;
        if (pid.ki==0.0f || elapsed>JY61_TIMEOUT_MS) integral_output=0.0f;
        else if (elapsed!=0U) {
            double delta=(double)pid.ki*h.yaw_error*((double)elapsed/1000.0);
            double candidate=(double)integral_output+delta;
            if (candidate>max_rpm) candidate=max_rpm;
            if (candidate< -max_rpm) candidate=-max_rpm;
            if (!((pd+candidate>max_rpm && delta>0.0) ||
                  (pd+candidate< -max_rpm && delta<0.0)))
                integral_output=(float)candidate;
        }
        correction=HEADING_CORRECTION_SIGN*(pd+integral_output);
        if (correction>max_rpm) correction=max_rpm;
        if (correction< -max_rpm) correction=-max_rpm;
        h.omega_correction=(float)correction;
        h.omega_final=(int16_t)correction;
    }
    return h.omega_final;
}

void Heading_Log(int16_t vx, int16_t vy, int16_t applied_omega)
{
    static const char *const names[]={"WAIT_REFERENCE","HOLD","CORRECTING","ANGLE_ADJUST","FAULT"};
    const JY61_Data *data=JY61_GetData();
    char line[96];
    if ((uint32_t)(HAL_GetTick()-log_ms)<HEADING_LOG_PERIOD_MS || !Debug_CanLog(12U)) return;
    log_ms=HAL_GetTick();
    LogValue("JY61 RAW_YAW",data->yaw);
    LogValue("JY61 GYRO_Z",data->gz);
    LogValue("YAW_ZERO",h.yaw_zero);
    LogValue("TARGET_YAW",h.target);
    LogValue("CURRENT_YAW",h.actual);
    (void)snprintf(line,sizeof(line),"HEADING_STATE=%s IMU_VALID=%u REF_VALID=%u FAULT=%u\r\n",
        names[h.state],(unsigned)data->valid,(unsigned)h.reference_valid,(unsigned)h.fault);
    Debug_Log(line);
    LogValue("YAW_ERROR",h.yaw_error);
    LogValue("YAW_CORRECTION",h.omega_correction);
    LogValue("OMEGA_APPLIED",(float)applied_omega);
    LogValue("VX",(float)vx);
    LogValue("VY",(float)vy);
}
