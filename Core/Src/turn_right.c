#include "turn_right.h"
#include "turn_config.h"
#include "heading_control.h"
#include "serial_io.h"
#include "usart2_dma.h"
#include <math.h>
#include <stdio.h>

static TurnRightStatus_t status;
static float previous_yaw;
static uint32_t yaw_sequence, phase_ms, stable_ms, unstable_sequence, stable_yaw_sequence;
static uint32_t action_ms, progress_ms, progress_log_ms;
static uint32_t last_process_ms, max_process_gap_ms, max_yaw_age_ms;
static uint32_t last_observed_yaw_ms, max_observed_yaw_gap_ms;
static uint32_t yaw_sequence_start, unread_start, overflow_start, rx_error_start;
static bool stable_started, settled_logged;
static float requested_degrees, progress_checkpoint, leg_start_degrees;
static float leg_initial_remaining;
static int8_t turn_direction, motion_direction;
static int16_t requested_rpm;
static uint8_t correction_count, slow_requested;

static int16_t ApproachRPM(void)
{
    int16_t limit = requested_degrees > TURN_RIGHT_TARGET_DEG ?
        TURN_RIGHT_180_APPROACH_RPM : TURN_APPROACH_RPM;
    return requested_rpm < limit ? requested_rpm : limit;
}

static void BeginLeg(uint32_t now, int8_t direction)
{
    motion_direction = direction;
    leg_start_degrees = status.turned_degrees;
    leg_initial_remaining = direction * (requested_degrees - status.turned_degrees);
    progress_checkpoint = 0.0f;
    progress_ms = progress_log_ms = phase_ms = now;
}

static void LogReceiveStats(uint32_t now)
{
    char line[80];
    uint32_t duration_ms = (uint32_t)(now - action_ms);
    uint32_t yaw_hz = duration_ms == 0U ? 0U :
        (uint32_t)(((uint64_t)(yaw_sequence - yaw_sequence_start) * 1000U) / duration_ms);
    (void)snprintf(line, sizeof(line),
                   "[TURN] RX hz=%lu yaw_gap=%lu loop_gap=%lu age=%lu\r\n",
                   (unsigned long)yaw_hz, (unsigned long)max_observed_yaw_gap_ms,
                   (unsigned long)max_process_gap_ms, (unsigned long)max_yaw_age_ms);
    Debug_Log(line);
    (void)snprintf(line, sizeof(line),
                   "[TURN] DMA poll=%u unread=%lu overflow=%lu error=%lu\r\n",
                   (unsigned)USART2_RX_PERIOD_MS,
                   (unsigned long)(usart2_rx_unread_batches - unread_start),
                   (unsigned long)(usart2_rx_overflows - overflow_start),
                   (unsigned long)(usart2_rx_errors - rx_error_start));
    Debug_Log(line);
}

static float StopLead(const JY61_Data *data, uint32_t now)
{
    int32_t skew_ms = (int32_t)(data->yaw_ms - data->gyro_ms);
    float toward_rate, lead;
    if ((uint32_t)(now - data->gyro_ms) > TURN_RATE_FRESH_MS ||
        skew_ms > (int32_t)TURN_RATE_FRESH_MS ||
        skew_ms < -(int32_t)TURN_RATE_FRESH_MS)
        return 0.0f;
    toward_rate = motion_direction * turn_direction * TURN_RIGHT_YAW_SIGN * data->gz;
    if (toward_rate <= 0.0f) return 0.0f;
    lead = toward_rate * ((float)TURN_STOP_LEAD_MS / 1000.0f);
    if (lead > TURN_STOP_LEAD_MAX_DEG) lead = TURN_STOP_LEAD_MAX_DEG;
    if (status.state == TURN_RIGHT_CORRECTING)
    {
        float correction_limit = leg_initial_remaining - TURN_CORRECTION_MIN_PROGRESS_DEG;
        if (correction_limit < 0.0f) correction_limit = 0.0f;
        if (lead > correction_limit) lead = correction_limit;
    }
    return lead;
}

static void LogStop(const char *reason, HAL_StatusTypeDef stop_result)
{
    char line[80];
    (void)snprintf(line, sizeof(line),
                   "[TURN] %s angle=%ld error=%u FE_QUEUE=%u correction=%u\r\n",
                   reason, (long)status.turned_degrees,
                   (unsigned)status.error, (unsigned)stop_result,
                   (unsigned)correction_count);
    Debug_Log(line);
}

static bool Active(void)
{
    return status.state == TURN_RIGHT_TURNING || status.state == TURN_RIGHT_STOPPING ||
           status.state == TURN_RIGHT_RESETTING || status.state == TURN_RIGHT_CORRECTING;
}

static void Fail(TurnRightError_t error)
{
    HAL_StatusTypeDef stop_result = brake();
    status.error = error;
    status.state = TURN_RIGHT_FAULT;
    LogStop("FAULT", stop_result);
    LogReceiveStats(HAL_GetTick());
}

static HAL_StatusTypeDef StartTurn(int16_t rpm, float degrees, int8_t direction)
{
    const JY61_Data *data = JY61_GetData();
    HAL_StatusTypeDef result;
    if (Active()) return HAL_BUSY;
    int16_t rpm_limit = degrees > TURN_RIGHT_TARGET_DEG ? TURN_RIGHT_180_RPM :
                        direction > 0 ? TURN_RIGHT_90_RPM : TURN_LEFT_90_RPM;
    if (rpm <= 0) { Debug_Log("[TURN] REJECT invalid_rpm\r\n"); return HAL_ERROR; }
    if (!data->valid) { Debug_Log("[TURN] REJECT imu_invalid\r\n"); return HAL_ERROR; }
    if (!isfinite(data->yaw) || !isfinite(data->gz))
    {
        Debug_Log("[TURN] REJECT imu_nonfinite\r\n");
        return HAL_ERROR;
    }
    if (Motor_HasFault()) { Debug_Log("[TURN] REJECT motor_fault\r\n"); return HAL_ERROR; }
    if (rpm > rpm_limit) rpm = rpm_limit;
    if (!Motor_IsIdle()) return HAL_BUSY;
    result = mecanum_drive(0, 0, (int16_t)(direction * rpm));
    if (result == HAL_ERROR || result == HAL_TIMEOUT)
        Debug_Log("[TURN] REJECT motor_command\r\n");
    if (result != HAL_OK) return result;
    previous_yaw = data->yaw;
    yaw_sequence = data->yaw_sequence;
    requested_degrees = degrees;
    requested_rpm = rpm;
    turn_direction = direction;
    correction_count = 0U;
    slow_requested = (uint8_t)(rpm <= ApproachRPM());
    status.turned_degrees = 0.0f;
    status.error = TURN_RIGHT_NO_ERROR;
    status.state = TURN_RIGHT_TURNING;
    action_ms = HAL_GetTick();
    last_process_ms = action_ms;
    max_process_gap_ms = max_yaw_age_ms = 0U;
    last_observed_yaw_ms = data->yaw_ms;
    max_observed_yaw_gap_ms = 0U;
    yaw_sequence_start = data->yaw_sequence;
    unread_start = usart2_rx_unread_batches;
    overflow_start = usart2_rx_overflows;
    rx_error_start = usart2_rx_errors;
    BeginLeg(action_ms, 1);
    stable_started = settled_logged = false;
    {
        char line[80];
        (void)snprintf(line, sizeof(line),
                       "[TURN] START dir=%d rpm=%d target=%ld yaw=%ld\r\n",
                       (int)direction, (int)rpm, (long)degrees, (long)data->yaw);
        Debug_Log(line);
    }
    return HAL_OK;
}

HAL_StatusTypeDef right90(int16_t rpm) { return StartTurn(rpm, TURN_RIGHT_TARGET_DEG, 1); }
HAL_StatusTypeDef right180(int16_t rpm) { return StartTurn(rpm, 180.0f, 1); }
HAL_StatusTypeDef left90(int16_t rpm) { return StartTurn(rpm, TURN_RIGHT_TARGET_DEG, -1); }

void TurnRight_Cancel(void)
{
    HAL_StatusTypeDef stop_result;
    if (!Active()) return;
    stop_result = brake();
    status.state = TURN_RIGHT_CANCELLED;
    LogStop("CANCEL", stop_result);
    LogReceiveStats(HAL_GetTick());
}

const TurnRightStatus_t *TurnRight_GetStatus(void)
{
    return &status;
}

void TurnRight_Process(bool motion_allowed)
{
    const JY61_Data *data;
    uint32_t now;
    bool fresh_yaw = false;
    if (!Active()) return;
    if (!motion_allowed) { TurnRight_Cancel(); return; }
    if (Motor_HasFault()) { Fail(TURN_RIGHT_MOTOR_ERROR); return; }
    data = JY61_GetData();
    now = HAL_GetTick();
    if ((uint32_t)(now - last_process_ms) > max_process_gap_ms)
        max_process_gap_ms = (uint32_t)(now - last_process_ms);
    last_process_ms = now;
    if ((uint32_t)(now - action_ms) >= TURN_RIGHT_TIMEOUT_MS)
    {
        Fail(TURN_RIGHT_TIMEOUT);
        return;
    }

    /* Reset intentionally invalidates yaw until a new complete sample arrives. */
    if (status.state == TURN_RIGHT_RESETTING)
    {
        if ((uint32_t)(now - phase_ms) > JY61_RESET_TIMEOUT_MS)
        {
            Fail(TURN_RIGHT_RESET_ERROR);
            return;
        }
        if (data->valid && data->yaw_sequence != yaw_sequence &&
            (int32_t)(data->yaw_ms - phase_ms) > 0 &&
            isfinite(data->yaw) && isfinite(data->gz) &&
            fabsf(data->yaw) <= JY61_RESET_CONFIRM_DEG && fabsf(data->gz) < HEADING_GZ_STABLE)
        {
            /* Align the existing software reference too, preserving tuned PID. */
            (void)Heading_Update(true);
            if (!Heading_RequestReference()) { Fail(TURN_RIGHT_RESET_ERROR); return; }
            status.state = TURN_RIGHT_DONE;
        }
        return;
    }

    if (!data->valid || !isfinite(data->yaw) || !isfinite(data->gz))
    {
        Fail(TURN_RIGHT_IMU_ERROR);
        return;
    }
    /* Keep measuring after FE is queued: coast is part of the final angle. */
    if (data->yaw_sequence != yaw_sequence)
    {
        float delta = data->yaw - previous_yaw;
        if (delta > 180.0f) delta -= 360.0f;
        if (delta < -180.0f) delta += 360.0f;
        status.turned_degrees += turn_direction * TURN_RIGHT_YAW_SIGN * delta;
        previous_yaw = data->yaw;
        yaw_sequence = data->yaw_sequence;
        fresh_yaw = true;
        if ((uint32_t)(data->yaw_ms - last_observed_yaw_ms) > max_observed_yaw_gap_ms)
            max_observed_yaw_gap_ms = (uint32_t)(data->yaw_ms - last_observed_yaw_ms);
        last_observed_yaw_ms = data->yaw_ms;
        if ((uint32_t)(now - data->yaw_ms) > max_yaw_age_ms)
            max_yaw_age_ms = (uint32_t)(now - data->yaw_ms);
    }

    if (status.state == TURN_RIGHT_TURNING || status.state == TURN_RIGHT_CORRECTING)
    {
        float remaining = motion_direction * (requested_degrees - status.turned_degrees);
        float leg_progress = motion_direction * (status.turned_degrees - leg_start_degrees);
        float lead = StopLead(data, now);
        float stop_at = lead > TURN_TOLERANCE_DEG ? lead : TURN_TOLERANCE_DEG;
        if (leg_progress <= -TURN_WRONG_WAY_DEG)
        {
            Fail(TURN_RIGHT_WRONG_WAY);
            return;
        }
        if (remaining <= stop_at)
        {
            char line[80];
            HAL_StatusTypeDef stop_result = brake();
            if (stop_result != HAL_OK) { Fail(TURN_RIGHT_MOTOR_ERROR); return; }
            phase_ms = now;
            stable_started = settled_logged = false;
            status.state = TURN_RIGHT_STOPPING;
            LogStop("APPROACH_STOP", stop_result);
            (void)snprintf(line, sizeof(line),
                           "[TURN] STOP_DETAIL a10=%ld gz10=%ld lead10=%ld n=%u\r\n",
                           (long)(status.turned_degrees * 10.0f),
                           (long)(data->gz * 10.0f), (long)(lead * 10.0f),
                           (unsigned)correction_count);
            Debug_Log(line);
            return;
        }
        if (status.state == TURN_RIGHT_TURNING && !slow_requested &&
            remaining <= TURN_APPROACH_DEG && Motor_IsIdle())
        {
            HAL_StatusTypeDef result = mecanum_drive(0, 0,
                (int16_t)(turn_direction * ApproachRPM()));
            if (result == HAL_OK)
            {
                slow_requested = 1U;
                {
                    char line[80];
                    (void)snprintf(line, sizeof(line),
                                   "[TURN] SLOW_APPROACH angle=%ld rpm=%d\r\n",
                                   (long)status.turned_degrees, (int)ApproachRPM());
                    Debug_Log(line);
                }
            }
            else if (result != HAL_BUSY) { Fail(TURN_RIGHT_MOTOR_ERROR); return; }
        }
        if (leg_progress >= progress_checkpoint + TURN_PROGRESS_DEG)
        {
            progress_checkpoint = leg_progress;
            progress_ms = now;
        }
        if (fresh_yaw && (uint32_t)(now - progress_log_ms) >= HEADING_LOG_PERIOD_MS &&
            Debug_CanLog(1U))
        {
            char line[64];
            (void)snprintf(line, sizeof(line), "[TURN] progress=%ld/%ld\r\n",
                           (long)status.turned_degrees, (long)requested_degrees);
            Debug_Log(line);
            progress_log_ms = now;
        }
        if ((uint32_t)(now - progress_ms) >= TURN_PROGRESS_TIMEOUT_MS)
            Fail(TURN_RIGHT_NO_PROGRESS);
        return;
    }

    if ((uint32_t)(now - phase_ms) >= TURN_STOP_TIMEOUT_MS)
    {
        Fail(TURN_RIGHT_TIMEOUT);
        return;
    }
    if (!Motor_IsIdle() || (int32_t)(data->gyro_ms - phase_ms) <= 0 ||
        fabsf(data->gz) >= HEADING_GZ_STABLE)
    {
        stable_started = settled_logged = false;
        return;
    }
    if (!stable_started || data->gyro_unstable_sequence != unstable_sequence)
    {
        stable_started = true;
        settled_logged = false;
        stable_ms = data->gyro_ms;
        unstable_sequence = data->gyro_unstable_sequence;
        stable_yaw_sequence = data->yaw_sequence;
        return;
    }
    /* Require both stable gyro acquisition time and a yaw sample after it began. */
    if ((uint32_t)(data->gyro_ms - stable_ms) >= TURN_STOP_STABLE_MS &&
        data->yaw_sequence != stable_yaw_sequence)
    {
        float error = requested_degrees - status.turned_degrees;
        char line[80];
        if (!settled_logged)
        {
            (void)snprintf(line, sizeof(line),
                           "[TURN] SETTLED a10=%ld err10=%ld correction=%u\r\n",
                           (long)(status.turned_degrees * 10.0f), (long)(error * 10.0f),
                           (unsigned)correction_count);
            Debug_Log(line);
            settled_logged = true;
        }
        if (fabsf(error) <= TURN_TOLERANCE_DEG)
        {
            LogReceiveStats(now);
            yaw_sequence = data->yaw_sequence;
            if (JY61_ResetHeading() != HAL_OK) { Fail(TURN_RIGHT_RESET_ERROR); return; }
            phase_ms = HAL_GetTick();
            status.state = TURN_RIGHT_RESETTING;
            return;
        }
        if (correction_count >= TURN_MAX_CORRECTIONS)
        {
            Fail(TURN_RIGHT_TOLERANCE_ERROR);
            return;
        }
        {
            int8_t correction_direction = error > 0.0f ? 1 : -1;
            HAL_StatusTypeDef result = mecanum_drive(0, 0,
                (int16_t)(turn_direction * correction_direction * ApproachRPM()));
            if (result == HAL_BUSY) return;
            if (result != HAL_OK) { Fail(TURN_RIGHT_MOTOR_ERROR); return; }
            correction_count++;
            BeginLeg(now, correction_direction);
            status.state = TURN_RIGHT_CORRECTING;
            (void)snprintf(line, sizeof(line),
                           "[TURN] CORRECT n=%u dir=%d rpm=%d angle=%ld\r\n",
                           (unsigned)correction_count, (int)correction_direction,
                           (int)ApproachRPM(), (long)status.turned_degrees);
            Debug_Log(line);
        }
    }
}
