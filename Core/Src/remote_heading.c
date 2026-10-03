#include "remote_heading.h"
#include "serial_io.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static RemoteHeadingStatus_t status;
static uint32_t sample_sequence, log_tick, severe_start_tick, severe_last_tick;
static uint8_t severe_samples;
static bool have_sample, last_translating;

void RemoteHeading_Reset(void)
{
    memset(&status, 0, sizeof(status));
    status.phase = REMOTE_WAIT_REFERENCE;
    have_sample = false;
    last_translating = false;
    severe_samples = 0U;
    Heading_ClearReference();
}

void RemoteHeading_Suspend(void)
{
    if (status.phase != REMOTE_SUSPENDED) Heading_Suspend();
    status.phase = REMOTE_SUSPENDED;
    status.correction_rpm = 0;
    have_sample = false;
    severe_samples = 0U;
}

void RemoteHeading_BeginTurn(void)
{
    Heading_Suspend();
    status.phase = REMOTE_TURNING;
    status.correction_rpm = 0;
    severe_samples = 0U;
}

void RemoteHeading_CompleteTurn(int8_t clockwise_quarters)
{
    if (status.phase != REMOTE_TURNING || !status.reference_valid ||
        (clockwise_quarters != -1 && clockwise_quarters != 1 && clockwise_quarters != 2)) return;
    int next = ((int)status.direction + clockwise_quarters + 4) % 4;
    status.direction = (RemoteDirection_t)next;
    /* Installed yaw decreases clockwise. All four targets share one origin. */
    status.target = next == 3 ? 90.0f : -90.0f * (float)next;
    (void)Heading_SetReference(status.reference_yaw, status.target);
    status.phase = REMOTE_WAIT_REFERENCE;
    status.correction_rpm = 0;
    have_sample = false;
    severe_samples = 0U;
}

const RemoteHeadingStatus_t *RemoteHeading_GetStatus(void) { return &status; }

void RemoteHeading_Update(bool translating)
{
    const JY61_Data *data = JY61_GetData();
    const HeadingStatus *heading;
    float rpm, start_degrees, stop_degrees;
    int16_t max_rpm;
    bool new_sample, mode_changed;
    status.imu_valid = data->valid && isfinite(data->yaw) && isfinite(data->gz);
    if (status.phase == REMOTE_TURNING) return;
    if (!status.imu_valid) {
        if (status.phase != REMOTE_NO_IMU) Heading_Suspend();
        status.phase = REMOTE_NO_IMU;
        status.correction_rpm = 0;
        have_sample = false;
        severe_samples = 0U;
        return;
    }
    new_sample = !have_sample || data->yaw_sequence != sample_sequence;
    mode_changed = translating != last_translating;
    if (!new_sample && !mode_changed) return;
    /* A mode change may reuse a sample to apply its new limits, but never
     * integrates that sample twice or carries the old mode's integral. */
    if (!have_sample || mode_changed) Heading_Suspend();
    if (mode_changed) severe_samples = 0U;
    sample_sequence = data->yaw_sequence;
    have_sample = true;
    last_translating = translating;
    if (!status.reference_valid) {
        status.reference_yaw = data->yaw;
        status.reference_valid = true;
        (void)Heading_SetReference(status.reference_yaw, status.target);
    }
    start_degrees = translating ? REMOTE_MOVING_START_DEG : REMOTE_HEADING_START_DEG;
    stop_degrees = translating ? REMOTE_MOVING_STOP_DEG : REMOTE_HEADING_STOP_DEG;
    max_rpm = translating ? REMOTE_MOVING_MAX_RPM : REMOTE_HEADING_MAX_RPM;
    (void)Heading_UpdateWithLimits(true, start_degrees, stop_degrees, (float)max_rpm);
    heading = Heading_GetStatus();
    status.target = heading->target;
    status.actual = heading->actual;
    status.error = heading->yaw_error;
    if (heading->heading_hold) {
        status.phase = REMOTE_ALIGNED;
        status.correction_rpm = 0;
    } else {
        status.phase = translating ? REMOTE_DRIFTING : REMOTE_ALIGNING;
        rpm = heading->omega_correction;
        if (fabsf(rpm) < 1.0f)
            rpm = HEADING_CORRECTION_SIGN * status.error > 0.0f ? 1.0f : -1.0f;
        if (rpm > max_rpm) rpm = (float)max_rpm;
        if (rpm < -max_rpm) rpm = (float)-max_rpm;
        status.correction_rpm = (int16_t)rpm;
    }
    if (!translating || !new_sample) return;
    if (fabsf(status.error) >= REMOTE_MOVING_BRAKE_DEG) {
        if (severe_samples == 0U ||
            (uint32_t)(data->yaw_ms - severe_last_tick) > JY61_TIMEOUT_MS) {
            severe_start_tick = data->yaw_ms;
            severe_samples = 0U;
        }
        severe_last_tick = data->yaw_ms;
        if (severe_samples < UINT8_MAX) ++severe_samples;
        if (severe_samples >= REMOTE_MOVING_BRAKE_MIN_SAMPLES &&
            (uint32_t)(data->yaw_ms - severe_start_tick) >= REMOTE_MOVING_BRAKE_CONFIRM_MS)
            status.phase = REMOTE_ALIGNING;
    } else severe_samples = 0U;
}

void RemoteHeading_Log(void)
{
    static const char *const directions[] = {"FRONT", "RIGHT", "BACK", "LEFT"};
    static const char *const phases[] = {"WAIT", "ALIGNED", "ALIGNING", "DRIFTING", "TURN", "NO_IMU", "PAUSED"};
    char line[80];
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - log_tick) < 1000U || !Debug_CanLog(1U)) return;
    log_tick = now;
    (void)snprintf(line, sizeof(line), "[RHEAD] %s %s T100=%ld E100=%ld rpm=%d imu=%u\r\n",
                   directions[status.direction], phases[status.phase],
                   (long)(status.target * 100.0f), (long)(status.error * 100.0f),
                   status.correction_rpm, (unsigned)status.imu_valid);
    Debug_Log(line);
}
