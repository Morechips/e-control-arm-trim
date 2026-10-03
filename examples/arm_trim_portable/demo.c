#include "arm_trim.h"
#include <math.h>
#include <stdio.h>

static ArmTrim_t trim;
static uint32_t tick;
static unsigned sends;
static bool Send(void *user, const uint16_t p[3], uint16_t ms)
{
    (void)user; (void)p; (void)ms;
    ++sends;
    return true; /* Dry run: replace with the target application's transport. */
}
static bool Stop(void *user, unsigned joint) { (void)user; return joint < 3U; }
static uint32_t Now(void *user) { (void)user; return tick; }

static int Failed(const char *stage, ArmTrimResult_t result)
{
    const ArmTrimStatus_t status = ArmTrim_GetStatus(&trim);
    fprintf(stderr, "%s failed: result=%u, state=%u, error=%u, reference=%u, "
            "model=[%.3f, %.3f] mm, enabled=[%.3f, %.3f] mm\n",
            stage, (unsigned)result, (unsigned)status.state, (unsigned)status.error,
            (unsigned)status.reference_valid, (double)status.model_min_mm,
            (double)status.model_max_mm, (double)status.enabled_min_mm,
            (double)status.enabled_max_mm);
    return 1;
}

int main(void)
{
    ArmTrimConfig_t config;
    const ArmTrimIO_t io = {Send, Stop, Now, NULL, NULL};
    const uint16_t reference[3] = {1200U, 1800U, 1600U};
    unsigned i;
    ArmTrimStatus_t status;
    ArmTrimResult_t result;
    ArmTrim_DefaultConfig(&config);
    /* Example measurements, independent of any robot project. These link
     * lengths admit a positive 1 mm range scan under the default joint limits. */
    config.geometry = (ArmKinematicsGeometry_t){100.0f, 80.0f, 20.0f, 10.0f, 0.0f};
    for (i = 0U; i < 3U; ++i)
        config.calibration[i] = (ArmServoCalibration_t){500U, 2500U, 1000U, 2000U, 0.0f, 1.5707963268f};
    result = ArmTrim_Init(&trim, &config, &io);
    if (result != ARM_TRIM_OK) return Failed("Init", result);
    result = ArmTrim_Synchronize(&trim, reference);
    if (result != ARM_TRIM_OK) return Failed("Synchronize", result);
    result = ArmTrim_MoveRelativeX(&trim, 0.5f);
    if (result != ARM_TRIM_OK) return Failed("MoveRelativeX(+0.5 mm)", result);
    for (i = 0U; i < 1000U; ++i) {
        ArmTrim_Process(&trim);
        status = ArmTrim_GetStatus(&trim);
        if (status.state == ARM_TRIM_COMPLETE_ESTIMATED) {
            if (!status.reference_valid || sends == 0U || fabsf(status.offset_mm - 0.5f) > 0.0001f)
                return Failed("Completed estimate", ARM_TRIM_PATH_INVALID);
            printf("Portable dry run passed: %u segments, offset %.3f mm, no HAL.\n", sends, (double)status.offset_mm);
            return 0;
        }
        if (status.state == ARM_TRIM_FAULT) return Failed("Process", status.error);
        tick += 10U;
    }
    return Failed("Process loop deadline", ARM_TRIM_SERVICE_TIMEOUT);
}
