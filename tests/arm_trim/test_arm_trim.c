#include "arm_trim_project.h"
#include "arm_trim.h"
#include "arm_trim_project_config.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(c) do { ++checks; if (!(c)) { fprintf(stderr, "FAIL %u: %s\n", (unsigned)__LINE__, #c); exit(1); } } while (0)
static unsigned checks, sends, stops;
static uint32_t tick;
static bool send_ok = true, stop_ok = true;
static ArmTrim_t trim;
static float lateral;
static uint32_t send_ticks[1024];
static uint16_t send_times[1024];
static bool Send(void *user, const uint16_t p[3], uint16_t ms)
{
    unsigned i;
    CHECK(user == &trim && ms > 0U && ms <= 9999U);
    for (i = 0U; i < 3U; ++i)
        CHECK(p[i] >= trim.config.calibration[i].min_position &&
              p[i] <= trim.config.calibration[i].max_position);
    CHECK(sends < 1024U);
    send_ticks[sends] = tick;
    send_times[sends] = ms;
    ++sends;
    tick += 3U; /* Physical UART callback time counts before the move timer. */
    return send_ok;
}
static bool Stop(void *user, unsigned joint)
{ CHECK(user == &trim && joint < 3U); ++stops; return stop_ok; }
static uint32_t Now(void *user) { CHECK(user == &trim); return tick; }
static void Lateral(void *user, float error) { CHECK(user == &trim); lateral = error; }
static ArmKinematicsResult_t ProjectForward(const uint16_t p[3], ArmPose2D_t *pose)
{
    ArmKinematicsGeometry_t geometry;
    ArmServoCalibration_t calibration[3];
    ArmJointAngles_t q;
    float *angle[] = {&q.q0_rad, &q.q1_rad, &q.q2_rad};
    unsigned i;
    ArmTrimProject_Geometry(&geometry);
    ArmTrimProject_Calibrations(calibration);
    for (i = 0U; i < 3U; ++i) {
        ArmKinematicsResult_t result = ArmKinematics_PositionToAngle(&calibration[i], p[i], angle[i]);
        if (result != ARM_KINEMATICS_OK) return result;
    }
    return ArmKinematics_Forward(&geometry, &q, pose);
}
static ArmTrimConfig_t Config(bool full_range)
{
    ArmTrimConfig_t c;
    ArmTrim_DefaultConfig(&c);
    ArmTrimProject_Geometry(&c.geometry);
    ArmTrimProject_Calibrations(c.calibration);
    c.calibration[0].max_position = ARM_TRIM_PROJECT_P0_MAX;
    if (full_range) { c.enabled_min_mm = -75.0f; c.enabled_max_mm = 75.0f; }
    return c;
}
static void Init(bool full_range)
{
    ArmTrimConfig_t c = Config(full_range);
    ArmTrimIO_t io = {Send, Stop, Now, Lateral, &trim};
    CHECK(ArmTrim_Init(&trim, &c, &io) == ARM_TRIM_OK);
    sends = stops = 0U;
    send_ok = stop_ok = true;
}
static void Finish(void)
{
    unsigned iterations;
    for (iterations = 0U; iterations < 20000U; ++iterations) {
        ArmTrim_Process(&trim);
        if (trim.status.state == ARM_TRIM_COMPLETE_ESTIMATED ||
            trim.status.state == ARM_TRIM_CANCELLED || trim.status.state == ARM_TRIM_FAULT) return;
        tick += 5U;
    }
    CHECK(false);
}
static void TestProfiles(void)
{
    const uint16_t refs[][3] = {{1356U,1850U,698U}, {1684U,2136U,785U}, {1566U,1896U,673U}};
    unsigned i;
    for (i = 0U; i < 3U; ++i) {
        ArmPose2D_t actual;
        ArmJointAngles_t q;
        float *angles[] = {&q.q0_rad, &q.q1_rad, &q.q2_rad};
        unsigned j;
        Init(false);
        CHECK(ArmTrim_Synchronize(&trim, refs[i]) == ARM_TRIM_OK);
        CHECK(sends == 0U && trim.status.reference_valid);
        CHECK(trim.status.model_min_mm < 0.0f && trim.status.model_max_mm > 0.0f);
        printf("profile %u model %.0f..%.0f enabled %.0f..%.0f mm\n", i,
               (double)trim.status.model_min_mm, (double)trim.status.model_max_mm,
               (double)trim.status.enabled_min_mm, (double)trim.status.enabled_max_mm);
        CHECK(ArmTrim_MoveRelativeX(&trim, 0.0f) == ARM_TRIM_OK && sends == 0U);
        CHECK(ArmTrim_MoveRelativeX(&trim, 2.0f) == ARM_TRIM_OK);
        CHECK(ArmTrim_MoveRelativeX(&trim, -2.0f) == ARM_TRIM_BUSY);
        CHECK(ArmTrim_Synchronize(&trim, refs[i]) == ARM_TRIM_BUSY);
        CHECK(ArmTrim_Exit(&trim) == ARM_TRIM_BUSY);
        CHECK(trim.status.offset_mm == 0.0f && sends == 0U);
        Finish();
        CHECK(trim.status.offset_mm == 2.0f && trim.status.state == ARM_TRIM_COMPLETE_ESTIMATED);
        for (j = 0U; j < 3U; ++j)
            CHECK(ArmKinematics_PositionToAngle(&trim.config.calibration[j], trim.status.estimated_position[j], angles[j]) == ARM_KINEMATICS_OK);
        CHECK(ArmKinematics_Forward(&trim.config.geometry, &q, &actual) == ARM_KINEMATICS_OK);
        CHECK(fabsf(actual.z_mm - trim.status.origin.z_mm) < 1.0f);
        CHECK(fabsf(actual.phi_rad - trim.status.origin.phi_rad) < 0.02f);
        CHECK(fabsf(actual.x_mm - trim.status.origin.x_mm - 2.0f) < 1.0f);
        CHECK(ArmTrim_MoveRelativeX(&trim, -4.0f) == ARM_TRIM_OK);
        Finish(); CHECK(trim.status.offset_mm == -2.0f);
        j = sends;
        CHECK(ArmTrim_MoveRelativeX(&trim, -75.0f) == ARM_TRIM_OUT_OF_RANGE);
        CHECK(sends == j && trim.status.offset_mm == -2.0f);
        ArmTrim_ReportLateral(&trim, 7.0f); CHECK(lateral == 7.0f);
        CHECK(ArmTrim_Exit(&trim) == ARM_TRIM_OK);
        CHECK(ArmTrim_MoveRelativeX(&trim, 1.0f) == ARM_TRIM_REFERENCE_REQUIRED);
    }
}
static void TestLongPathAndFailures(void)
{
    const uint16_t ref[] = {1356U,1850U,698U};
    unsigned before;
    Init(true);
    CHECK(ArmTrim_Synchronize(&trim, ref) == ARM_TRIM_OK);
    CHECK(ArmTrim_MoveRelativeX(&trim, -40.0f) == ARM_TRIM_OK);
    CHECK(trim.status.segment_count > 16U && sends == 0U);
    Finish(); CHECK(sends == 20U && trim.status.offset_mm == -40.0f);
    /* Strict interpolation fails late in preflight without any partial TX. */
    trim.config.interpolation_position_tolerance_mm = 0.000001f;
    before = sends;
    CHECK(ArmTrim_MoveRelativeX(&trim, 10.0f) == ARM_TRIM_PATH_INVALID);
    CHECK(sends == before && trim.status.offset_mm == -40.0f);
    Init(false); CHECK(ArmTrim_Synchronize(&trim, ref) == ARM_TRIM_OK);
    CHECK(ArmTrim_MoveRelativeX(&trim, -2.0f) == ARM_TRIM_OK);
    CHECK(ArmTrim_Cancel(&trim) == ARM_TRIM_OK);
    Finish(); CHECK(sends == 0U && stops == 3U && !trim.status.reference_valid);
    CHECK(ArmTrim_MoveRelativeX(&trim, 1.0f) == ARM_TRIM_REFERENCE_REQUIRED);
    CHECK(ArmTrim_Synchronize(&trim, ref) == ARM_TRIM_OK);
    CHECK(ArmTrim_MoveRelativeX(&trim, -2.0f) == ARM_TRIM_OK);
    send_ok = false; stop_ok = false;
    Finish(); CHECK(trim.status.state == ARM_TRIM_FAULT && trim.status.stop_failed && stops == 6U);
    CHECK(ArmTrim_Synchronize(&trim, ref) == ARM_TRIM_FAULT_LATCHED);
    CHECK(ArmTrim_Exit(&trim) == ARM_TRIM_FAULT_LATCHED);
    CHECK(ArmTrim_ClearFault(&trim) == ARM_TRIM_OK);
    CHECK(!trim.status.reference_valid);
    send_ok = stop_ok = true;
    CHECK(ArmTrim_Synchronize(&trim, ref) == ARM_TRIM_OK);
    CHECK(ArmTrim_MoveRelativeX(&trim, 1.0f) == ARM_TRIM_OK);
    tick += trim.config.service_timeout_ms + 1U;
    Finish(); CHECK(trim.status.error == ARM_TRIM_SERVICE_TIMEOUT && !trim.status.reference_valid);
    Init(false); CHECK(ArmTrim_Synchronize(&trim, ref) == ARM_TRIM_OK);
    tick = UINT32_MAX - 100U;
    CHECK(ArmTrim_MoveRelativeX(&trim, -2.0f) == ARM_TRIM_OK);
    Finish(); CHECK(trim.status.state == ARM_TRIM_COMPLETE_ESTIMATED);
    CHECK(ArmTrim_MoveRelativeX(&trim, NAN) == ARM_TRIM_INVALID);
}

static void TestHeldJog(void)
{
    const uint16_t refs[][3] = {{1356U,1850U,698U}, {1684U,2136U,785U}, {1566U,1896U,673U}};
    unsigned profile, i, before;
    for (profile = 0U; profile < 3U; ++profile) {
        int direction;
        for (direction = -1; direction <= 1; direction += 2) {
            float previous, bound;
            size_t count;
            Init(true); CHECK(ArmTrim_Synchronize(&trim, refs[profile]) == ARM_TRIM_OK);
            CHECK(ArmTrim_StartJog(&trim, direction) == ARM_TRIM_OK && sends == 0U);
            CHECK(trim.status.jogging && trim.status.jog_direction == direction);
            bound = direction > 0 ? trim.status.enabled_max_mm : trim.status.enabled_min_mm;
            CHECK(trim.status.target_offset_mm == bound);
            CHECK(ArmTrim_StartJog(&trim, direction) == ARM_TRIM_BUSY);
            previous = trim.status.offset_mm;
            count = trim.status.segment_count;
            for (i = 0U; i < count; ++i) {
                ArmPose2D_t pose;
                float distance = fabsf(trim.segments[i].offset_mm - previous);
                CHECK(trim.segments[i].move_ms <= trim.config.update_period_ms);
                CHECK(distance <= trim.config.speed_mm_s * (float)trim.segments[i].move_ms / 1000.0f + 0.001f);
                CHECK(trim.segments[i].end_speed_mm_s >= 0.0f && trim.segments[i].end_speed_mm_s <= trim.config.speed_mm_s + 0.001f);
                CHECK(ProjectForward(trim.segments[i].position, &pose) == ARM_KINEMATICS_OK);
                CHECK(fabsf(pose.z_mm - trim.status.origin.z_mm) < 1.0f);
                CHECK(fabsf(pose.phi_rad - trim.status.origin.phi_rad) < 0.02f);
                previous = trim.segments[i].offset_mm;
            }
            CHECK(previous == bound && trim.segments[count - 1U].end_speed_mm_s == 0.0f);
            for (i = 0U; i < 20000U && trim.status.state != ARM_TRIM_COMPLETE_ESTIMATED; ++i) {
                ArmTrim_Process(&trim); ++tick;
            }
            CHECK(trim.status.state == ARM_TRIM_COMPLETE_ESTIMATED && trim.status.reference_valid);
            CHECK(trim.status.offset_mm == bound && sends == count);
            /* Blocking UART time belongs inside the 50ms command cadence. */
            for (i = 1U; i < sends; ++i) CHECK(send_ticks[i] - send_ticks[i - 1U] == send_times[i - 1U]);
            before = sends;
            CHECK(ArmTrim_StartJog(&trim, direction) == ARM_TRIM_OUT_OF_RANGE && sends == before);
            CHECK(ArmTrim_StartJog(&trim, -direction) == ARM_TRIM_OK);
            CHECK(ArmTrim_ReleaseJog(&trim) == ARM_TRIM_OK);
            Finish(); CHECK(sends == before && trim.status.reference_valid && trim.status.offset_mm == bound);
        }
    }
    Init(true); CHECK(ArmTrim_Synchronize(&trim, refs[0]) == ARM_TRIM_OK);
    CHECK(ArmTrim_StartJog(&trim, -1) == ARM_TRIM_OK && trim.status.segment_count > 16U);
    for (i = 0U; i < 1200U; ++i) { ArmTrim_Process(&trim); ++tick; }
    {
        float release_offset = trim.status.offset_mm;
        CHECK(release_offset < -5.0f && trim.status.jogging);
        CHECK(ArmTrim_ReleaseJog(&trim) == ARM_TRIM_OK);
        Finish();
        CHECK(trim.status.state == ARM_TRIM_COMPLETE_ESTIMATED && trim.status.reference_valid);
        CHECK(trim.status.offset_mm < release_offset && trim.status.offset_mm >= release_offset - 3.001f);
        CHECK(!trim.status.jogging && trim.segments[trim.status.segment_count - 1U].end_speed_mm_s == 0.0f);
        CHECK(ArmTrim_StartJog(&trim, 1) == ARM_TRIM_OK);
        CHECK(ArmTrim_Cancel(&trim) == ARM_TRIM_OK);
        Finish(); CHECK(!trim.status.reference_valid && stops == 3U);
    }
    Init(true); CHECK(ArmTrim_Synchronize(&trim, refs[0]) == ARM_TRIM_OK);
    CHECK(ArmTrim_StartJog(&trim, -1) == ARM_TRIM_OK);
    ArmTrim_Process(&trim); tick += trim.config.update_period_ms + trim.config.dispatch_lateness_ms + 1U;
    Finish(); CHECK(trim.status.state == ARM_TRIM_FAULT && trim.status.error == ARM_TRIM_SERVICE_TIMEOUT);
    Init(true); CHECK(ArmTrim_Synchronize(&trim, refs[0]) == ARM_TRIM_OK);
    CHECK(ArmTrim_StartJog(&trim, -1) == ARM_TRIM_OK); send_ok = false;
    Finish(); CHECK(trim.status.state == ARM_TRIM_FAULT && !trim.status.reference_valid);
    Init(true); CHECK(ArmTrim_Synchronize(&trim, refs[0]) == ARM_TRIM_OK);
    trim.config.interpolation_position_tolerance_mm = 0.000001f;
    CHECK(ArmTrim_StartJog(&trim, -1) == ARM_TRIM_PATH_INVALID && sends == 0U);
    /* Release within a fractional final millisecond must finish normally,
     * rather than fault on a braking distance below floating-point resolution. */
    Init(true); trim.config.enabled_max_mm = 2.4501f;
    CHECK(ArmTrim_Synchronize(&trim, refs[0]) == ARM_TRIM_OK);
    CHECK(ArmTrim_StartJog(&trim, 1) == ARM_TRIM_OK);
    for (i = 0U; i < 1000U; ++i) {
        ArmTrim_Process(&trim); ++tick;
        if (trim.segment_sent && trim.status.segment_index + 2U == trim.status.segment_count) break;
    }
    CHECK(i < 1000U && ArmTrim_ReleaseJog(&trim) == ARM_TRIM_OK);
    Finish(); CHECK(trim.status.state == ARM_TRIM_COMPLETE_ESTIMATED && trim.status.reference_valid);
    CHECK(trim.status.offset_mm <= 2.4501f && trim.status.offset_mm > 2.449f);
    Init(true); CHECK(ArmTrim_Synchronize(&trim, refs[0]) == ARM_TRIM_OK);
    trim.config.speed_mm_s = 0.000001f;
    CHECK(ArmTrim_StartJog(&trim, -1) == ARM_TRIM_PATH_INVALID && sends == 0U);
}
static void TestRearBoxGuard(void)
{
    const uint16_t refs[][3] = {{1356U,1850U,698U}, {1684U,2136U,785U}, {1566U,1896U,673U}};
    const uint16_t lift[] = {1800U,1855U,528U};
    ArmTrimConfig_t c = Config(true);
    ArmTrimIO_t io = {Send, Stop, Now, Lateral, &trim};
    unsigned i;
    ArmTrimProject_Collision(&c.collision);
    CHECK(ArmTrim_Init(&trim, &c, &io) == ARM_TRIM_OK);
    sends = stops = 0U;
    CHECK(ArmTrim_Synchronize(&trim, lift) == ARM_TRIM_PATH_INVALID);
    CHECK(!trim.status.reference_valid && sends == 0U);
    for (i = 0U; i < 3U; ++i) {
        float pure_min, guarded_min;
        size_t j;
        Init(true);
        CHECK(ArmTrim_Synchronize(&trim, refs[i]) == ARM_TRIM_OK);
        pure_min = trim.status.model_min_mm;
        CHECK(ArmTrim_Init(&trim, &c, &io) == ARM_TRIM_OK);
        sends = stops = 0U;
        CHECK(ArmTrim_Synchronize(&trim, refs[i]) == ARM_TRIM_OK);
        guarded_min = trim.status.model_min_mm;
        CHECK(guarded_min >= pure_min && sends == 0U);
        printf("guard profile %u %.0f..%.0f (pure minimum %.0f)\n", i,
               (double)guarded_min, (double)trim.status.model_max_mm, (double)pure_min);
        CHECK(ArmTrim_MoveRelativeX(&trim, 2.0f) == ARM_TRIM_OK);
        Finish();
        CHECK(ArmTrim_MoveRelativeX(&trim, -4.0f) == ARM_TRIM_OK);
        Finish();
        CHECK(ArmTrim_StartJog(&trim, -1) == ARM_TRIM_OK);
        for (j = 0U; j < trim.status.segment_count; ++j) {
            ArmJointAngles_t q;
            float *angles[] = {&q.q0_rad, &q.q1_rad, &q.q2_rad};
            unsigned k;
            for (k = 0U; k < 3U; ++k)
                CHECK(ArmKinematics_PositionToAngle(&c.calibration[k],
                    trim.segments[j].position[k], angles[k]) == ARM_KINEMATICS_OK);
            CHECK(ArmCollision_CheckPose(&c.collision, &c.geometry, &q) == ARM_COLLISION_CLEAR);
        }
        Finish();
        CHECK(trim.status.offset_mm == guarded_min);
        j = sends;
        CHECK(ArmTrim_MoveRelativeX(&trim, -1.0f) == ARM_TRIM_OUT_OF_RANGE);
        CHECK(sends == j);
    }
    /* Valid pose with only an obstacle on the requested positive path.
     * A two-mm request is rejected in full before any transport call. */
    c = Config(true);
    ArmTrimProject_Collision(&c.collision);
    {
        ArmPose2D_t origin;
        CHECK(ProjectForward(refs[0], &origin) == ARM_KINEMATICS_OK);
        c.collision.box_min[0] = origin.x_mm + 1.5f;
        c.collision.box_max[0] = origin.x_mm + 2.5f;
        c.collision.box_min[2] = origin.z_mm - 0.1f;
        c.collision.box_max[2] = origin.z_mm + 0.1f;
        c.collision.radius_mm[0] = c.collision.radius_mm[1] = c.collision.radius_mm[2] = 0.0f;
        c.collision.clearance_mm = 0.0f;
    }
    CHECK(ArmTrim_Init(&trim, &c, &io) == ARM_TRIM_OK);
    sends = 0U;
    CHECK(ArmTrim_Synchronize(&trim, refs[0]) == ARM_TRIM_OK);
    CHECK(ArmTrim_MoveRelativeX(&trim, 2.0f) == ARM_TRIM_OUT_OF_RANGE);
    CHECK(sends == 0U);
    c.collision.radius_mm[0] = -1.0f;
    CHECK(ArmTrim_Init(&trim, &c, &io) == ARM_TRIM_INVALID);
}

static void TestReplacementReference(void)
{
    const uint16_t good[] = {1356U, 1850U, 698U};
    const uint16_t outside[] = {1801U, 1850U, 698U};
    const uint16_t singular[] = {1356U, 1687U, 698U};
    ArmTrimConfig_t project;
    ArmTrimProject_DefaultConfig(&project);
    CHECK(fabsf(project.geometry.link_2_mm - 84.75f) < 0.001f);
    CHECK(project.calibration[0].min_position == 915U && project.calibration[0].max_position == 1800U);
    CHECK(project.calibration[1].min_position == 821U && project.calibration[2].max_position == 1874U);
    CHECK(project.collision.enabled && project.enabled_min_mm == -75.0f && project.enabled_max_mm == 75.0f);
    Init(true);
    CHECK(ArmTrim_Synchronize(&trim, good) == ARM_TRIM_OK);
    CHECK(ArmTrim_Synchronize(&trim, outside) == ARM_TRIM_OUT_OF_RANGE);
    CHECK(!trim.status.reference_valid);
    CHECK(ArmTrim_MoveRelativeX(&trim, 1.0f) == ARM_TRIM_REFERENCE_REQUIRED && sends == 0U);
    CHECK(ArmTrim_Synchronize(&trim, good) == ARM_TRIM_OK);
    CHECK(ArmTrim_Synchronize(&trim, singular) == ARM_TRIM_PATH_INVALID);
    CHECK(!trim.status.reference_valid && ArmTrim_StartJog(&trim, 1) == ARM_TRIM_REFERENCE_REQUIRED);
    CHECK(ArmTrim_Synchronize(&trim, good) == ARM_TRIM_OK);
    CHECK(ArmTrim_Synchronize(&trim, NULL) == ARM_TRIM_INVALID && !trim.status.reference_valid);
}

int main(void)
{
    TestProfiles();
    TestLongPathAndFailures();
    TestRearBoxGuard();
    TestHeldJog();
    TestReplacementReference();
    printf("arm trim: %u checks passed\n", checks);
    return 0;
}
