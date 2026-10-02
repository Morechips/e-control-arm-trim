#include "arm_trim.h"
#include <math.h>
#include <string.h>

#define TRIM_PI 3.14159265358979323846f

static bool Active(const ArmTrim_t *t)
{
    return t->status.state == ARM_TRIM_MOVING ||
           t->status.state == ARM_TRIM_SETTLING || t->status.state == ARM_TRIM_STOPPING;
}

void ArmTrim_DefaultConfig(ArmTrimConfig_t *c)
{
    if (c == NULL) return;
    memset(c, 0, sizeof(*c));
    c->search_mm = 75.0f;
    c->enabled_min_mm = -5.0f;
    c->enabled_max_mm = 5.0f;
    c->max_segment_mm = 2.0f;
    c->speed_mm_s = 10.0f;
    c->acceleration_mm_s2 = 20.0f;
    c->interpolation_position_tolerance_mm = 1.0f;
    c->interpolation_angle_tolerance_rad = TRIM_PI / 180.0f;
    c->min_elbow_sine = 0.05f;
    c->max_joint_delta = 80U;
    c->update_period_ms = 50U;
    c->dispatch_lateness_ms = 20U;
    c->settle_ms = 300U;
    c->service_timeout_ms = 250U;
}

ArmTrimResult_t ArmTrim_Init(ArmTrim_t *t, const ArmTrimConfig_t *c, const ArmTrimIO_t *io)
{
    ArmJointAngles_t zero = {0};
    ArmPose2D_t pose;
    unsigned i;
    float angle;
    if (t == NULL || c == NULL || io == NULL || io->send == NULL ||
        io->stop == NULL || io->now == NULL ||
        !ArmCollision_ModelValid(&c->collision) ||
        ArmKinematics_Forward(&c->geometry, &zero, &pose) != ARM_KINEMATICS_OK ||
        !isfinite(c->search_mm) || c->search_mm < 1.0f || c->search_mm > 75.0f ||
        !isfinite(c->enabled_min_mm) || !isfinite(c->enabled_max_mm) ||
        c->enabled_min_mm > 0.0f || c->enabled_max_mm < 0.0f ||
        c->enabled_min_mm < -c->search_mm || c->enabled_max_mm > c->search_mm ||
        !isfinite(c->max_segment_mm) || c->max_segment_mm < 0.1f || c->max_segment_mm > 2.0f ||
        !isfinite(c->speed_mm_s) || c->speed_mm_s <= 0.0f || c->speed_mm_s > 10.0f ||
        !isfinite(c->acceleration_mm_s2) || c->acceleration_mm_s2 <= 0.0f || c->acceleration_mm_s2 > 20.0f ||
        !isfinite(c->interpolation_position_tolerance_mm) || c->interpolation_position_tolerance_mm <= 0.0f ||
        !isfinite(c->interpolation_angle_tolerance_rad) || c->interpolation_angle_tolerance_rad <= 0.0f ||
        !isfinite(c->min_elbow_sine) || c->min_elbow_sine <= 0.0f || c->min_elbow_sine >= 1.0f ||
        c->max_joint_delta == 0U || c->settle_ms > 60000U ||
        c->update_period_ms < 20U || c->update_period_ms > 100U ||
        c->dispatch_lateness_ms >= c->update_period_ms ||
        c->service_timeout_ms == 0U || c->service_timeout_ms > 1000U)
        return ARM_TRIM_INVALID;
    for (i = 0U; i < 3U; ++i) {
        if (c->calibration[i].min_position < 500U || c->calibration[i].max_position > 2500U ||
            ArmKinematics_PositionToAngle(&c->calibration[i], c->calibration[i].position_a,
                                           &angle) != ARM_KINEMATICS_OK)
            return ARM_TRIM_INVALID;
    }
    memset(t, 0, sizeof(*t));
    t->config = *c;
    t->io = *io;
    t->configured = true;
    return ARM_TRIM_OK;
}

static ArmTrimResult_t Forward(const ArmTrim_t *t, const uint16_t p[3], ArmPose2D_t *pose,
                              ArmJointAngles_t *q)
{
    float *angles[] = {&q->q0_rad, &q->q1_rad, &q->q2_rad};
    unsigned i;
    for (i = 0U; i < 3U; ++i)
        if (ArmKinematics_PositionToAngle(&t->config.calibration[i], p[i], angles[i]) != ARM_KINEMATICS_OK)
            return ARM_TRIM_OUT_OF_RANGE;
    return ArmKinematics_Forward(&t->config.geometry, q, pose) == ARM_KINEMATICS_OK ?
           ARM_TRIM_OK : ARM_TRIM_INVALID;
}

static ArmTrimResult_t Solve(const ArmTrim_t *t, float offset, uint16_t p[3])
{
    ArmPose2D_t pose = t->status.origin;
    ArmJointAngles_t q;
    const float *angles[] = {&q.q0_rad, &q.q1_rad, &q.q2_rad};
    unsigned i;
    pose.x_mm += offset;
    if (ArmKinematics_Inverse(&t->config.geometry, &pose, t->branch, &q) != ARM_KINEMATICS_OK ||
        fabsf(sinf(q.q1_rad)) < t->config.min_elbow_sine)
        return ARM_TRIM_PATH_INVALID;
    for (i = 0U; i < 3U; ++i)
        if (ArmKinematics_AngleToPosition(&t->config.calibration[i], *angles[i], &p[i]) != ARM_KINEMATICS_OK)
            return ARM_TRIM_OUT_OF_RANGE;
    return ARM_TRIM_OK;
}

/* Check rounded endpoints, posture tolerance and configured collision model
 * over the controller's joint interpolation, before transmitting any point. */
static bool EdgeValid(const ArmTrim_t *t, const uint16_t from[3], const uint16_t to[3],
                      float offset_from, float offset_to)
{
    ArmJointAngles_t a, b, q;
    ArmPose2D_t pose;
    unsigned i, sample;
    if (Forward(t, from, &pose, &a) != ARM_TRIM_OK || Forward(t, to, &pose, &b) != ARM_TRIM_OK)
        return false;
    for (i = 0U; i < 3U; ++i) {
        int delta = (int)to[i] - (int)from[i];
        if (delta > (int)t->config.max_joint_delta || delta < -(int)t->config.max_joint_delta)
            return false;
    }
    if (ArmCollision_CheckEdge(&t->config.collision, &t->config.geometry, &a, &b) !=
        ARM_COLLISION_CLEAR) return false;
    for (sample = 0U; sample <= 8U; ++sample) {
        float u = (float)sample / 8.0f;
        float expected_x = t->status.origin.x_mm + offset_from + u * (offset_to - offset_from);
        q.q0_rad = a.q0_rad + u * (b.q0_rad - a.q0_rad);
        q.q1_rad = a.q1_rad + u * (b.q1_rad - a.q1_rad);
        q.q2_rad = a.q2_rad + u * (b.q2_rad - a.q2_rad);
        if (fabsf(sinf(q.q1_rad)) < t->config.min_elbow_sine ||
            ArmKinematics_Forward(&t->config.geometry, &q, &pose) != ARM_KINEMATICS_OK ||
            fabsf(pose.x_mm - expected_x) > t->config.interpolation_position_tolerance_mm ||
            fabsf(pose.z_mm - t->status.origin.z_mm) > t->config.interpolation_position_tolerance_mm ||
            fabsf(pose.phi_rad - t->status.origin.phi_rad) > t->config.interpolation_angle_tolerance_rad)
            return false;
    }
    return true;
}

ArmTrimResult_t ArmTrim_Synchronize(ArmTrim_t *t, const uint16_t p[3])
{
    ArmPose2D_t pose;
    ArmJointAngles_t q;
    uint16_t previous[3], candidate[3];
    int direction;
    if (t == NULL || !t->configured || p == NULL) return ARM_TRIM_INVALID;
    if (Active(t)) return ARM_TRIM_BUSY;
    if (t->status.state == ARM_TRIM_FAULT) return ARM_TRIM_FAULT_LATCHED;
    if (Forward(t, p, &pose, &q) != ARM_TRIM_OK) return ARM_TRIM_OUT_OF_RANGE;
    if (fabsf(sinf(q.q1_rad)) < t->config.min_elbow_sine) return ARM_TRIM_PATH_INVALID;
    memset(&t->status, 0, sizeof(t->status));
    t->status.origin = pose;
    t->branch = sinf(q.q1_rad) < 0.0f ? ARM_ELBOW_NEGATIVE : ARM_ELBOW_POSITIVE;
    if (Solve(t, 0.0f, candidate) != ARM_TRIM_OK || !EdgeValid(t, p, candidate, 0.0f, 0.0f))
        return ARM_TRIM_PATH_INVALID;
    memcpy(t->status.estimated_position, p, sizeof(t->status.estimated_position));
    memcpy(t->status.target_position, p, sizeof(t->status.target_position));
    /* A conservative contiguous range at 1 mm resolution, scanned from zero. */
    for (direction = -1; direction <= 1; direction += 2) {
        float offset = 0.0f, limit = 0.0f;
        memcpy(previous, p, sizeof(previous));
        while (fabsf(offset) + 1.0f <= t->config.search_mm) {
            float next = offset + (float)direction;
            if (Solve(t, next, candidate) != ARM_TRIM_OK || !EdgeValid(t, previous, candidate, offset, next)) break;
            memcpy(previous, candidate, sizeof(previous));
            offset = limit = next;
        }
        if (direction < 0) t->status.model_min_mm = limit;
        else t->status.model_max_mm = limit;
    }
    t->status.enabled_min_mm = fmaxf(t->status.model_min_mm, t->config.enabled_min_mm);
    t->status.enabled_max_mm = fminf(t->status.model_max_mm, t->config.enabled_max_mm);
    t->status.reference_valid = true;
    t->status.state = ARM_TRIM_READY;
    t->jog_release_requested = false;
    return ARM_TRIM_OK;
}

/* Time at distance s on a triangular/trapezoidal Cartesian speed profile. */
static float TimeAt(const ArmTrim_t *t, float distance, float s)
{
    float accel = t->config.acceleration_mm_s2;
    float peak = fminf(t->config.speed_mm_s, sqrtf(accel * distance));
    float ramp_distance = peak * peak / (2.0f * accel);
    float total = 2.0f * peak / accel + (distance - 2.0f * ramp_distance) / peak;
    if (s <= ramp_distance) return sqrtf(2.0f * s / accel);
    if (s >= distance - ramp_distance) return total - sqrtf(2.0f * fmaxf(0.0f, distance - s) / accel);
    return peak / accel + (s - ramp_distance) / peak;
}

ArmTrimResult_t ArmTrim_MoveRelativeX(ArmTrim_t *t, float dx)
{
    float destination, distance, last_time = 0.0f, last_offset;
    uint16_t previous[3];
    size_t count, index;
    if (t == NULL || !t->configured || !isfinite(dx)) return ARM_TRIM_INVALID;
    if (Active(t)) return ARM_TRIM_BUSY;
    if (t->status.state == ARM_TRIM_FAULT) return ARM_TRIM_FAULT_LATCHED;
    if (!t->status.reference_valid) return ARM_TRIM_REFERENCE_REQUIRED;
    if (dx == 0.0f) return ARM_TRIM_OK;
    destination = t->status.offset_mm + dx;
    if (destination < t->status.enabled_min_mm || destination > t->status.enabled_max_mm)
        return ARM_TRIM_OUT_OF_RANGE;
    distance = fabsf(dx);
    count = (size_t)ceilf(distance / t->config.max_segment_mm);
    if (count == 0U || count > ARM_TRIM_MAX_SEGMENTS) return ARM_TRIM_PATH_INVALID;
    memcpy(previous, t->status.estimated_position, sizeof(previous));
    last_offset = t->status.offset_mm;
    for (index = 0U; index < count; ++index) {
        float fraction = (float)(index + 1U) / (float)count;
        float offset = t->status.offset_mm + dx * fraction;
        float time = TimeAt(t, distance, distance * fraction);
        float ms = ceilf((time - last_time) * 1000.0f);
        ArmTrimResult_t result = Solve(t, offset, t->segments[index].position);
        if (result != ARM_TRIM_OK) return result;
        if (!EdgeValid(t, previous, t->segments[index].position, last_offset, offset) || ms > 9999.0f)
            return ARM_TRIM_PATH_INVALID;
        t->segments[index].move_ms = (uint16_t)fmaxf(1.0f, ms);
        t->segments[index].offset_mm = offset;
        t->segments[index].end_speed_mm_s = 0.0f;
        memcpy(previous, t->segments[index].position, sizeof(previous));
        last_time = time;
        last_offset = offset;
    }
    t->status.segment_count = count;
    t->status.segment_index = 0U;
    t->status.target_offset_mm = destination;
    memcpy(t->status.target_position, previous, sizeof(previous));
    t->status.state = ARM_TRIM_MOVING;
    t->status.error = ARM_TRIM_OK;
    t->status.jogging = false;
    t->status.jog_direction = 0;
    t->jog_release_requested = false;
    t->segment_sent = false;
    t->service_tick = t->io.now(t->io.user);
    return ARM_TRIM_OK;
}

/* Uniform-time samples of an acceleration/cruise/deceleration profile.  An
 * initial speed is used only for the normal release braking tail.  Planning
 * checks every rounded target and edge before any position is transmitted. */
static ArmTrimResult_t PlanTimed(ArmTrim_t *t, float destination, float initial_speed)
{
    float delta = destination - t->status.offset_mm, distance = fabsf(delta);
    float a = t->config.acceleration_mm_s2, peak, ramp, cruise, down, total;
    float up_distance, down_distance, previous_offset = t->status.offset_mm;
    uint16_t previous[3];
    uint32_t duration_ms, previous_ms = 0U;
    size_t count, i;
    int direction = delta > 0.0f ? 1 : -1;
    if (distance <= 0.00001f || initial_speed < 0.0f ||
        distance + 0.0001f < initial_speed * initial_speed / (2.0f * a))
        return ARM_TRIM_PATH_INVALID;
    /* Clamp only the admitted floating-point remainder of a braking tail. */
    initial_speed = fminf(initial_speed, sqrtf(2.0f * a * distance));
    peak = fminf(t->config.speed_mm_s, sqrtf(a * distance + initial_speed * initial_speed / 2.0f));
    if (peak + 0.0001f < initial_speed) return ARM_TRIM_PATH_INVALID;
    ramp = fmaxf(0.0f, (peak - initial_speed) / a);
    down = peak / a;
    up_distance = (peak * peak - initial_speed * initial_speed) / (2.0f * a);
    down_distance = peak * peak / (2.0f * a);
    cruise = fmaxf(0.0f, distance - up_distance - down_distance) / peak;
    total = ramp + cruise + down;
    if (!isfinite(total) || total <= 0.0f ||
        total * 1000.0f > (float)ARM_TRIM_MAX_SEGMENTS * (float)t->config.update_period_ms)
        return ARM_TRIM_PATH_INVALID;
    duration_ms = (uint32_t)ceilf(total * 1000.0f);
    count = (duration_ms + t->config.update_period_ms - 1U) / t->config.update_period_ms;
    if (count == 0U || count > ARM_TRIM_MAX_SEGMENTS) return ARM_TRIM_PATH_INVALID;
    memcpy(previous, t->status.estimated_position, sizeof(previous));
    for (i = 0U; i < count; ++i) {
        uint32_t at_ms = (uint32_t)(i + 1U) * t->config.update_period_ms;
        float at, s, speed, offset;
        ArmTrimResult_t result;
        if (at_ms > duration_ms) at_ms = duration_ms;
        at = (float)at_ms / 1000.0f;
        if (at >= total) { s = distance; speed = 0.0f; }
        else if (at <= ramp) {
            s = initial_speed * at + a * at * at / 2.0f;
            speed = initial_speed + a * at;
        } else if (at <= ramp + cruise) {
            s = up_distance + peak * (at - ramp); speed = peak;
        } else {
            float remaining = total - at;
            s = distance - a * remaining * remaining / 2.0f;
            speed = a * remaining;
        }
        offset = t->status.offset_mm + (float)direction * s;
        if (offset < t->status.enabled_min_mm - 0.0001f ||
            offset > t->status.enabled_max_mm + 0.0001f ||
            fabsf(offset - previous_offset) > t->config.max_segment_mm + 0.0001f)
            return ARM_TRIM_PATH_INVALID;
        result = Solve(t, offset, t->segments[i].position);
        if (result != ARM_TRIM_OK) return result;
        if (!EdgeValid(t, previous, t->segments[i].position, previous_offset, offset))
            return ARM_TRIM_PATH_INVALID;
        t->segments[i].move_ms = (uint16_t)(at_ms - previous_ms);
        t->segments[i].offset_mm = offset;
        t->segments[i].end_speed_mm_s = speed;
        memcpy(previous, t->segments[i].position, sizeof(previous));
        previous_offset = offset;
        previous_ms = at_ms;
    }
    t->status.segment_index = 0U;
    t->status.segment_count = count;
    t->status.target_offset_mm = destination;
    memcpy(t->status.target_position, previous, sizeof(previous));
    t->status.state = ARM_TRIM_MOVING;
    t->status.error = ARM_TRIM_OK;
    t->segment_sent = false;
    t->service_tick = t->io.now(t->io.user);
    return ARM_TRIM_OK;
}

ArmTrimResult_t ArmTrim_StartJog(ArmTrim_t *t, int direction)
{
    ArmTrimResult_t result;
    float destination;
    if (t == NULL || !t->configured || (direction != 1 && direction != -1)) return ARM_TRIM_INVALID;
    if (Active(t)) return ARM_TRIM_BUSY;
    if (t->status.state == ARM_TRIM_FAULT) return ARM_TRIM_FAULT_LATCHED;
    if (!t->status.reference_valid) return ARM_TRIM_REFERENCE_REQUIRED;
    destination = direction > 0 ? t->status.enabled_max_mm : t->status.enabled_min_mm;
    if (fabsf(destination - t->status.offset_mm) < 0.0001f) return ARM_TRIM_OUT_OF_RANGE;
    result = PlanTimed(t, destination, 0.0f);
    if (result == ARM_TRIM_OK) {
        t->status.jogging = true;
        t->status.jog_direction = (int8_t)direction;
        t->jog_release_requested = false;
    }
    return result;
}

ArmTrimResult_t ArmTrim_ReleaseJog(ArmTrim_t *t)
{
    if (t == NULL || !t->configured) return ARM_TRIM_INVALID;
    if (t->status.jogging && t->status.state == ARM_TRIM_MOVING)
        t->jog_release_requested = true;
    return ARM_TRIM_OK;
}

ArmTrimResult_t ArmTrim_Cancel(ArmTrim_t *t)
{
    if (t == NULL || !t->configured) return ARM_TRIM_INVALID;
    if (t->status.state == ARM_TRIM_FAULT) return ARM_TRIM_FAULT_LATCHED;
    if (t->status.state == ARM_TRIM_STOPPING) return ARM_TRIM_OK;
    t->status.reference_valid = false;
    t->status.jogging = false;
    t->status.jog_direction = 0;
    t->jog_release_requested = false;
    t->status.state = ARM_TRIM_STOPPING;
    t->status.stop_failed = false;
    t->stop_index = 0U;
    return ARM_TRIM_OK;
}

static void Fail(ArmTrim_t *t, ArmTrimResult_t error)
{
    (void)ArmTrim_Cancel(t);
    t->status.error = error;
}

static void CompletedSegment(ArmTrim_t *t)
{
    const ArmTrimSegment_t *segment = &t->segments[t->status.segment_index];
    t->status.offset_mm = segment->offset_mm;
    memcpy(t->status.estimated_position, segment->position, sizeof(segment->position));
}

static void SendSegment(ArmTrim_t *t, uint32_t now)
{
    const ArmTrimSegment_t *segment = &t->segments[t->status.segment_index];
    /* Start-to-start cadence includes UART TX time. Waiting T after TX and
     * then sending the next frame used to add a transmission gap every step. */
    t->dispatch_tick = now;
    if (!t->io.send(t->io.user, segment->position, segment->move_ms)) {
        Fail(t, ARM_TRIM_TRANSPORT); return;
    }
    t->segment_tick = t->io.now(t->io.user);
    t->service_tick = t->segment_tick;
    t->segment_sent = true;
}

void ArmTrim_Process(ArmTrim_t *t)
{
    uint32_t now;
    if (t == NULL || !t->configured) return;
    if (t->status.state == ARM_TRIM_STOPPING) {
        if (t->stop_index < 3U) {
            if (!t->io.stop(t->io.user, t->stop_index)) {
                t->status.stop_failed = true;
                if (t->status.error == ARM_TRIM_OK) t->status.error = ARM_TRIM_TRANSPORT;
            }
            ++t->stop_index;
        } else t->status.state = t->status.error == ARM_TRIM_OK ? ARM_TRIM_CANCELLED : ARM_TRIM_FAULT;
        return;
    }
    if (!Active(t)) return;
    now = t->io.now(t->io.user);
    if ((uint32_t)(now - t->service_tick) > t->config.service_timeout_ms) {
        Fail(t, ARM_TRIM_SERVICE_TIMEOUT);
        return;
    }
    t->service_tick = now;
    if (t->status.state == ARM_TRIM_SETTLING) {
        if ((uint32_t)(now - t->segment_tick) >= t->config.settle_ms) {
            t->status.offset_mm = t->status.target_offset_mm;
            memcpy(t->status.estimated_position, t->status.target_position,
                   sizeof(t->status.estimated_position));
            t->status.state = ARM_TRIM_COMPLETE_ESTIMATED;
            t->status.jogging = false;
            t->status.jog_direction = 0;
        }
        return;
    }
    if (!t->segment_sent) {
        if (t->jog_release_requested) {
            t->status.segment_index = t->status.segment_count = 0U;
            t->status.target_offset_mm = t->status.offset_mm;
            memcpy(t->status.target_position, t->status.estimated_position, sizeof(t->status.target_position));
            t->status.jogging = false;
            t->jog_release_requested = false;
            t->status.state = ARM_TRIM_SETTLING;
            t->segment_tick = now;
            return;
        }
        SendSegment(t, now);
    } else if (t->jog_release_requested || t->status.segment_index + 1U == t->status.segment_count) {
        const ArmTrimSegment_t segment = t->segments[t->status.segment_index];
        float stopping_distance = segment.end_speed_mm_s * segment.end_speed_mm_s /
                                  (2.0f * t->config.acceleration_mm_s2);
        if ((uint32_t)(now - t->segment_tick) < segment.move_ms) return;
        CompletedSegment(t);
        if (t->jog_release_requested && stopping_distance > 0.00001f) {
            float destination = t->status.offset_mm + (float)t->status.jog_direction * stopping_distance;
            ArmTrimResult_t result;
            destination = fminf(t->status.enabled_max_mm, fmaxf(t->status.enabled_min_mm, destination));
            t->jog_release_requested = false;
            t->status.jogging = false;
            result = PlanTimed(t, destination, segment.end_speed_mm_s);
            if (result != ARM_TRIM_OK) { Fail(t, result); return; }
            SendSegment(t, t->io.now(t->io.user));
        } else {
            if (t->jog_release_requested) {
                t->status.segment_count = t->status.segment_index + 1U;
                t->status.target_offset_mm = t->status.offset_mm;
                memcpy(t->status.target_position, t->status.estimated_position, sizeof(t->status.target_position));
            }
            ++t->status.segment_index;
            t->segment_sent = false;
            t->status.jogging = false;
            t->jog_release_requested = false;
            t->status.state = ARM_TRIM_SETTLING;
            t->segment_tick = now;
        }
    } else if ((uint32_t)(now - t->dispatch_tick) >= t->segments[t->status.segment_index].move_ms) {
        uint32_t late = (uint32_t)(now - t->dispatch_tick) - t->segments[t->status.segment_index].move_ms;
        if (late > t->config.dispatch_lateness_ms) { Fail(t, ARM_TRIM_SERVICE_TIMEOUT); return; }
        CompletedSegment(t);
        ++t->status.segment_index;
        t->segment_sent = false;
        /* Do not burst old points or shorten T to catch up with a delayed loop. */
        SendSegment(t, now);
    }
}

ArmTrimResult_t ArmTrim_Exit(ArmTrim_t *t)
{
    if (t == NULL || !t->configured) return ARM_TRIM_INVALID;
    if (Active(t)) return ARM_TRIM_BUSY;
    if (t->status.state == ARM_TRIM_FAULT) return ARM_TRIM_FAULT_LATCHED;
    t->status.reference_valid = false;
    t->status.state = ARM_TRIM_IDLE;
    return ARM_TRIM_OK;
}

ArmTrimResult_t ArmTrim_ClearFault(ArmTrim_t *t)
{
    if (t == NULL || !t->configured) return ARM_TRIM_INVALID;
    if (t->status.state != ARM_TRIM_FAULT) return ARM_TRIM_INVALID;
    t->status.reference_valid = false;
    t->status.error = ARM_TRIM_OK;
    t->status.stop_failed = false;
    t->status.state = ARM_TRIM_CANCELLED;
    return ARM_TRIM_OK;
}

ArmTrimStatus_t ArmTrim_GetStatus(const ArmTrim_t *t)
{
    ArmTrimStatus_t empty = {0};
    return t != NULL ? t->status : empty;
}

void ArmTrim_ReportLateral(ArmTrim_t *t, float error)
{
    if (t != NULL && t->configured && isfinite(error) && t->io.lateral_report != NULL)
        t->io.lateral_report(t->io.user, error);
}
