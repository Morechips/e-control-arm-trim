#include "arm_trim_service.h"
#include <string.h>

typedef struct {
    ArmTrim_t trim;
    ArmTrimServiceConfig_t config;
    uint32_t (*clock)(void *user);
    void *clock_user;
    ArmTrimServiceState_t state;
    ArmTrimServiceProfile_t profile;
    ArmTrimResult_t last_request;
    ServoStatus_t transport_result;
    uint32_t operation_tick, completed_tick;
    unsigned stop_index;
    uint8_t stop_mask;
    bool configured, owns, synchronize_profile, ending;
    bool completed, stop_waiting, stop_failed, fault_latched;
} Service_t;

static Service_t service;

static uint32_t Now(void *user)
{
    Service_t *s = user;
    return s->clock(s->clock_user);
}

static ArmTrimResult_t Result(ArmTrimResult_t result)
{
    service.last_request = result;
    return result;
}

static ArmTrimResult_t FromServo(ServoStatus_t result)
{
    service.transport_result = result;
    if (result == SERVO_OK) return ARM_TRIM_OK;
    if (result == SERVO_BUSY) return ARM_TRIM_BUSY;
    if (result == SERVO_INVALID_PARAM || result == SERVO_BUFFER_OVERFLOW)
        return ARM_TRIM_INVALID;
    return ARM_TRIM_TRANSPORT;
}

static bool Send(void *user, const uint16_t positions[3], uint16_t move_ms)
{
    Service_t *s = user;
    s->transport_result = Servo_SetValuesOwned(s, positions, 3U, move_ms);
    return s->transport_result == SERVO_OK;
}

static bool Stop(void *user, unsigned index)
{
    Service_t *s = user;
    s->transport_result = Servo_StopServoOwned(s, (uint16_t)index);
    return s->transport_result == SERVO_OK;
}

static ArmTrimTxState_t Poll(void *user, uint32_t *started, uint32_t *completed)
{
    Service_t *s = user;
    ServoTransferStatus_t status = Servo_GetTransferStatus(s);
    if (status.state == SERVO_TRANSFER_PENDING) return ARM_TRIM_TX_PENDING;
    if (status.state != SERVO_TRANSFER_COMPLETE || !status.started) {
        s->transport_result = status.result;
        return ARM_TRIM_TX_FAILED;
    }
    *started = status.started_tick;
    *completed = status.completed_tick;
    return ARM_TRIM_TX_COMPLETE;
}

static void CancelPending(void *user)
{
    Service_t *s = user;
    s->transport_result = Servo_CancelPendingOwned(s);
}

static bool Busy(void)
{
    return service.state == ARM_TRIM_SERVICE_PROFILE ||
           service.state == ARM_TRIM_SERVICE_MOTION ||
           service.state == ARM_TRIM_SERVICE_GRIP ||
           service.state == ARM_TRIM_SERVICE_STOPPING || service.ending;
}

static bool Release(void)
{
    ServoStatus_t result;
    if (!service.owns) return true;
    result = Servo_Release(&service);
    if (result == SERVO_BUSY) return false;
    service.transport_result = result;
    if (result != SERVO_OK) return false;
    service.owns = false;
    return true;
}

static ArmTrimResult_t Acquire(void)
{
    ArmTrimResult_t result;
    if (service.owns) return ARM_TRIM_OK;
    result = FromServo(Servo_Acquire(&service));
    if (result == ARM_TRIM_OK) service.owns = true;
    return result;
}

static ArmTrimResult_t Available(void)
{
    if (!service.configured) return ARM_TRIM_INVALID;
    if (service.fault_latched || service.state == ARM_TRIM_SERVICE_FAULT)
        return ARM_TRIM_FAULT_LATCHED;
    if (Busy()) return ARM_TRIM_BUSY;
    return ARM_TRIM_OK;
}

static bool CompletionTimedOut(const ServoTransferStatus_t *transfer, uint32_t now)
{
    uint32_t limit = service.config.core.service_timeout_ms;
    return (uint32_t)(transfer->started_tick - service.operation_tick) > limit ||
           (uint32_t)(transfer->completed_tick - transfer->started_tick) > limit ||
           (uint32_t)(transfer->completed_tick - service.operation_tick) > limit ||
           (uint32_t)(now - transfer->completed_tick) >= UINT32_C(0x80000000);
}

static void Invalidate(void)
{
    (void)ArmTrim_Exit(&service.trim);
    service.profile = ARM_TRIM_PROFILE_NONE;
}

static void StartStops(uint8_t mask, ArmTrimResult_t error)
{
    CancelPending(&service);
    Invalidate();
    service.state = ARM_TRIM_SERVICE_STOPPING;
    service.stop_mask = mask;
    service.stop_index = 0U;
    service.stop_waiting = false;
    service.completed = false;
    service.stop_failed = false;
    if (error != ARM_TRIM_OK) {
        service.last_request = error;
        service.fault_latched = true;
    }
}

ArmTrimResult_t ArmTrimService_Init(const ArmTrimServiceConfig_t *config,
                                 uint32_t (*now)(void *user), void *user)
{
    ArmTrimIO_t io = {Send, Stop, Now, NULL, &service};
    ArmTrimAsyncIO_t async = {Poll, CancelPending};
    ArmTrimResult_t result;
    unsigned profile, joint;
    if (service.owns) return Result(ARM_TRIM_BUSY);
    if (config == NULL || now == NULL || config->profile_move_ms == 0U ||
        config->profile_move_ms > 9999U || config->profile_guard_ms > 60000U ||
        config->grip_move_ms == 0U || config->grip_move_ms > 9999U ||
        config->grip_guard_ms > 60000U) return Result(ARM_TRIM_INVALID);
    if ((config->grip_min_pwm != 0U || config->grip_max_pwm != 0U) &&
        (config->grip_min_pwm < 500U || config->grip_max_pwm > 2500U ||
         config->grip_min_pwm >= config->grip_max_pwm)) return Result(ARM_TRIM_INVALID);
    for (profile = 0U; profile < ARM_TRIM_PROFILE_COUNT; ++profile)
        for (joint = 0U; joint < 3U; ++joint)
            if (config->references[profile][joint] < config->core.calibration[joint].min_position ||
                config->references[profile][joint] > config->core.calibration[joint].max_position)
                return Result(ARM_TRIM_INVALID);
    /* Validate before replacing the service's previous idle configuration. */
    result = ArmTrim_Init(&service.trim, &config->core, &io);
    if (result != ARM_TRIM_OK) return Result(result);
    result = ArmTrim_SetAsyncIO(&service.trim, &async);
    if (result != ARM_TRIM_OK) return Result(result);
    service.config = *config;
    if (service.config.grip_min_pwm == 0U && service.config.grip_max_pwm == 0U) {
        service.config.grip_min_pwm = 500U;
        service.config.grip_max_pwm = 2500U;
    }
    service.clock = now;
    service.clock_user = user;
    service.state = ARM_TRIM_SERVICE_IDLE;
    service.profile = ARM_TRIM_PROFILE_NONE;
    service.configured = true;
    service.owns = service.ending = service.completed = false;
    service.stop_waiting = service.stop_failed = service.fault_latched = false;
    service.transport_result = SERVO_OK;
    return Result(ARM_TRIM_OK);
}

ArmTrimResult_t ArmTrimService_ReadyProfile(ArmTrimServiceProfile_t profile,
                                         bool synchronize)
{
    ArmTrimResult_t result = Available();
    if (result != ARM_TRIM_OK) return Result(result);
    if ((unsigned)profile >= ARM_TRIM_PROFILE_COUNT) return Result(ARM_TRIM_INVALID);
    result = Acquire();
    if (result != ARM_TRIM_OK) return Result(result);
    Invalidate();
    service.operation_tick = Now(&service);
    result = FromServo(Servo_SetValuesOwned(&service, service.config.references[profile],
                                         3U, service.config.profile_move_ms));
    if (result != ARM_TRIM_OK) {
        (void)Release();
        service.state = ARM_TRIM_SERVICE_IDLE;
        return Result(result);
    }
    service.profile = profile;
    service.synchronize_profile = synchronize;
    service.completed = false;
    service.state = ARM_TRIM_SERVICE_PROFILE;
    return Result(ARM_TRIM_OK);
}

ArmTrimResult_t ArmTrimService_Begin(const uint16_t positions[3], bool parked_stable)
{
    ArmTrimResult_t result = Available();
    if (result != ARM_TRIM_OK) return Result(result);
    if (positions == NULL || !parked_stable) return Result(ARM_TRIM_INVALID);
    result = Acquire();
    if (result != ARM_TRIM_OK) return Result(result);
    /* A failed new reference must never leave the previous estimate usable. */
    Invalidate();
    result = ArmTrim_Synchronize(&service.trim, positions);
    if (result == ARM_TRIM_OK) service.state = ARM_TRIM_SERVICE_REFERENCE;
    else {
        service.state = ARM_TRIM_SERVICE_IDLE;
        (void)Release();
    }
    return Result(result);
}

static ArmTrimResult_t ReferenceAvailable(void)
{
    ArmTrimResult_t result = Available();
    if (result != ARM_TRIM_OK) return result;
    if (!service.owns || !ArmTrim_GetStatus(&service.trim).reference_valid)
        return ARM_TRIM_REFERENCE_REQUIRED;
    return ARM_TRIM_OK;
}

ArmTrimResult_t ArmTrimService_MoveRelativeX(float dx_mm)
{
    ArmTrimResult_t result = ReferenceAvailable();
    if (result == ARM_TRIM_OK) result = ArmTrim_MoveRelativeX(&service.trim, dx_mm);
    if (result == ARM_TRIM_OK && ArmTrim_GetStatus(&service.trim).state == ARM_TRIM_MOVING)
        service.state = ARM_TRIM_SERVICE_MOTION;
    return Result(result);
}

ArmTrimResult_t ArmTrimService_StartJog(int direction)
{
    ArmTrimResult_t result = ReferenceAvailable();
    if (result == ARM_TRIM_OK) result = ArmTrim_StartJog(&service.trim, direction);
    if (result == ARM_TRIM_OK) service.state = ARM_TRIM_SERVICE_MOTION;
    return Result(result);
}

ArmTrimResult_t ArmTrimService_ReleaseJog(void)
{
    if (!service.configured) return Result(ARM_TRIM_INVALID);
    if (service.fault_latched) return Result(ARM_TRIM_FAULT_LATCHED);
    if (service.state != ARM_TRIM_SERVICE_MOTION) return Result(ARM_TRIM_BUSY);
    return Result(ArmTrim_ReleaseJog(&service.trim));
}

ArmTrimResult_t ArmTrimService_Grip(uint16_t pwm)
{
    ArmTrimResult_t result = ReferenceAvailable();
    ServoCommand_t command = {3U, pwm, service.config.grip_move_ms};
    if (result != ARM_TRIM_OK) return Result(result);
    if (pwm < service.config.grip_min_pwm || pwm > service.config.grip_max_pwm)
        return Result(ARM_TRIM_INVALID);
    service.operation_tick = Now(&service);
    result = FromServo(Servo_SetCommandsOwned(&service, &command, 1U));
    if (result != ARM_TRIM_OK) return Result(result);
    service.completed = false;
    service.state = ARM_TRIM_SERVICE_GRIP;
    return Result(ARM_TRIM_OK);
}

ArmTrimResult_t ArmTrimService_Cancel(void)
{
    ArmTrimResult_t result;
    if (!service.configured) return Result(ARM_TRIM_INVALID);
    if (service.fault_latched || service.state == ARM_TRIM_SERVICE_FAULT)
        return Result(ARM_TRIM_FAULT_LATCHED);
    if (service.state == ARM_TRIM_SERVICE_STOPPING) return Result(ARM_TRIM_OK);
    if (!service.owns) {
        Invalidate();
        service.state = ARM_TRIM_SERVICE_IDLE;
        return Result(ARM_TRIM_OK);
    }
    if (service.state == ARM_TRIM_SERVICE_PROFILE || service.state == ARM_TRIM_SERVICE_GRIP) {
        StartStops(service.state == ARM_TRIM_SERVICE_GRIP ? 8U : 7U, ARM_TRIM_OK);
        return Result(ARM_TRIM_OK);
    }
    result = ArmTrim_Cancel(&service.trim);
    if (result == ARM_TRIM_OK) {
        service.profile = ARM_TRIM_PROFILE_NONE;
        service.state = ARM_TRIM_SERVICE_MOTION;
    }
    return Result(result);
}

ArmTrimResult_t ArmTrimService_End(void)
{
    ArmTrimResult_t result;
    if (!service.configured) return Result(ARM_TRIM_INVALID);
    if (service.state == ARM_TRIM_SERVICE_FAULT || service.fault_latched) {
        service.ending = true;
        /* A latched fault may still be running the required stop sequence. */
        if (!Busy() || service.state == ARM_TRIM_SERVICE_FAULT) {
            (void)Release();
            if (!service.owns) service.ending = false;
        }
        return Result(ARM_TRIM_FAULT_LATCHED);
    }
    if (Busy()) {
        result = ArmTrimService_Cancel();
        if (result == ARM_TRIM_OK) service.ending = true;
        return Result(result);
    }
    Invalidate();
    service.state = ARM_TRIM_SERVICE_IDLE;
    service.ending = !Release();
    return Result(ARM_TRIM_OK);
}

ArmTrimResult_t ArmTrimService_ClearFault(void)
{
    ArmTrimResult_t result;
    if (!service.configured) return Result(ARM_TRIM_INVALID);
    if (Busy() || service.owns) return Result(ARM_TRIM_BUSY);
    if (!service.fault_latched && service.state != ARM_TRIM_SERVICE_FAULT)
        return Result(ARM_TRIM_INVALID);
    if (ArmTrim_GetStatus(&service.trim).state == ARM_TRIM_FAULT) {
        result = ArmTrim_ClearFault(&service.trim);
        if (result != ARM_TRIM_OK) return Result(result);
    }
    Invalidate();
    service.fault_latched = service.stop_failed = false;
    service.state = ARM_TRIM_SERVICE_IDLE;
    return Result(ARM_TRIM_OK);
}

static void ProcessWait(void)
{
    ServoTransferStatus_t transfer;
    uint32_t now = Now(&service);
    uint32_t wait = service.state == ARM_TRIM_SERVICE_PROFILE ?
        (uint32_t)service.config.profile_move_ms + service.config.profile_guard_ms :
        (uint32_t)service.config.grip_move_ms + service.config.grip_guard_ms;
    if (!service.completed) {
        transfer = Servo_GetTransferStatus(&service);
        if (transfer.state == SERVO_TRANSFER_COMPLETE && transfer.started) {
            if (CompletionTimedOut(&transfer, now)) {
                StartStops(service.state == ARM_TRIM_SERVICE_GRIP ? 8U : 7U,
                           ARM_TRIM_SERVICE_TIMEOUT);
                return;
            }
            service.completed_tick = transfer.completed_tick;
            service.completed = true;
        } else if (transfer.state == SERVO_TRANSFER_FAILED || transfer.state == SERVO_TRANSFER_NONE) {
            service.transport_result = transfer.result;
            StartStops(service.state == ARM_TRIM_SERVICE_GRIP ? 8U : 7U, ARM_TRIM_TRANSPORT);
            return;
        } else if ((uint32_t)(now - service.operation_tick) > service.config.core.service_timeout_ms) {
            StartStops(service.state == ARM_TRIM_SERVICE_GRIP ? 8U : 7U, ARM_TRIM_SERVICE_TIMEOUT);
            return;
        }
    }
    if (!service.completed || (uint32_t)(now - service.completed_tick) < wait) return;
    if (service.state == ARM_TRIM_SERVICE_GRIP) service.state = ARM_TRIM_SERVICE_REFERENCE;
    else if (service.synchronize_profile) {
        ArmTrimResult_t result = ArmTrim_Synchronize(&service.trim,
            service.config.references[service.profile]);
        if (result == ARM_TRIM_OK) service.state = ARM_TRIM_SERVICE_REFERENCE;
        else {
            Invalidate();
            service.last_request = result;
            service.fault_latched = true;
            service.state = ARM_TRIM_SERVICE_FAULT;
            (void)Release();
        }
    } else {
        Invalidate();
        service.state = ARM_TRIM_SERVICE_IDLE;
        service.ending = !Release();
    }
}

static void ProcessStops(void)
{
    ServoTransferStatus_t transfer;
    uint32_t now = Now(&service);
    if (service.stop_waiting) {
        bool timed_out;
        transfer = Servo_GetTransferStatus(&service);
        if (transfer.state == SERVO_TRANSFER_PENDING &&
            (uint32_t)(now - service.operation_tick) <= service.config.core.service_timeout_ms)
            return;
        timed_out = transfer.state == SERVO_TRANSFER_PENDING ||
            (transfer.state == SERVO_TRANSFER_COMPLETE && CompletionTimedOut(&transfer, now));
        if (transfer.state != SERVO_TRANSFER_COMPLETE || !transfer.started || timed_out) {
            service.transport_result = transfer.result;
            service.stop_failed = service.fault_latched = true;
            service.last_request = timed_out ?
                ARM_TRIM_SERVICE_TIMEOUT : ARM_TRIM_TRANSPORT;
            (void)Servo_CancelPendingOwned(&service);
        }
        service.stop_waiting = false;
        ++service.stop_index;
    }
    while (service.stop_index < 4U && !(service.stop_mask & (1U << service.stop_index)))
        ++service.stop_index;
    if (service.stop_index < 4U) {
        ServoStatus_t result = Servo_StopServoOwned(&service, (uint16_t)service.stop_index);
        service.transport_result = result;
        if (result == SERVO_OK) {
            service.operation_tick = now;
            service.stop_waiting = true;
        } else {
            service.stop_failed = service.fault_latched = true;
            service.last_request = ARM_TRIM_TRANSPORT;
            ++service.stop_index;
        }
        return;
    }
    if (!Release()) return;
    service.ending = false;
    service.state = service.fault_latched ? ARM_TRIM_SERVICE_FAULT : ARM_TRIM_SERVICE_IDLE;
}

void ArmTrimService_Process(void)
{
    ArmTrimStatus_t core;
    if (!service.configured) return;
    Servo_Process();
    if (service.state == ARM_TRIM_SERVICE_PROFILE || service.state == ARM_TRIM_SERVICE_GRIP)
        ProcessWait();
    else if (service.state == ARM_TRIM_SERVICE_STOPPING) ProcessStops();
    else if (service.state == ARM_TRIM_SERVICE_MOTION) {
        ArmTrim_Process(&service.trim);
        core = ArmTrim_GetStatus(&service.trim);
        if (core.state == ARM_TRIM_COMPLETE_ESTIMATED || core.state == ARM_TRIM_READY)
            service.state = ARM_TRIM_SERVICE_REFERENCE;
        else if (core.state == ARM_TRIM_CANCELLED || core.state == ARM_TRIM_FAULT) {
            service.stop_failed = core.stop_failed;
            service.fault_latched = core.state == ARM_TRIM_FAULT;
            if (core.error != ARM_TRIM_OK) service.last_request = core.error;
            if (Release()) {
                service.ending = false;
                service.state = service.fault_latched ? ARM_TRIM_SERVICE_FAULT : ARM_TRIM_SERVICE_IDLE;
            }
        }
    } else if (service.ending || service.state == ARM_TRIM_SERVICE_FAULT) {
        if (Release()) service.ending = false;
    }
}

ArmTrimServiceStatus_t ArmTrimService_GetStatus(void)
{
    ArmTrimServiceStatus_t status;
    memset(&status, 0, sizeof(status));
    status.state = service.state;
    status.last_request = service.last_request;
    status.transport_result = service.transport_result;
    status.profile = service.profile;
    status.owns_motion = service.owns;
    status.busy = Busy();
    status.stop_failed = service.stop_failed;
    status.core = ArmTrim_GetStatus(&service.trim);
    if (service.owns) status.transfer = Servo_GetTransferStatus(&service);
    return status;
}

bool ArmTrimService_IsBusy(void) { return Busy(); }
bool ArmTrimService_OwnsMotion(void) { return service.owns; }
