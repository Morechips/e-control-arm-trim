#include "arm_control.h"
#include <string.h>

static bool initialized;
static ArmConfig_t configuration;
static ArmStep_t sequence[ARM_MAX_SEQUENCE_STEPS];
static ArmStatus_t status;
static bool step_sent;
static uint32_t step_tick;
static uint32_t service_tick;
static size_t stop_joint;
static void (*external_stop)(void *user);
static void *external_user;

static bool Active(void)
{
    return status.state == ARM_RUNNING || status.state == ARM_STOPPING;
}

void Arm_DefaultConfig(ArmConfig_t *config)
{
    size_t i;
    if (config == NULL) return;
    memset(config, 0, sizeof(*config));
    for (i = 0U; i < ARM_JOINT_COUNT; ++i)
        config->joints[i].servo_id = ARM_SERVO_ID_UNASSIGNED;
}

void Arm_ProjectConfig(ArmConfig_t *config)
{
    static const uint16_t servo_id[] = {0U, 1U, 2U, 3U};
    static const uint16_t minimum[] = {
        ARM_P0_MIN, ARM_P1_MIN, ARM_P2_MIN, ARM_GRIPPER_MIN_P
    };
    static const uint16_t maximum[] = {
        ARM_P0_MAX, ARM_P1_MAX, ARM_P2_MAX, ARM_GRIPPER_MAX_P
    };
    size_t i;
    if (config == NULL) return;
    Arm_DefaultConfig(config);
    for (i = 0U; i < sizeof(servo_id) / sizeof(servo_id[0]); ++i)
    {
        config->joints[i].enabled = true;
        config->joints[i].calibrated = true;
        config->joints[i].servo_id = servo_id[i];
        config->joints[i].min_position = minimum[i];
        config->joints[i].max_position = maximum[i];
    }
}

void Arm_Init(void)
{
    if (initialized) return;
    initialized = true;
    Arm_DefaultConfig(&configuration);
    memset(&status, 0, sizeof(status));
    status.state = ARM_UNCONFIGURED;
}

ArmResult_t Arm_Configure(const ArmConfig_t *config)
{
    size_t i, j;
    bool any_enabled = false;
    if (!initialized) return ARM_NOT_CONFIGURED;
    if (external_stop != NULL) return ARM_BUSY;
    if (Active()) return ARM_BUSY;
    if (status.state == ARM_FAULT) return ARM_FAULT_LATCHED;
    if (config == NULL) return ARM_INVALID_ARGUMENT;
    for (i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        const ArmJointConfig_t *joint = &config->joints[i];
        if (!joint->enabled) continue;
        any_enabled = true;
        if (!joint->calibrated || joint->servo_id > 254U ||
            joint->min_position < 500U || joint->max_position > 2500U ||
            joint->min_position >= joint->max_position)
            return ARM_INVALID_ARGUMENT;
        for (j = 0U; j < i; ++j)
            if (config->joints[j].enabled &&
                config->joints[j].servo_id == joint->servo_id)
                return ARM_INVALID_ARGUMENT;
    }
    if (!any_enabled) return ARM_INVALID_ARGUMENT;
    configuration = *config;
    memset(&status, 0, sizeof(status));
    status.state = ARM_IDLE;
    step_sent = false;
    return ARM_OK;
}

ArmResult_t Arm_SetMotionAllowed(bool allowed)
{
    if (external_stop != NULL) {
        if (allowed) return ARM_BUSY;
        return Arm_Stop();
    }
    if (!allowed)
    {
        status.motion_allowed = false;
        if (status.state == ARM_RUNNING) return Arm_Stop();
        return ARM_OK;
    }
    if (!initialized || status.state == ARM_UNCONFIGURED)
        return ARM_NOT_CONFIGURED;
    if (status.state == ARM_STOPPING) return ARM_BUSY;
    if (status.state == ARM_FAULT) return ARM_FAULT_LATCHED;
    status.motion_allowed = true;
    return ARM_OK;
}

static bool StepValid(const ArmStep_t *step, bool original_preset)
{
    size_t i;
    const uint8_t valid_mask = (uint8_t)((1U << ARM_JOINT_COUNT) - 1U);
    if (step->joint_mask == 0U || (step->joint_mask & valid_mask) != step->joint_mask ||
        step->move_ms == 0U || step->move_ms > 9999U || step->hold_ms > ARM_MAX_HOLD_MS)
        return false;
    for (i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        const ArmJointConfig_t *joint = &configuration.joints[i];
        const bool original_gripper = original_preset && i == ARM_JOINT_GRIPPER;
        if ((step->joint_mask & (1U << i)) == 0U) continue;
        if (!joint->enabled ||
            step->position[i] < (original_gripper ? 500U : joint->min_position) ||
            step->position[i] > (original_gripper ? 2500U : joint->max_position))
            return false;
    }
    return true;
}

static ArmResult_t StartSequence(const ArmStep_t *steps, size_t count,
                                 bool original_preset)
{
    size_t i;
    if (!initialized || status.state == ARM_UNCONFIGURED) return ARM_NOT_CONFIGURED;
    if (external_stop != NULL) return ARM_BUSY;
    if (Active()) return ARM_BUSY;
    if (status.state == ARM_FAULT) return ARM_FAULT_LATCHED;
    if (!status.motion_allowed) return ARM_INHIBITED;
    if (steps == NULL || count == 0U || count > ARM_MAX_SEQUENCE_STEPS)
        return ARM_INVALID_ARGUMENT;
    /* Validate every step BEFORE copying or starting any motion. */
    for (i = 0U; i < count; ++i)
        if (!StepValid(&steps[i], original_preset)) return ARM_INVALID_ARGUMENT;
    memcpy(sequence, steps, count * sizeof(sequence[0]));
    status.step_index = 0U;
    status.step_count = count;
    status.error = ARM_ERROR_NONE;
    status.transport_result = ZLIS2_OK;
    status.stop_delivery_failed = false;
    status.state = ARM_RUNNING;
    step_sent = false;
    service_tick = HAL_GetTick();
    return ARM_OK;
}

ArmResult_t Arm_StartSequence(const ArmStep_t *steps, size_t count)
{
    return StartSequence(steps, count, false);
}

ArmResult_t Arm_StartOriginalPreset(const ArmStep_t *step)
{
    return StartSequence(step, 1U, true);
}

ArmResult_t Arm_ResetController(void)
{
    ZLIS2_Status result;
    if (!initialized || status.state == ARM_UNCONFIGURED) return ARM_NOT_CONFIGURED;
    if (external_stop != NULL) return ARM_BUSY;
    if (Active() || status.motion_allowed) return ARM_BUSY;
    if (status.state == ARM_FAULT) return ARM_FAULT_LATCHED;
    result = ZLIS2_Reset();
    status.transport_result = result;
    if (result != ZLIS2_OK)
    {
        status.state = ARM_FAULT;
        status.error = ARM_ERROR_TRANSPORT;
        return ARM_FAULT_LATCHED;
    }
    status.state = ARM_IDLE;
    status.error = ARM_ERROR_NONE;
    status.step_count = 0U;
    return ARM_OK;
}

ArmResult_t Arm_SendImmediate(const ArmStep_t *step)
{
    ZLIS2_ServoCommand commands[ARM_JOINT_COUNT];
    ZLIS2_Status tx_result;
    size_t i, count = 0U;
    if (!initialized || status.state == ARM_UNCONFIGURED) return ARM_NOT_CONFIGURED;
    if (external_stop != NULL) return ARM_BUSY;
    if (step == NULL || !StepValid(step, false)) return ARM_INVALID_ARGUMENT;
    for (i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        if ((step->joint_mask & (1U << i)) == 0U) continue;
        commands[count].id = configuration.joints[i].servo_id;
        commands[count].pwm = step->position[i];
        commands[count].time_ms = step->move_ms;
        ++count;
    }
    tx_result = ZLIS2_SetServos(commands, count);
    status.transport_result = tx_result;
    if (tx_result != ZLIS2_OK)
    {
        status.motion_allowed = false;
        status.state = ARM_FAULT;
        status.error = ARM_ERROR_TRANSPORT;
        status.step_count = 0U;
        step_sent = false;
        return ARM_FAULT_LATCHED;
    }
    sequence[0] = *step;
    status.step_index = 0U;
    status.step_count = 1U;
    status.error = ARM_ERROR_NONE;
    status.stop_delivery_failed = false;
    status.motion_allowed = true;
    status.state = ARM_RUNNING;
    step_sent = true;
    step_tick = service_tick = HAL_GetTick();
    return ARM_OK;
}

ArmResult_t Arm_MoveJoint(ArmJoint_t joint, uint16_t position,
                          uint16_t move_ms, uint32_t hold_ms)
{
    ArmStep_t step = {0};
    if ((unsigned)joint >= ARM_JOINT_COUNT) return ARM_INVALID_ARGUMENT;
    step.joint_mask = (uint8_t)(1U << (unsigned)joint);
    step.position[(unsigned)joint] = position;
    step.move_ms = move_ms;
    step.hold_ms = hold_ms;
    return Arm_StartSequence(&step, 1U);
}

ArmResult_t Arm_Stop(void)
{
    status.motion_allowed = false;
    if (external_stop != NULL) {
        external_stop(external_user);
        return ARM_OK;
    }
    if (!initialized || status.state == ARM_UNCONFIGURED) return ARM_NOT_CONFIGURED;
    if (status.state == ARM_STOPPING) return ARM_OK;
    status.state = ARM_STOPPING;
    stop_joint = 0U;
    step_sent = false;
    return ARM_OK;
}

static void Fail(ArmError_t error)
{
    status.error = error;
    (void)Arm_Stop();
}

static void ProcessStop(void)
{
    ZLIS2_Status result;
    while (stop_joint < ARM_JOINT_COUNT && !configuration.joints[stop_joint].enabled)
        ++stop_joint;
    if (stop_joint == ARM_JOINT_COUNT)
    {
        status.state = status.error == ARM_ERROR_NONE ? ARM_CANCELLED : ARM_FAULT;
        return;
    }
    result = ZLIS2_StopServo(configuration.joints[stop_joint].servo_id);
    ++stop_joint;
    if (result != ZLIS2_OK)
    {
        status.transport_result = result;
        status.stop_delivery_failed = true;
        if (status.error == ARM_ERROR_NONE) status.error = ARM_ERROR_STOP_TRANSPORT;
    }
    /* Continue attempting remaining joints even after a transport failure. */
}

void Arm_Process(void)
{
    uint32_t now;
    if (status.state == ARM_STOPPING)
    {
        ProcessStop();
        return;
    }
    if (status.state != ARM_RUNNING) return;
    now = HAL_GetTick();
    if ((uint32_t)(now - service_tick) > ARM_SERVICE_TIMEOUT_MS)
    {
        Fail(ARM_ERROR_SERVICE_TIMEOUT);
        ProcessStop();
        return;
    }
    service_tick = now;
    if (!step_sent)
    {
        ZLIS2_ServoCommand commands[ARM_JOINT_COUNT];
        const ArmStep_t *step = &sequence[status.step_index];
        size_t i, count = 0U;
        for (i = 0U; i < ARM_JOINT_COUNT; ++i)
        {
            if ((step->joint_mask & (1U << i)) == 0U) continue;
            commands[count].id = configuration.joints[i].servo_id;
            commands[count].pwm = step->position[i];
            commands[count].time_ms = step->move_ms;
            ++count;
        }
        status.transport_result = ZLIS2_SetServos(commands, count);
        if (status.transport_result != ZLIS2_OK)
        {
            Fail(ARM_ERROR_TRANSPORT);
            return;
        }
        /* Conservative start: count from TX completion, not queue acceptance. */
        step_tick = HAL_GetTick();
        step_sent = true;
        return;
    }
    if ((uint32_t)(now - step_tick) >=
        (uint32_t)sequence[status.step_index].move_ms + sequence[status.step_index].hold_ms)
    {
        if (status.step_index + 1U == status.step_count)
            status.state = ARM_COMPLETE_ESTIMATED;
        else
        {
            ++status.step_index;
            step_sent = false;
        }
    }
}

ArmResult_t Arm_ClearFault(void)
{
    if (!initialized || status.state == ARM_UNCONFIGURED) return ARM_NOT_CONFIGURED;
    if (Active()) return ARM_BUSY;
    if (status.state != ARM_FAULT) return ARM_INVALID_ARGUMENT;
    status.error = ARM_ERROR_NONE;
    status.transport_result = ZLIS2_OK;
    status.stop_delivery_failed = false;
    status.motion_allowed = false;
    status.state = ARM_IDLE;
    return ARM_OK;
}

ArmStatus_t Arm_GetStatus(void)
{
    return status;
}

ArmResult_t Arm_AcquireExternalMotion(void (*stop)(void *user), void *user)
{
    if (!initialized || status.state == ARM_UNCONFIGURED) return ARM_NOT_CONFIGURED;
    if (stop == NULL || user == NULL) return ARM_INVALID_ARGUMENT;
    if (external_stop != NULL || Active() || status.motion_allowed) return ARM_BUSY;
    if (status.state == ARM_FAULT) return ARM_FAULT_LATCHED;
    external_stop = stop;
    external_user = user;
    return ARM_OK;
}

ArmResult_t Arm_ReleaseExternalMotion(void *user)
{
    if (external_stop == NULL || external_user != user) return ARM_INVALID_ARGUMENT;
    external_stop = NULL;
    external_user = NULL;
    return ARM_OK;
}

bool Arm_ExternalMotionOwned(void) { return external_stop != NULL; }

