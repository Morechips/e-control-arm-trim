#include "arm_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint32_t tick;
static unsigned checks, tx_count;
static char frames[64][ZLIS2_MAX_TX_LENGTH + 1U];
static HAL_StatusTypeDef next_tx = HAL_OK;
static UART_HandleTypeDef uart;

#define CHECK(condition) do { ++checks; if (!(condition)) { \
    fprintf(stderr, "FAIL line %u: %s\n", (unsigned)__LINE__, #condition); \
    exit(1); } } while (0)

uint32_t HAL_GetTick(void) { return tick; }

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *port, const uint8_t *data,
                                   uint16_t length, uint32_t timeout)
{
    HAL_StatusTypeDef result = next_tx;
    CHECK(port == &uart && timeout == ZLIS2_TX_TIMEOUT_MS);
    CHECK(length <= ZLIS2_MAX_TX_LENGTH && tx_count < 64U);
    memcpy(frames[tx_count], data, length);
    frames[tx_count++][length] = '\0';
    next_tx = HAL_OK;
    tick += 3U; /* Make timing distinguish request time from TX completion. */
    return result;
}

static ArmConfig_t ValidConfig(void)
{
    ArmConfig_t config;
    Arm_DefaultConfig(&config);
    /* Synthetic fixture values, NOT real-car calibration. */
    config.joints[0].enabled = config.joints[0].calibrated = true;
    config.joints[0].servo_id = 7U;
    config.joints[0].min_position = 1100U;
    config.joints[0].max_position = 1900U;
    config.joints[1] = config.joints[0];
    config.joints[1].servo_id = 12U;
    return config;
}

static void FinishStop(void)
{
    unsigned i;
    for (i = 0U; i < ARM_JOINT_COUNT + 1U; ++i) Arm_Process();
    CHECK(Arm_GetStatus().state != ARM_STOPPING);
}

static void Prepare(void)
{
    ArmConfig_t config = ValidConfig();
    if (Arm_GetStatus().state == ARM_RUNNING) CHECK(Arm_Stop() == ARM_OK);
    if (Arm_GetStatus().state == ARM_STOPPING) FinishStop();
    if (Arm_GetStatus().state == ARM_FAULT) CHECK(Arm_ClearFault() == ARM_OK);
    CHECK(Arm_Configure(&config) == ARM_OK);
    CHECK(!Arm_GetStatus().motion_allowed);
    CHECK(Arm_SetMotionAllowed(true) == ARM_OK);
    tx_count = 0U;
}

static void TestConfiguration(void)
{
    ArmConfig_t config;
    Arm_Init();
    Arm_DefaultConfig(&config);
    CHECK(!config.joints[0].enabled && !config.joints[0].calibrated);
    CHECK(config.joints[0].servo_id == ARM_SERVO_ID_UNASSIGNED);
    Arm_Process();
    CHECK(tx_count == 0U && Arm_GetStatus().state == ARM_UNCONFIGURED);
    CHECK(Arm_SetMotionAllowed(true) == ARM_NOT_CONFIGURED);
    CHECK(Arm_MoveJoint(ARM_JOINT_SHOULDER, 1500U, 100U, 0U) == ARM_NOT_CONFIGURED);
    CHECK(Arm_Stop() == ARM_NOT_CONFIGURED);
    CHECK(Arm_Configure(NULL) == ARM_INVALID_ARGUMENT);
    CHECK(Arm_Configure(&config) == ARM_INVALID_ARGUMENT);
    config = ValidConfig();
    config.joints[0].calibrated = false;
    CHECK(Arm_Configure(&config) == ARM_INVALID_ARGUMENT);
    config = ValidConfig(); config.joints[1].servo_id = 7U;
    CHECK(Arm_Configure(&config) == ARM_INVALID_ARGUMENT);
    config = ValidConfig(); config.joints[1].servo_id = 255U;
    CHECK(Arm_Configure(&config) == ARM_INVALID_ARGUMENT);
    config = ValidConfig(); config.joints[0].min_position = 499U;
    CHECK(Arm_Configure(&config) == ARM_INVALID_ARGUMENT);
    config = ValidConfig(); config.joints[0].max_position = 2501U;
    CHECK(Arm_Configure(&config) == ARM_INVALID_ARGUMENT);
    config = ValidConfig(); config.joints[0].max_position = 1000U;
    CHECK(Arm_Configure(&config) == ARM_INVALID_ARGUMENT);
    config = ValidConfig(); CHECK(Arm_Configure(&config) == ARM_OK);
    config.joints[0].servo_id = 99U; /* Driver owns a copy. */
    CHECK(Arm_MoveJoint(ARM_JOINT_SHOULDER, 1500U, 100U, 0U) == ARM_INHIBITED);
    CHECK(Arm_SetMotionAllowed(true) == ARM_OK);
    CHECK(Arm_MoveJoint(ARM_JOINT_SHOULDER, 1500U, 100U, 0U) == ARM_OK);
    CHECK(tx_count == 0U);
    Arm_Process();
    CHECK(strcmp(frames[0], "{#007P1500T0100!}") == 0);
    CHECK(Arm_Configure(&config) == ARM_BUSY);
    Arm_Init(); /* Cannot silently erase an active operation. */
    CHECK(Arm_GetStatus().state == ARM_RUNNING);
    CHECK(Arm_SetMotionAllowed(false) == ARM_OK);
    FinishStop();
    CHECK(strcmp(frames[1], "$DST:7!") == 0);
    CHECK(strcmp(frames[2], "$DST:12!") == 0);
    CHECK(Arm_GetStatus().state == ARM_CANCELLED);
    CHECK(!Arm_GetStatus().motion_allowed);
}

static void TestValidation(void)
{
    ArmStep_t steps[2] = {{0}, {0}};
    Prepare();
    CHECK(Arm_StartSequence(NULL, 1U) == ARM_INVALID_ARGUMENT);
    CHECK(Arm_StartSequence(steps, 0U) == ARM_INVALID_ARGUMENT);
    CHECK(Arm_StartSequence(steps, ARM_MAX_SEQUENCE_STEPS + 1U) == ARM_INVALID_ARGUMENT);
    CHECK(Arm_MoveJoint((ArmJoint_t)-1, 1500U, 100U, 0U) == ARM_INVALID_ARGUMENT);
    CHECK(Arm_MoveJoint(ARM_JOINT_AUX_1, 1500U, 100U, 0U) == ARM_INVALID_ARGUMENT);
    CHECK(Arm_MoveJoint(ARM_JOINT_SHOULDER, 1099U, 100U, 0U) == ARM_INVALID_ARGUMENT);
    CHECK(Arm_MoveJoint(ARM_JOINT_SHOULDER, 1901U, 100U, 0U) == ARM_INVALID_ARGUMENT);
    CHECK(Arm_MoveJoint(ARM_JOINT_SHOULDER, 1500U, 0U, 0U) == ARM_INVALID_ARGUMENT);
    CHECK(Arm_MoveJoint(ARM_JOINT_SHOULDER, 1500U, 10000U, 0U) == ARM_INVALID_ARGUMENT);
    CHECK(Arm_MoveJoint(ARM_JOINT_SHOULDER, 1500U, 100U, ARM_MAX_HOLD_MS + 1U) == ARM_INVALID_ARGUMENT);
    steps[0].joint_mask = 1U; steps[0].position[0] = 1500U; steps[0].move_ms = 100U;
    CHECK(Arm_StartSequence(steps, 2U) == ARM_INVALID_ARGUMENT); /* Bad second step. */
    steps[1] = steps[0]; steps[1].joint_mask = 0x80U;
    CHECK(Arm_StartSequence(steps, 2U) == ARM_INVALID_ARGUMENT);
    Arm_Process();
    CHECK(tx_count == 0U && Arm_GetStatus().state == ARM_IDLE);
}

static void TestSequence(void)
{
    ArmStep_t steps[2] = {{0}, {0}};
    uint32_t sent_at;
    Prepare(); tick = UINT32_MAX - 60U;
    steps[0].joint_mask = 3U; steps[0].position[0] = 1100U;
    steps[0].position[1] = 1900U; steps[0].move_ms = 100U; steps[0].hold_ms = 20U;
    steps[1].joint_mask = 2U; steps[1].position[1] = 1400U; steps[1].move_ms = 50U;
    CHECK(Arm_StartSequence(steps, 2U) == ARM_OK);
    memset(steps, 0, sizeof(steps)); /* Stack/caller changes cannot alter accepted plan. */
    CHECK(Arm_MoveJoint(ARM_JOINT_SHOULDER, 1500U, 100U, 0U) == ARM_BUSY);
    Arm_Process(); sent_at = tick;
    CHECK(strcmp(frames[0], "{#007P1100T0100!#012P1900T0100!}") == 0);
    tick = sent_at + 119U; Arm_Process();
    CHECK(tx_count == 1U && Arm_GetStatus().step_index == 0U);
    ++tick; Arm_Process();
    CHECK(tx_count == 1U && Arm_GetStatus().step_index == 1U);
    Arm_Process();
    CHECK(strcmp(frames[1], "{#012P1400T0050!}") == 0);
    tick += 49U; Arm_Process(); CHECK(Arm_GetStatus().state == ARM_RUNNING);
    ++tick; Arm_Process(); CHECK(Arm_GetStatus().state == ARM_COMPLETE_ESTIMATED);
    Arm_Process(); CHECK(tx_count == 2U); /* No repeated commands after completion. */
}

static void TestOriginalPresetAndReset(void)
{
    ArmStep_t step = {0};
    Prepare();
    step.joint_mask = 1U;
    step.position[0] = 2192U;
    step.move_ms = 100U;
    CHECK(Arm_StartSequence(&step, 1U) == ARM_INVALID_ARGUMENT);
    step.position[0] = 2501U;
    CHECK(Arm_StartOriginalPreset(&step) == ARM_INVALID_ARGUMENT);
    step.position[0] = 499U;
    CHECK(Arm_StartOriginalPreset(&step) == ARM_INVALID_ARGUMENT);
    step.position[0] = 2192U;
    CHECK(Arm_StartOriginalPreset(&step) == ARM_INVALID_ARGUMENT);
    CHECK(tx_count == 0U && Arm_GetStatus().state == ARM_IDLE);
    step.position[0] = 1900U;
    CHECK(Arm_StartOriginalPreset(&step) == ARM_OK);
    CHECK(Arm_ResetController() == ARM_BUSY);
    Arm_Process();
    CHECK(strcmp(frames[0], "{#007P1900T0100!}") == 0);
    tick += 100U; Arm_Process();
    CHECK(Arm_GetStatus().state == ARM_COMPLETE_ESTIMATED);
    CHECK(Arm_SetMotionAllowed(false) == ARM_OK);
    CHECK(Arm_ResetController() == ARM_OK);
    CHECK(strcmp(frames[1], "$RST!") == 0);
    next_tx = HAL_ERROR;
    CHECK(Arm_ResetController() == ARM_FAULT_LATCHED);
    CHECK(Arm_GetStatus().state == ARM_FAULT &&
          Arm_GetStatus().error == ARM_ERROR_TRANSPORT);
    CHECK(Arm_ResetController() == ARM_FAULT_LATCHED);
    CHECK(Arm_ClearFault() == ARM_OK);
}

static void TestProjectTravel(void)
{
    const uint16_t minimum[] = {915U, 947U, 500U};
    const uint16_t maximum[] = {1800U, 2500U, 1874U};
    ArmConfig_t config;
    ArmStep_t step = {0};
    unsigned joint, side;
    Prepare();
    Arm_ProjectConfig(&config);
    CHECK(Arm_Configure(&config) == ARM_OK);
    CHECK(Arm_SetMotionAllowed(true) == ARM_OK);
    step.move_ms = 100U;
    for (joint = 0U; joint < 3U; ++joint) {
        CHECK(config.joints[joint].min_position == minimum[joint]);
        CHECK(config.joints[joint].max_position == maximum[joint]);
        step.joint_mask = (uint8_t)(1U << joint);
        for (side = 0U; side < 2U; ++side) {
            step.position[joint] = side == 0U ? minimum[joint] - 1U : maximum[joint] + 1U;
            CHECK(Arm_StartSequence(&step, 1U) == ARM_INVALID_ARGUMENT);
            CHECK(Arm_StartOriginalPreset(&step) == ARM_INVALID_ARGUMENT);
            CHECK(Arm_SendImmediate(&step) == ARM_INVALID_ARGUMENT);
            Arm_Process();
            CHECK(tx_count == 0U && Arm_GetStatus().state == ARM_IDLE);
        }
    }
    /* The measured endpoints remain inclusive; rejected requests above
     * were neither transmitted nor clamped into the valid interval. */
    step.joint_mask = 0x07U;
    memcpy(step.position, minimum, sizeof(minimum));
    CHECK(Arm_StartOriginalPreset(&step) == ARM_OK);
    Arm_Process();
    CHECK(strcmp(frames[0], "{#000P0915T0100!#001P0947T0100!#002P0500T0100!}") == 0);
    tick += 100U; Arm_Process();
    memcpy(step.position, maximum, sizeof(maximum));
    CHECK(Arm_StartOriginalPreset(&step) == ARM_OK);
    Arm_Process();
    CHECK(strcmp(frames[1], "{#000P1800T0100!#001P2500T0100!#002P1874T0100!}") == 0);
    tick += 100U; Arm_Process();
}

static void TestGapPreset(void)
{
    ArmConfig_t config = ValidConfig();
    ArmStep_t step = {0};
    unsigned i;
    Prepare();
    config.joints[ARM_JOINT_GRIPPER] = config.joints[0];
    config.joints[ARM_JOINT_GRIPPER].servo_id = 3U;
    CHECK(Arm_Configure(&config) == ARM_OK);
    CHECK(Arm_SetMotionAllowed(true) == ARM_OK);
    step.joint_mask = (uint8_t)(1U << ARM_JOINT_GRIPPER);
    step.position[ARM_JOINT_GRIPPER] = 500U;
    step.move_ms = 1000U;
    CHECK(Arm_StartSequence(&step, 1U) == ARM_INVALID_ARGUMENT);
    CHECK(Arm_StartOriginalPreset(&step) == ARM_OK);
    Arm_Process();
    CHECK(strcmp(frames[0], "{#003P0500T1000!}") == 0);
    for (i = 0U; i < 10U; ++i) { tick += 100U; Arm_Process(); }
    CHECK(Arm_GetStatus().state == ARM_COMPLETE_ESTIMATED);
    CHECK(Arm_SetMotionAllowed(false) == ARM_OK);
    CHECK(Arm_SetMotionAllowed(true) == ARM_OK);
    step.position[ARM_JOINT_GRIPPER] = 2500U;
    CHECK(Arm_StartOriginalPreset(&step) == ARM_OK);
    Arm_Process();
    CHECK(strcmp(frames[1], "{#003P2500T1000!}") == 0);
    CHECK(Arm_SetMotionAllowed(false) == ARM_OK);
    FinishStop();
}

static void TestCancellationAndFaults(void)
{
    ArmConfig_t config = ValidConfig();
    Prepare();
    CHECK(Arm_MoveJoint(ARM_JOINT_SHOULDER, 1500U, 100U, 0U) == ARM_OK);
    CHECK(Arm_SetMotionAllowed(false) == ARM_OK); /* Before first TX. */
    CHECK(Arm_Stop() == ARM_OK); /* Repeated cancellation does not reset progress. */
    CHECK(Arm_SetMotionAllowed(true) == ARM_BUSY);
    Arm_Process(); CHECK(tx_count == 1U && strcmp(frames[0], "$DST:7!") == 0);
    CHECK(Arm_Stop() == ARM_OK);
    FinishStop(); CHECK(tx_count == 2U && strcmp(frames[1], "$DST:12!") == 0);
    CHECK(Arm_MoveJoint(ARM_JOINT_SHOULDER, 1500U, 100U, 0U) == ARM_INHIBITED);

    Prepare();
    CHECK(Arm_MoveJoint(ARM_JOINT_SHOULDER, 1500U, 100U, 0U) == ARM_OK);
    next_tx = HAL_TIMEOUT; Arm_Process();
    CHECK(Arm_GetStatus().state == ARM_STOPPING);
    CHECK(Arm_GetStatus().error == ARM_ERROR_TRANSPORT);
    CHECK(Arm_GetStatus().transport_result == ZLIS2_UART_ERROR);
    next_tx = HAL_ERROR; Arm_Process(); /* Stop failure still attempts the other joint. */
    FinishStop();
    CHECK(tx_count == 3U && strcmp(frames[2], "$DST:12!") == 0);
    CHECK(Arm_GetStatus().state == ARM_FAULT && Arm_GetStatus().stop_delivery_failed);
    CHECK(Arm_SetMotionAllowed(true) == ARM_FAULT_LATCHED);
    CHECK(Arm_Configure(&config) == ARM_FAULT_LATCHED);
    CHECK(Arm_MoveJoint(ARM_JOINT_SHOULDER, 1500U, 100U, 0U) == ARM_FAULT_LATCHED);
    Arm_Process(); CHECK(tx_count == 3U);
    CHECK(Arm_ClearFault() == ARM_OK && !Arm_GetStatus().motion_allowed);

    Prepare();
    CHECK(Arm_MoveJoint(ARM_JOINT_SHOULDER, 1500U, 100U, 0U) == ARM_OK);
    tick += ARM_SERVICE_TIMEOUT_MS + 1U; Arm_Process();
    CHECK(tx_count == 1U && strcmp(frames[0], "$DST:7!") == 0);
    FinishStop(); CHECK(Arm_GetStatus().error == ARM_ERROR_SERVICE_TIMEOUT);

    Prepare(); CHECK(Arm_Stop() == ARM_OK);
    next_tx = HAL_BUSY; Arm_Process(); FinishStop();
    CHECK(Arm_GetStatus().error == ARM_ERROR_STOP_TRANSPORT);
}

static void TestExtensionAndMissingTransport(void)
{
    ArmConfig_t config = ValidConfig();
    Prepare();
    config.joints[4] = config.joints[0]; config.joints[4].servo_id = 254U;
    CHECK(Arm_Configure(&config) == ARM_OK);
    CHECK(Arm_SetMotionAllowed(true) == ARM_OK);
    CHECK(Arm_MoveJoint(ARM_JOINT_AUX_1, 1200U, 100U, 0U) == ARM_OK);
    Arm_Process(); CHECK(strcmp(frames[0], "{#254P1200T0100!}") == 0);
    CHECK(Arm_Stop() == ARM_OK); FinishStop();
    CHECK(tx_count == 4U && strcmp(frames[3], "$DST:254!") == 0);

    Prepare(); CHECK(ZLIS2_Init(NULL) == ZLIS2_INVALID_PARAM);
    CHECK(Arm_MoveJoint(ARM_JOINT_SHOULDER, 1500U, 100U, 0U) == ARM_OK);
    Arm_Process(); FinishStop();
    CHECK(tx_count == 0U && Arm_GetStatus().state == ARM_FAULT);
    CHECK(Arm_GetStatus().transport_result == ZLIS2_NOT_INITIALIZED);
}

int main(void)
{
    uart.Instance = &uart;
    uart.Init.BaudRate = ZLIS2_BAUD_RATE;
    uart.Init.Mode = UART_MODE_TX_RX;
    uart.gState = HAL_UART_STATE_READY;
    CHECK(ZLIS2_Init(&uart) == ZLIS2_OK);
    TestConfiguration(); TestValidation(); TestSequence();
    TestOriginalPresetAndReset();
    TestProjectTravel();
    TestGapPreset();
    TestCancellationAndFaults(); TestExtensionAndMissingTransport();
    printf("PASS arm: %u checks; limits, silent boot, ownership, sequence copies, timing/wrap, stop/fault paths.\n", checks);
    puts("Host HAL simulation only; no hardware movement or feedback validation.");
    return 0;
}

