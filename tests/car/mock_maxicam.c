#include "mock_maxicam.h"

static MaxiCamTargetData_t test_target;
static uint32_t test_sequence;
static uint32_t test_mode_count, test_mode_attempts;
static uint8_t test_last_mode;
static HAL_StatusTypeDef test_send_result;

void TestMaxiCam_Reset(void)
{
    test_target.type = (uint8_t)DETECT_UNKNOWN;
    test_target.offset_x = test_target.offset_y = 0;
    test_target.target_valid = false;
    test_sequence = 0U;
    test_mode_count = test_mode_attempts = 0U;
    test_last_mode = UINT8_MAX;
    test_send_result = HAL_OK;
}

void TestMaxiCam_SetSendResult(HAL_StatusTypeDef result)
{
    test_send_result = result;
}

uint32_t TestMaxiCam_ModeCount(void) { return test_mode_count; }
uint32_t TestMaxiCam_ModeAttempts(void) { return test_mode_attempts; }
uint8_t TestMaxiCam_LastMode(void) { return test_last_mode; }

HAL_StatusTypeDef MaxiCam_SendMode(uint8_t command)
{
    ++test_mode_attempts;
    if (test_send_result != HAL_OK) return test_send_result;
    test_last_mode = command;
    ++test_mode_count;
    test_target.target_valid = false;
    test_target.type = UINT8_MAX;
    ++test_sequence;
    return HAL_OK;
}

void TestMaxiCam_Publish(uint8_t type, int16_t offset_x, int16_t offset_y,
                         uint8_t valid)
{
    test_target.type = type;
    test_target.offset_x = offset_x;
    test_target.offset_y = offset_y;
    test_target.target_valid = valid != 0U;
    ++test_sequence;
}

void MaxiCam_GetTargetData(MaxiCamTargetData_t *data, uint32_t *frame_sequence)
{
    if (data != NULL) *data = test_target;
    if (frame_sequence != NULL) *frame_sequence = test_sequence;
}

void MaxiCam_RxCallback(UART_HandleTypeDef *uart)
{
    (void)uart;
}

void MaxiCam_ErrorCallback(UART_HandleTypeDef *uart)
{
    (void)uart;
}
