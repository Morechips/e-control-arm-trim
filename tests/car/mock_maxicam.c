#include "mock_maxicam.h"

static MaxiCamTargetData_t test_target;
static uint32_t test_sequence;
static uint32_t test_qr_notices;
static MaxiCamMode_t test_requested_mode, test_current_mode;
static uint32_t test_request_count, test_mode_count, test_mode_attempts;
static uint8_t test_last_mode;
static HAL_StatusTypeDef test_send_result;

void TestMaxiCam_Reset(void)
{
    test_target.type = (uint8_t)DETECT_UNKNOWN;
    test_target.offset_x = test_target.offset_y = 0;
    test_target.target_valid = false;
    test_sequence = 0U;
    test_qr_notices = 0U;
    test_requested_mode = test_current_mode = MAXICAM_MODE_NONE;
    test_request_count = test_mode_count = test_mode_attempts = 0U;
    test_last_mode = UINT8_MAX;
    test_send_result = HAL_OK;
}

void TestMaxiCam_SetSendResult(HAL_StatusTypeDef result)
{
    test_send_result = result;
}

uint8_t TestMaxiCam_LastMode(void) { return test_last_mode; }
uint32_t TestMaxiCam_ModeCount(void) { return test_mode_count; }
uint32_t TestMaxiCam_ModeAttempts(void) { return test_mode_attempts; }
MaxiCamMode_t TestMaxiCam_RequestedMode(void) { return test_requested_mode; }
uint32_t TestMaxiCam_RequestCount(void) { return test_request_count; }
bool TestMaxiCam_ModePending(void) { return MaxiCam_ModePending(); }

/* Mirrors the module: the request is only a wish, and a transmit that fails
 * simply leaves it pending for the next MaxiCam_Process() call. */
void MaxiCam_RequestMode(MaxiCamMode_t mode)
{
    if (mode != MAXICAM_MODE_NONE && mode != MAXICAM_MODE_OBJECT &&
        mode != MAXICAM_MODE_AIM) return;
    ++test_request_count;
    test_requested_mode = mode;
}

bool MaxiCam_ModePending(void)
{
    return test_requested_mode != MAXICAM_MODE_NONE &&
           test_requested_mode != test_current_mode;
}

MaxiCamMode_t MaxiCam_GetMode(void) { return test_current_mode; }

uint32_t MaxiCam_GetQrNoticeCount(void) { return test_qr_notices; }

bool MaxiCam_QrNotified(void) { return test_qr_notices != 0U; }

void MaxiCam_ResetQrNotice(void) { test_qr_notices = 0U; }

void TestMaxiCam_ModeStep(void)
{
    uint8_t command;
    if (!MaxiCam_ModePending()) return;   /* nothing wanted, or already sent */
    if (!MaxiCam_QrNotified()) return;    /* deferred until the camera notifies */
    ++test_mode_attempts;
    if (test_send_result != HAL_OK) return; /* stays pending and is retried */
    command = (test_requested_mode == MAXICAM_MODE_AIM) ? MODE_CMD_AIM
                                                        : MODE_CMD_OBJECT;
    test_last_mode = command;
    ++test_mode_count;
    test_current_mode = test_requested_mode;
}

void TestMaxiCam_PublishQrNotice(void)
{
    ++test_qr_notices;
    if (test_qr_notices == 0U) ++test_qr_notices;
    TestMaxiCam_ModeStep();
}

void MaxiCam_Init(void) { TestMaxiCam_Reset(); }

void MaxiCam_Process(void) { TestMaxiCam_ModeStep(); }

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
