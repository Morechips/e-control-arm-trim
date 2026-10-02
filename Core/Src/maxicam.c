#include "maxicam.h"
#include "action_fsm.h"
#include "serial_io.h"
#include "vision_config.h"
#include <string.h>

static SerialRx maxicam_rx;
static MaxiCamTargetData_t target_data;
static uint32_t target_frame_sequence;
static uint8_t target_frame[sizeof(DetectData)];
static uint8_t target_frame_length;

static int16_t DecodeInt16LE(const uint8_t *bytes)
{
    return (int16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8));
}

static void ParseTargetFrame(void)
{
    target_data.type = target_frame[0];
    target_data.offset_x = DecodeInt16LE(&target_frame[1]);
    target_data.offset_y = DecodeInt16LE(&target_frame[3]);
    target_data.target_valid = target_frame[0] < (uint8_t)DETECT_UNKNOWN;
    ++target_frame_sequence;
    if (target_frame_sequence == 0U) ++target_frame_sequence;
    /* offset_y is decoded but is not used for vehicle alignment. */
}

void MaxiCam_Init(void)
{
    memset(&target_data, 0, sizeof(target_data));
    target_frame_sequence = 0U;
    target_frame_length = 0U;
    Serial_Init(&maxicam_rx, &huart4);
}

void MaxiCam_RxCallback(UART_HandleTypeDef *uart)
{
    Serial_RxCallback(&maxicam_rx, uart);
}

void MaxiCam_ErrorCallback(UART_HandleTypeDef *uart)
{
    Serial_ErrorCallback(&maxicam_rx, uart);
}

void MaxiCam_Process(void)
{
    uint8_t rx_byte;
    uint32_t rx_tick;

    if (Serial_Recover(&maxicam_rx))
    {
        /* A UART error can split a 5-byte position packet. Discard that partial
         * packet and publish target loss; never reinterpret its tail as QR. */
        target_frame_length = 0U;
        target_data.target_valid = false;
        target_data.type = UINT8_MAX; /* UART failure is not an explicit UNKNOWN frame. */
        ++target_frame_sequence;
        if (target_frame_sequence == 0U) ++target_frame_sequence;
    }
    while (Serial_Pop(&maxicam_rx, &rx_byte, &rx_tick))
    {
        (void)rx_tick;
        if (target_frame_length != 0U)
        {
            /* Already inside a position packet: 0x80 is payload here, not QR. */
            target_frame[target_frame_length++] = rx_byte;
            if (target_frame_length == sizeof(target_frame))
            {
                ParseTargetFrame();
                target_frame_length = 0U;
            }
        }
        else if (rx_byte == QR_SUCCESS_CODE)
        {
            /* QR success is a standalone one-byte packet and is recognized
             * only while waiting for the first byte of a new packet. */
            if (!qr_success)
            {
                Debug_Log("[QR] RX: 0x80\r\n");
                ActionFSM_SetQRSuccess(true);
            }
        }
        else if (rx_byte <= (uint8_t)DETECT_UNKNOWN)
        {
            target_frame[0] = rx_byte;
            target_frame_length = 1U;
        }
    }
}

void MaxiCam_GetTargetData(MaxiCamTargetData_t *data, uint32_t *frame_sequence)
{
    if (data != NULL) *data = target_data;
    if (frame_sequence != NULL) *frame_sequence = target_frame_sequence;
}

HAL_StatusTypeDef MaxiCam_SendMode(uint8_t command)
{
    HAL_StatusTypeDef status;
    if (command != MODE_CMD_AIM && command != MODE_CMD_OBJECT) return HAL_ERROR;
    status = HAL_UART_Transmit(&huart4, &command, 1U, MAXICAM_MODE_TX_TIMEOUT_MS);
    if (status == HAL_OK)
    {
        /* The previous mode may have left a partial position packet. */
        target_frame_length = 0U;
        target_data.target_valid = false;
        target_data.type = UINT8_MAX;
        ++target_frame_sequence;
        if (target_frame_sequence == 0U) ++target_frame_sequence;
    }
    return status;
}
