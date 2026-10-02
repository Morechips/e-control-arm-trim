#include "vision_data.h"

VisionDataStatus_t VisionData_DecodeDigits(const uint8_t *digits, size_t length,
                                         VisionTask_t *task)
{
    VisionTask_t decoded;
    size_t i;

    if (digits == NULL || task == NULL)
    {
        return VISION_DATA_INVALID_ARGUMENT;
    }
    if (length != VISION_TASK_DIGIT_COUNT)
    {
        return VISION_DATA_INVALID_LENGTH;
    }
    for (i = 0U; i < VISION_TASK_DIGIT_COUNT; ++i)
    {
        if (digits[i] < 1U || digits[i] > 3U)
        {
            return VISION_DATA_INVALID_VALUE;
        }
    }

    decoded.explosive_color = (VisionColor_t)digits[0];
    decoded.counterterror_target_color = (VisionColor_t)digits[1];
    decoded.rescue_target_shape = (VisionShape_t)digits[2];
    *task = decoded;
    return VISION_DATA_OK;
}

VisionDataStatus_t VisionData_DecodeAscii(const uint8_t *ascii, size_t length,
                                        VisionTask_t *task)
{
    uint8_t digits[VISION_TASK_DIGIT_COUNT];
    size_t i;

    if (ascii == NULL || task == NULL)
    {
        return VISION_DATA_INVALID_ARGUMENT;
    }
    if (length != VISION_TASK_DIGIT_COUNT)
    {
        return VISION_DATA_INVALID_LENGTH;
    }
    for (i = 0U; i < VISION_TASK_DIGIT_COUNT; ++i)
    {
        if (ascii[i] < (uint8_t)'1' || ascii[i] > (uint8_t)'3')
        {
            return VISION_DATA_INVALID_VALUE;
        }
        digits[i] = (uint8_t)(ascii[i] - (uint8_t)'0');
    }
    return VisionData_DecodeDigits(digits, sizeof(digits), task);
}
