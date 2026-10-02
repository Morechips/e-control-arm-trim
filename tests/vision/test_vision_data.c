#include "vision_data.h"
#include <assert.h>
#include <stdio.h>

typedef VisionDataStatus_t (*DecodeFn)(const uint8_t *, size_t, VisionTask_t *);

static void expect_task(const VisionTask_t *task, VisionColor_t explosive,
                        VisionColor_t target, VisionShape_t rescue)
{
    assert(task->explosive_color == explosive);
    assert(task->counterterror_target_color == target);
    assert(task->rescue_target_shape == rescue);
}

static void check_combinations(void)
{
    const VisionColor_t colors[] = {
        VISION_COLOR_RED, VISION_COLOR_GREEN, VISION_COLOR_BLUE
    };
    const VisionShape_t shapes[] = {
        VISION_SHAPE_CYLINDER, VISION_SHAPE_CONE, VISION_SHAPE_WAIST_DRUM
    };
    size_t a, b, c;
    VisionTask_t task = {0};

    for (a = 0U; a < 3U; ++a)
    {
        for (b = 0U; b < 3U; ++b)
        {
            for (c = 0U; c < 3U; ++c)
            {
                const uint8_t digits[] = {
                    (uint8_t)(a + 1U), (uint8_t)(b + 1U), (uint8_t)(c + 1U)
                };
                /* Exactly three bytes: no NUL terminator required. */
                const uint8_t ascii[] = {
                    (uint8_t)('1' + a), (uint8_t)('1' + b), (uint8_t)('1' + c)
                };
                assert(VisionData_DecodeDigits(digits, sizeof(digits), &task) == VISION_DATA_OK);
                expect_task(&task, colors[a], colors[b], shapes[c]);
                assert(VisionData_DecodeAscii(ascii, sizeof(ascii), &task) == VISION_DATA_OK);
                expect_task(&task, colors[a], colors[b], shapes[c]);
            }
        }
    }
}

static void check_invalid_bytes(DecodeFn decode, const uint8_t valid[3],
                                unsigned int minimum, unsigned int maximum)
{
    size_t position;
    unsigned int value;
    for (position = 0U; position < 3U; ++position)
    {
        for (value = 0U; value <= UINT8_MAX; ++value)
        {
            uint8_t input[] = {valid[0], valid[1], valid[2]};
            VisionTask_t task = {0};
            if (value >= minimum && value <= maximum)
            {
                continue;
            }
            /* Seed a valid result; a bad update must not partially overwrite it. */
            assert(decode(valid, 3U, &task) == VISION_DATA_OK);
            input[position] = (uint8_t)value;
            assert(decode(input, sizeof(input), &task) == VISION_DATA_INVALID_VALUE);
            expect_task(&task, VISION_COLOR_RED, VISION_COLOR_GREEN, VISION_SHAPE_WAIST_DRUM);
            assert(decode(valid, 3U, &task) == VISION_DATA_OK);
        }
    }
}

static void check_arguments(DecodeFn decode, const uint8_t valid[3])
{
    const size_t lengths[] = {0U, 1U, 2U, 4U, 6U, SIZE_MAX};
    VisionTask_t task = {0};
    size_t i;

    assert(decode(NULL, 3U, &task) == VISION_DATA_INVALID_ARGUMENT);
    expect_task(&task, VISION_COLOR_UNKNOWN, VISION_COLOR_UNKNOWN, VISION_SHAPE_UNKNOWN);
    assert(decode(valid, 3U, NULL) == VISION_DATA_INVALID_ARGUMENT);
    assert(decode(NULL, 0U, NULL) == VISION_DATA_INVALID_ARGUMENT);
    assert(decode(valid, 3U, &task) == VISION_DATA_OK);
    for (i = 0U; i < sizeof(lengths) / sizeof(lengths[0]); ++i)
    {
        assert(decode(valid, lengths[i], &task) == VISION_DATA_INVALID_LENGTH);
        expect_task(&task, VISION_COLOR_RED, VISION_COLOR_GREEN, VISION_SHAPE_WAIST_DRUM);
    }
}

int main(void)
{
    const uint8_t digits[] = {1U, 2U, 3U};
    const uint8_t ascii[] = {'1', '2', '3'};
    const uint8_t framed[] = {'[', '1', '2', '3', ']'};
    const uint8_t terminated[] = "123";
    VisionTask_t task = {0};

    check_combinations();
    check_invalid_bytes(VisionData_DecodeDigits, digits, 1U, 3U);
    check_invalid_bytes(VisionData_DecodeAscii, ascii, '1', '3');
    check_arguments(VisionData_DecodeDigits, digits);
    check_arguments(VisionData_DecodeAscii, ascii);
    assert(VisionData_DecodeAscii(terminated, sizeof(terminated), &task) == VISION_DATA_INVALID_LENGTH);
    assert(VisionData_DecodeAscii(terminated, sizeof(terminated) - 1U, &task) == VISION_DATA_OK);
    assert(VisionData_DecodeAscii(framed, sizeof(framed), &task) == VISION_DATA_INVALID_LENGTH);
    assert(VisionData_DecodeDigits(ascii, sizeof(ascii), &task) == VISION_DATA_INVALID_VALUE);
    assert(VisionData_DecodeAscii(digits, sizeof(digits), &task) == VISION_DATA_INVALID_VALUE);
    expect_task(&task, VISION_COLOR_RED, VISION_COLOR_GREEN, VISION_SHAPE_WAIST_DRUM);

    puts("PASS vision: all 27 combinations in both encodings / 1518 invalid-byte cases");
    puts("PASS vision: nulls / lengths / encoding separation / unchanged on failure / recovery");
    return 0;
}
