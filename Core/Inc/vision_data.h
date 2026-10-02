#ifndef VISION_DATA_H
#define VISION_DATA_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VISION_TASK_DIGIT_COUNT 3U

typedef enum
{
    VISION_COLOR_UNKNOWN = 0,
    VISION_COLOR_RED = 1,
    VISION_COLOR_GREEN = 2,
    VISION_COLOR_BLUE = 3
} VisionColor_t;

typedef enum
{
    VISION_SHAPE_UNKNOWN = 0,
    VISION_SHAPE_CYLINDER = 1,
    VISION_SHAPE_CONE = 2,
    VISION_SHAPE_WAIST_DRUM = 3
} VisionShape_t;

typedef struct
{
    VisionColor_t explosive_color;
    VisionColor_t counterterror_target_color;
    VisionShape_t rescue_target_shape;
} VisionTask_t;

typedef enum
{
    VISION_DATA_OK = 0,
    VISION_DATA_INVALID_ARGUMENT,
    VISION_DATA_INVALID_LENGTH,
    VISION_DATA_INVALID_VALUE
} VisionDataStatus_t;

/* Decode one complete payload, with exactly three values in task order.
 * Digits accepts numeric bytes 1..3; Ascii accepts characters '1'..'3'.
 * Ascii length excludes any NUL terminator. Neither API scans for a frame,
 * auto-detects encoding, checks a transport checksum, or accepts separators.
 * The caller must supply length readable bytes and a writable output object.
 * On failure, output is unchanged; only OK authorizes using a new result.
 * No retained state, UART access, allocation, or control actions. Concurrent
 * callers must use separate output objects or serialize access themselves.
 */
VisionDataStatus_t VisionData_DecodeDigits(const uint8_t *digits, size_t length,
                                         VisionTask_t *task);
VisionDataStatus_t VisionData_DecodeAscii(const uint8_t *ascii, size_t length,
                                        VisionTask_t *task);

#ifdef __cplusplus
}
#endif

#endif
