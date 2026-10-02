#ifndef DETECT_DATA_H
#define DETECT_DATA_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief MaxiCam QR-code recognition notification packet.
 * @note Recognition success is one binary byte: 0x80.
 */
#define QRCODE_NOTIFY_PACK 0x80U

/* Host -> MaxiCam: one byte, effective after the QR notification. */
#define MODE_CMD_AIM    0x00U
#define MODE_CMD_OBJECT 0x01U

typedef enum
{
    DETECT_REDBALL   = 0,
    DETECT_BLUEBALL  = 1,
    DETECT_GREANBALL = 2,

    DETECT_HOSTAGE_1 = 3,
    DETECT_HOSTAGE_2 = 4,
    DETECT_HOSTAGE_3 = 5,

    DETECT_BARREL    = 6,
    DETECT_TARGET    = 7,
    DETECT_UNKNOWN   = 8,
} DetectType;

/**
 * @brief Object-recognition position packet.
 * @note Five bytes: type, little-endian signed offset_x, little-endian signed
 * offset_y. Types below DETECT_UNKNOWN are valid targets; DETECT_UNKNOWN is an
 * explicit target-loss frame. Single-byte aligned as required by the protocol.
 */
#pragma pack(push, 1)
typedef struct
{
    uint8_t type;
    int16_t offset_x;
    int16_t offset_y;
} DetectData;
#pragma pack(pop)

#endif
