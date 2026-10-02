#ifndef JY61_H
#define JY61_H
#include "main.h"
#include "heading_config.h"
#include <stdbool.h>
typedef struct {
    float ax, ay, az, gx, gy, gz, roll, pitch, yaw;
    uint32_t last_update_ms;
    bool valid;
    uint32_t gyro_ms, yaw_ms, gyro_sequence, yaw_sequence;
    uint32_t gyro_unstable_sequence;
    bool gyro_valid, yaw_valid;
} JY61_Data;
void JY61_Init(void);
void JY61_Process(void);
bool JY61_IsValid(void);
float JY61_GetYaw(void);
float JY61_GetGyroZ(void);
const JY61_Data *JY61_GetData(void);
HAL_StatusTypeDef JY61_ResetHeading(void);
/* Foreground parser entry, acquisition timestamp (not processing timestamp). */
void JY61_Parse(const uint8_t *data, uint16_t length, uint32_t received_ms);
extern volatile uint32_t jy61_checksum_errors, jy61_stream_errors;
#endif
