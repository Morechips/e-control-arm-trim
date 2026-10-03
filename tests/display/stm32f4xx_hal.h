#ifndef DISPLAY_TEST_HAL_H
#define DISPLAY_TEST_HAL_H
#include <stdint.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { uint32_t busy, state, error; } I2C_HandleTypeDef;
#define RESET 0U
#define I2C_FLAG_BUSY 1U
#define HAL_I2C_STATE_READY 0U
#define HAL_I2C_ERROR_NONE 0U
#define __HAL_I2C_GET_FLAG(i, f) ((void)(f), (i)->busy)
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *, uint16_t, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *, uint16_t, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_I2C_Master_Transmit_IT(I2C_HandleTypeDef *, uint16_t, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *);
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *);
uint32_t HAL_I2C_GetState(I2C_HandleTypeDef *);
uint32_t HAL_I2C_GetError(I2C_HandleTypeDef *);
#endif
