#ifndef PID_TUNER_H
#define PID_TUNER_H
#include "main.h"
#include <stdbool.h>

#define PID_DEBUG_INTERVAL_MS 100U
#define PID_TX_TIMEOUT_MS 500U
#define PID_TX_LINE_SIZE 192U
#define PID_REPLY_QUEUE_SIZE 8U

typedef enum { PID_STEP_COARSE, PID_STEP_FINE } PID_StepMode;
typedef struct {
    PID_StepMode step_mode;
    bool debug_enabled;
    uint32_t replies_dropped, debug_skipped, tx_errors;
} PID_TunerStatus;

/* Foreground APIs. Init is silent; gains belong to Heading_Init/GetPID.
 * USART6 TX is owned here after the existing startup AT exchange.
 * Commands use one little-endian short: A5 cmd_lo cmd_hi sum 5A.
 * Valid commands are 1..11. Never feed unframed joystick bytes here.
 */
void PID_Tuner_Init(void);
void PID_Tuner_HandleCommand(uint8_t cmd);
void PID_Tuner_PrintParameters(void);
/* Call after safety/motor processing. Nonblocking; no periodic TX by default. */
void PID_Tuner_Process(void);
void PID_Tuner_TxCallback(UART_HandleTypeDef *uart);
const PID_TunerStatus *PID_Tuner_GetStatus(void);
bool PID_Tuner_QueueReply(const char *line);
#endif
