#include "uart_tx_queue.h"
#include "pid_tuner.h"
#include "heading_control.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static PID_TunerStatus status;
static uint8_t replies[PID_REPLY_QUEUE_SIZE][PID_TX_LINE_SIZE];
static uint16_t lengths[PID_REPLY_QUEUE_SIZE];
static uint8_t transmitting[PID_TX_LINE_SIZE];
static UartTxQueue_t tx;
static uint32_t debug_tick;

/* Integer-only printf: newlib-nano float printf need not be enabled.
 * Scientific notation only for values too large for safe scaled uint32.
 * No artificial upper limit is imposed on the stored PID gains. */
static void Fixed(char *out, float value, unsigned int digits)
{
    uint32_t scale=digits==3U ? 1000U : 10U;
    uint32_t scaled;
    uint8_t exponent=0U; /* A finite float32 exponent is at most 38 here. */
    int length;
    double magnitude=value<0.0f ? -(double)value : (double)value;
    if (!isfinite(value)) { (void)snprintf(out,24U,"NA"); return; }
    if (magnitude>4000000.0) {
        while (magnitude>=10.0) { magnitude/=10.0; ++exponent; }
    }
    scaled=(uint32_t)(magnitude*scale+0.5);
    if (exponent) {
        length=snprintf(out,24U,"%s%lu.%0*luE+%u",value<0.0f ? "-" : "",
            (unsigned long)(scaled/scale),(int)digits,(unsigned long)(scaled%scale),(unsigned int)exponent);
    } else {
        length=snprintf(out,24U,"%s%lu.%0*lu",value<0.0f ? "-" : "",
            (unsigned long)(scaled/scale),(int)digits,(unsigned long)(scaled%scale));
    }
    if (length<0 || length>=24) (void)snprintf(out,24U,"NA");
}

static void TxEvent(UartTxQueue_t *q, UartQueueEvent_t event, const uint8_t *data,
                    uint16_t length, uint32_t tag, HAL_StatusTypeDef result)
{
    (void)q; (void)data; (void)length; (void)tag; (void)result;
    if (event == UART_QUEUE_START_FAILED || event == UART_QUEUE_TIMED_OUT) ++status.tx_errors;
}
static void Queue(const char *line, int length)
{
    if (length <= 0 || (size_t)length >= PID_TX_LINE_SIZE ||
        UART_TxQueue_Submit(&tx, (const uint8_t *)line, (size_t)length, 0U) != HAL_OK)
        ++status.replies_dropped;
}
void PID_Tuner_Init(void)
{
    memset(&status,0,sizeof(status));
    status.step_mode=PID_STEP_COARSE;
    const UartTxQueueConfig_t config = {
        .policy = UART_QUEUE_FIFO, .capacity = PID_REPLY_QUEUE_SIZE, .frame_size = PID_TX_LINE_SIZE,
        .timeout_ms = PID_TX_TIMEOUT_MS, .timeout_inclusive = 1U,
        .abort_failure_keeps_active = 1U, .retain_failed = 1U, .notify = TxEvent
    };
    (void)UART_TxQueue_Init(&tx, &huart6, &config, &replies[0][0], lengths, NULL, transmitting);
    debug_tick=HAL_GetTick();
}

bool PID_Tuner_QueueReply(const char *line)
{
    size_t length;
    if (line == NULL) return false;
    for (length = 0U; length < PID_TX_LINE_SIZE && line[length] != '\0'; ++length) {}
    if (length == 0U || length == PID_TX_LINE_SIZE || UART_TxQueue_Free(&tx) == 0U) {
        ++status.replies_dropped;
        return false;
    }
    Queue(line, (int)length);
    return true;
}

const PID_TunerStatus *PID_Tuner_GetStatus(void) { return &status; }

void PID_Tuner_PrintParameters(void)
{
    const HeadingPIDParameters *pid=Heading_GetPID();
    char p[24],i[24],d[24],line[PID_TX_LINE_SIZE];
    int length;
    Fixed(p,pid->kp,3U); Fixed(i,pid->ki,3U); Fixed(d,pid->kd,3U);
    length=snprintf(line,sizeof(line),"PID P=%s I=%s D=%s STEP=%s\r\n",p,i,d,
                    status.step_mode==PID_STEP_COARSE ? "COARSE" : "FINE");
    Queue(line,length);
}

void PID_Tuner_HandleCommand(uint8_t cmd)
{
    const HeadingPIDParameters *pid=Heading_GetPID();
    float p=pid->kp,i=pid->ki,d=pid->kd;
    float p_step=status.step_mode==PID_STEP_COARSE ? 0.1f : 0.01f;
    float id_step=status.step_mode==PID_STEP_COARSE ? 0.01f : 0.001f;
    switch (cmd) {
    case 1: p+=p_step; break;
    case 2: p-=p_step; break;
    case 3: i+=id_step; break;
    case 4: i-=id_step; break;
    case 5: d+=id_step; break;
    case 6: d-=id_step; break;
    case 7: status.step_mode=PID_STEP_COARSE; break;
    case 8: status.step_mode=PID_STEP_FINE; break;
    case 9: PID_Tuner_PrintParameters(); return;
    case 10:
        if (!status.debug_enabled) debug_tick=HAL_GetTick();
        status.debug_enabled=true;
        Queue("PID DEBUG=ON\r\n",(int)(sizeof("PID DEBUG=ON\r\n")-1U));
        return;
    case 11:
        status.debug_enabled=false;
        Queue("PID DEBUG=OFF\r\n",(int)(sizeof("PID DEBUG=OFF\r\n")-1U));
        return;
    default: return;
    }
    if (p<0.0f) p=0.0f;
    if (i<0.0f) i=0.0f;
    if (d<0.0f) d=0.0f;
    if (cmd<=6U) (void)Heading_SetPID(p,i,d);
    PID_Tuner_PrintParameters();
}

void PID_Tuner_TxCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart6) UART_TxQueue_Complete(&tx);
}
void PID_Tuner_Process(void)
{
    uint32_t now=HAL_GetTick();
    bool debug_due=status.debug_enabled && (uint32_t)(now-debug_tick)>=PID_DEBUG_INTERVAL_MS;
    uint8_t replies_due = (uint8_t)(UART_TxQueue_Pending(&tx) != 0U);
    if (UART_TxQueue_Process(&tx) != HAL_OK && UART_TxQueue_IsSending(&tx)) return;
    if (debug_due) debug_tick=now;
    if (replies_due || !UART_TxQueue_IsIdle(&tx)) {
        if (debug_due) ++status.debug_skipped;
        return;
    }
    if (debug_due) {
        const HeadingStatus *h=Heading_GetStatus();
        const HeadingPIDParameters *pid=Heading_GetPID();
        char t[24],a[24],e[24],o[24],p[24],i[24],d[24],line[PID_TX_LINE_SIZE];
        int length;
        Fixed(t,h->target,1U); Fixed(a,h->actual,1U);
        Fixed(e,h->yaw_error,1U); Fixed(o,h->omega_correction,1U);
        Fixed(p,pid->kp,3U); Fixed(i,pid->ki,3U); Fixed(d,pid->kd,3U);
        length=snprintf(line,sizeof(line),"CTRL T=%s A=%s E=%s O=%s P=%s I=%s D=%s\r\n",t,a,e,o,p,i,d);
        if (length<=0 || (size_t)length>=sizeof(line) ||
            UART_TxQueue_Submit(&tx, (const uint8_t *)line, (size_t)length, 0U) != HAL_OK) {
            ++status.debug_skipped;
        } else if (UART_TxQueue_Process(&tx) != HAL_OK) {
            UART_TxQueue_CancelPending(&tx, 0U);
            ++status.debug_skipped;
        }
    }
}
