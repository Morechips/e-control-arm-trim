#include "pid_tuner.h"
#include "heading_control.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static PID_TunerStatus status;
static char replies[PID_REPLY_QUEUE_SIZE][PID_TX_LINE_SIZE];
static uint16_t lengths[PID_REPLY_QUEUE_SIZE];
static uint8_t head, tail, count;
static char transmitting[PID_TX_LINE_SIZE];
static volatile uint8_t active;
static uint32_t tx_tick, debug_tick;

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

static void Queue(const char *line, int length)
{
    if (length<=0 || (size_t)length>=PID_TX_LINE_SIZE || count==PID_REPLY_QUEUE_SIZE) {
        ++status.replies_dropped;
        return;
    }
    memcpy(replies[head],line,(size_t)length);
    lengths[head]=(uint16_t)length;
    head=(uint8_t)((head+1U)%PID_REPLY_QUEUE_SIZE);
    ++count;
}

void PID_Tuner_Init(void)
{
    memset(&status,0,sizeof(status));
    status.step_mode=PID_STEP_COARSE;
    head=tail=count=active=0U;
    tx_tick=debug_tick=HAL_GetTick();
}

bool PID_Tuner_QueueReply(const char *line)
{
    size_t length;
    if (line == NULL) return false;
    for (length = 0U; length < PID_TX_LINE_SIZE && line[length] != '\0'; ++length) {}
    if (length == 0U || length == PID_TX_LINE_SIZE || count == PID_REPLY_QUEUE_SIZE) {
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
    if (uart==&huart6) active=0U;
}

static HAL_StatusTypeDef Start(const char *line, uint16_t length)
{
    HAL_StatusTypeDef result;
    memcpy(transmitting,line,length);
    tx_tick=HAL_GetTick();
    active=1U; /* Set before starting: completion may interrupt this call. */
    result=HAL_UART_Transmit_IT(&huart6,(const uint8_t *)transmitting,length);
    if (result!=HAL_OK) { active=0U; ++status.tx_errors; }
    return result;
}

void PID_Tuner_Process(void)
{
    uint32_t now=HAL_GetTick();
    bool debug_due=status.debug_enabled && (uint32_t)(now-debug_tick)>=PID_DEBUG_INTERVAL_MS;
    if (active && (uint32_t)(now-tx_tick)>=PID_TX_TIMEOUT_MS) {
        /* TX-only abort: never stop the joystick RX interrupt. No DMA here. */
        if (HAL_UART_AbortTransmit(&huart6)!=HAL_OK) return;
        active=0U; ++status.tx_errors;
    }
    if (debug_due) debug_tick=now; /* No backlog or catch-up burst. */
    if (active || count!=0U) {
        if (debug_due) ++status.debug_skipped;
        if (!active && Start(replies[tail],lengths[tail])==HAL_OK) {
            tail=(uint8_t)((tail+1U)%PID_REPLY_QUEUE_SIZE); --count;
        }
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
            Start(line,(uint16_t)length)!=HAL_OK) ++status.debug_skipped;
    }
}
