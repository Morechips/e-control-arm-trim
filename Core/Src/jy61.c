#include "uart_driver.h"
#include "jy61.h"
#include "serial_io.h"
#include "usart2_dma.h"
#include <stdio.h>
#include <string.h>
static JY61_Data imu;
static uint8_t frame[11], used;
static uint8_t raw_probe[44], raw_probe_count;
static bool raw_probe_logged;
static uint32_t stream_errors, previous_byte_ms, diagnostics_ms;
volatile uint32_t jy61_checksum_errors, jy61_stream_errors;
static const uint8_t jy61_heading_zero_cmd[5] = {0xFF,0xAA,0x01,0x04,0x00};
static int16_t Decode(const uint8_t *p)
{
    uint16_t u = (uint16_t)((uint16_t)p[0] | (uint16_t)p[1] << 8);
    return (int16_t)(u <= 32767U ? (int32_t)u : (int32_t)u - 65536);
}
static uint32_t StreamErrors(void)
{
    /* UART framing/noise errors affect individual bytes. The 11-byte checksum
     * and resynchronizer reject corrupt frames; only actual stream gaps must
     * invalidate both gyro and yaw until fresh packets arrive. */
    return usart2_rx_dma_errors + usart2_rx_overflows + usart2_rx_unread_batches;
}
static void LogRawProbe(void)
{
    char line[80];
    unsigned chunk, i;
    int written;
    if (raw_probe_count != sizeof(raw_probe) || raw_probe_logged || !Debug_CanLog(3U)) return;
    for (chunk = 0U; chunk < 3U; ++chunk) {
        unsigned start = chunk * 16U;
        unsigned end = start + 16U;
        if (end > sizeof(raw_probe)) end = sizeof(raw_probe);
        written = snprintf(line, sizeof(line), "[IMU RAW +%u]", start);
        for (i = start; i < end; ++i)
            written += snprintf(line + written, sizeof(line) - (size_t)written,
                                " %02X", (unsigned)raw_probe[i]);
        (void)snprintf(line + written, sizeof(line) - (size_t)written, "\r\n");
        Debug_Log(line);
    }
    raw_probe_logged = true;
}
static void LogReceiveDiagnostics(void)
{
    char line[80];
    uint32_t now = HAL_GetTick();
    long gyro_age, yaw_age;
    if ((uint32_t)(now - diagnostics_ms) < 1000U || !Debug_CanLog(6U)) return;
    diagnostics_ms = now;
    gyro_age = imu.gyro_sequence ? (long)(now - imu.gyro_ms) : -1L;
    yaw_age = imu.yaw_sequence ? (long)(now - imu.yaw_ms) : -1L;
    (void)snprintf(line, sizeof(line),
                   "[IMU] valid=%u poll=%lu rx=%lu ndtr=%u\r\n",
                   (unsigned)JY61_IsValid(), (unsigned long)usart2_rx_periods,
                   (unsigned long)usart2_rx_bytes, (unsigned)USART2_DMA_Remaining());
    Debug_Log(line);
    (void)snprintf(line, sizeof(line),
                   "[IMU] gyro=%lu age=%ld yaw=%lu age=%ld\r\n",
                   (unsigned long)imu.gyro_sequence, gyro_age,
                   (unsigned long)imu.yaw_sequence, yaw_age);
    Debug_Log(line);
    (void)snprintf(line, sizeof(line),
                   "[IMU] crc=%lu stream=%lu\r\n",
                   (unsigned long)jy61_checksum_errors,
                   (unsigned long)jy61_stream_errors);
    Debug_Log(line);
    (void)snprintf(line, sizeof(line),
                   "[IMU] rx_err=%lu ovf=%lu drop=%lu\r\n",
                   (unsigned long)usart2_rx_errors,
                   (unsigned long)usart2_rx_overflows,
                   (unsigned long)usart2_rx_unread_batches);
    Debug_Log(line);
    (void)snprintf(line, sizeof(line),
                   "[IMU] uart=%lu dma=%lu sr=%lX hisr=%lX\r\n",
                   (unsigned long)usart2_rx_uart_errors,
                   (unsigned long)usart2_rx_dma_errors,
                   (unsigned long)usart2_last_uart_flags,
                   (unsigned long)usart2_last_dma_flags);
    Debug_Log(line);
    (void)snprintf(line, sizeof(line),
                   "[IMU] ore=%lu ne=%lu fe=%lu pe=%lu\r\n",
                   (unsigned long)usart2_rx_ore_errors,
                   (unsigned long)usart2_rx_ne_errors,
                   (unsigned long)usart2_rx_fe_errors,
                   (unsigned long)usart2_rx_pe_errors);
    Debug_Log(line);
}
void JY61_Init(void)
{
    memset(&imu,0,sizeof(imu)); used = 0U;
    raw_probe_count = 0U;
    raw_probe_logged = false;
    jy61_checksum_errors = jy61_stream_errors = 0U;
    USART2_DMA_Init();
    (void)UART_BindDma(&huart2, USART2_DMA_ReadTimed);
    stream_errors = StreamErrors();
    previous_byte_ms = diagnostics_ms = HAL_GetTick();
}
bool JY61_IsValid(void)
{
    uint32_t now = HAL_GetTick();
    imu.valid = imu.gyro_valid && imu.yaw_valid &&
        (uint32_t)(now - imu.gyro_ms) <= JY61_TIMEOUT_MS &&
        (uint32_t)(now - imu.yaw_ms) <= JY61_TIMEOUT_MS;
    return imu.valid;
}
const JY61_Data *JY61_GetData(void) { (void)JY61_IsValid(); return &imu; }
float JY61_GetYaw(void) { return imu.yaw; }
float JY61_GetGyroZ(void) { return imu.gz; }
void JY61_Parse(const uint8_t *data, uint16_t length, uint32_t received_ms)
{
    uint16_t n;
    if (data == NULL) return;
    if ((uint32_t)(HAL_GetTick() - received_ms) > JY61_TIMEOUT_MS) {
        used = 0U; imu.gyro_valid = imu.yaw_valid = false; return;
    }
    if (used && (uint32_t)(received_ms - previous_byte_ms) > JY61_TIMEOUT_MS) used = 0U;
    previous_byte_ms = received_ms;
    for (n=0U; n<length; n++) {
        unsigned i;
        uint8_t sum = 0U;
        if (!used && data[n] != 0x55U) continue;
        frame[used++] = data[n];
        if (used == 2U && (frame[1] < 0x51U || frame[1] > 0x53U)) {
            used = frame[1] == 0x55U ? 1U : 0U; continue;
        }
        if (used < sizeof(frame)) continue;
        for (i=0U;i<10U;i++) sum = (uint8_t)(sum + frame[i]);
        if (sum == frame[10]) {
            float x = (float)Decode(frame+2), y = (float)Decode(frame+4), z = (float)Decode(frame+6);
            if (frame[1] == 0x51U) {
                imu.ax=x*(16.0f/32768.0f); imu.ay=y*(16.0f/32768.0f); imu.az=z*(16.0f/32768.0f);
            } else if (frame[1] == 0x52U) {
                imu.gx=x*(2000.0f/32768.0f); imu.gy=y*(2000.0f/32768.0f); imu.gz=z*(2000.0f/32768.0f);
                imu.gyro_ms=received_ms; imu.gyro_valid=true; imu.gyro_sequence++;
                /* Retain evidence of motion even if later frames in a DMA batch are stable. */
                if (imu.gz >= HEADING_GZ_STABLE || imu.gz <= -HEADING_GZ_STABLE) imu.gyro_unstable_sequence++;
            } else {
                imu.roll=x*(180.0f/32768.0f); imu.pitch=y*(180.0f/32768.0f); imu.yaw=z*(180.0f/32768.0f);
                imu.yaw_ms=received_ms; imu.yaw_valid=true; imu.yaw_sequence++;
            }
            imu.last_update_ms=received_ms; used=0U;
        } else {
            jy61_checksum_errors++;
            /* A corrupt sample is not evidence of continuous stillness. */
            imu.gyro_unstable_sequence++;
            /* Retain a possible nested header; DMA boundaries are not frames. */
            for (i=1U;i<sizeof(frame);i++) {
                if (frame[i]==0x55U && (i==10U || (frame[i+1]>=0x51U && frame[i+1]<=0x53U))) break;
            }
            used=(uint8_t)(sizeof(frame)-i);
            memmove(frame,frame+i,used);
        }
    }
    (void)JY61_IsValid();
}
void JY61_Process(void)
{
    uint8_t bytes[USART2_RX_BUFFER_SIZE];
    uint32_t errors;
    size_t count;
    USART2_DMA_Process();
    errors=StreamErrors();
    if (errors != stream_errors) {
        stream_errors=errors; jy61_stream_errors++;
        used=0U; imu.gyro_valid=imu.yaw_valid=false;
        USART2_DMA_DiscardPending();
    }
    (void)UART_RECV(bytes, sizeof(bytes), &huart2, &count, NULL);
    if (count != 0U) {
        uint32_t tick = UART_RxTick(&huart2);
        if (!raw_probe_logged && raw_probe_count < sizeof(raw_probe)) {
            uint16_t copy = (uint16_t)(sizeof(raw_probe) - raw_probe_count);
            if (copy > count) copy = count;
            memcpy(raw_probe + raw_probe_count, bytes, copy);
            raw_probe_count = (uint8_t)(raw_probe_count + copy);
        }
        JY61_Parse(bytes,count,tick);
    }
    (void)JY61_IsValid();
    LogRawProbe();
    LogReceiveDiagnostics();
}
HAL_StatusTypeDef JY61_ResetHeading(void)
{
    HAL_StatusTypeDef result=UART_SEND(jy61_heading_zero_cmd, sizeof(jy61_heading_zero_cmd),
                                       &huart2, UART_TX_BLOCKING, JY61_RESET_TX_TIMEOUT_MS);
    if (result==HAL_OK) {
        /* Discard pre-command software bytes/partial frame. Do NOT alter yaw,
         * and do NOT stop RX DMA. Confirmation requires a new full 0x53. */
        USART2_DMA_DiscardPending(); used=0U; imu.yaw_valid=false;
    }
    return result;
}
