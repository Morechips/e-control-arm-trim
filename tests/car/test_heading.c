#include "heading_control.h"
#include "usart2_dma.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

extern unsigned mock_reset_count,mock_discard_count;
extern HAL_StatusTypeDef mock_reset_result;
static uint32_t now;
uint32_t HAL_GetTick(void) { return now; }
void Debug_Log(const char *text) { (void)text; }
uint8_t Debug_CanLog(uint8_t n) { (void)n;return 1U; }

static void Build(uint8_t *f,uint8_t kind,int16_t x,int16_t y,int16_t z)
{
    unsigned i;int16_t values[3]={x,y,z};memset(f,0,11);f[0]=0x55;f[1]=kind;
    for(i=0;i<3;i++){ f[2+i*2]=(uint8_t)values[i];f[3+i*2]=(uint8_t)((uint16_t)values[i]>>8); }
    for(i=0;i<10;i++) f[10]=(uint8_t)(f[10]+f[i]);
}
static void Sample(float yaw,float gz)
{
    uint8_t f[22];Build(f,0x52,0,0,(int16_t)(gz*32768.0f/2000.0f));
    Build(f+11,0x53,0,0,(int16_t)(yaw*32768.0f/180.0f));JY61_Parse(f,22,now);
}
static void Reset(void)
{
    now=0;mock_reset_count=mock_discard_count=0;mock_reset_result=HAL_OK;JY61_Init();Heading_Init();
}
static void ExpectState(HeadingState state) { assert(Heading_GetStatus()->state==state); }
static void Establish(float raw_yaw)
{
    Sample(raw_yaw,0);assert(Heading_Update(true)==0);
    assert(Heading_GetStatus()->reference_valid && Heading_GetStatus()->target==0.0f);
    ExpectState(HEADING_HOLD);
}

static void TestParser(void)
{
    uint8_t f[11], noise[]={0x12,0x55,0x55,0x54,0x77};unsigned i;
    Reset();JY61_Parse(noise,sizeof(noise),now);
    Build(f,0x51,16384,-16384,-32768);
    for(i=0;i<11;i++) JY61_Parse(f+i,1,now);
    assert(JY61_GetData()->ax==8 && JY61_GetData()->ay==-8 && JY61_GetData()->az==-16);
    Build(f,0x52,16384,-16384,-32768);JY61_Parse(f,11,now);
    assert(JY61_GetData()->gx==1000 && JY61_GetData()->gy==-1000 && JY61_GetGyroZ()==-2000);
    Build(f,0x53,16384,-16384,-32768);JY61_Parse(f,11,now);
    assert(JY61_IsValid() && JY61_GetData()->roll==90 && JY61_GetData()->pitch==-90 && JY61_GetYaw()==-180);
    f[10]++;JY61_Parse(f,11,now);assert(jy61_checksum_errors==1 && JY61_GetYaw()==-180);
    Build(f,0x53,0,0,16384);JY61_Parse(f,5,now);JY61_Parse(f,11,now);assert(JY61_GetYaw()==90);
    now=JY61_TIMEOUT_MS+1U;Build(f,0x51,0,0,0);JY61_Parse(f,11,now);assert(!JY61_IsValid());
    Sample(5,0);assert(JY61_IsValid());now+=JY61_TIMEOUT_MS;assert(JY61_IsValid());now++;assert(!JY61_IsValid());
    JY61_Parse(f,11,0);assert(!JY61_IsValid());
    Sample(5,0);usart2_rx_uart_errors++;usart2_rx_errors++;
    JY61_Process();assert(JY61_IsValid()); /* A byte error is not a stream gap. */
    Sample(5,0);usart2_rx_unread_batches++;JY61_Process();assert(!JY61_IsValid());
    puts("PASS JY61 parser: split/noise/resync/checksum/51-52-53/signed scaling/freshness/stream loss");
}

static void TestCapturedUSART2Frames(void)
{
    /* Two consecutive groups captured on PD6. Type 0x54 must not disturb the
     * following 0x52 gyro and 0x53 yaw packets. */
    static const uint8_t captured[] = {
        0x55,0x51,0xCB,0xFF,0xC5,0xFF,0xFC,0xF7,0x15,0x0C,0x48,
        0x55,0x52,0xD1,0xFF,0xF6,0xFF,0xD4,0xFF,0x15,0x0C,0x60,
        0x55,0x53,0x34,0x81,0x02,0x01,0x08,0x00,0xAA,0x46,0x58,
        0x55,0x54,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xA9,
        0x55,0x51,0xCC,0xFF,0xC8,0xFF,0xF7,0xF7,0x03,0x0C,0x35,
        0x55,0x52,0x04,0x00,0x00,0x00,0xFD,0xFF,0x03,0x0C,0xB6,
        0x55,0x53,0x31,0x81,0x03,0x01,0x0D,0x00,0xAA,0x46,0x5B,
        0x55,0x54,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xA9
    };
    const JY61_Data *data;
    Reset();
    now = 10U;
    JY61_Parse(captured, 19U, now); /* Split inside the first gyro frame. */
    assert(!JY61_IsValid());
    JY61_Parse(captured + 19U, (uint16_t)(sizeof(captured) - 19U), now);
    data = JY61_GetData();
    assert(data->valid && data->gyro_sequence == 2U && data->yaw_sequence == 2U);
    assert(fabsf(data->gz - (-3.0f * 2000.0f / 32768.0f)) < 0.001f);
    assert(fabsf(data->yaw - (13.0f * 180.0f / 32768.0f)) < 0.001f);
    assert(jy61_checksum_errors == 0U);
    puts("PASS JY61 captured USART2 stream: gyro/yaw valid across 0x54 frames");
}

static void TestSoftwareZeroAndHold(void)
{
    Reset();assert(Heading_Update(true)==0);ExpectState(HEADING_WAIT_REFERENCE);
    Establish(20);assert(fabsf(Heading_GetStatus()->yaw_zero-20.0f)<0.02f && mock_reset_count==0);
    now+=10;Sample(25.1f,0);assert(Heading_Update(true)>0);ExpectState(HEADING_CORRECTING);
    now+=10;Sample(24.8f,0);assert(Heading_Update(true)>0);ExpectState(HEADING_CORRECTING);
    now+=10;Sample(22.0f,0);assert(Heading_Update(true)==0);ExpectState(HEADING_HOLD);
    now+=10;Sample(13,0);assert(Heading_Update(true)<0);
    assert(Heading_GetStatus()->target==0.0f && mock_reset_count==0);
    now+=JY61_TIMEOUT_MS+1U;assert(Heading_Update(true)==0);ExpectState(HEADING_FAULT);
    assert(Heading_GetStatus()->fault==HEADING_IMU_LOST && Heading_GetStatus()->reference_valid);
    now+=1;Sample(13,0);assert(Heading_Update(true)<0);
    assert(Heading_Update(false)==0 && !Heading_GetStatus()->heading_hold && !Heading_RequestReference());
    puts("PASS heading hold: software zero / hysteresis / PID sign / IMU zero fallback / target persistence / inhibit");
}

static void TestTargetAndWrap(void)
{
    Reset();Establish(0);
    assert(Heading_AdjustTarget(ANGLE_ADJUST_STEP_DEG));
    assert(Heading_GetStatus()->target==5.0f && Heading_Update(true)==-2);
    now+=10;Sample(5,0);assert(Heading_Update(true)==0);ExpectState(HEADING_HOLD);
    assert(Heading_GetStatus()->target==5.0f);
    now+=10;Sample(10.1f,0);assert(Heading_Update(true)>0);
    assert(Heading_GetStatus()->target==5.0f);
    assert(Heading_SetTarget(179.0f));
    now+=10;Sample(-179.0f,0);Heading_Update(true);
    assert(fabsf(Heading_GetStatus()->yaw_error+2.0f)<0.05f);
    assert(Heading_SetTarget(-179.0f));
    now+=10;Sample(179.0f,0);Heading_Update(true);
    assert(fabsf(Heading_GetStatus()->yaw_error-2.0f)<0.05f);
    assert(!Heading_SetTarget(NAN) && !Heading_AdjustTarget(INFINITY));
    assert(Heading_RequestReference());
    assert(Heading_GetStatus()->target==0.0f && Heading_GetStatus()->actual==0.0f);
    puts("PASS angle target: +/-5 adjustment / reached target retained / +/-180 wrap / explicit software re-zero");
}

static void TestRuntimePIDAndClamp(void)
{
    unsigned i;float first;
    Reset();Establish(0);now+=10;Sample(6,2);Heading_Update(true);
    assert(Heading_SetPID(0.5f,0,0));Heading_Update(true);
    assert(fabsf(Heading_GetStatus()->omega_correction+0.5f*Heading_GetStatus()->yaw_error)<0.0001f);
    assert(Heading_SetPID(1,0,0.1f));Heading_Update(true);
    assert(fabsf(Heading_GetStatus()->omega_correction+
        (Heading_GetStatus()->yaw_error-0.1f*JY61_GetGyroZ()))<0.001f);
    assert(Heading_SetPID(0,1,0));Heading_Update(true);
    assert(Heading_GetStatus()->omega_correction==0);
    now+=10;Sample(6,0);Heading_Update(true);first=Heading_GetStatus()->omega_correction;
    assert(fabsf(first-0.06f)<0.001f);
    for(i=0;i<100;i++) Heading_Update(true);
    assert(Heading_GetStatus()->omega_correction==first);
    now+=10;Sample(6,0);Heading_Update(true);
    assert(fabsf(Heading_GetStatus()->omega_correction-2*first)<0.001f);
    assert(Heading_SetPID(1000,0,0));
    now+=10;Sample(80,0);assert(Heading_Update(true)==(int16_t)MAX_YAW_CORRECTION_RPM);
    assert(fabsf(Heading_GetStatus()->omega_correction-MAX_YAW_CORRECTION_RPM)<0.001f);
    assert(!Heading_SetPID(-1,0,0) && !Heading_SetPID(0,-1,0) && !Heading_SetPID(0,0,-1));
    assert(!Heading_SetPID(NAN,0,0) && !Heading_SetPID(0,INFINITY,0));
    Heading_Init();assert(fabsf(Heading_GetPID()->kp-HEADING_KP)<0.000001f &&
                          fabsf(Heading_GetPID()->ki-HEADING_KI)<0.000001f &&
                          fabsf(Heading_GetPID()->kd-HEADING_KD)<0.000001f);
    puts("PASS runtime PID: live gains / elapsed-time I / no call-count integration / 100 RPM clamp / bounds / defaults");
}

int main(void)
{
    TestParser();TestCapturedUSART2Frames();TestSoftwareZeroAndHold();TestTargetAndWrap();TestRuntimePIDAndClamp();
    puts("ALL JY61/HEADING TESTS PASSED (simulation only)");return 0;
}
