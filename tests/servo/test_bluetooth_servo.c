#include "bluetooth_driver.h"
#include "board_inputs.h"
#include "board_input_config.h"
#include "car_control.h"
#include "servo.h"
#include "serial_io.h"
#include "heading_control.h"
#include "pid_tuner.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../phone20_packet.h"

UART_HandleTypeDef huart1, huart3, huart6;
GPIO_TypeDef mock_gpiob, mock_gpioc, mock_gpiod, mock_gpioe;
static uint32_t now;
static uint8_t pe4;
static unsigned tx_count, checks, conflicts;
static HAL_StatusTypeDef tx_result;
static char last_tx[SERVO_MAX_TX_LENGTH + 1U];
static char input_diagnostic[81], rx_diagnostic[81], servo_diagnostic[81];
static unsigned raw_header_observed;
static HeadingPIDParameters pid = {1.0f, 0.0f, 0.2f};
static HeadingStatus heading;
#define CHECK(c) do { ++checks; assert(c); } while (0)
static const char aim_tx[] = "{#000P1058T2000!#001P0821T2000!#002P0554T2000!#003P1499T2000!}";
static const char rst_tx[] = "{#000P1524T2000!#001P1163T2000!#002P1693T2000!#003P1486T2000!}";
static const char ref_tx[] = "{#000P1532T2000!#001P2219T2000!#002P1202T2000!}";
static const uint16_t buttons[] = {1U, 2U, 4U, 8U, 16U, 32U, 64U, 128U,
                                   BT_SERVO_BUTTON_TH_L, BT_SERVO_BUTTON_RST,
                                   BT_CONTROL_BOOL_AIM_BIT};
static const char *const expected[] = {
    "{#000P1717T2000!#001P2297T2000!#002P0884T2000!#003P0500T2000!}",
    "{#000P1356T2000!#001P1850T2000!#002P0698T2000!#003P1480T2000!}",
    "{#000P1356T2000!#001P1850T2000!#002P0698T2000!#003P0500T2000!}",
    "{#000P1566T2000!#001P1896T2000!#002P0673T2000!#003P0500T2000!}",
    "{#000P1566T2000!#001P1896T2000!#002P0673T2000!#003P1200T2000!}",
    "{#000P1667T2000!#001P1882T2000!#002P0651T2000!#003P1483T2000!}",
    "{#000P1667T2000!#001P1882T2000!#002P0651T2000!#003P0500T2000!}",
    "{#000P1800T2000!#001P1855T2000!#002P0528T2000!#003P0500T2000!}",
    "{#000P1673T2000!#001P1394T2000!#002P0732T2000!#003P0500T2000!}",
    rst_tx, aim_tx
};

uint32_t HAL_GetTick(void) { return now; }
static CarRemoteInput_t submitted_input;
static unsigned submitted_count, invalidations;
void Car_Control_SubmitRemoteInput(const CarRemoteInput_t *input) { submitted_input = *input; ++submitted_count; }
void Car_Control_InvalidateRemoteInput(void) { ++invalidations; }
void Car_Control_SubmitLocalInput(const CarLocalInput_t *input) { (void)input; }
void Laser_SetManualRequest(bool requested) { (void)requested; }
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *gpio)
{ (void)port; CHECK(gpio->Mode == GPIO_MODE_INPUT && gpio->Pull == GPIO_PULLUP); }
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{ (void)port; (void)pin; (void)state; }
static void Dispatch(void)
{
    mock_gpioe.IDR = UINT16_MAX & (pe4 ? (uint32_t)~GPIO_PIN_4 : UINT16_MAX);
    Bluetooth_DispatchServoActions(BoardInputs_TakeServoAimPress());
}
const HeadingPIDParameters *Heading_GetPID(void) { return &pid; }
const HeadingStatus *Heading_GetStatus(void) { return &heading; }
bool Heading_SetPID(float p, float i, float d) { pid.kp=p; pid.ki=i; pid.kd=d; return true; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u, uint8_t *p, uint16_t n)
{ CHECK(n == 1U); u->rx=p; u->RxState=1U; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *u) { (void)u; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u, const uint8_t *p, uint16_t n)
{
    if (u == &huart1) {
        char log[81];
        CHECK(n < sizeof(log)); memcpy(log,p,n); log[n]='\0';
        if (strstr(log,"[PE4]")) memcpy(input_diagnostic,log,n+1U);
        if (strstr(log,"[BT RX]")) memcpy(rx_diagnostic,log,n+1U);
        if (strstr(log,"BT RAW len=41: A5")) ++raw_header_observed;
        if (strstr(log,"[SERVO]")) memcpy(servo_diagnostic,log,n+1U);
        if (strstr(log,"[SERVO]") && !strstr(log,"dropped=0")) ++conflicts;
        Debug_TxCallback(u);
    } else if (u == &huart3) {
        CHECK(n < sizeof(last_tx));
        memcpy(last_tx,p,n); last_tx[n]='\0'; ++tx_count;
        if (tx_result != HAL_OK) return tx_result;
        u->gState = HAL_UART_STATE_READY;
        Servo_TxCallback(u);
    } else { CHECK(u == &huart6); PID_Tuner_TxCallback(u); }
    return HAL_OK;
}
static void Drain(void) { unsigned i; for(i=0U;i<16U;++i) Debug_Process(); }
static void Feed(const void *p, size_t n)
{
    const uint8_t *bytes=p;
    while(n--) { *huart6.rx=*bytes++; huart6.RxState=HAL_UART_STATE_READY; Bluetooth_RxCallback(&huart6); }
}
static void Reset(void)
{
    Drain(); now += 2000U; pe4=0U; tx_count=conflicts=0U; tx_result=HAL_OK;
    mock_gpiob.IDR = mock_gpioc.IDR = mock_gpiod.IDR = mock_gpioe.IDR = UINT16_MAX;
    last_tx[0]='\0'; Bluetooth_Init(); PID_Tuner_Init(); BoardInputs_Init();
    submitted_count=invalidations=0U;
    Dispatch(); CHECK(tx_count == 0U);
}
static void Packet(unsigned length, uint16_t mask, uint16_t gap, int consume)
{
    uint8_t p[BT_MAX_FRAME_SIZE]={0xA5U};
    unsigned i;
    p[1]=(uint8_t)mask; p[2]=(uint8_t)(mask>>8);
    if(length!=41U) { p[27]=0xF8U; p[28]=0xFFU; }
    if(length==35U) { p[29]=(uint8_t)gap; p[30]=(uint8_t)(gap>>8); }
    if(length==41U) { p[35]=(uint8_t)gap; p[36]=(uint8_t)(gap>>8); }
    for(i=1U;i<length-2U;++i) p[length-2U]+=p[i];
    p[length-1U]=0x5AU;
    Feed(p,length); Bluetooth_Process();
    if(consume) { Dispatch(); Drain(); }
}
static void Short(uint8_t value, int consume)
{
    uint8_t p[5]={0xA5U,value,0U,value,0x5AU};
    Feed(p,sizeof(p)); Bluetooth_Process();
    if(consume) { Dispatch(); Drain(); }
}
static void Expect(unsigned count, const char *bytes)
{ CHECK(tx_count==count && strcmp(last_tx,bytes)==0); }

static void TestPoses(void)
{
    const unsigned lengths[] = {31U,35U,41U};
    unsigned length,index,i,before;
    for(index=0U;index<3U;++index) {
        length=lengths[index];
        Reset();
        for(i=0U;i<sizeof(buttons)/sizeof(buttons[0]);++i) {
            before=tx_count;
            Packet(length,buttons[i],500U,1);
            Expect(before+1U,expected[i]);
            const ServoCode poses[] = {TakeBall_Before, TakeBall_Mid,
                TakeBall_Gap, BarrelDown_Up, BarrelDown_Down,
                TakeHostage_Catch, TakeHostage_Gap, TakeHostage_Up,
                TakeHostage_Leave, Servo_RST, Servo_AIM};
            CHECK(strstr(servo_diagnostic, Servo_GetName(poses[i])) != NULL);
            Packet(length,buttons[i],500U,1); CHECK(tx_count==before+1U);
            Packet(length,0U,500U,1);
            Packet(length,buttons[i],500U,1); Expect(before+2U,expected[i]);
            /* Next iteration directly switches without waiting for motion. */
        }
        CHECK(Bluetooth_GetAimSequence()==2U);
    }
    puts("PASS direct servo: every phone pose, 31/35/41-byte layouts, exact bytes, edge/hold/rapid switch");
}

static void TestGapAndLink(void)
{
    Reset();
    Packet(35U,0U,500U,1); CHECK(tx_count==0U);
    Packet(35U,0U,1500U,1); Expect(1U,"{#003P1500T2000!}");
    Packet(35U,0U,1500U,1); CHECK(tx_count==1U);
    Packet(35U,0U,0U,1); CHECK(tx_count==1U);
    Packet(35U,0U,500U,1); Expect(2U,"{#003P0500T2000!}");
    Packet(35U,BT_CONTROL_BOOL_AIM_BIT,800U,1); Expect(3U,aim_tx);
    Packet(35U,BT_CONTROL_BOOL_AIM_BIT,800U,1); CHECK(tx_count==3U);
    now+=501U; Dispatch(); CHECK(tx_count==3U);
    Short(BT_SERVO_ONE_PRESS_CMD,1); CHECK(tx_count==3U); /* Offline short is consumed. */
    Packet(35U,0U,2000U,1); CHECK(tx_count==3U);
    now+=501U; /* Reconnect received before servo dispatch observed the timeout. */
    Packet(35U,0U,500U,1); CHECK(tx_count==3U);
    Packet(35U,0U,600U,1); Expect(4U,"{#003P0600T2000!}");
    Packet(35U,BT_CONTROL_BOOL_AIM_BIT,600U,1); Expect(5U,aim_tx);
    now+=501U; Packet(35U,BT_CONTROL_BOOL_AIM_BIT,500U,1); Expect(6U,aim_tx);
    Reset(); Packet(35U,BT_CONTROL_BOOL_AIM_BIT,500U,1); Expect(1U,aim_tx);
    now=UINT32_MAX-200U; Reset(); /* Explicit wrap for lease and edge handling. */
    now=UINT32_MAX-100U; Packet(35U,0U,500U,1);
    now=20U; Packet(35U,0U,700U,1); Expect(1U,"{#003P0700T2000!}");
    puts("PASS GAP/link: baseline, change, zero rearm, reconnect, wrap and no automatic stop");
}

static void TestPriority(void)
{
    unsigned before;
    Reset(); Packet(35U,0U,500U,1);
    Packet(35U,BT_CONTROL_BOOL_AIM_BIT|BT_SERVO_BUTTON_RST|3U,800U,1);
    Expect(1U,aim_tx); CHECK(conflicts==1U);
    Packet(35U,BT_CONTROL_BOOL_AIM_BIT|BT_SERVO_BUTTON_RST|3U,800U,1);
    CHECK(tx_count==1U);
    Packet(35U,0U,800U,1);
    Packet(35U,BT_SERVO_BUTTON_RST|3U,900U,1); Expect(2U,rst_tx);
    Packet(35U,0U,900U,1);
    Packet(35U,3U,1000U,1); Expect(3U,expected[0]);
    Packet(35U,2U,1000U,1); CHECK(tx_count==3U); /* Losing edge not deferred. */
    Packet(35U,0U,1000U,1);
    Short(BT_SERVO_ONE_PRESS_CMD,0);
    Packet(35U,BT_SERVO_BUTTON_TH_L,1200U,1); Expect(4U,expected[8]);
    Short(BT_SERVO_ONE_RELEASE_CMD,1);
    Short(BT_SERVO_ONE_PRESS_CMD,0);
    Packet(35U,0U,1300U,1); Expect(5U,ref_tx);
    Packet(35U,0U,1300U,1); CHECK(tx_count==5U);
    pe4=1U; Dispatch(); now+=20U;
    Short(BT_SERVO_ONE_RELEASE_CMD,0); Short(BT_SERVO_ONE_PRESS_CMD,0);
    Packet(35U,BT_CONTROL_BOOL_AIM_BIT|BT_SERVO_BUTTON_RST|1U,1400U,1);
    Expect(6U,aim_tx); CHECK(Bluetooth_GetAimSequence()==2U);
    before=tx_count;
    now+=50U; Packet(35U,BT_CONTROL_BOOL_AIM_BIT|BT_SERVO_BUTTON_RST|1U,1400U,1);
    CHECK(tx_count==before);
    puts("PASS priority: PE4>AIM>RST>bool index>reference>GAP; one TX and no delayed losers");
}

static void TestErrorsAndPhysical(void)
{
    unsigned failure;
    Reset();
    for(failure=HAL_ERROR;failure<=HAL_TIMEOUT;++failure) {
        unsigned before=tx_count;
        tx_result=(HAL_StatusTypeDef)failure;
        Packet(35U,BT_CONTROL_BOOL_AIM_BIT,500U,1); Expect(before+1U,aim_tx);
        if (failure == HAL_BUSY) CHECK(strstr(servo_diagnostic,"TX=6 dropped=1") != NULL);
        Dispatch(); Packet(35U,BT_CONTROL_BOOL_AIM_BIT,500U,1);
        CHECK(tx_count==before+1U); /* No retry or stop commands. */
        Packet(35U,0U,500U,1); tx_result=HAL_OK;
        Packet(35U,BT_CONTROL_BOOL_AIM_BIT,500U,1); Expect(before+2U,aim_tx);
        Packet(35U,0U,500U,1);
    }
#if CAR_TEST_INPUTS_ENABLE
    Reset(); pe4=1U; Dispatch(); now+=19U; Dispatch();
    CHECK(tx_count==0U); now+=1U; Dispatch(); Expect(1U,aim_tx);
    now+=1000U; Dispatch(); CHECK(tx_count==1U);
    pe4=0U; Dispatch(); now+=20U; Dispatch();
    pe4=1U; Dispatch(); now+=20U; Dispatch(); Expect(2U,aim_tx);
    mock_gpioe.IDR &= ~GPIO_PIN_4; BoardInputs_Init(); now+=50U; Dispatch(); CHECK(tx_count==2U);
#else
    Reset(); pe4=1U; Dispatch(); now+=1000U; Dispatch();
    CHECK(tx_count==0U); /* A held PE4 cannot enter the production chain. */
#endif
    Reset(); Packet(35U,0U,500U,1);
    Short(BT_SERVO_ONE_PRESS_CMD,1); Expect(1U,ref_tx);
    Short(BT_SERVO_ONE_PRESS_CMD,1); CHECK(tx_count==1U);
    Short(BT_SERVO_ONE_RELEASE_CMD,1); Short(BT_SERVO_ONE_PRESS_CMD,1); Expect(2U,ref_tx);
    Feed("@ARM DUAL START\n",16U); Bluetooth_Process(); Dispatch();
    CHECK(tx_count==2U); /* Production profile never installs a legacy ARM handler. */
    puts("PASS transport errors: no latch/retry/stop; offline PE4 AIM, debounce, short reference, legacy text inactive");
}

static void TestInputDiagnostics(void)
{
    uint8_t unsupported[45] = {0xA5U};
    Reset();
    Feed("+OK\r\n",5U); Bluetooth_Process(); Drain();
    now+=1000U; Bluetooth_Process(); Dispatch(); Drain();
    CHECK(strstr(rx_diagnostic,"bytes=5 seq=0 online=0 recover=0") != NULL);
#if CAR_TEST_INPUTS_ENABLE
    CHECK(strstr(input_diagnostic,"pin=1 presses=0 tx=0 last=255") != NULL);
#endif
    CHECK(tx_count==0U);
    Packet(35U,0U,500U,1);
    pe4=1U; Dispatch(); now+=20U; Dispatch(); Drain();
    now+=1000U; Bluetooth_Process(); Dispatch(); Drain();
    CHECK(strstr(rx_diagnostic,"bytes=40 seq=1 online=0 recover=0") != NULL);
#if CAR_TEST_INPUTS_ENABLE
    CHECK(strstr(input_diagnostic,"pin=0 presses=1 tx=1 last=0") != NULL);
    Expect(1U,aim_tx);
#else
    CHECK(input_diagnostic[0]=='\0' && tx_count==0U);
#endif
    Reset(); raw_header_observed=0U;
    unsupported[33]=0x2CU; unsupported[34]=0x02U;
    unsupported[37]=0x2EU; unsupported[38]=0x5AU;
    Feed(unsupported,sizeof(unsupported)); Bluetooth_Process(); Drain();
    CHECK(raw_header_observed > 0U);
    CHECK(Bluetooth_GetSequence()==0U && tx_count==0U);
    puts("PASS input diagnostics: AT bytes versus control frames, PE4 level and UART attempts");
}

static void TestPhone20Parsing(void)
{
    int16_t v[16] = {-128,1,0,1,0,1,0,1,0,1,0,1,-777,13,-1000,1000};
    uint8_t p[41], bad[41];
    unsigned i;
    const BluetoothControlFrame *c;
    /* Captured log tails: GAP=556 neutral checksum 2E, forward checksum 2F. */
    {
        int16_t neutral[16] = {0};
        PackPhone20(p,neutral,0U,556U); CHECK(p[35]==0x2CU && p[36]==2U && p[39]==0x2EU);
        neutral[1]=1; PackPhone20(p,neutral,0U,556U); CHECK(p[39]==0x2FU);
    }
    Reset(); PackPhone20(p,v,BT_CONTROL_BOOL_AIM_BIT|BT_SERVO_BUTTON_TH_L,556U);
    for(i=0U;i<40U;++i) {
        Feed(p+i,1U); Bluetooth_Process(); Dispatch();
        CHECK(Bluetooth_GetSequence()==0U && tx_count==0U);
    }
    Feed(p+40U,1U); Bluetooth_Process(); Dispatch(); Drain();
    c=&Bluetooth_GetControl()->frame;
    CHECK(Bluetooth_GetSequence()==1U && Bluetooth_IsConnected());
    CHECK(c->joy_y==-128 && c->joy_x==-777 && c->forward==1 && c->backward==0);
    CHECK(c->stop==1 && c->strafe_left==0 && c->strafe_right==1);
    CHECK(c->right_90==0 && c->right_180==1 && c->left_90==0 && c->Cam_T==0);
    CHECK(c->Shot==1 && c->aim==1 && c->gap_pwm==556U);
    CHECK(c->servo_buttons==BT_SERVO_BUTTON_TH_L && Bluetooth_GetArmSequence()==0U);
    Expect(1U,aim_tx);
    /* Corrupted checksum, out-of-range controls and GAP must stay rejected. */
    memcpy(bad,p,sizeof(bad)); bad[39]^=1U;
    Feed(bad,sizeof(bad)); Bluetooth_Process(); CHECK(Bluetooth_GetSequence()==1U);
    v[1]=2; PackPhone20(bad,v,0U,556U);
    Feed(bad,sizeof(bad)); Bluetooth_Process(); CHECK(Bluetooth_GetSequence()==1U);
    v[1]=0; PackPhone20(bad,v,0U,499U);
    Feed(bad,sizeof(bad)); Bluetooth_Process(); CHECK(Bluetooth_GetSequence()==1U);
    /* Valid frame after malformed input, and concatenated release/repress. */
    memset(v,0,sizeof(v)); PackPhone20(p,v,0U,556U);
    Feed(p,sizeof(p)); PackPhone20(p,v,BT_CONTROL_BOOL_AIM_BIT,556U); Feed(p,sizeof(p));
    Bluetooth_Process(); Dispatch(); CHECK(Bluetooth_GetSequence()==2U);
    Bluetooth_Process(); Dispatch(); CHECK(Bluetooth_GetSequence()==3U);
    Expect(2U,aim_tx);
    CHECK(Bluetooth_GetArmSequence()==0U);
    puts("PASS phone20: actual field order, byte splits, corrupt/range rejection, resync, burst AIM");
}

static void TestPublication(void)
{
    Reset();
    Packet(35U, 0U, 500U, 0);
    CHECK(submitted_count==1U && submitted_input.valid && submitted_input.sequence==1U && tx_count==0U);
    uint32_t received = submitted_input.received_tick;
    now += 10U;
    Short(BT_SERVO_ONE_PRESS_CMD, 0);
    CHECK(submitted_count==1U && submitted_input.received_tick==received && tx_count==0U);
    Short(9U, 0); /* PID query is not a chassis heartbeat. */
    CHECK(submitted_count==1U);
    unsigned before = invalidations;
    Bluetooth_ErrorCallback(&huart6);
    CHECK(invalidations==before+1U && tx_count==0U);
    Bluetooth_Process();
    Reset(); Packet(35U, 0U, 500U, 0);
    const uint8_t noise = 0x77U;
    for (unsigned i=0U;i<UART_RX_CAPACITY;++i) Feed(&noise,1U);
    CHECK(invalidations!=0U && submitted_count==1U && tx_count==0U);
    puts("PASS direct publication: control only, short commands do not refresh, ISR error/overflow invalidate without actuator calls");
}
int main(void)
{
    huart3.Instance=&huart3; huart3.Init.BaudRate=115200U;
    huart3.Init.Mode=UART_MODE_TX_RX; huart3.gState=HAL_UART_STATE_READY;
    CHECK(Servo_Init(&huart3)==SERVO_OK);
    TestPoses(); TestGapAndLink(); TestPriority(); TestErrorsAndPhysical();
    TestInputDiagnostics();
    TestPhone20Parsing(); TestPublication();
    printf("PASS direct servo: %u checks (HAL simulation, not hardware)\n",checks);
    return 0;
}
