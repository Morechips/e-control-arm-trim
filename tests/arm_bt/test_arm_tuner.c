#include "arm_tuner.h"
#include "arm_control.h"
#include "bluetooth_driver.h"
#include "heading_control.h"
#include "pid_tuner.h"
#include "car_control.h"
#include "motor_driver.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

UART_HandleTypeDef huart1, huart3, huart6;
static uint32_t now;
static unsigned checks, tx_count, reply_count;
static char last_frame[SERVO_MAX_TX_LENGTH+1U], last_reply[PID_TX_LINE_SIZE];
static HAL_StatusTypeDef arm_tx = HAL_OK;
static HeadingPIDParameters pid = {1.0f, 0.0f, 0.2f};
static HeadingStatus heading;
void Car_Control_SubmitRemoteInput(const CarRemoteInput_t *input) { (void)input; }
void Car_Control_InvalidateRemoteInput(void) {}
void BoardInputs_TraceServo(uint32_t transmissions, unsigned result) { (void)transmissions; (void)result; }
CarState_t Car_Control_GetState(void) { return CAR_READY; }
uint8_t Motor_IsIdle(void) { return 1U; }
uint8_t Motor_HasFault(void) { return 0U; }
#define CHECK(c) do { ++checks; if (!(c)) { \
    fprintf(stderr,"FAIL line %u: %s\n",(unsigned)__LINE__,#c); exit(1); } } while (0)
uint32_t HAL_GetTick(void) { return now; }
const HeadingPIDParameters *Heading_GetPID(void) { return &pid; }
const HeadingStatus *Heading_GetStatus(void) { return &heading; }
bool Heading_SetPID(float p, float i, float d) { pid.kp=p; pid.ki=i; pid.kd=d; return true; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u, uint8_t *p, uint16_t n)
{ CHECK(n==1U); u->rx=p; u->RxState=1U; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *u) { (void)u; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u, const uint8_t *p, uint16_t n)
{
    if (u == &huart3) {
        HAL_StatusTypeDef result = arm_tx;
        CHECK(n<sizeof(last_frame));
        memcpy(last_frame,p,n); last_frame[n]='\0'; ++tx_count; arm_tx=HAL_OK;
        if (result == HAL_OK) { u->gState=HAL_UART_STATE_READY; Servo_TxCallback(u); }
        return result;
    }
    CHECK(u==&huart6 && n<sizeof(last_reply));
    memcpy(last_reply,p,n); last_reply[n]='\0'; ++reply_count;
    PID_Tuner_TxCallback(u);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u, const uint8_t *p, uint16_t n, uint32_t timeout)
{
    HAL_StatusTypeDef result=arm_tx;
    CHECK(u==&huart3 && n<sizeof(last_frame) && timeout==SERVO_TX_TIMEOUT_MS);
    memcpy(last_frame,p,n); last_frame[n]='\0'; ++tx_count; arm_tx=HAL_OK;
    return result;
}
static void Feed(const void *data, size_t length)
{
    const uint8_t *p=data;
    while (length--) { *huart6.rx=*p++; huart6.RxState=HAL_UART_STATE_READY; Bluetooth_RxCallback(&huart6); }
}
static void DrainReplies(void)
{
    unsigned i;
    for (i=0U;i<PID_REPLY_QUEUE_SIZE+1U;++i) PID_Tuner_Process();
}
static void Command(const char *line)
{
    last_reply[0]='\0';
    Feed(line,strlen(line)); Feed("\n",1U); Bluetooth_Process();
    ArmTuner_Process(); Arm_Process(); DrainReplies();
}
static void Expect(const char *part)
{
    if (strstr(last_reply,part)==NULL)
        fprintf(stderr,"Expected [%s], received [%s], tick=%lu state=%u\n",part,last_reply,
                (unsigned long)now,(unsigned)Arm_GetStatus().state);
    CHECK(strstr(last_reply,part)!=NULL);
}
static void Advance(unsigned ms)
{
    while (ms--) { ++now; ArmTuner_Process(); Arm_Process(); }
}
static void MakeCombined(uint8_t *packet, int command, int x, int y,
                         int arm_x, int arm_y, int brake, int forward,
                         int cam, int shot)
{
    const int values[16] = {
        x, y, forward, 0, brake, 0, 0, 0, 0,
        cam, shot, 0, 0, command, arm_x, arm_y
    };
    unsigned i;
    memset(packet, 0, BT_ARM_COMBINED_FRAME_SIZE);
    packet[0] = 0xA5U;
    for (i = 0U; i < 16U; ++i) {
        uint16_t value = (uint16_t)values[i];
        packet[1U + 2U * i] = (uint8_t)value;
        packet[2U + 2U * i] = (uint8_t)(value >> 8);
    }
    for (i = 1U; i <= 32U; ++i) packet[33] += packet[i];
    packet[34] = 0x5AU;
}
static void FinishStop(void) { Advance(8U); CHECK(Arm_GetStatus().state!=ARM_STOPPING); }
static void Move(void)
{
    Command("@ARM ARM"); Expect("RESULT=0");
    Command("@ARM MOVE 1500"); Expect("RESULT=0");
    CHECK(Arm_GetStatus().state==ARM_RUNNING);
}

static void RemotePacket(int direction, int x, int y, int brake)
{
    uint8_t packet[BT_ARM_COMBINED_FRAME_SIZE];
    MakeCombined(packet, direction, x, y, 0, 0, brake, 0, 0, 0);
    Feed(packet,sizeof(packet)); Bluetooth_Process();
    ArmTuner_Process(); Arm_Process(); DrainReplies();
}

static void PrepareRemote(void)
{
    Command("@ARM ENTER");
    Command("@ARM SYNC 1532 2219 1202"); Expect("RESULT=0");
    Command("@ARM TIME 300");
    Command("@ARM REMOTE ON"); Expect("REMOTE=1 NO_MOTION");
    RemotePacket(0,0,0,0);
    Command("@ARM ARM"); Expect("REMOTE_STREAM_REQUIRED");
}

static void TestRemote(void)
{
    unsigned before;
    RemotePacket(0,0,0,0);
    ArmTuner_Init(); before=tx_count;
    Command("@ARM ENTER");
    Command("@ARM SYNC 1532 2219 1202");
    Command("@ARM TIME 300");
    Command("@ARM REMOTE ON");
    Command("@ARM ARM"); Expect("REMOTE_SYNC_CENTER_TIME_REQUIRED");
    RemotePacket(13,0,0,0);
    Command("@ARM ARM"); Expect("REMOTE_SYNC_CENTER_TIME_REQUIRED");
    CHECK(tx_count==before);
    RemotePacket(0,0,0,0);
    Command("@ARM REMOTE SHOW"); Expect("REF=1 CENTER=1 CONNECTED=1");
    Command("@ARM ARM"); Expect("REMOTE_STREAM_REQUIRED");
    Command("@ARM JOG 1 0 0"); Expect("REMOTE_ACTIVE");
    Command("@ARM MOVE 1532"); Expect("REMOTE_ACTIVE");
    RemotePacket(13,0,0,0); Expect("DX=2 DZ=0 DPHI=0");
    CHECK(tx_count==before+1U);
    CHECK(strstr(last_frame,"#000P") && strstr(last_frame,"#001P") &&
          strstr(last_frame,"#002P") && !strstr(last_frame,"#003P"));
    RemotePacket(13,0,0,0); CHECK(tx_count==before+1U); /* No backlog. */
    Advance(100U); RemotePacket(0,0,0,0);
    Advance(201U); RemotePacket(0,0,0,0);
    CHECK(Arm_GetStatus().state==ARM_COMPLETE_ESTIMATED);
    CHECK(tx_count==before+1U); /* Release preserves reference. */
    RemotePacket(12,0,0,0); Expect("DX=-2 DZ=0 DPHI=0");
    Advance(301U);
    RemotePacket(11,0,0,0); Expect("DX=0 DZ=-2 DPHI=0");
    Advance(301U);
    RemotePacket(10,0,0,0); Expect("DX=0 DZ=2 DPHI=0");
    Advance(301U);
    before=tx_count;
    RemotePacket(0,100,-100,0); CHECK(tx_count==before); /* Deadzone. */
    RemotePacket(0,400,-400,0); Expect("DX=1 DZ=-1 DPHI=0");
    Advance(301U);
    RemotePacket(0,1000,-1000,0); Expect("DX=2 DZ=-2 DPHI=0");
    Advance(301U); RemotePacket(0,0,0,0);
    before=tx_count;
    Command("@ARM GRIP 899"); Expect("RESULT=1"); CHECK(tx_count==before);
    Command("@ARM GRIP 1500"); Expect("RESULT=0");
    CHECK(strcmp(last_frame,"{#003P1500T0300!}")==0);
    Advance(301U); RemotePacket(0,0,0,0);
    Command("@ARM REMOTE SHOW"); Expect("REF=1");
    RemotePacket(13,0,0,0); Expect("DX=2 DZ=0 DPHI=0");
    RemotePacket(0,0,0,1); FinishStop();
    CHECK(!Arm_GetStatus().motion_allowed);
    RemotePacket(0,0,0,0);
    Command("@ARM ARM"); Expect("REMOTE_SYNC_CENTER_TIME_REQUIRED");
    Command("@ARM SYNC 1532 2219 1202");
    Command("@ARM ARM"); Expect("REMOTE_STREAM_REQUIRED");
    Advance(301U); RemotePacket(0,0,0,0);
    Advance(201U); RemotePacket(0,0,0,0);
    CHECK(Arm_GetStatus().motion_allowed); /* Stream substitutes for PING. */
    Advance(501U); FinishStop();
    CHECK(!Arm_GetStatus().motion_allowed);
    RemotePacket(13,0,0,0);
    CHECK(!Arm_GetStatus().motion_allowed); /* Reconnection never rearms. */
    Command("@ARM EXIT");

    /* A late packet arriving in the same loop must not revive the lease. */
    PrepareRemote();
    now += BT_FAILSAFE_TIMEOUT_MS + 1U;
    RemotePacket(13,0,0,0); FinishStop();
    CHECK(!Arm_GetStatus().motion_allowed);
    Command("@ARM EXIT");
    PrepareRemote();
    Advance(400U); Command("@ARM PING");
    Advance(101U); FinishStop();
    CHECK(!Arm_GetStatus().motion_allowed); /* PING cannot hide stream loss. */
    Command("@ARM EXIT");

    before=tx_count; RemotePacket(13,0,0,0);
    CHECK(tx_count==before); /* Original car packets outside an arm session. */
    Command("@ARM ENTER");
    Command("@ARM SYNC 1532 2219 1202");
    Command("@ARM LIMIT 7 902 1569"); Expect("RESULT=0");
    Command("@ARM TIME 300");
    Command("@ARM REMOTE ON");
    Command("@ARM SYNC 1532 2219 1202");
    RemotePacket(0,0,0,0);
    before=tx_count;
    Command("@ARM ARM"); Expect("REMOTE_SYNC_CENTER_TIME_REQUIRED");
    RemotePacket(13,0,0,0); CHECK(tx_count==before);
    Command("@ARM EXIT");
}
static void TestNumericButtons(void)
{
    unsigned before, replies;
    uint32_t sequence, arm_sequence;
    ArmTuner_Init();
    RemotePacket(0,0,0,0);
    before=tx_count; sequence=Bluetooth_GetSequence();
    arm_sequence=Bluetooth_GetArmSequence();
    RemotePacket(20,0,0,0); Expect("SYNC_AND_CENTER_THEN_ARM");
    CHECK(tx_count==before && Bluetooth_GetSequence()==sequence+1U &&
          Bluetooth_GetArmSequence()==arm_sequence+1U);
    replies=reply_count;
    RemotePacket(20,0,0,0); CHECK(reply_count==replies);
    RemotePacket(0,0,0,0);
    RemotePacket(21,0,0,0); Expect("REMOTE_SYNC_CENTER_TIME_REQUIRED");
    CHECK(!Arm_GetStatus().motion_allowed);
    RemotePacket(0,0,0,0);
    Command("@ARM SYNC 1532 2219 1202"); Expect("RESULT=0");
    RemotePacket(0,0,0,0);
    RemotePacket(21,0,0,0); Expect("REMOTE_STREAM_REQUIRED");
    CHECK(Arm_GetStatus().motion_allowed && tx_count==before);
    RemotePacket(0,0,0,0);
    RemotePacket(13,0,0,0); Expect("DX=2 DZ=0 DPHI=0");
    Advance(301U); RemotePacket(0,0,0,0);
    RemotePacket(24,0,0,0); Expect("RESULT=0");
    CHECK(strcmp(last_frame,"{#003P1500T0200!}")==0);
    Advance(201U); RemotePacket(0,0,0,0);
    RemotePacket(25,0,0,0); Expect("RESULT=0");
    CHECK(strcmp(last_frame,"{#003P0900T0200!}")==0);
    RemotePacket(0,0,0,0);
    RemotePacket(22,0,0,0); FinishStop();
    CHECK(!Arm_GetStatus().motion_allowed);
    RemotePacket(0,0,0,0);
    RemotePacket(21,0,0,0); Expect("REMOTE_SYNC_CENTER_TIME_REQUIRED");
    RemotePacket(0,0,0,0);
    replies=reply_count; sequence=Bluetooth_GetSequence();
    arm_sequence=Bluetooth_GetArmSequence();
    RemotePacket(23,1,0,0); /* Arm button and chassis axes coexist. */
    CHECK(reply_count==replies+1U && Bluetooth_GetSequence()==sequence+1U &&
          Bluetooth_GetArmSequence()==arm_sequence+1U);
    CHECK(Bluetooth_GetControl()->frame.joy_x==1);
    RemotePacket(23,0,0,0);
    RemotePacket(0,0,0,0);
}

static void DualPacket(int button, int x, int y, int ax, int ay, int brake)
{
    uint8_t packet[BT_ARM_COMBINED_FRAME_SIZE];
    MakeCombined(packet, button, x, y, ax, ay, brake, 0, 0, 0);
    Feed(packet,sizeof(packet)); Bluetooth_Process();
    ArmTuner_Process(); Arm_Process(); DrainReplies();
}

static void ChassisPacket(int x, int y, int forward)
{
    uint8_t packet[BT_CONTROL_FRAME_SIZE] = {0xA5U};
    const int values[9] = {x, y, forward, 0, 0, 0, 0, 0, 0};
    unsigned i;
    for (i=0U; i<9U; ++i) {
        uint16_t value=(uint16_t)values[i];
        packet[1U+2U*i]=(uint8_t)value;
        packet[2U+2U*i]=(uint8_t)(value>>8);
    }
    for (i=1U; i<=18U; ++i) packet[19]+=packet[i];
    packet[20]=0x5AU;
    Feed(packet,sizeof(packet)); Bluetooth_Process();
    ArmTuner_Process(); Arm_Process(); DrainReplies();
}

static void CombinedPacketVariant(unsigned base_shorts, int arm_command,
                                  int x, int y, int arm_x, int arm_y,
                                  int forward, int stop, int cam, int shot)
{
    uint8_t packet[BT_ARM_COMBINED_FRAME_SIZE];
    CHECK(base_shorts>=9U && base_shorts<=11U);
    MakeCombined(packet, arm_command, x, y, arm_x, arm_y, stop, forward,
                 base_shorts >= 10U ? cam : 0, base_shorts >= 11U ? shot : 0);
    Feed(packet,sizeof(packet)); Bluetooth_Process();
    ArmTuner_Process(); Arm_Process(); DrainReplies();
}

static void CombinedPacket(int arm_command, int x, int y, int arm_x, int arm_y,
                           int forward, int stop)
{
    CombinedPacketVariant(9U,arm_command,x,y,arm_x,arm_y,forward,stop,0,0);
}

static void TestCombinedFraming(void)
{
    uint8_t packet[BT_ARM_COMBINED_FRAME_SIZE];
    uint32_t car_before, arm_before;
    unsigned i;
    MakeCombined(packet, 0, -400, 250, 700, -800, 0, 0, 1, 1);
    packet[23] = 1U; /* LEFT_90 at bytes 23..24. */
    packet[25] = 3U; /* Reserved short at bytes 25..26. */
    packet[33] = 0U;
    for (i = 1U; i <= 32U; ++i) packet[33] += packet[i];
    car_before = Bluetooth_GetSequence();
    arm_before = Bluetooth_GetArmSequence();
    for (i = 0U; i + 1U < sizeof(packet); ++i) {
        Feed(&packet[i], 1U); Bluetooth_Process();
        CHECK(Bluetooth_GetSequence() == car_before &&
              Bluetooth_GetArmSequence() == arm_before);
    }
    Feed(&packet[34], 1U); Bluetooth_Process();
    CHECK(Bluetooth_GetSequence() == car_before + 1U &&
          Bluetooth_GetArmSequence() == arm_before + 1U);
    CHECK(Bluetooth_GetControl()->frame.left_90 == 1 &&
          Bluetooth_GetControl()->frame.Cam_T == 1 &&
          Bluetooth_GetControl()->frame.Shot == 1);
    CHECK(Bluetooth_GetArmControl()->arm_x == 700 &&
          Bluetooth_GetArmControl()->arm_y == -800);
    packet[33] ^= 1U;
    Feed(packet, sizeof(packet)); Bluetooth_Process();
    CHECK(Bluetooth_GetSequence() == car_before + 1U);
    packet[25] = 11U;
    packet[33] = 0U;
    for (i = 1U; i <= 32U; ++i) packet[33] += packet[i];
    Feed(packet, sizeof(packet)); Bluetooth_Process();
    CHECK(Bluetooth_GetSequence() == car_before + 2U &&
          Bluetooth_GetArmSequence() == arm_before + 2U);
    packet[25] = 0U;
    packet[27] = 26U; /* ARM_CMD outside 0/1/10..13/20..25. */
    packet[33] = 0U;
    for (i = 1U; i <= 32U; ++i) packet[33] += packet[i];
    Feed(packet, sizeof(packet)); Bluetooth_Process();
    CHECK(Bluetooth_GetArmSequence() == arm_before + 2U);
    now += BT_FRAME_GAP_TIMEOUT_MS + 1U;
    { uint8_t legacy_dual[17] = {0xA5U};
      legacy_dual[16] = 0x5AU;
      Feed(legacy_dual, sizeof(legacy_dual)); Bluetooth_Process(); }
    CHECK(Bluetooth_GetArmSequence() == arm_before + 2U);
    now += BT_FRAME_GAP_TIMEOUT_MS + 1U;
}

static void TestDualAndSetup(void)
{
    unsigned before, i;
    uint32_t sequence, arm_sequence;
    ArmTuner_Init();
    Command("@ARM SETUP"); Expect("SINGLE_MOVE NO_PING");
    Command("@ARM SELECT 0");
    Command("@ARM ARM"); Expect("SINGLE_MOVE_NO_PING");
    Command("@ARM MOVE 1532"); Expect("RESULT=0");
    Advance(1600U); CHECK(Arm_GetStatus().state==ARM_RUNNING);
    Advance(401U); CHECK(!Arm_GetStatus().motion_allowed);
    CHECK(Arm_GetStatus().state==ARM_COMPLETE_ESTIMATED);
    before=tx_count;
    Command("@ARM MOVE 1980"); Expect("RESULT=4"); CHECK(tx_count==before);
    Command("@ARM SELECT 1");
    Command("@ARM ARM");
    Command("@ARM MOVE 2219"); Expect("RESULT=0");
    Advance(2001U); CHECK(!Arm_GetStatus().motion_allowed);
    Command("@ARM SYNC LAST"); Expect("SYNC_LAST_REQUIRES_KNOWN_P0_P1_P2");
    Command("@ARM SELECT 2");
    Command("@ARM ARM");
    Command("@ARM MOVE 1202"); Expect("RESULT=0");
    Advance(2001U); CHECK(!Arm_GetStatus().motion_allowed);
    Command("@ARM SYNC LAST"); Expect("RESULT=0 P=1532,2219,1202");
    before=tx_count;
    Command("@ARM DUAL START");
    Expect("MOVING_REFERENCE P=1532,2219,1202");
    CHECK(Bluetooth_IsExtended());
    CHECK(Arm_GetStatus().state==ARM_RUNNING && tx_count==before+1U);
    CHECK(strcmp(last_frame,
                 "{#000P1532T2000!#001P2219T2000!#002P1202T2000!#003P1200T2000!}")==0);
    Advance(2001U); DrainReplies(); Expect("DUAL READY SHORTS=16");
    CHECK(!Arm_GetStatus().motion_allowed);
    before=tx_count;
    CombinedPacket(0,0,0,0,0,0,0); Expect("AUTO_GRANT RESULT=0");
    CHECK(Arm_GetStatus().motion_allowed);
    arm_sequence=Bluetooth_GetArmSequence();
    CombinedPacketVariant(10U,0,100,-100,0,0,0,0,1,0);
    CHECK(Bluetooth_GetArmSequence()==arm_sequence+1U &&
          Bluetooth_GetControl()->frame.Cam_T==1 &&
          Bluetooth_GetControl()->frame.Shot==0);
    arm_sequence=Bluetooth_GetArmSequence();
    CombinedPacketVariant(11U,0,-100,100,0,0,0,0,1,1);
    CHECK(Bluetooth_GetArmSequence()==arm_sequence+1U &&
          Bluetooth_GetControl()->frame.Cam_T==1 &&
          Bluetooth_GetControl()->frame.Shot==1);
    /* Reproduce the real phone workload: chassis and arm packets alternate.
     * Normal 21-byte traffic must never reset the independent arm lease,
     * reference, direction state or arm sequence. */
    for (i=0U; i<250U; ++i) {
        int x=(int)(i%2001U)-1000;
        int y=1000-(int)(i%2001U);
        arm_sequence=Bluetooth_GetArmSequence();
        ChassisPacket(x,y,(int)(i&1U));
        CHECK(Bluetooth_GetArmSequence()==arm_sequence);
        CHECK(Bluetooth_GetControl()->frame.joy_x==x);
        CHECK(Bluetooth_GetControl()->frame.joy_y==y);
        CHECK(Arm_GetStatus().motion_allowed);
        CombinedPacket(0,x,y,0,0,(int)(i&1U),0);
        CHECK(Bluetooth_GetArmSequence()==arm_sequence+1U);
        CHECK(Bluetooth_GetArmControl()->valid);
        CHECK(Bluetooth_GetControl()->frame.forward==(int)(i&1U));
        CHECK(Arm_GetStatus().motion_allowed);
        Advance(1U);
    }
    sequence=Bluetooth_GetArmSequence();
    /* A period containing only chassis traffic expires ArmIsConnected, but
     * must not destroy the safe reference/authorization. */
    for (i=0U; i<20U; ++i) {
        Advance(30U);
        ChassisPacket(600,-300,1);
        CHECK(Bluetooth_GetArmSequence()==sequence);
        CHECK(Arm_GetStatus().motion_allowed);
    }
    CHECK(Bluetooth_GetArmSequence()==sequence);
    CHECK(Bluetooth_GetControl()->frame.joy_x==600 &&
          Bluetooth_GetControl()->frame.joy_y==-300 &&
          Bluetooth_GetControl()->frame.forward==1);
    CHECK(Bluetooth_IsConnected() && !Bluetooth_ArmIsConnected());
    CHECK(Arm_GetStatus().motion_allowed);
    Command("@ARM REMOTE SHOW"); Expect("REF=1 CENTER=1 CONNECTED=0");
    Expect("ALLOW=1");
    DualPacket(0,600,-300,0,0,0);
    CHECK(Bluetooth_ArmIsConnected() && Arm_GetStatus().motion_allowed);
    DualPacket(11,0,0,0,0,0); Expect("WRIST RESULT=0 DP=-35 P=1167");
    CHECK(strcmp(last_frame,"{#002P1167T0200!}")==0);
    Advance(201U);
    DualPacket(10,0,0,0,0,0); Expect("WRIST RESULT=0 DP=35 P=1202");
    CHECK(strcmp(last_frame,"{#002P1202T0200!}")==0);
    Advance(201U);
    before=tx_count;
    DualPacket(0,1000,-600,400,400,0); Expect("DX=1 DZ=1 DPHI=0");
    CHECK(tx_count==before+1U);
    CHECK(Bluetooth_GetControl()->frame.joy_x==1000 &&
          Bluetooth_GetControl()->frame.joy_y==-600);
    CHECK(Bluetooth_GetArmControl()->arm_x==400 &&
          Bluetooth_GetArmControl()->arm_y==400);
    Advance(100U); DualPacket(0,1000,0,0,0,0);
    Advance(201U); DualPacket(0,1000,0,0,0,0);
    CHECK(tx_count==before+1U); /* Wheels alone never jog the arm. */
    DualPacket(24,800,0,0,0,0); Expect("RESULT=0");
    CHECK(strcmp(last_frame,"{#003P1500T0300!}")==0);
    CHECK(Bluetooth_GetControl()->frame.joy_x==800); /* Grip preserves wheels. */
    Advance(501U); DualPacket(0,0,0,0,0,0);
    DualPacket(13,0,0,0,0,0); Expect("REMOTE GRIP RESULT=0 P=1460");
    CHECK(strcmp(last_frame,"{#003P1460T0300!}")==0);
    /* A held/repeated discrete direction is one bounded step.  It must be
     * released before another identical wrist/gripper step is accepted. */
    Advance(301U); before=tx_count;
    DualPacket(13,0,0,0,0,0); CHECK(tx_count==before);
    DualPacket(0,0,0,0,0,0);
    DualPacket(13,0,0,0,0,0); Expect("REMOTE GRIP RESULT=0 P=1420");
    CHECK(strcmp(last_frame,"{#003P1420T0300!}")==0);
    Advance(301U); DualPacket(0,0,0,0,0,0);
    DualPacket(12,0,0,0,0,0); Expect("REMOTE GRIP RESULT=0 P=1460");
    Advance(301U); DualPacket(0,0,0,0,0,0);
    DualPacket(12,0,0,0,0,0); Expect("REMOTE GRIP RESULT=0 P=1500");
    CHECK(strcmp(last_frame,"{#003P1500T0300!}")==0);
    Advance(501U); DualPacket(0,0,0,0,0,0);
    Command("@ARM TARGETS");
    Expect("KNOWN=0x0F");
    Expect("P=");
    Expect(",1500,0 COMMAND_ESTIMATES_NOT_FEEDBACK");
    sequence=Bluetooth_GetSequence();
    { uint8_t bad[17]={0xA5U}; bad[16]=0x5AU; bad[15]=1U;
      Feed(bad,sizeof(bad)); Bluetooth_Process(); }
    CHECK(Bluetooth_GetSequence()==sequence);
    now += BT_FRAME_GAP_TIMEOUT_MS + 1U;
    DualPacket(0,0,0,-400,0,0); Expect("DX=-1 DZ=0");
    DualPacket(0,0,0,0,0,1); FinishStop();
    CHECK(!Arm_GetStatus().motion_allowed &&
          Bluetooth_GetArmControl()->brake==1);
    DualPacket(0,0,0,0,0,0);
    Command("@ARM SYNC 1532 2219 1202");
    DualPacket(0,0,0,0,0,0);
    DualPacket(21,0,0,0,0,0);
    Advance(501U);
    CHECK(Arm_GetStatus().motion_allowed && !Bluetooth_ArmIsConnected());
    /* A link error does not revoke the whole dual session after a bounded
     * segment was accepted. The accepted segment completes, then waits. */
    before=tx_count; DualPacket(0,0,0,1000,0,0);
    CHECK(tx_count==before+1U); /* A fresh event may start one bounded jog. */
    Bluetooth_ErrorCallback(&huart6); Bluetooth_Process(); ArmTuner_Process();
    CHECK(Arm_GetStatus().motion_allowed && Arm_GetStatus().state==ARM_RUNNING);
    Advance(201U);
    CHECK(Arm_GetStatus().motion_allowed);
    Command("@ARM REMOTE SHOW"); Expect("REF=1"); Expect("ALLOW=1 ERR=0");
    DualPacket(0,0,0,0,0,0); /* Reconnect and cancel any held vector. */
    /* One full-scale phone event is smoothed into at most three chained 2 mm
     * / 200 ms segments. The 600 ms latch expires even if release is lost. */
    before=tx_count; DualPacket(0,0,0,1000,0,0);
    CHECK(tx_count==before+1U);
    Advance(201U); CHECK(tx_count==before+2U);
    Advance(201U); CHECK(tx_count==before+3U);
    Advance(201U); CHECK(tx_count==before+3U);
    DualPacket(0,0,0,0,0,0);
    /* START is intentionally idempotent.  If an old bounded segment is still
     * running, it is stopped and the reference restarts automatically
     * instead of exposing STOP_BEFORE_DUAL_START to the phone operator. */
    before=tx_count;
    DualPacket(0,0,0,1000,0,0);
    CHECK(tx_count==before+1U && Arm_GetStatus().state==ARM_RUNNING);
    Command("@ARM DUAL START");
    Expect("STOPPING_PREVIOUS AUTO_RESTART_PENDING");
    Advance(ARM_TUNER_DUAL_RESTART_SETTLE_MS + 8U); DrainReplies();
    Expect("MOVING_REFERENCE P=1532,2219,1202");
    CHECK(Arm_GetStatus().state==ARM_RUNNING);
    CHECK(strcmp(last_frame,
                 "{#000P1532T2000!#001P2219T2000!#002P1202T2000!#003P1200T2000!}")==0);
    Advance(2001U); DrainReplies(); Expect("DUAL READY SHORTS=16");
    Command("@ARM STOP"); FinishStop();
    Command("@ARM DUAL OFF"); Expect("SHORTS=13");
    CHECK(!Bluetooth_IsExtended());
    RemotePacket(0,0,0,0);
    Command("@ARM EXIT");
}

int main(void)
{
    unsigned before, replies;
    uint32_t seq, stamp;
    uint8_t joy[BT_ARM_COMBINED_FRAME_SIZE];
    uint8_t tune[5]={0xA5,1,0,1,0x5A};
    char long_line[110];
    huart3.Instance=&huart3; huart3.Init.BaudRate=SERVO_BAUD_RATE;
    huart3.Init.Mode=UART_MODE_TX_RX; huart3.gState=HAL_UART_STATE_READY;
    CHECK(Servo_Init(&huart3)==SERVO_OK);
    MakeCombined(joy, 0, 64, 0, 0, 0, 0, 0, 0, 0);
    Bluetooth_Init(); PID_Tuner_Init(); Arm_Init(); ArmTuner_Init();
    ArmTuner_Process(); Arm_Process(); CHECK(tx_count==0 && reply_count==0);
    TestCombinedFraming();
    seq=Bluetooth_GetSequence(); stamp=Bluetooth_GetLastRxTick();
    Command("@ARM SHOW"); Expect("NAME=SHOULDER"); Expect("STATE=IDLE");
    Expect("ID=0"); Expect("MIN=902"); Expect("MAX=1569");
    before=tx_count;
    Command("@ARM IK 210 277 0");
    Expect("DRY_RUN NO_MOTION"); Expect("POS=0"); Expect("PP=");
    CHECK(tx_count==before);
    Command("@ARM IK -200 35 -90"); Expect("DRY_RUN NO_MOTION");
    CHECK(tx_count==before);
    Command("@ARM IK 210 277 181"); Expect("IK X_MM GROUND_Z_MM PHI_DEG");
    Command("@ARM IK 210 277"); Expect("IK X_MM GROUND_Z_MM PHI_DEG");
    CHECK(tx_count==before);
    Command("@ARM MOVE 1500"); Expect("ENTER_REQUIRED"); CHECK(tx_count==0);
    Command("@ARM ENTER"); Expect("CHASSIS_UNCHANGED"); CHECK(tx_count==0);
    Command("@ARM ENTER"); Expect("CHASSIS_UNCHANGED");
    Command("@ARM ARM"); Expect("PARK_CONFIG_TIME_REQUIRED");
    Command("@ARM LIMIT 7 1400 1600"); Expect("RESULT=0");
    Command("@ARM LIMIT 255 1400 1600"); Expect("SYNTAX");
    Command("@ARM LIMIT 7 499 1600"); Expect("RESULT=1");
    Command("@ARM LIMIT 7 1600 1400"); Expect("RESULT=1");
    Command("@ARM TIME 100"); Expect("TIME=100");
    Command("@ARM TIME 0"); Expect("SYNTAX");
    Command("@ARM TIME 10000"); Expect("SYNTAX");
    Command("@ARM TIME -1"); Expect("SYNTAX");
    Command("@ARM TIME 99999999999999999999"); Expect("SYNTAX");
    Command("@ARM TIME 100 extra"); Expect("SYNTAX");
    Command("@ARM STEP 51"); Expect("SYNTAX");
    Command("@ARM STEP 0"); Expect("SYNTAX");
    Command("@ARM STEP 5"); Expect("STEP=5");
    Command("@ARM ARM"); Expect("RESULT=0");
    Command("@ARM +"); Expect("MOVE_FIRST");
    Command("@ARM LIMIT 7 1400 1600"); Expect("STOP_BEFORE_CONFIG");
    Command("@ARM MOVE 1399"); Expect("RESULT=1"); CHECK(tx_count==0);
    Command("@ARM MOVE 1500"); Expect("RESULT=0");
    CHECK(strcmp(last_frame,"{#007P1500T0100!}")==0);
    Command("@ARM +"); Expect("BUSY"); CHECK(tx_count==1);
    Advance(100U); CHECK(Arm_GetStatus().state==ARM_COMPLETE_ESTIMATED);
    Command("@ARM MARK 0"); Expect("P=1500");
    Command("@ARM SLOT 0"); Expect("VALID=1");
    Command("@ARM +"); CHECK(strcmp(last_frame,"{#007P1505T0100!}")==0);
    Advance(100U);
    Command("@ARM -"); CHECK(strcmp(last_frame,"{#007P1500T0100!}")==0);
    Advance(100U);
    Command("@ARM SHOW"); Expect("LAST=1500 KNOWN=1");
    CHECK(Bluetooth_GetSequence()==seq && Bluetooth_GetLastRxTick()==stamp);
    Command("@ARM STOP"); FinishStop(); CHECK(!Arm_GetStatus().motion_allowed);
    Command("@ARM +"); Expect("MOVE_FIRST");
    Command("@ARM SLOT 0"); Expect("VALID=1");
    Command("@ARM LIMIT 7 1400 1600"); Expect("RESULT=0");
    Command("@ARM SLOT 0"); Expect("VALID=0");
    Command("@ARM SELECT 1"); Expect("J=1"); Expect("NAME=ELBOW");
    Command("@ARM LIMIT 7 1400 1600"); Expect("RESULT=1");
    Command("@ARM LIMIT 12 1400 1600"); Expect("RESULT=0");
    Command("@ARM SELECT 0");
    Move(); Advance(100U);
    Advance(ARM_TUNER_HEARTBEAT_MS+1U); FinishStop();
    CHECK(!Arm_GetStatus().motion_allowed);
    Command("@ARM PING"); Command("@ARM MOVE 1500"); Expect("RESULT=4");
    Command("@ARM TIME 2000"); Move();
    Advance(500U); Command("@ARM PING");
    Advance(500U); Command("@ARM PING");
    Advance(500U); Command("@ARM PING"); Advance(500U);
    CHECK(Arm_GetStatus().state==ARM_COMPLETE_ESTIMATED);
    Command("@ARM TIME 100"); Move();
    Bluetooth_ErrorCallback(&huart6); Bluetooth_Process(); FinishStop();
    CHECK(!Arm_GetStatus().motion_allowed);
    Move(); MakeCombined(joy, 0, 64, 0, 0, 0, 1, 0, 0, 0);
    Feed(joy,sizeof(joy)); Bluetooth_Process(); ArmTuner_Process(); FinishStop();
    CHECK(!Arm_GetStatus().motion_allowed && Bluetooth_GetArmControl()->brake);
    Command("@ARM ARM"); Expect("RELEASE_JOYSTICK_BRAKE");
    MakeCombined(joy, 0, 64, 0, 0, 0, 0, 0, 0, 0);
    Feed(joy,sizeof(joy)); Bluetooth_Process(); ArmTuner_Process();
    /* Release alone does not reauthorize the arm. */
    CHECK(!Arm_GetStatus().motion_allowed);
    Move(); arm_tx=HAL_ERROR; Command("@ARM STOP"); FinishStop();
    CHECK(Arm_GetStatus().state==ARM_FAULT);
    Command("@ARM EXIT"); Expect("STOP_OR_CLEAR_FIRST");
    Command("@ARM CLEAR"); Expect("RESULT=0"); CHECK(!Arm_GetStatus().motion_allowed);
    Command("@ARM EXIT");
    before=tx_count;
    /* Split text / CRLF / malformed / stale / overlong / binary coexistence. */
    Feed("@ARM EN",7U); Bluetooth_Process(); CHECK(tx_count==before);
    Feed("TER\r\n",5U); Bluetooth_Process(); DrainReplies();
    Expect("CHASSIS_UNCHANGED");
    Command("@ARM STOP extra"); Expect("SYNTAX"); CHECK(tx_count==before);
    Feed("@ARM MOVE 1500\n",15U); now+=BT_FAILSAFE_TIMEOUT_MS+1U;
    Bluetooth_Process(); CHECK(tx_count==before);
    memset(long_line,'X',sizeof(long_line)); memcpy(long_line,"@ARM ",5U);
    long_line[sizeof(long_line)-1U]='\n'; Feed(long_line,sizeof(long_line)); Bluetooth_Process();
    CHECK(tx_count==before);
    Feed("@ARM EX",7U); Bluetooth_Process(); now+=BT_FRAME_GAP_TIMEOUT_MS+1U;
    Feed("IT\n",3U); Bluetooth_Process(); CHECK(tx_count==before);
    Feed("@ARM EX",7U); Bluetooth_Process(); Bluetooth_ErrorCallback(&huart6); Bluetooth_Process();
    before=tx_count; /* Recovery may issue one bounded arm STOP. */
    Feed("IT\n",3U); Bluetooth_Process(); CHECK(tx_count==before); FinishStop();
    Feed("@ARM EX",7U); Bluetooth_Process();
    MakeCombined(joy, 0, 64, 0, 0, 0, 1, 0, 0, 0);
    Feed(joy,sizeof(joy)); Bluetooth_Process(); CHECK(Bluetooth_GetArmControl()->brake);
    Feed(tune,sizeof(tune)); Bluetooth_Process(); CHECK(pid.kp>1.0f);
    Command("@ARM SHOW"); Expect("RAM_ONLY");
    replies=reply_count; Feed("@ARM SHOW\n@ARM SHOW\n",20U);
    Bluetooth_Process(); DrainReplies(); CHECK(reply_count==replies+1U);
    Bluetooth_Process(); DrainReplies(); CHECK(reply_count==replies+2U);
    FinishStop();
    /* Wrap-safe heartbeat and process deadline. */
    now=UINT32_MAX-50U; Move(); Advance(100U);
    CHECK(Arm_GetStatus().state==ARM_COMPLETE_ESTIMATED);
    Command("@ARM STOP"); FinishStop();
    Command("@ARM EXIT");
    /* Cartesian control requires an explicitly declared, user-confirmed pose.
     * SYNC itself is calculation/state only; JOG is small, negative-branch and
     * sends all three planar joints in one synchronized frame. */
    MakeCombined(joy, 0, 64, 0, 0, 0, 0, 0, 0, 0);
    Feed(joy,sizeof(joy)); Bluetooth_Process(); ArmTuner_Process();
    ArmTuner_Init(); before=tx_count;
    Command("@ARM ENTER");
    Command("@ARM JOG 1 0 0"); Expect("SYNC_REQUIRED"); CHECK(tx_count==before);
    Command("@ARM SYNC 1533 2219 1201");
    Expect("REFERENCE_ONLY NO_MOTION"); CHECK(tx_count==before);
    Command("@ARM JOG 1 0 0"); Expect("ARM_REQUIRED"); CHECK(tx_count==before);
    Command("@ARM TIME 500"); Command("@ARM ARM"); Expect("RESULT=0");
    Command("@ARM JOG 1 0 0"); Expect("RESULT=0");
    Expect("ACCEPTED_NOT_ARRIVED"); CHECK(tx_count==before+1U);
    CHECK(strstr(last_frame,"#000P")!=NULL);
    CHECK(strstr(last_frame,"#001P")!=NULL);
    CHECK(strstr(last_frame,"#002P")!=NULL);
    Command("@ARM JOG 1 0 0"); Expect("BUSY"); CHECK(tx_count==before+1U);
    Advance(500U);
    Command("@ARM JOG 11 0 0"); Expect("JOG DX_MM DZ_MM DPHI_DEG");
    Command("@ARM JOG 0 0 0"); Expect("ZERO_JOG");
    Command("@ARM STOP"); FinishStop();
    Command("@ARM JOG 1 0 0"); Expect("SYNC_REQUIRED");
    Command("@ARM EXIT");
    TestRemote();
    TestNumericButtons();
    TestDualAndSetup();
    printf("PASS arm Bluetooth: %u checks (HAL simulation, not hardware)\n",checks);
    return 0;
}
