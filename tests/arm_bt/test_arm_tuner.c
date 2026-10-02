#include "arm_tuner.h"
#include "arm_control.h"
#include "arm_trim_bluetooth.h"
#include "arm_trim_bench.h"
#include "arm_trim_project_config.h"
#include "bluetooth_driver.h"
#include "heading_control.h"
#include "pid_tuner.h"
#include "serial_io.h"
#include "servo_remote.h"
#include "car_control.h"
#include "motor_driver.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

UART_HandleTypeDef huart1, huart3, huart6;
static uint32_t now;
static uint8_t pe4_low;
static CarState_t mock_car_state = CAR_READY;
static uint8_t mock_motor_idle = 1U;
static uint8_t mock_motor_fault;
static unsigned checks, tx_count, reply_count, gripper_stops;
static unsigned gripper_commands;
static uint16_t last_gripper_target;
static char last_frame[ZLIS2_MAX_TX_LENGTH+1U], last_reply[PID_TX_LINE_SIZE];
static char debug_log[4096];
static HAL_StatusTypeDef arm_tx = HAL_OK;
static HeadingPIDParameters pid = {1.0f, 0.0f, 0.2f};
static HeadingStatus heading;
CarState_t Car_Control_GetState(void) { return mock_car_state; }
uint8_t Motor_IsIdle(void) { return mock_motor_idle; }
uint8_t Motor_HasFault(void) { return mock_motor_fault; }
uint8_t Board_ServoButtonIsLow(void) { return pe4_low; }
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
    if (u==&huart1) {
        size_t used_log=strlen(debug_log);
        CHECK(n<80U);
        if (used_log+n<sizeof(debug_log)) {
            memcpy(debug_log+used_log,p,n); debug_log[used_log+n]='\0';
        }
        Debug_TxCallback(u);
        return HAL_OK;
    }
    CHECK(u==&huart6 && n<sizeof(last_reply));
    memcpy(last_reply,p,n); last_reply[n]='\0'; ++reply_count;
    PID_Tuner_TxCallback(u);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u, const uint8_t *p, uint16_t n, uint32_t timeout)
{
    HAL_StatusTypeDef result=arm_tx;
    CHECK(u==&huart3 && n<sizeof(last_frame) && timeout==ZLIS2_TX_TIMEOUT_MS);
    memcpy(last_frame,p,n); last_frame[n]='\0'; ++tx_count; arm_tx=HAL_OK;
    if (result == HAL_OK && strstr(last_frame, "#003P") != NULL) {
        last_gripper_target = (uint16_t)strtoul(strstr(last_frame, "#003P") + 5U, NULL, 10);
        ++gripper_commands;
    }
    if (ArmTrimBluetooth_OwnsMotion() && !ArmTrimBench_IsActive() && last_frame[0] == '{') {
        CHECK(strstr(last_frame, "#000P") && strstr(last_frame, "#001P") && strstr(last_frame, "#002P"));
        CHECK(strstr(last_frame, "#003P") == NULL);
    }
    if (strcmp(last_frame, "$DST:3!") == 0) ++gripper_stops;
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

static void MakeBoolArmGap41(uint8_t *packet, uint16_t buttons,
                             int command, int x, int y,
                             int arm_x, int arm_y, uint32_t gap)
{
    const int values[16] = {
        x, y, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        BT_SERVO_GAP_MARKER, command, arm_x, arm_y
    };
    unsigned i;
    memset(packet, 0, BT_ARM_BOOL_GAP_FRAME_SIZE);
    packet[0] = 0xA5U;
    packet[1] = (uint8_t)buttons;
    packet[2] = (uint8_t)(buttons >> 8);
    for (i = 0U; i < 16U; ++i) {
        uint16_t value = (uint16_t)values[i];
        packet[3U + 2U * i] = (uint8_t)value;
        packet[4U + 2U * i] = (uint8_t)(value >> 8);
    }
    packet[35] = (uint8_t)gap;
    packet[36] = (uint8_t)(gap >> 8);
    packet[37] = (uint8_t)(gap >> 16);
    packet[38] = (uint8_t)(gap >> 24);
    for (i = 1U; i <= 38U; ++i) packet[39] += packet[i];
    packet[40] = 0x5AU;
}

static void MakePhone20(uint8_t *packet, uint16_t buttons,
                        int joy_y, int forward, int cam, int shot,
                        int left_90, int servo_mode, int joy_x,
                        int arm_command, int arm_x, int arm_y, uint32_t gap)
{
    const int values[16] = {
        joy_y, forward, 0, 0, 0, 0, 0, 0,
        cam, shot, left_90, servo_mode, joy_x,
        arm_command, arm_x, arm_y
    };
    unsigned i;
    memset(packet, 0, BT_CONTROL_FRAME_PHONE20_SIZE);
    packet[0] = 0xA5U;
    packet[1] = (uint8_t)buttons;
    packet[2] = (uint8_t)(buttons >> 8);
    for (i = 0U; i < 16U; ++i) {
        uint16_t value = (uint16_t)values[i];
        packet[3U + 2U * i] = (uint8_t)value;
        packet[4U + 2U * i] = (uint8_t)(value >> 8);
    }
    packet[35] = (uint8_t)gap;
    packet[36] = (uint8_t)(gap >> 8);
    packet[37] = (uint8_t)(gap >> 16);
    packet[38] = (uint8_t)(gap >> 24);
    for (i = 1U; i <= 38U; ++i) packet[39] += packet[i];
    packet[40] = 0x5AU;
}

static void MakeArmVariant(uint8_t *packet, unsigned length, int command,
                           int x, int y, int arm_x, int arm_y)
{
    int values[16] = {0};
    unsigned base_shorts, total_shorts, i;
    CHECK(length == 27U || length == 29U || length == 31U ||
          length == 33U || length == 35U);
    base_shorts = (length - 9U) / 2U;
    total_shorts = base_shorts + 3U;
    values[0] = x;
    values[1] = y;
    values[2] = 1; /* FORWARD proves the chassis half is retained. */
    if (base_shorts > 9U) values[9] = 1;  /* Cam_T */
    if (base_shorts > 10U) values[10] = 1; /* Shot */
    if (base_shorts > 11U) values[11] = 1; /* LEFT_90 */
    if (base_shorts > 12U) values[12] = 3; /* SERVO_MODE */
    values[base_shorts] = command;
    values[base_shorts + 1U] = arm_x;
    values[base_shorts + 2U] = arm_y;
    memset(packet, 0, BT_ARM_COMBINED_FRAME_SIZE);
    packet[0] = 0xA5U;
    for (i = 0U; i < total_shorts; ++i) {
        uint16_t value = (uint16_t)values[i];
        packet[1U + 2U * i] = (uint8_t)value;
        packet[2U + 2U * i] = (uint8_t)(value >> 8);
    }
    for (i = 1U; i + 2U < length; ++i)
        packet[length - 2U] = (uint8_t)(packet[length - 2U] + packet[i]);
    packet[length - 1U] = 0x5AU;
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

static void Phone7Packet(uint16_t buttons, int arm_command,
                         int arm_x, int arm_y, uint32_t gap)
{
    uint8_t packet[BT_ARM_BOOL_GAP_FRAME_SIZE];
    MakeBoolArmGap41(packet, buttons, arm_command, 0, 0,
                     arm_x, arm_y, gap);
    Feed(packet, sizeof(packet)); Bluetooth_Process();
    ArmTuner_Process(); Arm_Process(); DrainReplies();
}

static void TestArmProtocolMatrix(void)
{
    static const unsigned lengths[] = {27U, 29U, 31U, 33U, 35U};
    uint8_t packet[BT_ARM_COMBINED_FRAME_SIZE];
    uint8_t dual[BT_ARM_DUAL_FRAME_SIZE] = {0};
    uint32_t car_before, arm_before, invalid_before;
    unsigned i, j;

    Bluetooth_SetExtended(1U);
    for (i = 0U; i < sizeof(lengths) / sizeof(lengths[0]); ++i) {
        unsigned length = lengths[i];
        MakeArmVariant(packet, length, BT_DIRECTION_ARM_PRESET_NEXT,
                       -300, 250, 700, -800);
        car_before = Bluetooth_GetSequence();
        arm_before = Bluetooth_GetArmSequence();
        Feed(packet, length / 2U); Bluetooth_Process();
        CHECK(Bluetooth_GetSequence() == car_before &&
              Bluetooth_GetArmSequence() == arm_before);
        Feed(packet + length / 2U, length - length / 2U); Bluetooth_Process();
        CHECK(Bluetooth_GetSequence() == car_before + 1U &&
              Bluetooth_GetArmSequence() == arm_before + 1U);
        CHECK(Bluetooth_GetLastFrameLength() == length &&
              Bluetooth_GetLastArmFrameLength() == length);
        CHECK(Bluetooth_GetControl()->frame.joy_x == -300 &&
              Bluetooth_GetControl()->frame.joy_y == 250 &&
              Bluetooth_GetControl()->frame.forward == 1);
        CHECK(Bluetooth_GetArmControl()->direction ==
              BT_DIRECTION_ARM_PRESET_NEXT &&
              Bluetooth_GetArmControl()->arm_x == 700 &&
              Bluetooth_GetArmControl()->arm_y == -800);
    }

    dual[0] = 0xA5U;
    { const int values[7] = {13, 1, 0, -200, 300, -400, 500};
      for (i = 0U; i < 7U; ++i) {
          uint16_t value = (uint16_t)values[i];
          dual[1U + 2U * i] = (uint8_t)value;
          dual[2U + 2U * i] = (uint8_t)(value >> 8);
      } }
    for (i = 1U; i <= 14U; ++i) dual[15] = (uint8_t)(dual[15] + dual[i]);
    dual[16] = 0x5AU;
    car_before = Bluetooth_GetSequence(); arm_before = Bluetooth_GetArmSequence();
    Feed(dual, sizeof(dual)); Bluetooth_Process();
    CHECK(Bluetooth_GetSequence() == car_before + 1U &&
          Bluetooth_GetArmSequence() == arm_before + 1U);
    CHECK(Bluetooth_GetLastFrameLength() == 17U &&
          Bluetooth_GetLastArmFrameLength() == 17U);
    CHECK(Bluetooth_GetArmControl()->direction == 13 &&
          Bluetooth_GetArmControl()->arm_x == -400 &&
          Bluetooth_GetArmControl()->arm_y == 500);

    MakeArmVariant(packet, 35U, 0, 0, 0, 0, 0);
    packet[33] ^= 1U;
    invalid_before = Bluetooth_GetInvalidFrameCount();
    car_before = Bluetooth_GetSequence(); arm_before = Bluetooth_GetArmSequence();
    Feed(packet, 35U); Bluetooth_Process();
    CHECK(Bluetooth_GetInvalidFrameCount() == invalid_before + 1U);
    CHECK(Bluetooth_GetSequence() == car_before &&
          Bluetooth_GetArmSequence() == arm_before);
    now += BT_FRAME_GAP_TIMEOUT_MS + 1U;

    /* Outside an arm session the ambiguous 29-byte packet remains the
     * teammate SERVO_MODE profile and must not create an arm event. */
    Bluetooth_SetExtended(0U);
    memset(packet, 0, BT_CONTROL_FRAME_SERVO_MODE_SIZE);
    packet[0] = 0xA5U;
    packet[25] = 3U;
    for (j = 1U; j <= 26U; ++j)
        packet[27] = (uint8_t)(packet[27] + packet[j]);
    packet[28] = 0x5AU;
    car_before = Bluetooth_GetSequence(); arm_before = Bluetooth_GetArmSequence();
    Feed(packet, BT_CONTROL_FRAME_SERVO_MODE_SIZE); Bluetooth_Process();
    CHECK(Bluetooth_GetSequence() == car_before + 1U &&
          Bluetooth_GetArmSequence() == arm_before);
    CHECK(Bluetooth_GetControl()->frame.servo_mode == 3 &&
          Bluetooth_GetLastFrameLength() == BT_CONTROL_FRAME_SERVO_MODE_SIZE);

    /* Exact incremental form of (7).pro: its valid 35-byte prefix must wait
     * for ARM_CMD/X/Y + GAP instead of being rejected early. */
    {
        uint8_t extended[BT_ARM_BOOL_GAP_FRAME_SIZE];
        uint16_t buttons = (uint16_t)(BT_SERVO_BUTTON_TH_G |
            BT_CONTROL_BUTTON_SHOT | BT_CONTROL_BUTTON_AIM |
            BT_SERVO_BUTTON_TH_L);
        Bluetooth_SetExtended(1U);
        MakeBoolArmGap41(extended, buttons, BT_DIRECTION_ARM_PRESET_NEXT,
                         -111, 222, 333, -444, 1800U);
        car_before = Bluetooth_GetSequence();
        arm_before = Bluetooth_GetArmSequence();
        Feed(extended, 35U); Bluetooth_Process();
        CHECK(Bluetooth_GetSequence() == car_before &&
              Bluetooth_GetArmSequence() == arm_before);
        Feed(extended + 35U, sizeof(extended) - 35U); Bluetooth_Process();
        CHECK(Bluetooth_GetSequence() == car_before + 1U &&
              Bluetooth_GetArmSequence() == arm_before + 1U);
        CHECK(Bluetooth_GetLastFrameLength() == BT_ARM_BOOL_GAP_FRAME_SIZE &&
              Bluetooth_GetLastArmFrameLength() == BT_ARM_BOOL_GAP_FRAME_SIZE);
        CHECK(Bluetooth_GetControl()->frame.servo_buttons ==
                  (BT_SERVO_BUTTON_TH_G | BT_SERVO_BUTTON_TH_L) &&
              Bluetooth_GetControl()->frame.Shot == 1 &&
              Bluetooth_GetControl()->frame.aim == 1 &&
              Bluetooth_GetControl()->frame.Cam_T == 0 &&
              Bluetooth_GetControl()->frame.left_90 == 0 &&
              Bluetooth_GetControl()->frame.gap_pwm == 1800U);
        CHECK(Bluetooth_GetArmControl()->direction ==
                  BT_DIRECTION_ARM_PRESET_NEXT &&
              Bluetooth_GetArmControl()->arm_x == 333 &&
              Bluetooth_GetArmControl()->arm_y == -444);
        extended[39] ^= 1U;
        invalid_before = Bluetooth_GetInvalidFrameCount();
        Feed(extended, sizeof(extended)); Bluetooth_Process();
        CHECK(Bluetooth_GetInvalidFrameCount() == invalid_before + 1U &&
              Bluetooth_GetSequence() == car_before + 1U);
        now += BT_FRAME_GAP_TIMEOUT_MS + 1U;
    }

    /* Exact teammate (20).pro layout from origin/main@3021961.  It differs
     * from (7).pro by placing JOY_X after SERVO_MODE. */
    {
        uint8_t phone20[BT_CONTROL_FRAME_PHONE20_SIZE];
        uint16_t buttons = (uint16_t)(BT_SERVO_BUTTON_TB_M |
            BT_CONTROL_BUTTON_SHOT | BT_CONTROL_BUTTON_AIM |
            BT_SERVO_BUTTON_TH_L);
        MakePhone20(phone20, buttons, 600, 1, 0, 0, 1, 0, -450,
                    BT_DIRECTION_ARM_PRESET_NEXT, 333, -444, 1800U);
        car_before = Bluetooth_GetSequence();
        arm_before = Bluetooth_GetArmSequence();
        Feed(phone20, 35U); Bluetooth_Process();
        CHECK(Bluetooth_GetSequence() == car_before &&
              Bluetooth_GetArmSequence() == arm_before);
        Feed(phone20 + 35U, sizeof(phone20) - 35U); Bluetooth_Process();
        CHECK(Bluetooth_GetSequence() == car_before + 1U &&
              Bluetooth_GetArmSequence() == arm_before + 1U);
        CHECK(Bluetooth_GetLastFrameLength() == BT_CONTROL_FRAME_PHONE20_SIZE &&
              Bluetooth_GetLastArmFrameLength() == BT_CONTROL_FRAME_PHONE20_SIZE);
        CHECK(Bluetooth_GetControl()->frame.joy_x == -450 &&
              Bluetooth_GetControl()->frame.joy_y == 600 &&
              Bluetooth_GetControl()->frame.forward == 1 &&
              Bluetooth_GetControl()->frame.Cam_T == 0 &&
              Bluetooth_GetControl()->frame.Shot == 1 &&
              Bluetooth_GetControl()->frame.aim == 1 &&
              Bluetooth_GetControl()->frame.left_90 == 1 &&
              Bluetooth_GetControl()->frame.servo_buttons ==
                  (BT_SERVO_BUTTON_TB_M | BT_SERVO_BUTTON_TH_L) &&
              Bluetooth_GetControl()->frame.gap_pwm == 1800U);
        CHECK(Bluetooth_GetArmControl()->direction ==
                  BT_DIRECTION_ARM_PRESET_NEXT &&
              Bluetooth_GetArmControl()->arm_x == 333 &&
              Bluetooth_GetArmControl()->arm_y == -444);
    }
}

static void TestCombinedFraming(void)
{
    uint8_t packet[BT_ARM_COMBINED_FRAME_SIZE];
    uint32_t car_before, arm_before;
    unsigned i;
    TestArmProtocolMatrix();
    Bluetooth_SetExtended(1U);
    MakeCombined(packet, 0, -400, 250, 700, -800, 0, 0, 1, 1);
    packet[23] = 1U; /* LEFT_90 at bytes 23..24. */
    packet[25] = 3U; /* SERVO_MODE at bytes 25..26. */
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
          Bluetooth_GetControl()->frame.servo_mode == 3 &&
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
    CHECK(Bluetooth_GetSequence() == car_before + 1U);
    packet[25] = 0U;
    packet[27] = 26U; /* ARM_CMD outside 0/1/10..13/20..25. */
    packet[33] = 0U;
    for (i = 1U; i <= 32U; ++i) packet[33] += packet[i];
    Feed(packet, sizeof(packet)); Bluetooth_Process();
    CHECK(Bluetooth_GetArmSequence() == arm_before + 1U);
    now += BT_FRAME_GAP_TIMEOUT_MS + 1U;
    { uint8_t legacy_dual[17] = {0xA5U};
      legacy_dual[16] = 0x5AU;
      Feed(legacy_dual, sizeof(legacy_dual)); Bluetooth_Process(); }
    CHECK(Bluetooth_GetArmSequence() == arm_before + 2U);
    now += BT_FRAME_GAP_TIMEOUT_MS + 1U;
    Bluetooth_SetExtended(0U);
}

static void TestDualAndSetup(void)
{
    static const uint16_t preset[8][4] = {
        {1532U, 1972U, 573U, 2192U},
        {1421U, 2071U, 812U, 2192U},
        {1421U, 2071U, 812U, 500U},
        {1717U, 2297U, 884U, 500U},
        {1717U, 2297U, 884U, 1800U},
        {1499U, 1855U, 528U, 1800U},
        {1499U, 1855U, 528U, 500U},
        {1800U, 1855U, 528U, 500U}
    };
    unsigned before, i;
    uint32_t sequence, arm_sequence;
    char expected[96], reply_part[24];
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
    DualPacket(0,0,0,0,0,0); Expect("AUTO_GRANT RESULT=0");
    for (i=0U; i<8U; ++i) {
        before=tx_count;
        DualPacket(BT_DIRECTION_ARM_PRESET_NEXT,0,0,0,0,0);
        (void)snprintf(reply_part,sizeof(reply_part),"INDEX=%u",i);
        Expect(reply_part);
        (void)snprintf(expected,sizeof(expected),
            "{#000P%04uT1500!#001P%04uT1500!#002P%04uT1500!#003P%04uT1500!}",
            preset[i][0],preset[i][1],preset[i][2],preset[i][3]);
        CHECK(tx_count==before+1U && strcmp(last_frame,expected)==0);
        DualPacket(BT_DIRECTION_ARM_PRESET_NEXT,0,0,0,0,0);
        CHECK(tx_count==before+1U); /* Held button has no repeat/backlog. */
        Advance(ARM_TUNER_PRESET_MOVE_MS+1U);
        DualPacket(0,0,0,0,0,0); /* Release and refresh both leases. */
    }
    /* The existing eight bool buttons in (7).pro select the same fixed
     * actions while the added three shorts continue to provide fine jogs. */
    before=tx_count;
    Phone7Packet(0U,0,0,0,0U);
    Phone7Packet(BT_SERVO_BUTTON_TH_G,0,0,0,0U);
    Expect("INDEX=6");
    CHECK(tx_count==before+1U &&
          strcmp(last_frame,
                 "{#000P1499T1500!#001P1855T1500!#002P0528T1500!#003P0500T1500!}")==0);
    Phone7Packet(BT_SERVO_BUTTON_TH_G,0,0,0,0U);
    CHECK(tx_count==before+1U); /* Held bool has no repeat/backlog. */
    Advance(ARM_TUNER_PRESET_MOVE_MS+1U);
    Phone7Packet(0U,0,0,0,0U);
    before=tx_count;
    mock_car_state=CAR_RUNNING;
    DualPacket(BT_DIRECTION_ARM_PRESET_NEXT,0,0,0,0,0);
    Expect("PRESET_CHASSIS_NOT_READY"); CHECK(tx_count==before);
    mock_car_state=CAR_READY; DualPacket(0,0,0,0,0,0);
    mock_motor_idle=0U;
    DualPacket(BT_DIRECTION_ARM_PRESET_NEXT,0,0,0,0,0);
    Expect("PRESET_CHASSIS_NOT_READY"); CHECK(tx_count==before);
    mock_motor_idle=1U; DualPacket(0,0,0,0,0,0);
    DualPacket(BT_DIRECTION_ARM_PRESET_NEXT,500,0,0,0,0);
    Expect("PRESET_CHASSIS_NOT_READY"); CHECK(tx_count==before);
    DualPacket(0,0,0,0,0,0);
    Command("@ARM STOP"); FinishStop();
    Command("@ARM DUAL OFF"); Expect("SHORTS=13");
    CHECK(!Bluetooth_IsExtended());
    RemotePacket(0,0,0,0);
    Command("@ARM EXIT");
}

static void TestPresetArbitration(void)
{
    uint8_t packet[BT_ARM_COMBINED_FRAME_SIZE];
    uint8_t mode_frame[BT_CONTROL_FRAME_SERVO_MODE_SIZE] = {0xA5U};
    unsigned before, i;
    Bluetooth_Init(); ArmTuner_Init(); ServoRemote_Init();
    /* A 29-byte neutral mode zero frame arms preset control. */
    mode_frame[28] = 0x5AU;
    Feed(mode_frame, sizeof(mode_frame)); Bluetooth_Process(); ServoRemote_Process();
    before = tx_count;
    MakeCombined(packet, BT_ARM_REMOTE_ENTER, 0, 0, 0, 0, 0, 0, 0, 0);
    packet[25] = 3U;
    packet[33] = 0U;
    for (i = 1U; i <= 32U; ++i) packet[33] += packet[i];
    Feed(packet, sizeof(packet)); Bluetooth_Process(); ServoRemote_Process();
    CHECK(ArmTuner_IsSessionActive() && tx_count == before);
    Command("@ARM EXIT"); Expect("MODE=0");
    CHECK(!ArmTuner_IsSessionActive());
    Feed(mode_frame, sizeof(mode_frame)); Bluetooth_Process(); ServoRemote_Process();
    mode_frame[25] = 3U;
    mode_frame[27] = 3U;
    Feed(mode_frame, sizeof(mode_frame)); Bluetooth_Process(); ServoRemote_Process();
    Arm_Process();
    CHECK(tx_count == before + 1U);
    CHECK(strcmp(last_frame,
                 "{#000P1421T1000!#001P2071T1000!#002P0812T1000!}") == 0);
    Command("@ARM DUAL START"); Expect("PRESET_BUSY");
    CHECK(!ArmTuner_IsSessionActive() && Arm_GetStatus().state == ARM_RUNNING);
}

static void TestPe4ModeOneTx(void)
{
    static const char expected[] =
        "{#000P1532T1000!#001P2219T1000!#002P1202T1000!}";
    unsigned before;
    (void)Arm_Stop();
    FinishStop();
    now = 0U;
    pe4_low = 0U;
    Bluetooth_Init();
    Arm_Init();
    ArmTuner_Init();
    ServoRemote_Init();
    before = tx_count;
    CHECK(!Bluetooth_IsConnected());
    pe4_low = 1U;
    ServoRemote_Process();
    now = 19U;
    ServoRemote_Process(); Arm_Process();
    CHECK(tx_count == before);
    now = 20U;
    ServoRemote_Process(); Arm_Process();
    CHECK(tx_count == before + 1U && strcmp(last_frame, expected) == 0);
    pe4_low = 0U;
    ServoRemote_Process();
    now = 40U;
    ServoRemote_Process();
    pe4_low = 1U;
    ServoRemote_Process();
    now = 60U;
    ServoRemote_Process(); Arm_Process();
    CHECK(tx_count == before + 1U); /* Busy preset rejects a second PE4 press. */
    Advance(1001U);
    ServoRemote_Process();
    CHECK(!Arm_GetStatus().motion_allowed);
    pe4_low = 0U;
    ServoRemote_Process();
    now += 20U;
    ServoRemote_Process();
    pe4_low = 1U;
    ServoRemote_Process();
    now += 20U;
    ServoRemote_Process(); Arm_Process();
    CHECK(tx_count == before + 2U && strcmp(last_frame, expected) == 0);
    Advance(1001U); ServoRemote_Process();
    pe4_low = 0U;
    ServoRemote_Process(); now += 20U; ServoRemote_Process();
    arm_tx = HAL_ERROR;
    pe4_low = 1U;
    ServoRemote_Process(); now += 20U; ServoRemote_Process();
    Arm_Process();
    CHECK(tx_count == before + 3U && Arm_GetStatus().state == ARM_STOPPING &&
          Arm_GetStatus().error == ARM_ERROR_TRANSPORT);
    FinishStop();
    CHECK(Arm_GetStatus().state == ARM_FAULT);
    pe4_low = 0U;
    ServoRemote_Process(); now += 20U; ServoRemote_Process();
    pe4_low = 1U;
    ServoRemote_Process(); now += 20U; ServoRemote_Process();
    CHECK(Arm_GetStatus().state == ARM_FAULT); /* Fault is not overwritten. */
    CHECK(Arm_ClearFault() == ARM_OK);
}

static void TestThGGripTx(void)
{
    uint8_t neutral[BT_CONTROL_FRAME_SERVO_MODE_SIZE] = {0xA5U};
    uint8_t th_g[BT_CONTROL_FRAME_SERVO_BOOL_SIZE] = {0xA5U};
    unsigned before;
    (void)Arm_Stop();
    FinishStop();
    pe4_low = 0U;
    Bluetooth_Init();
    Arm_Init();
    ArmTuner_Init();
    ServoRemote_Init();
    neutral[sizeof(neutral) - 1U] = 0x5AU;
    Feed(neutral, sizeof(neutral));
    Bluetooth_Process(); ServoRemote_Process();
    before = tx_count;
    th_g[1] = BT_SERVO_BUTTON_TH_G;
    th_g[sizeof(th_g) - 2U] = th_g[1];
    th_g[sizeof(th_g) - 1U] = 0x5AU;
    Feed(th_g, sizeof(th_g));
    Bluetooth_Process(); ServoRemote_Process(); Arm_Process();
    CHECK(tx_count == before + 1U);
    CHECK(strcmp(last_frame,
                 "{#000P1499T1000!#001P1855T1000!#002P0528T1000!#003P0500T1000!}") == 0);
}

static void TestTrimTestCopy(void)
{
    unsigned before;
    ArmStep_t step = {0};
    (void)Arm_Stop(); FinishStop();
    ArmTuner_Init(); ServoRemote_Init();
    before = tx_count;
    Command("@BENCH PREP BALL"); Expect("RESULT=0 MASK=7");
    CHECK(strstr(last_frame, "#000P1356T2000!") && strstr(last_frame, "#001P1850T2000!"));
    CHECK(strstr(last_frame, "#002P0698T2000!") && strstr(last_frame, "#003") == NULL);
    CHECK(tx_count == before + 1U && ArmTuner_IsSessionActive());
    Command("@ARM TRIM BEGIN BALL"); Expect("LEGACY_EXIT_REQUIRED");
    Command("@BENCH GRIP 900"); Expect("BUSY_OR_SESSION");
    Advance(2310U); DrainReplies(); Expect("BENCH COMPLETE_ESTIMATED");
    CHECK(!Arm_GetStatus().motion_allowed && !ArmTrimBench_IsActive());
    before = tx_count;
    Feed("@ARM TRIM BE", 12U); Bluetooth_Process(); CHECK(tx_count == before);
    Feed("GIN BALL\r\n", 10U); Bluetooth_Process(); DrainReplies();
    Expect("PROFILE=BALL NO_MOTION"); CHECK(tx_count == before && Arm_ExternalMotionOwned());
    step.joint_mask = 0x07U; step.move_ms = 1000U;
    step.position[0] = 1356U; step.position[1] = 1850U; step.position[2] = 698U;
    CHECK(Arm_StartOriginalPreset(&step) == ARM_BUSY);
    CHECK(Arm_ResetController() == ARM_BUSY);
    CHECK(Arm_SendImmediate(&step) == ARM_BUSY);
    pe4_low = 0U; ServoRemote_Init(); pe4_low = 1U;
    ServoRemote_Process(); Advance(21U); ServoRemote_Process();
    CHECK(tx_count == before); /* Physical button cannot steal trim ownership. */
    pe4_low = 0U; ServoRemote_Process(); Advance(21U); ServoRemote_Process();
    Command("@ARM TRIM DX 2"); Expect("RESULT=0");
    CHECK(!strstr(last_frame, "#003P"));
    Command("@ARM TRIM DX 1"); Expect("RESULT=2");
    Advance(1300U); DrainReplies(); Expect("COMPLETE_ESTIMATED");
    CHECK(ArmTrimBluetooth_GetStatus().offset_mm == 2.0f);
    Command("@ARM TRIM DX -7"); Expect("RESULT=0");
    Advance(1800U); DrainReplies(); Expect("COMPLETE_ESTIMATED");
    Command("@ARM TRIM DX 9"); Expect("RESULT=0");
    Advance(2000U); DrainReplies(); Expect("COMPLETE_ESTIMATED");
    CHECK(ArmTrimBluetooth_GetStatus().offset_mm == 4.0f);
    /* L2=84.75 mm retains the BALL +4 mm boundary; +5 mm must send nothing. */
    before = tx_count;
    Command("@ARM TRIM DX 1"); Expect("RESULT=4"); CHECK(tx_count == before);
    Command("@BENCH GRIP 900"); Expect("BUSY_OR_SESSION");
    Command("@ARM TRIM END"); Expect("RESULT=0");
    Command("@BENCH GRIP 900"); Expect("RESULT=0 MASK=8");
    CHECK(strcmp(last_frame, "{#003P0900T1500!}") == 0); /* Completed trim joints stay unchanged. */
    Advance(1810U); DrainReplies(); Expect("BENCH COMPLETE_ESTIMATED");
    Command("@BENCH PREP HOSTAGE"); Expect("RESULT=0");
    CHECK(strstr(last_frame, "#000P1684T2000!") && strstr(last_frame, "#003") == NULL);
    Advance(2310U); DrainReplies();
    Command("@ARM TRIM BEGIN HOSTAGE"); Expect("PROFILE=HOSTAGE NO_MOTION");
    Command("@ARM TRIM DX -2"); Expect("RESULT=0");
    Command("@ARM STOP"); Expect("STOP REQUESTED");
    Advance(8U); DrainReplies(); Expect("CANCELLED");
    CHECK(!ArmTrimBluetooth_GetStatus().reference_valid);
    Command("@ARM TRIM END"); Expect("RESULT=0");
    Command("@BENCH PREP BUCKET"); Expect("RESULT=0");
    CHECK(strstr(last_frame, "#000P1566T2000!") && strstr(last_frame, "#003") == NULL);
    Advance(2310U); DrainReplies();
    Command("@ARM TRIM BEGIN BUCKET"); Expect("PROFILE=BUCKET NO_MOTION");
    Command("@ARM TRIM DX -2"); Expect("RESULT=0");
    Bluetooth_ErrorCallback(&huart6); Bluetooth_Process();
    Advance(8U); DrainReplies(); Expect("CANCELLED");
    Command("@ARM TRIM END"); Expect("RESULT=0");
    before = tx_count;
    Command("@BENCH GRIP 499"); Expect("GRIP_P_500_TO_2500"); CHECK(tx_count == before);
    Command("@BENCH PREP BALL"); Expect("RESULT=0");
    Command("@BENCH STOP"); Advance(8U); DrainReplies(); Expect("BENCH CANCELLED");
    ArmTuner_Init(); ServoRemote_Init();
}

static void MakeTrimTest(uint8_t *packet, int command, int dx, int close_p, int open_p)
{
    const int values[4] = {command, dx, close_p, open_p};
    unsigned i;
    memset(packet, 0, BT_TRIM_TEST_FRAME_SIZE);
    packet[0] = 0xA5U;
    for (i = 0U; i < 4U; ++i) {
        uint16_t v = (uint16_t)values[i];
        packet[1U + 2U * i] = (uint8_t)v;
        packet[2U + 2U * i] = (uint8_t)(v >> 8);
    }
    for (i = 1U; i <= 8U; ++i) packet[9] += packet[i];
    packet[10] = 0x5AU;
}

static void TestButton(int command, int dx, int close_p, int open_p)
{
    uint8_t packet[BT_TRIM_TEST_FRAME_SIZE];
    MakeTrimTest(packet, command, dx, close_p, open_p);
    last_reply[0] = '\0';
    Feed(packet, sizeof(packet)); Bluetooth_Process();
    ArmTuner_Process(); Arm_Process(); DrainReplies();
}

static void TestTrimTestPage(void)
{
    uint8_t packet[BT_TRIM_TEST_FRAME_SIZE];
    unsigned before;
    uint32_t base_seq = Bluetooth_GetSequence(), arm_seq = Bluetooth_GetArmSequence();
    uint32_t stamp = Bluetooth_GetLastRxTick(), invalid = Bluetooth_GetInvalidFrameCount();
    before = tx_count;
    /* All fields are signed little-endian shorts. Split packets and stale
     * packets must not produce movement or renew the chassis keepalive. */
    MakeTrimTest(packet, 1, 0, 0, 0);
    Feed(packet, 4U); Bluetooth_Process(); CHECK(tx_count == before);
    Feed(packet + 4U, sizeof(packet) - 4U); Bluetooth_Process();
    ArmTuner_Process(); Arm_Process(); DrainReplies(); Expect("RESULT=0");
    CHECK(tx_count == before + 1U);
    CHECK(strstr(last_frame, "#000P1356T2000!") && strstr(last_frame, "#001P1850T2000!") &&
          strstr(last_frame, "#002P0698T2000!") && strstr(last_frame, "#003") == NULL);
    TestButton(1, 0, 0, 0); CHECK(tx_count == before + 1U); /* Held button. */
    TestButton(2, 0, 0, 0); Expect("BUSY_OR_SESSION"); /* Not queued. */
    Advance(2310U); DrainReplies(); Expect("COMPLETE_ESTIMATED");
    TestButton(2, 0, 0, 0); CHECK(tx_count == before + 1U);
    TestButton(0, 0, 0, 0);
    TestButton(4, 0, 0, 0); Expect("PROFILE=BALL NO_MOTION");
    TestButton(0, 2, 0, 0); CHECK(tx_count == before + 1U); /* Input is not execution. */
    TestButton(7, 2, 0, 0); Expect("RESULT=0");
    CHECK(strstr(last_frame, "#003") == NULL);
    TestButton(7, -2, 0, 0); CHECK(tx_count == before + 2U); /* Held with changed input. */
    Advance(1310U); DrainReplies();
    CHECK(ArmTrimBluetooth_GetStatus().offset_mm > 1.9f);
    TestButton(0, -2, 0, 0);
    TestButton(7, -2, 0, 0); Expect("RESULT=0");
    Advance(1310U); DrainReplies();
    CHECK(ArmTrimBluetooth_GetStatus().offset_mm < 0.1f);
    TestButton(0, 0, 0, 0);
    TestButton(8, 0, 0, 0); Expect("TRIM END RESULT=0");
    TestButton(0, 0, 0, 0);
    before = tx_count;
    TestButton(9, 0, 0, 0); Expect("GRIP_P_500_TO_2500"); CHECK(tx_count == before);
    TestButton(0, 0, 900, 1480);
    TestButton(9, 0, 900, 1480); Expect("RESULT=0");
    CHECK(strcmp(last_frame, "{#003P0900T1500!}") == 0);
    Advance(1810U); DrainReplies();
    TestButton(0, 0, 900, 1480);
    TestButton(10, 0, 900, 1480); Expect("RESULT=0");
    CHECK(strcmp(last_frame, "{#003P1480T1500!}") == 0);
    Advance(1810U); DrainReplies();
    TestButton(0, 0, 0, 0);
    TestButton(2, 0, 0, 0); Expect("RESULT=0");
    CHECK(strstr(last_frame, "#000P1684T2000!") && strstr(last_frame, "#001P2136T2000!") &&
          strstr(last_frame, "#002P0785T2000!") && strstr(last_frame, "#003") == NULL);
    Advance(2310U); DrainReplies();
    TestButton(0, 0, 0, 0); TestButton(5, 0, 0, 0); Expect("PROFILE=HOSTAGE NO_MOTION");
    TestButton(0, 0, 0, 0); TestButton(8, 0, 0, 0); Expect("RESULT=0");
    TestButton(0, 0, 0, 0); TestButton(3, 0, 0, 0); Expect("RESULT=0");
    CHECK(strstr(last_frame, "#000P1566T2000!") && strstr(last_frame, "#001P1896T2000!") &&
          strstr(last_frame, "#002P0673T2000!") && strstr(last_frame, "#003") == NULL);
    Advance(2310U); DrainReplies();
    TestButton(0, 0, 0, 0); TestButton(6, 0, 0, 0); Expect("PROFILE=BUCKET NO_MOTION");
    TestButton(0, 0, 0, 0); TestButton(7, -2, 0, 0); Expect("RESULT=0");
    TestButton(0, 0, 0, 0); TestButton(12, 0, 0, 0); Expect("STOP REQUESTED");
    Advance(8U); DrainReplies(); CHECK(!ArmTrimBluetooth_GetStatus().reference_valid);
    TestButton(0, 0, 0, 0); TestButton(13, 0, 0, 0); Expect("TRIM CLEAR RESULT=1");
    TestButton(0, 0, 0, 0); TestButton(8, 0, 0, 0); Expect("RESULT=0");
    CHECK(Bluetooth_GetLastTestFrameLength() == 11U && Bluetooth_GetTestSequence() > 20U);
    CHECK(Bluetooth_GetSequence() == base_seq && Bluetooth_GetArmSequence() == arm_seq &&
          Bluetooth_GetLastRxTick() == stamp);
    before = tx_count;
    MakeTrimTest(packet, 1, 0, 0, 0); ++packet[9];
    Feed(packet, sizeof(packet)); Bluetooth_Process(); CHECK(tx_count == before);
    MakeTrimTest(packet, 1, 151, 0, 0);
    Feed(packet, sizeof(packet)); Bluetooth_Process(); CHECK(tx_count == before);
    MakeTrimTest(packet, 14, 0, 0, 0);
    Feed(packet, sizeof(packet)); Bluetooth_Process(); CHECK(tx_count == before);
    CHECK(Bluetooth_GetInvalidFrameCount() == invalid + 3U);
    /* Multiple frames in one RX burst are dispatched one at a time, including
     * button release; otherwise the next repeated action could be lost. */
    MakeTrimTest(packet, 11, 0, 0, 0); Feed(packet, sizeof(packet));
    MakeTrimTest(packet, 0, 0, 0, 0); Feed(packet, sizeof(packet));
    MakeTrimTest(packet, 11, 0, 0, 0); Feed(packet, sizeof(packet));
    Bluetooth_Process(); DrainReplies(); Expect("NOT_FEEDBACK");
    Bluetooth_Process(); Bluetooth_Process(); DrainReplies(); Expect("NOT_FEEDBACK");
    MakeTrimTest(packet, 1, 0, 0, 0); Feed(packet, sizeof(packet));
    now += BT_FAILSAFE_TIMEOUT_MS + 1U; Bluetooth_Process(); CHECK(tx_count == before);
    /* A stale press did not latch the command; a fresh press is accepted. */
    TestButton(1, 0, 0, 0); Expect("RESULT=0");
    TestButton(0, 0, 0, 0); TestButton(12, 0, 0, 0); Advance(8U); DrainReplies();
    CHECK(!ArmTrimBench_IsActive());
    Command("@ARM LINK"); Expect("TEST_LEN=11");
    ArmTuner_Init(); ServoRemote_Init();
}

static void MakeSimpleTest(uint8_t *packet, uint8_t buttons, int dx)
{
    uint16_t value = (uint16_t)dx;
    packet[0] = 0xA5U; packet[1] = buttons; packet[2] = BT_TRIM_SIMPLE_MARKER;
    packet[3] = (uint8_t)value; packet[4] = (uint8_t)(value >> 8);
    packet[5] = (uint8_t)(packet[1] + packet[2] + packet[3] + packet[4]);
    packet[6] = 0x5AU;
}

static void SimpleButton(uint8_t buttons, int dx)
{
    uint8_t packet[BT_TRIM_SIMPLE_FRAME_SIZE];
    MakeSimpleTest(packet, buttons, dx);
    last_reply[0] = '\0'; Feed(packet, sizeof(packet)); Bluetooth_Process();
    ArmTuner_Process(); Arm_Process(); DrainReplies();
}

static void HoldSimple(int direction, unsigned ms)
{
    while (ms != 0U) {
        unsigned step = ms > 100U ? 100U : ms;
        SimpleButton(BT_TRIM_MOVE, direction);
        Advance(step);
        ms -= step;
    }
}

static void TestSimplePage(void)
{
    uint8_t packet[BT_TRIM_SIMPLE_FRAME_SIZE];
    unsigned before, stopped;
    uint32_t seq = Bluetooth_GetSequence(), arm_seq = Bluetooth_GetArmSequence();
    uint32_t stamp = Bluetooth_GetLastRxTick(), invalid;
    ArmTrimStatus_t pose;
    before = tx_count;
    SimpleButton(BT_TRIM_CLOSE, 0); Expect("FIXED_ACTION_REQUIRED"); CHECK(tx_count == before);
    SimpleButton(0, 0);
    /* READY sends the confirmed fixed pose, then automatically initializes
     * trim only after the 2000ms move and 300ms hold have completed. */
    MakeSimpleTest(packet, BT_TRIM_BALL, 0);
    Feed(packet, 3U); Bluetooth_Process(); CHECK(tx_count == before);
    Feed(packet + 3U, sizeof(packet) - 3U); Bluetooth_Process();
    ArmTuner_Process(); Arm_Process(); DrainReplies(); Expect("RESULT=0");
    CHECK(strstr(last_frame, "#000P1356T2000!") && strstr(last_frame, "#003") == NULL);
    SimpleButton(BT_TRIM_BALL, 0); CHECK(tx_count == before + 1U);
    Advance(2299U); CHECK(!ArmTrimBluetooth_OwnsMotion());
    SimpleButton(0, 0); SimpleButton(BT_TRIM_MOVE, 2); Expect("BEGIN_REQUIRED");
    Advance(12U); DrainReplies(); Expect("TRIM_AUTO_INITIALIZED");
    CHECK(ArmTrimBluetooth_GetStatus().reference_valid);
    SimpleButton(0, 2); before = tx_count;
    SimpleButton(BT_TRIM_MOVE, 2); Expect("RESULT=0"); CHECK(tx_count == before + 1U);
    SimpleButton(BT_TRIM_MOVE, 30); CHECK(tx_count == before + 1U);
    Command("@BENCH READY HOSTAGE"); Expect("TRIM_BUSY_OR_FAULT");
    Command("@BENCH CLOSE"); Expect("TRIM_BUSY_OR_FAULT");
    HoldSimple(2, 350U); SimpleButton(0, 2);
    Advance(1310U); DrainReplies();
    pose = ArmTrimBluetooth_GetStatus(); CHECK(pose.offset_mm > 1.9f);
    SimpleButton(0, 0); before = tx_count;
    SimpleButton(BT_TRIM_CLOSE, 0); Expect("ONLY_003");
    CHECK(strcmp(last_frame, "#003P0500T1500!") == 0 && tx_count == before + 1U);
    CHECK(ArmTrimBluetooth_OwnsMotion());
    SimpleButton(0, 0); SimpleButton(BT_TRIM_HOSTAGE, 0); Expect("BUSY_OR_FAULT");
    SimpleButton(0, -2); SimpleButton(BT_TRIM_MOVE, -2); Expect("BENCH_BUSY");
    Advance(1810U); DrainReplies(); Expect("PLANAR_JOINTS_UNCHANGED");
    CHECK(ArmTrimBluetooth_GetStatus().reference_valid &&
          ArmTrimBluetooth_GetStatus().offset_mm == pose.offset_mm);
    CHECK(memcmp(ArmTrimBluetooth_GetStatus().estimated_position, pose.estimated_position,
                 sizeof(pose.estimated_position)) == 0);
    SimpleButton(0, 0); SimpleButton(BT_TRIM_OPEN, 0); Expect("ONLY_003");
    CHECK(strcmp(last_frame, "#003P1800T1500!") == 0);
    Advance(1810U); DrainReplies();
    SimpleButton(0, -2); SimpleButton(BT_TRIM_MOVE, -2); Expect("RESULT=0");
    HoldSimple(-2, 350U); SimpleButton(0, -2);
    Advance(1310U); DrainReplies(); CHECK(ArmTrimBluetooth_GetStatus().offset_mm < 0.1f);
    /* Switching scene automatically ends the idle old session and resets
     * reference/range/offset after the new fixed action, without BEGIN/END. */
    SimpleButton(0, 0); SimpleButton(BT_TRIM_HOSTAGE, 0); Expect("RESULT=0");
    CHECK(strstr(last_frame, "#000P1684T2000!") && strstr(last_frame, "#003") == NULL);
    CHECK(!ArmTrimBluetooth_OwnsMotion());
    Advance(2310U); DrainReplies(); Expect("TRIM_AUTO_INITIALIZED");
    SimpleButton(0, 0); SimpleButton(BT_TRIM_BUCKET, 0); Expect("RESULT=0");
    CHECK(strstr(last_frame, "#000P1566T2000!") && strstr(last_frame, "#003") == NULL);
    Advance(2310U); DrainReplies(); Expect("TRIM_AUTO_INITIALIZED");
    SimpleButton(0, 0); before = tx_count;
    SimpleButton(BT_TRIM_BALL | BT_TRIM_MOVE, 2); Expect("ONE_ACTION_BUTTON");
    CHECK(tx_count == before);
    SimpleButton(0, 0); SimpleButton(BT_TRIM_OPEN, 0); stopped = gripper_stops;
    SimpleButton(0, 0); SimpleButton(BT_TRIM_STOP | BT_TRIM_BALL, 999); Expect("STOP REQUESTED");
    Advance(8U); DrainReplies();
    CHECK(gripper_stops == stopped + 1U && !ArmTrimBluetooth_GetStatus().reference_valid);
    before = tx_count;
    SimpleButton(BT_TRIM_STOP | BT_TRIM_HOSTAGE, 0); CHECK(tx_count == before);
    /* Cancel during PREP suppresses its scheduled automatic reference. */
    SimpleButton(0, 0); SimpleButton(BT_TRIM_BALL, 0); Expect("RESULT=0");
    SimpleButton(0, 0); SimpleButton(BT_TRIM_STOP, 0); Advance(2310U); DrainReplies();
    CHECK(!ArmTrimBluetooth_OwnsMotion() && !ArmTrimBench_IsActive());
    SimpleButton(0, 0); SimpleButton(BT_TRIM_BALL, 0); Expect("RESULT=0");
    Advance(2310U); DrainReplies();
    stopped = gripper_stops; arm_tx = HAL_ERROR;
    SimpleButton(0, 0); SimpleButton(BT_TRIM_CLOSE, 0); Expect("BENCH GRIP FAULT");
    Advance(8U); DrainReplies(); CHECK(gripper_stops == stopped + 1U);
    SimpleButton(0, 0); SimpleButton(BT_TRIM_BALL, 0); Expect("BUSY_OR_FAULT");
    Command("@BENCH CLEAR"); Expect("FAULT_CLEARED");
    SimpleButton(0, 0); SimpleButton(BT_TRIM_BALL, 0); Expect("RESULT=0");
    Advance(2310U); DrainReplies();
    SimpleButton(0, 0); SimpleButton(BT_TRIM_OPEN, 0); stopped = gripper_stops;
    Bluetooth_ErrorCallback(&huart6); Bluetooth_Process(); Advance(8U); DrainReplies();
    CHECK(gripper_stops == stopped + 1U && !ArmTrimBluetooth_GetStatus().reference_valid);
    Command("@ARM TRIM END"); Expect("RESULT=0");
    before = tx_count; invalid = Bluetooth_GetInvalidFrameCount();
    MakeSimpleTest(packet, BT_TRIM_BALL, 0); ++packet[5];
    Feed(packet, sizeof(packet)); Bluetooth_Process(); CHECK(tx_count == before);
    MakeSimpleTest(packet, BT_TRIM_BALL, 151);
    Feed(packet, sizeof(packet)); Bluetooth_Process(); CHECK(tx_count == before);
    CHECK(Bluetooth_GetInvalidFrameCount() == invalid + 2U);
    SimpleButton(0, 0);
    MakeSimpleTest(packet, BT_TRIM_BALL, 0); Feed(packet, sizeof(packet));
    now += BT_FAILSAFE_TIMEOUT_MS + 1U; Bluetooth_Process(); CHECK(tx_count == before);
    SimpleButton(BT_TRIM_BALL, 0); Expect("RESULT=0");
    SimpleButton(0, 0); SimpleButton(BT_TRIM_STOP, 0); Advance(8U); DrainReplies();
    CHECK(Bluetooth_GetLastTestFrameLength() == 7U && Bluetooth_GetSequence() == seq &&
          Bluetooth_GetArmSequence() == arm_seq && Bluetooth_GetLastRxTick() == stamp);
    /* Failed fixed-pose transmission must not enable trim after its timer. */
    SimpleButton(0, 0); arm_tx = HAL_ERROR;
    SimpleButton(BT_TRIM_BALL, 0); Advance(2310U); DrainReplies(); Expect("BENCH FAULT");
    CHECK(!ArmTrimBluetooth_OwnsMotion() && !ArmTrimBluetooth_GetStatus().reference_valid);
    CHECK(Arm_ClearFault() == ARM_OK);
    /* Release and status packets in a single RX burst retain their edges. */
    MakeSimpleTest(packet, 0, 0); Feed(packet, sizeof(packet));
    MakeSimpleTest(packet, BT_TRIM_STATUS, 0); Feed(packet, sizeof(packet));
    MakeSimpleTest(packet, 0, 0); Feed(packet, sizeof(packet));
    MakeSimpleTest(packet, BT_TRIM_STATUS, 0); Feed(packet, sizeof(packet));
    Bluetooth_Process(); Bluetooth_Process(); DrainReplies(); Expect("NOT_FEEDBACK");
    Bluetooth_Process(); Bluetooth_Process(); DrainReplies(); Expect("NOT_FEEDBACK");
    ArmTuner_Init(); ServoRemote_Init();
}

static void TestJogWatchdogAndRelease(void)
{
    unsigned before;
    ArmTrimStatus_t s;
    SimpleButton(0, 0); SimpleButton(BT_TRIM_BALL, 0);
    Advance(2310U); DrainReplies(); CHECK(ArmTrimBluetooth_GetStatus().reference_valid);
    SimpleButton(0, 1); SimpleButton(BT_TRIM_MOVE, 1); Expect("RESULT=0");
    HoldSimple(1, 100U);
    SimpleButton(BT_TRIM_MOVE, -1); Expect("RELEASE_THEN_PRESS_AGAIN");
    Advance(1000U); DrainReplies();
    s = ArmTrimBluetooth_GetStatus(); CHECK(s.reference_valid && !s.jogging);
    before = tx_count; HoldSimple(-1, 300U); CHECK(tx_count == before);
    SimpleButton(0, -1); SimpleButton(BT_TRIM_MOVE, -1); Expect("RESULT=0");
    HoldSimple(-1, 350U); SimpleButton(0, -1);
    Advance(1000U); DrainReplies(); CHECK(ArmTrimBluetooth_GetStatus().reference_valid);
    /* A received press is not renewed by an unrelated query, old KEEP or a
     * damaged frame. Lost release stops motion and invalidates the estimate. */
    SimpleButton(BT_TRIM_MOVE, -1); Expect("RESULT=0");
    Advance(BT_FAILSAFE_TIMEOUT_MS + 12U); DrainReplies();
    CHECK(!ArmTrimBluetooth_GetStatus().reference_valid);
    Command("@ARM TRIM STATUS"); CHECK(strstr(last_reply, "NOT_FEEDBACK"));
    before = tx_count; HoldSimple(-1, 200U); CHECK(tx_count == before);
    SimpleButton(0, 0); SimpleButton(BT_TRIM_HOSTAGE, 0); Advance(2310U); DrainReplies();
    SimpleButton(0, 0); before = tx_count;
    SimpleButton(BT_TRIM_MOVE, 0); Expect("REJECTED_NO_MOTION"); CHECK(tx_count == before);
    SimpleButton(0, 1); SimpleButton(BT_TRIM_MOVE, 1); HoldSimple(1, 7000U);
    CHECK(ArmTrimBluetooth_GetStatus().state == ARM_TRIM_COMPLETE_ESTIMATED);
    before = tx_count; HoldSimple(1, 500U); CHECK(tx_count == before);
    SimpleButton(0, -1); SimpleButton(BT_TRIM_MOVE, -1); HoldSimple(-1, 150U);
    SimpleButton(BT_TRIM_STOP | BT_TRIM_MOVE, 999); Advance(8U); DrainReplies();
    CHECK(!ArmTrimBluetooth_GetStatus().reference_valid);
    SimpleButton(0, 0); Command("@ARM TRIM END"); Expect("RESULT=0");
}

static void TestDefaultByteAndDiagnostics(void)
{
    const uint8_t query[] = {0xA5U,0x40U,0U,0U,0U,0x40U,0x5AU};
    uint8_t packet[BT_TRIM_SIMPLE_FRAME_SIZE];
    unsigned before=tx_count, i;
    uint32_t bytes=Bluetooth_GetRxByteCount(), test=Bluetooth_GetTestSequence();
    uint32_t base=Bluetooth_GetSequence(), invalid;
    PID_StepMode mode=PID_Tuner_GetStatus()->step_mode;
    SimpleButton(0,0);
    last_reply[0]='\0'; Feed(query,sizeof(query)); Bluetooth_Process(); DrainReplies();
    Expect("NOT_FEEDBACK");
    CHECK(Bluetooth_GetRxByteCount()==bytes+14U && Bluetooth_GetTestSequence()==test+2U);
    CHECK(tx_count==before && Bluetooth_GetSequence()==base);
    /* A page configured with the old 84 byte remains usable. */
    SimpleButton(0,0); MakeSimpleTest(packet,BT_TRIM_STATUS,0);
    packet[2]=BT_TRIM_SIMPLE_LEGACY_MARKER; packet[5]+=BT_TRIM_SIMPLE_LEGACY_MARKER;
    last_reply[0]='\0'; Feed(packet,sizeof(packet)); Bluetooth_Process(); DrainReplies();
    Expect("NOT_FEEDBACK"); CHECK(tx_count==before);
    /* Reject another byte even when its checksum is valid. */
    invalid=Bluetooth_GetInvalidFrameCount(); test=Bluetooth_GetTestSequence();
    MakeSimpleTest(packet,BT_TRIM_BALL,0); packet[2]=1U; ++packet[5];
    Feed(packet,sizeof(packet)); Bluetooth_Process();
    CHECK(Bluetooth_GetInvalidFrameCount()>invalid && Bluetooth_GetTestSequence()==test);
    CHECK(tx_count==before);
    /* Invalid trim input must not change PID via an overlapping short-frame
     * prefix, including a continuation split just under the 100ms deadline. */
    invalid=Bluetooth_GetInvalidFrameCount(); MakeSimpleTest(packet,BT_TRIM_MOVE,0x5A08);
    Feed(packet,5U); Bluetooth_Process();
    CHECK(PID_Tuner_GetStatus()->step_mode==mode);
    now+=BT_FRAME_GAP_TIMEOUT_MS-1U;
    Feed(packet+5U,2U); Bluetooth_Process();
    CHECK(PID_Tuner_GetStatus()->step_mode==mode && tx_count==before);
    CHECK(Bluetooth_GetInvalidFrameCount()==invalid+1U);
    SimpleButton(0,0); SimpleButton(BT_TRIM_STOP,0x5A80); Expect("STOP REQUESTED");
    Advance(8U); DrainReplies(); SimpleButton(0,0);
    /* Make space for the periodic real RX counters on the debug UART. */
    for(i=0U;i<20U;++i) Debug_Process();
    debug_log[0]='\0'; now+=1000U; Bluetooth_Process();
    for(i=0U;i<20U;++i) Debug_Process();
    CHECK(strstr(debug_log,"[BT RX] bytes=") && strstr(debug_log,"test_len=7"));
    CHECK(Bluetooth_GetSequence()==base);
}

static void TestIndependentPageParkedGripper(void)
{
    unsigned before;
    mock_car_state=CAR_WAIT_CENTER;
    SimpleButton(0,0); SimpleButton(BT_TRIM_BALL,0); Expect("RESULT=0");
    Advance(2310U); DrainReplies(); Expect("TRIM_AUTO_INITIALIZED");
    before=tx_count;
    SimpleButton(0,0); SimpleButton(BT_TRIM_CLOSE,0); Expect("ONLY_003");
    CHECK(tx_count==before+1U && strcmp(last_frame,"#003P0500T1500!")==0);
    Advance(1810U); DrainReplies();
    /* This page never refreshes chassis keepalive: a parked LINK_LOST also
     * permits the gripper once its motor stop queue has finished. */
    mock_car_state=CAR_LINK_LOST; mock_motor_idle=0U; before=tx_count;
    SimpleButton(0,0); SimpleButton(BT_TRIM_OPEN,0); Expect("PARKED_IDLE_REQUIRED");
    CHECK(tx_count==before);
    mock_motor_idle=1U;
    SimpleButton(0,0); SimpleButton(BT_TRIM_OPEN,0); Expect("ONLY_003");
    CHECK(tx_count==before+1U && strcmp(last_frame,"#003P1800T1500!")==0);
    Advance(1810U); DrainReplies();
    mock_car_state=CAR_RUNNING; before=tx_count;
    SimpleButton(0,0); SimpleButton(BT_TRIM_CLOSE,0); Expect("PARKED_IDLE_REQUIRED");
    CHECK(tx_count==before);
    mock_car_state=CAR_OFF; mock_motor_fault=1U;
    SimpleButton(0,0); SimpleButton(BT_TRIM_CLOSE,0); Expect("PARKED_IDLE_REQUIRED");
    CHECK(tx_count==before);
    mock_motor_fault=0U;
    SimpleButton(0,0); SimpleButton(BT_TRIM_CLOSE,0); Expect("ONLY_003");
    Advance(1810U); DrainReplies();
    SimpleButton(0,0); SimpleButton(BT_TRIM_STOP,0); Advance(8U); DrainReplies();
    Command("@ARM TRIM END"); Expect("RESULT=0");
    mock_car_state=CAR_READY; ArmTuner_Init(); ServoRemote_Init();
}

static void TestReferencePreservesGripper(void)
{
    const uint8_t buttons[] = {BT_TRIM_BALL, BT_TRIM_HOSTAGE, BT_TRIM_BUCKET};
    const char *const prep[] = {"@BENCH PREP BALL", "@BENCH PREP HOSTAGE", "@BENCH PREP BUCKET"};
    const uint16_t targets[] = {ARM_TRIM_BENCH_CLOSE_P, ARM_TRIM_BENCH_OPEN_P};
    unsigned state, i, before, grip_before;
    for (state = 0U; state < 2U; ++state) {
        SimpleButton(0, 0); SimpleButton(BT_TRIM_BALL, 0);
        Advance(2310U); DrainReplies(); CHECK(ArmTrimBluetooth_GetStatus().reference_valid);
        SimpleButton(0, 0);
        grip_before = gripper_commands;
        SimpleButton(state == 0U ? BT_TRIM_CLOSE : BT_TRIM_OPEN, 0);
        Expect("ONLY_003");
        CHECK(gripper_commands == grip_before + 1U && last_gripper_target == targets[state]);
        Advance(1810U); DrainReplies();
        grip_before = gripper_commands;
        /* Reproduce gripping a ball, changing to BUCKET (or another scene),
         * then releasing. Scene switches must preserve either gripper target. */
        for (i = 0U; i < 3U; ++i) {
            SimpleButton(0, 0); before = tx_count;
            SimpleButton(buttons[i], 0); Expect("RESULT=0 MASK=7");
            CHECK(tx_count == before + 1U && strstr(last_frame, "#003") == NULL);
            CHECK(gripper_commands == grip_before && last_gripper_target == targets[state]);
            Advance(2310U); DrainReplies(); Expect("TRIM_AUTO_INITIALIZED");
            CHECK(ArmTrimBluetooth_GetStatus().reference_valid);
        }
        Command("@ARM TRIM END"); Expect("RESULT=0");
        /* The non-auto PREP entry and the legacy binary page share this
         * helper: neither may restore a reference-specific gripper value. */
        for (i = 0U; i < 3U; ++i) {
            before = tx_count;
            Command(prep[i]); Expect("RESULT=0 MASK=7");
            CHECK(tx_count == before + 1U && strstr(last_frame, "#003") == NULL);
            CHECK(gripper_commands == grip_before && last_gripper_target == targets[state]);
            Advance(2310U); DrainReplies(); Expect("BENCH COMPLETE_ESTIMATED");
            CHECK(!ArmTrimBluetooth_OwnsMotion());
        }
    }
}

static void TestRearBoxJogBoundary(void)
{
    unsigned before;
    SimpleButton(0, 0);
    SimpleButton(BT_TRIM_HOSTAGE, 0);
    Advance(2310U); DrainReplies();
    CHECK(ArmTrimBluetooth_GetStatus().reference_valid);
    CHECK(ArmTrimBluetooth_GetStatus().enabled_min_mm == -11.0f);
    SimpleButton(0, -1);
    SimpleButton(BT_TRIM_MOVE, -1);
    HoldSimple(-1, 6000U);
    CHECK(ArmTrimBluetooth_GetStatus().state == ARM_TRIM_COMPLETE_ESTIMATED);
    CHECK(ArmTrimBluetooth_GetStatus().offset_mm == -11.0f);
    before = tx_count;
    HoldSimple(-1, 500U);
    CHECK(tx_count == before);
    SimpleButton(0, 0);
    Command("@ARM TRIM END"); Expect("RESULT=0");
}

int main(void)
{
    unsigned before, replies;
    uint32_t seq, stamp;
    uint8_t joy[BT_ARM_COMBINED_FRAME_SIZE];
    uint8_t tune[5]={0xA5,1,0,1,0x5A};
    char long_line[110];
    huart3.Instance=&huart3; huart3.Init.BaudRate=ZLIS2_BAUD_RATE;
    huart3.Init.Mode=UART_MODE_TX_RX; huart3.gState=HAL_UART_STATE_READY;
    CHECK(ZLIS2_Init(&huart3)==ZLIS2_OK);
    MakeCombined(joy, 0, 64, 0, 0, 0, 0, 0, 0, 0);
    Bluetooth_Init(); PID_Tuner_Init(); Arm_Init(); ArmTuner_Init();
    ArmTuner_Process(); Arm_Process(); CHECK(tx_count==0 && reply_count==0);
    TestCombinedFraming();
    Command("@ARM LINK"); Expect("ARM LINK RX_LEN="); Expect("INVALID=");
    seq=Bluetooth_GetSequence(); stamp=Bluetooth_GetLastRxTick();
    Command("@ARM SHOW"); Expect("NAME=SHOULDER"); Expect("STATE=IDLE");
    Expect("ID=0"); Expect("MIN=915"); Expect("MAX=1800");
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
    Feed(tune,sizeof(tune)); Bluetooth_Process(); now+=BT_FRAME_GAP_TIMEOUT_MS+1U;
    Bluetooth_Process(); CHECK(pid.kp>1.0f);
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
    TestPresetArbitration();
    TestPe4ModeOneTx();
    TestThGGripTx();
    TestTrimTestCopy();
    TestTrimTestPage();
    TestSimplePage();
    TestJogWatchdogAndRelease();
    TestDefaultByteAndDiagnostics();
    TestIndependentPageParkedGripper();
    TestReferencePreservesGripper();
    TestRearBoxJogBoundary();
    printf("PASS arm Bluetooth: %u checks (HAL simulation, not hardware)\n",checks);
    return 0;
}
