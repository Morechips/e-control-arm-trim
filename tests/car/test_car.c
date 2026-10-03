#include "board_inputs.h"
#include "board_input_config.h"
#include "bluetooth_driver.h"
#include "motor_driver.h"
#include "car_control.h"
#include "serial_io.h"
#include "mecanum_test.h"
#include "heading_control.h"
#include "turn_right.h"
#include "turn_config.h"
#include "pid_tuner.h"
#include "mock_maxicam.h"
#include "laser.h"
#include "board_inputs.h"
#include "servo.h"
#include "remote_heading.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include <math.h>
#include <float.h>
UART_HandleTypeDef huart1, huart3, huart5, huart6;
GPIO_TypeDef mock_gpiob, mock_gpioc, mock_gpiod, mock_gpioe;
I2C_HandleTypeDef hi2c1;
static uint32_t now;
static uint8_t stall_motor;
static uint8_t pd10_low;
static uint8_t pe0_low, pe4_low;
static uint8_t pc1_low, test_laser_on;
static unsigned test_laser_enable_count;
static unsigned test_servo_pose_calls, test_servo_aim_calls;
extern unsigned mock_reset_count;
extern HAL_StatusTypeDef mock_reset_result;
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *gpio)
{
    assert(port == GPIOB || port == GPIOC || port == GPIOD || port == GPIOE);
    assert(gpio->Mode == GPIO_MODE_INPUT && gpio->Pull == GPIO_PULLUP);
}
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{ (void)port; (void)pin; (void)state; }
void Laser_SetManualRequest(bool requested) { (void)requested; }
static void SyncInputs(void)
{
    mock_gpiob.IDR = UINT16_MAX;
    mock_gpioc.IDR = UINT16_MAX & (pc1_low ? (uint32_t)~GPIO_PIN_1 : UINT16_MAX);
    mock_gpiod.IDR = UINT16_MAX & (pd10_low ? (uint32_t)~GPIO_PIN_10 : UINT16_MAX);
    mock_gpioe.IDR = UINT16_MAX;
    if (pe0_low) mock_gpioe.IDR &= ~GPIO_PIN_0;
    if (pe4_low) mock_gpioe.IDR &= ~GPIO_PIN_4;
}
void Laser_Enable(void) { test_laser_on = 1U; ++test_laser_enable_count; }
void Laser_Disable(void) { test_laser_on = 0U; }
bool Laser_IsEnabled(void) { return test_laser_on != 0U; }
static HAL_StatusTypeDef next_tx_result;
static HAL_StatusTypeDef next_bt_tx_result;
static uint8_t stall_bt;
static char bt_sent[4096][PID_TX_LINE_SIZE];
static uint32_t bt_ticks[4096], bt_start_tick, bt_wire_ms;
static unsigned bt_sent_count;
static const uint8_t *bt_inflight;
static uint16_t bt_inflight_length;
static unsigned bt_frame_ok_count;
static unsigned bt_forward_frame_count, bt_control_forward_count;
typedef struct { uint8_t bytes[8]; uint16_t length; uint32_t tick; } Sent;
static Sent sent[8192];
static unsigned sent_count;
uint32_t HAL_GetTick(void) { return now; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u, uint8_t *p, uint16_t n)
{
    assert(n == 1U);
    if (u->RxState != HAL_UART_STATE_READY) return HAL_BUSY;
    u->rx = p; u->RxState = 1U; return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u, const uint8_t *p, uint16_t n)
{
    if (u == &huart3) {
        ++test_servo_pose_calls;
        if (n == 62U && memcmp(p, "{#000P1058T2000!", 16U) == 0) ++test_servo_aim_calls;
        u->gState = HAL_UART_STATE_READY;
        HAL_UART_TxCpltCallback(u);
        return HAL_OK;
    }
    if (u==&huart6 && next_bt_tx_result!=HAL_OK) {
        HAL_StatusTypeDef r=next_bt_tx_result; next_bt_tx_result=HAL_OK; return r;
    }
    if (u == &huart5 && next_tx_result != HAL_OK) {
        HAL_StatusTypeDef r = next_tx_result; next_tx_result = HAL_OK; return r;
    }
    if (u->pending) return HAL_BUSY;
    u->pending = 1U;
    if (u==&huart6) {
        assert(n>0 && n<PID_TX_LINE_SIZE && bt_sent_count<4096U);
        memcpy(bt_sent[bt_sent_count],p,n); bt_sent[bt_sent_count][n]='\0';
        bt_ticks[bt_sent_count++]=now;
        bt_start_tick=now; bt_wire_ms=((uint32_t)n*10000U+9599U)/9600U;
        bt_inflight=p; bt_inflight_length=n;
    }
    if (u == &huart5) {
        assert(n <= 8U && sent_count < 8192U);
        memcpy(sent[sent_count].bytes, p, n);
        sent[sent_count].length = n;
        sent[sent_count++].tick = now;
    }
    if (u == &huart1) {
        if (n >= 11U && memcmp(p, "BT FRAME OK", 11U) == 0)
            ++bt_frame_ok_count;
        if (n >= 15U && memcmp(p, "BT FRAME OK F=1", 15U) == 0)
            ++bt_forward_frame_count;
        if (n >= 19U && memcmp(p, "BT CONTROL: FORWARD", 19U) == 0)
            ++bt_control_forward_count;
    }
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *u) { u->pending = 0U; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *u) { u->RxState = HAL_UART_STATE_READY; return HAL_OK; }
static void Complete(UART_HandleTypeDef *u)
{
    if (u==&huart6 && u->pending)
        assert(memcmp(bt_inflight,bt_sent[bt_sent_count-1U],bt_inflight_length)==0);
    if (u->pending) { u->pending = 0U; HAL_UART_TxCpltCallback(u); }
}
static void Loop(void)
{
    SyncInputs(); Bluetooth_Process(); BoardInputs_ProcessControl(); Car_Control_Process(); Motor_Process(); Car_Control_Process();
    PID_Tuner_Process(); Bluetooth_DispatchServoActions(BoardInputs_TakeServoAimPress()); Debug_Process();
}
static void Step(unsigned ms)
{
    while (ms--) {
        ++now;
        if (!stall_motor) Complete(&huart5);
        Complete(&huart1);
        if (!stall_bt && (uint32_t)(now-bt_start_tick)>=bt_wire_ms) Complete(&huart6);
        Loop();
    }
}
static void Inject(UART_HandleTypeDef *u, const uint8_t *p, unsigned n)
{
    while (n--) {
        assert(u->rx != NULL);
        *u->rx = *p++;
        u->RxState = HAL_UART_STATE_READY;
        HAL_UART_RxCpltCallback(u);
    }
}
static void PackControl(uint8_t *p, const BluetoothControlFrame *c)
{
    int16_t v[] = {c->joy_x, c->joy_y, c->forward, c->backward, c->stop,
                   c->strafe_left, c->strafe_right, c->right_90, c->right_180};
    uint8_t sum = 0U;
    unsigned i;
    p[0] = 0xA5U;
    for (i = 0; i < 9U; ++i) {
        uint16_t value = (uint16_t)v[i];
        p[1U + 2U*i] = (uint8_t)value;
        p[2U + 2U*i] = (uint8_t)(value >> 8);
        sum = (uint8_t)(sum + p[1U + 2U*i] + p[2U + 2U*i]);
    }
    p[19] = sum; p[20] = 0x5AU;
}
static void Pack(uint8_t *p, int16_t brake_value, int16_t off, int16_t x, int16_t y)
{
    BluetoothControlFrame c = {0};
    c.stop = brake_value; (void)off; c.joy_x = x; c.joy_y = y;
    PackControl(p, &c);
}
static void ControlFrame(BluetoothControlFrame c);
static void Frame(int16_t brake, int16_t off, int16_t x, int16_t y)
{
    BluetoothControlFrame c = {0};
    c.brake = brake; c.disable = off; c.joy_x = x; c.joy_y = y;
    ControlFrame(c);
}
static void ControlFrame(BluetoothControlFrame c)
{
    uint8_t p[BT_CONTROL_FRAME_SIZE];
    PackControl(p,&c); Inject(&huart6,p,sizeof(p)); Bluetooth_Process();
    /* These safety fields are not present in the active phone .pro. Keep the
     * existing control-layer safety tests by injecting them after wire decode. */
    const BluetoothControlFrame *decoded = &Bluetooth_GetControl()->frame;
    CarRemoteInput_t input = {
        .command = { .joy_x=decoded->joy_x, .joy_y=decoded->joy_y,
            .forward=decoded->forward, .backward=decoded->backward, .stop=decoded->stop,
            .strafe_left=decoded->strafe_left, .strafe_right=decoded->strafe_right,
            .right_90=decoded->right_90, .right_180=decoded->right_180, .left_90=decoded->left_90,
            .vision_follow=decoded->Cam_T, .shot=decoded->Shot, .brake=c.brake, .disable=c.disable },
        .sequence=Bluetooth_GetSequence(), .received_tick=Bluetooth_GetLastRxTick(), .valid=1U
    };
    Car_Control_SubmitRemoteInput(&input);
    Loop();
}
/* Translation stimulus for safety/heading tests, independent of JOY mode. */
static void ButtonFrame(int16_t brake, int16_t off, int16_t forward)
{
    BluetoothControlFrame c = {0};
    c.brake = brake; c.disable = off;
    c.forward = forward > 0; c.backward = forward < 0;
    ControlFrame(c);
}

static void Reset(uint32_t tick)
{
    mock_reset_count = 0U; mock_reset_result = HAL_OK;
    now = tick; stall_motor = 0U; next_tx_result = HAL_OK;
    stall_bt=0U; next_bt_tx_result=HAL_OK; bt_sent_count=0U;
    bt_inflight=NULL; bt_inflight_length=0U; bt_start_tick=now; bt_wire_ms=0U;
    pd10_low = pe0_low = pe4_low = pc1_low = test_laser_on = 0U;
    test_laser_enable_count = 0U; bt_frame_ok_count = 0U;
    test_servo_pose_calls = test_servo_aim_calls = 0U;
    bt_forward_frame_count = bt_control_forward_count = 0U;
    memset(&huart1,0,sizeof(huart1)); memset(&huart5,0,sizeof(huart5)); memset(&huart6,0,sizeof(huart6));
    huart1.RxState = huart5.RxState = huart6.RxState = HAL_UART_STATE_READY;
    Debug_TxCallback(&huart1);
    JY61_Init(); Bluetooth_Init(); Motor_Init(); TestMaxiCam_Reset(); Car_Control_Init();
    SyncInputs(); BoardInputs_Init();
    memset(&huart3, 0, sizeof(huart3)); huart3.Instance = &huart3;
    huart3.Init.BaudRate = SERVO_BAUD_RATE; huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.gState = HAL_UART_STATE_READY; assert(Servo_Init(&huart3) == SERVO_OK);
    PID_Tuner_Init();
    sent_count = 0U;
    Step(40U); assert(Motor_IsIdle());
    if (!CAR_PD10_STANDALONE_TEST && CAR_BOOT_AUTO_ENABLE) {
        unsigned i;
        assert(sent_count==12U && Car_Control_GetState()==CAR_WAIT_CENTER);
        for(i=8U;i<12U;++i) assert(sent[i].bytes[0]==i-7U && sent[i].bytes[1]==0xF3U && sent[i].bytes[3]==1U);
    }
    sent_count = 0U;
}
static void Ready(void)
{
    ButtonFrame(0,0,1); assert(Car_Control_GetState()==CAR_WAIT_CENTER);
    Step(40U); ButtonFrame(0,0,1); assert(Car_Control_GetState()==CAR_WAIT_CENTER);
    Frame(0,0,0,0); assert(Car_Control_GetState()==CAR_READY);
}
static unsigned Count(uint8_t cmd)
{
    unsigned count=0U,i; for(i=0;i<sent_count;++i) if(sent[i].bytes[1]==cmd) ++count; return count;
}
static void AssertLatestPhysicalSpeed(uint8_t address, int16_t rpm)
{
    unsigned i=sent_count;
    while (i--) {
        if (sent[i].bytes[0]==address && sent[i].bytes[1]==0xF6U) {
            int16_t actual=(int16_t)((uint16_t)sent[i].bytes[3]*256U+sent[i].bytes[4]);
            if (sent[i].bytes[2]) actual=(int16_t)-actual;
            if (sent[i].length != 8U || actual != rpm)
                fprintf(stderr, "wheel %u speed: expected %d, got %d at %lu ms\n",
                        (unsigned)address, (int)rpm, (int)actual,
                        (unsigned long)now);
            assert(sent[i].length==8U && actual==rpm);
            return;
        }
    }
    assert(0);
}
static void AssertLatestSyncTransaction(void)
{
    unsigned i = sent_count;
    while (i-- > 0U) {
        unsigned wheel;
        if (sent[i].bytes[1] != 0xFFU) continue;
        assert(i >= 4U && sent[i].length == 4U && sent[i].bytes[0] == 0U);
        for (wheel = 0U; wheel < 4U; ++wheel) {
            const Sent *frame = &sent[i - 4U + wheel];
            assert(frame->bytes[0] == wheel + 1U && frame->bytes[1] == 0xF6U &&
                   frame->bytes[6] == 1U);
        }
        return;
    }
    assert(0);
}
static void TestParser(void)
{
    uint8_t p[BT_CONTROL_FRAME_SIZE], q[BT_CONTROL_FRAME_SIZE];
    uint8_t burst[2U * BT_CONTROL_FRAME_SIZE];
    uint8_t legacy[13] = {0xA5U, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x5AU};
    uint8_t bare_short[2] = {1U, 0U};
    BluetoothControlFrame c = {0};
    uint32_t sequence, stamp;
    unsigned i;
    Reset(0U);
    c.forward = 1; c.joy_x = -1000; c.joy_y = 1000;
    PackControl(p, &c);
    for (i = 0U; i < BT_CONTROL_FRAME_SIZE - 1U; ++i) {
        Inject(&huart6, p + i, 1U); Bluetooth_Process();
        assert(!Bluetooth_GetControl()->valid);
    }
    Inject(&huart6, p + BT_CONTROL_FRAME_SIZE - 1U, 1U);
    Bluetooth_Process();
    assert(Bluetooth_GetControl()->frame.forward == 1);
    assert(Bluetooth_GetControl()->frame.joy_x == -1000 &&
           Bluetooth_GetControl()->frame.joy_y == 1000);
    sequence = Bluetooth_GetSequence(); stamp = Bluetooth_GetLastRxTick();
    p[19] ^= 1U; Inject(&huart6, p, sizeof(p)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence && Bluetooth_GetLastRxTick() == stamp);
    PackControl(p, &c); p[20] = 0U;
    Inject(&huart6, p, sizeof(p)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence);
    c.forward = 2; PackControl(p, &c);
    Inject(&huart6, p, sizeof(p)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence);
    c.forward = 0; c.joy_x = 1001; PackControl(p, &c);
    Inject(&huart6, p, sizeof(p)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence);
    c.joy_x = -1000;
    c.forward = 0; c.stop = 1; PackControl(p, &c);
    c.stop = 0; c.backward = 1; PackControl(q, &c);
    memcpy(burst, p, sizeof(p)); memcpy(burst + sizeof(p), q, sizeof(q));
    Inject(&huart6, burst, sizeof(burst));
    Bluetooth_Process(); assert(Bluetooth_GetControl()->frame.stop == 1);
    Bluetooth_Process(); assert(Bluetooth_GetControl()->frame.backward == 1);
    assert(Bluetooth_GetSequence() == sequence + 2U);
    Inject(&huart6, q, 7U); Bluetooth_Process();
    now += BT_FRAME_GAP_TIMEOUT_MS + 1U;
    Inject(&huart6, q + 7U, sizeof(q) - 7U); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence + 2U);
    Inject(&huart6, q, sizeof(q)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence + 3U);
    Inject(&huart6, legacy, sizeof(legacy)); Bluetooth_Process();
    Inject(&huart6, bare_short, sizeof(bare_short)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence + 3U);
    now += BT_FRAME_GAP_TIMEOUT_MS + 1U;
    Inject(&huart6, q, sizeof(q)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence + 4U);
    now += BT_FAILSAFE_TIMEOUT_MS + 1U;
    assert(!Bluetooth_IsConnected());
    puts("PASS parser: active 21-byte APP profile / split / checksum / domain / burst / old-frame rejection / gap / timeout");
}

static void TestActualPhoneProfile(void)
{
    static const uint8_t observed_suffix[17] = {
        0x00U, 0x01U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
        0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x01U, 0x5AU
    };
    BluetoothControlFrame c = {0};
    uint8_t packet[BT_CONTROL_FRAME_SIZE], expected[BT_CONTROL_FRAME_SIZE];
    const uint8_t button_offsets[7] = {5U, 7U, 11U, 13U, 9U, 15U, 17U};
    int16_t *buttons[7] = {&c.forward, &c.backward, &c.strafe_left,
        &c.strafe_right, &c.stop, &c.right_90, &c.right_180};
    unsigned speed;
    uint32_t sequence;
    unsigned i;
    Reset(0U); Ready();
    memset(expected, 0, sizeof(expected));
    expected[0] = 0xA5U; expected[19] = 1U; expected[20] = 0x5AU;
    for (i = 0U; i < 7U; ++i) {
        *buttons[i] = 1;
        expected[button_offsets[i]] = 1U;
        PackControl(packet, &c);
        assert(memcmp(packet, expected, sizeof(packet)) == 0);
        *buttons[i] = 0;
        expected[button_offsets[i]] = 0U;
    }
    c.joy_x = 256; c.joy_y = -1; PackControl(packet, &c);
    assert(packet[1] == 0U && packet[2] == 1U &&
           packet[3] == 0xFFU && packet[4] == 0xFFU);
    c.joy_x = 600; c.joy_y = -600; PackControl(packet, &c);
    assert(packet[1] == 0x58U && packet[2] == 0x02U &&
           packet[3] == 0xA8U && packet[4] == 0xFDU);
    c.joy_x = c.joy_y = 0;
    c.forward = 1;
    PackControl(packet, &c);
    assert(packet[0] == 0xA5U && packet[5] == 1U &&
           packet[19] == 1U && packet[20] == 0x5AU);
    /* The old 25-byte parser consumed a 21-byte packet plus four bytes of
     * the second repeat, leaving exactly this observed 17-byte suffix. */
    assert(memcmp(packet + 4U, observed_suffix, sizeof(observed_suffix)) == 0);
    sequence = Bluetooth_GetSequence();
    Inject(&huart6, observed_suffix, sizeof(observed_suffix));
    Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence);
    bt_frame_ok_count = bt_forward_frame_count = bt_control_forward_count = 0U;
    speed = Count(0xF6U);
    Inject(&huart6, packet, sizeof(packet));
    Inject(&huart6, packet, sizeof(packet));
    Loop(); assert(Bluetooth_GetSequence() > 0U);
    Loop(); assert(Bluetooth_GetControl()->frame.forward == 1);
    Step(40U);
    assert(Count(0xF6U) > speed && bt_frame_ok_count > 0U &&
           bt_forward_frame_count > 0U && bt_control_forward_count > 0U);
    puts("PASS phone profile: 9 shorts / 21-byte duplicate burst / observed 17-byte suffix / BT FRAME OK / Forward output");
}

static void ImuSample(float yaw, float gz);
static void TestIndependentButtons(void)
{
    BluetoothControlFrame c = {0};
    unsigned speed, stopped;
    assert(BLUETOOTH_TEST_MOVE_RPM == 35);
    Reset(0U); Ready();
    c.forward = 1; ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_RUNNING);
    AssertLatestPhysicalSpeed(1U, -BLUETOOTH_TEST_MOVE_RPM);
    AssertLatestPhysicalSpeed(2U, BLUETOOTH_TEST_MOVE_RPM);
    AssertLatestPhysicalSpeed(3U, BLUETOOTH_TEST_MOVE_RPM);
    AssertLatestPhysicalSpeed(4U, -BLUETOOTH_TEST_MOVE_RPM);
    stopped = Count(0xFE);
    c.forward = 0; ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_READY && Count(0xFE) == stopped + 4U);
    ControlFrame(c); Step(40U); assert(Count(0xFE) == stopped + 4U);
    c.backward = 1; ControlFrame(c); Step(40U);
    AssertLatestPhysicalSpeed(1U, BLUETOOTH_TEST_MOVE_RPM);
    AssertLatestPhysicalSpeed(2U, -BLUETOOTH_TEST_MOVE_RPM);
    AssertLatestPhysicalSpeed(3U, -BLUETOOTH_TEST_MOVE_RPM);
    AssertLatestPhysicalSpeed(4U, BLUETOOTH_TEST_MOVE_RPM);
    c.backward = 0; ControlFrame(c); Step(40U);
    c.strafe_left = 1; ControlFrame(c); Step(40U);
    AssertLatestPhysicalSpeed(1U, BLUETOOTH_TEST_MOVE_RPM);
    AssertLatestPhysicalSpeed(2U, BLUETOOTH_TEST_MOVE_RPM);
    AssertLatestPhysicalSpeed(3U, -BLUETOOTH_TEST_MOVE_RPM);
    AssertLatestPhysicalSpeed(4U, -BLUETOOTH_TEST_MOVE_RPM);
    c.strafe_left = 0; ControlFrame(c); Step(40U);
    c.strafe_right = 1; ControlFrame(c); Step(40U);
    AssertLatestPhysicalSpeed(1U, -BLUETOOTH_TEST_MOVE_RPM);
    AssertLatestPhysicalSpeed(2U, -BLUETOOTH_TEST_MOVE_RPM);
    AssertLatestPhysicalSpeed(3U, BLUETOOTH_TEST_MOVE_RPM);
    AssertLatestPhysicalSpeed(4U, BLUETOOTH_TEST_MOVE_RPM);
    c.strafe_right = 0; ControlFrame(c); Step(40U);
    c.forward = c.backward = 1; speed = Count(0xF6); ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_READY && Count(0xF6) == speed);
    c.forward = c.backward = 0; c.strafe_left = c.strafe_right = 1;
    ControlFrame(c); Step(40U); assert(Count(0xF6) == speed);
    c.strafe_left = c.strafe_right = 0; c.forward = 1;
    ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_RUNNING);
    c.stop = 1; ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_BRAKE_LOCK);
    speed = Count(0xF6);
    c.stop = 0; ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_BRAKE_LOCK && Count(0xF6) == speed);
    c.forward = 0; ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_READY);
    c.forward = 1; ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_RUNNING);
    c.forward = 0; stopped = Count(0xFE); ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_READY && Count(0xFE) == stopped + 4U);
    assert(Heading_GetStatus()->omega_final == 0);
    c.brake = 1; ControlFrame(c); assert(Car_Control_GetState() == CAR_BRAKE_LOCK);
    c.brake = 0; ControlFrame(c); Step(40U);
    ControlFrame(c); assert(Car_Control_GetState() == CAR_READY);
    speed = Count(0xF6);
    c.disable = 1; c.forward = 1; ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_OFF && Count(0xF6) == speed);
    puts("PASS independent buttons: four vectors / release / conflicts / joystick / STOP lock / internal BRAKE/DISABLE");
}

static void FinishRemoteTurn(BluetoothControlFrame c, float yaw)
{
    unsigned i;
    while (yaw > 180.0f) yaw -= 360.0f;
    while (yaw < -180.0f) yaw += 360.0f;
    ImuSample(yaw, 0); Loop(); Step(30U);
    for (i = 0U; i < 15U; ++i) {
        Step(10U); ImuSample(yaw, 0); ControlFrame(c);
    }
    assert(TurnRight_GetStatus()->state == TURN_RIGHT_DONE);
    assert(mock_reset_count == 0U);
}

static void TestIndependentTurns(void)
{
    BluetoothControlFrame c = {0};
    unsigned before;
    Reset(0U); Ready(); ImuSample(15,0); Loop();
    c.right_90 = 1; ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_TURNING);
    AssertLatestPhysicalSpeed(1U, TURN_RIGHT_90_RPM);
    before = Count(0xF6); ControlFrame(c); Step(30U); assert(Count(0xF6) == before);
    FinishRemoteTurn(c, -75);
    assert(RemoteHeading_GetStatus()->direction == REMOTE_RIGHT);
    assert(Car_Control_GetState() == CAR_READY);
    before = Count(0xF6); ControlFrame(c); Step(30U); assert(Count(0xF6) == before);
    c.right_90 = 0; ControlFrame(c);
    c.right_180 = 1; ControlFrame(c); Step(40U);
    AssertLatestPhysicalSpeed(1U, TURN_RIGHT_180_RPM);
    ImuSample(-165, -20); Loop(); Step(10U);
    FinishRemoteTurn(c, 105);
    assert(RemoteHeading_GetStatus()->direction == REMOTE_LEFT);
    c.right_180 = 0; ControlFrame(c);
    c.right_90 = 1; ControlFrame(c); Step(30U);
    c.right_180 = 1; ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_READY);
    assert(RemoteHeading_GetStatus()->direction == REMOTE_LEFT);
    c.right_90 = c.right_180 = 0; ControlFrame(c);
    c.right_90 = 1; ImuSample(105,0); ControlFrame(c); Step(30U);
    assert(Car_Control_GetState() == CAR_TURNING);
    c.stop = 1; ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_BRAKE_LOCK && mock_reset_count == 0U);
    puts("PASS remote turns: edge/hold/rearm, 90/180 composition, persistent coordinates, conflicts and STOP");
}

static void TestTranslationAfterTurn(void)
{
    BluetoothControlFrame c = {0};
    Reset(0U); Ready(); ImuSample(0,0); Loop();
    c.right_90 = 1; ControlFrame(c); Step(30U);
    FinishRemoteTurn(c, -90);
    c.right_90 = 0; c.forward = 1; ControlFrame(c); Step(40U);
    AssertLatestPhysicalSpeed(1U, -BLUETOOTH_TEST_MOVE_RPM);
    AssertLatestPhysicalSpeed(2U, BLUETOOTH_TEST_MOVE_RPM);
    ImuSample(-87,0); Loop(); Step(40U);
    assert(RemoteHeading_GetStatus()->phase == REMOTE_DRIFTING);
    Step(30U);
    AssertLatestPhysicalSpeed(1U, (int16_t)(-BLUETOOTH_TEST_MOVE_RPM +
        RemoteHeading_GetStatus()->correction_rpm));
    AssertLatestPhysicalSpeed(2U, (int16_t)(BLUETOOTH_TEST_MOVE_RPM +
        RemoteHeading_GetStatus()->correction_rpm));
    c.forward = 0; ControlFrame(c); Step(20U);
    assert(RemoteHeading_GetStatus()->phase == REMOTE_ALIGNING);
    ImuSample(-90,0); Loop(); Step(40U);
    assert(Car_Control_GetState() == CAR_READY && RemoteHeading_GetStatus()->direction == REMOTE_RIGHT);
    c.backward = 1; ControlFrame(c); Step(40U);
    AssertLatestPhysicalSpeed(1U, BLUETOOTH_TEST_MOVE_RPM);
    AssertLatestPhysicalSpeed(2U, -BLUETOOTH_TEST_MOVE_RPM);
    c.backward = 0; ControlFrame(c); Step(30U);
    ImuSample(-93,0); Loop(); Step(40U);
    AssertLatestPhysicalSpeed(1U,-1); AssertLatestPhysicalSpeed(2U,-1);
    assert(RemoteHeading_GetStatus()->direction == REMOTE_RIGHT && mock_reset_count == 0U);
    puts("PASS post-turn hold: common reference, correction mixed while driving, stationary alignment after release");
}

static void TestTurnWrongWayLocks(void)
{
    BluetoothControlFrame c = {0};
    unsigned speed;
    Reset(0U); Ready(); ImuSample(0.0f, 0.0f); Loop();
    c.right_90 = 1; ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_TURNING);
    speed = Count(0xF6);
    ImuSample(11.0f, 20.0f); Loop(); Step(40U);
    assert(TurnRight_GetStatus()->state == TURN_RIGHT_FAULT &&
           TurnRight_GetStatus()->error == TURN_RIGHT_WRONG_WAY);
    assert(Car_Control_GetState() == CAR_BRAKE_LOCK && Count(0xFE) >= 4U);
    assert(Count(0xF6) == speed);
    puts("PASS turn safety: wrong-way IMU feedback queues four stops and locks Bluetooth motion");
}

static void TestTurnSettledCorrection(void)
{
    BluetoothControlFrame c = {0};
    unsigned i, stops;
    Reset(0U); Ready(); ImuSample(0.0f, 0.0f); Loop();
    c.right_90 = 1; ControlFrame(c); Step(40U);
    ImuSample(-86.0f, -20.0f); Loop(); Step(40U);
    assert(TurnRight_GetStatus()->state == TURN_RIGHT_STOPPING);
    for (i = 0U; i < 14U; ++i) {
        Step(10U); ImuSample(-120.0f, 0.0f); Loop();
    }
    assert(Car_Control_GetState() == CAR_TURNING &&
           TurnRight_GetStatus()->state == TURN_RIGHT_CORRECTING);
    Step(40U);
    AssertLatestPhysicalSpeed(1U, -TURN_RIGHT_90_RPM);
    AssertLatestPhysicalSpeed(2U, -TURN_RIGHT_90_RPM);
    AssertLatestPhysicalSpeed(3U, -TURN_RIGHT_90_RPM);
    AssertLatestPhysicalSpeed(4U, -TURN_RIGHT_90_RPM);
    stops = Count(0xFE);
    ImuSample(-94.0f, 20.0f); Loop(); Step(40U);
    assert(TurnRight_GetStatus()->state == TURN_RIGHT_STOPPING &&
           Count(0xFE) >= stops + 4U);
    for (i = 0U; i < 14U; ++i) {
        Step(10U); ImuSample(-94.0f, 0.0f); Loop();
    }
    assert(TurnRight_GetStatus()->state == TURN_RIGHT_DONE);
    Step(10U); ImuSample(-90.0f, 0.0f); Loop(); Step(40U);
    assert(Car_Control_GetState() == CAR_READY);
    puts("PASS turn correction: Bluetooth stays turning through reverse 20 RPM and completes only after settled tolerance");
}

static void TestButtonRequiresReady(void)
{
    BluetoothControlFrame c = {0};
    Reset(0U); Step(40U);
    c.forward = 1; ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_WAIT_CENTER && Count(0xF6) == 0U);
    c.forward = 0; ControlFrame(c);
    assert(Car_Control_GetState() == CAR_READY);
    c.forward = 1; ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_RUNNING && Count(0xF6) > 0U);
    puts("PASS independent startup gate: centered neutral frame required");
}
static void TestMotor(void)
{
    const uint8_t expected[][8]={{1,0xF6,1,1,244,10,1,0x6B},{2,0xF6,1,1,244,10,1,0x6B},
        {3,0xF6,0,0,0,10,1,0x6B},{4,0xF6,1,0,100,10,1,0x6B}};
    const uint8_t reply[]={0,0xFF,1,0xF3,2,0x6B,2,0xF6,0xE2,0x6B,3,0xFE,0xEE,0x6B};
    unsigned i;
    Reset(0U);
    assert(Motor_Enable(0U)==HAL_ERROR && Motor_SetSpeed(5U,1)==HAL_ERROR);
    assert(Motor_GetLastStatus(5U)==NULL && Motor_EStop(0U)==HAL_ERROR);
    assert(Motor_SetSpeedSync4(INT16_MAX,INT16_MIN,0,100)==HAL_OK);
    assert(Motor_SetSpeed(1U,1)==HAL_BUSY); Step(30U);
    assert(sent_count==5U);
    for(i=0;i<4U;++i) assert(sent[i].length==8U && memcmp(sent[i].bytes,expected[i],8U)==0);
    assert(memcmp(sent[4].bytes,(uint8_t[]){0,0xFF,0x66,0x6B},4U)==0);
    Motor_ProcessRx(reply,5U);Motor_ProcessRx(reply+5U,(uint16_t)(sizeof(reply)-5U));
    assert(Motor_GetLastStatus(1)->command==0xF3 && Motor_GetLastStatus(1)->response_status==2);
    assert(Motor_GetLastStatus(2)->response_status==0xE2 && Motor_GetLastStatus(3)->response_status==0xEE);
    assert(Motor_HasFault()); Motor_ClearFault();
    sent_count=0U; assert(Motor_SetSpeedSync4(1,2,3,4)==HAL_OK);Motor_Process();
    assert(sent_count==1U);(void)Motor_EStopAll();(void)Motor_DisableAll();Step(40U);
    assert(sent_count==9U && Count(0xFF)==0U);
    for(i=1;i<=4;++i) assert(memcmp(sent[i].bytes,(uint8_t[]){(uint8_t)i,0xFE,0x98,0,0x6B},5U)==0);
    for(i=5;i<=8;++i) assert(memcmp(sent[i].bytes,(uint8_t[]){(uint8_t)(i-4),0xF3,0xAB,0,0,0x6B},6U)==0);
    puts("PASS motor: wire frames/signed saturation/addresses/replies/sync/safety queue preemption");
}
static void TestControl(void)
{
    BluetoothControlFrame c = {0};
    uint8_t burst[2U * BT_CONTROL_FRAME_SIZE];
    unsigned before;
    Reset(0U); Ready();
    ButtonFrame(0,0,1); Step(35U);
    assert(Car_Control_GetState() == CAR_RUNNING);
    AssertLatestPhysicalSpeed(1U, -BLUETOOTH_TEST_MOVE_RPM);
    AssertLatestPhysicalSpeed(2U, BLUETOOTH_TEST_MOVE_RPM);
    AssertLatestSyncTransaction();
    c.forward = 1; c.stop = 1; PackControl(burst, &c);
    c.stop = 0; PackControl(burst + BT_CONTROL_FRAME_SIZE, &c);
    before = Count(0xFE);
    Inject(&huart6, burst, sizeof(burst)); Loop(); Step(40U);
    assert(Car_Control_GetState() == CAR_BRAKE_LOCK && Count(0xFE) == before + 4U);
    before = Count(0xF6); Step(40U); assert(Count(0xF6) == before);
    ButtonFrame(0,0,0); assert(Car_Control_GetState() == CAR_READY);
    ButtonFrame(0,0,1); Step(30U); assert(Car_Control_GetState() == CAR_RUNNING);
    ButtonFrame(1,1,1); Step(40U); assert(Car_Control_GetState() == CAR_OFF);
    before = sent_count; ButtonFrame(0,1,1); Step(40U); assert(sent_count == before);
    puts("PASS control: button translation / sync / STOP burst / center rearm / disable priority");
}
static void TestFailsafe(void)
{
    unsigned before;
    Reset(UINT32_MAX-100U);Ready();ButtonFrame(0,0,1);Step(40);
    Step(460); assert(Bluetooth_IsConnected()); /* Exactly 500 ms remains connected. */
    Step(1);assert(Car_Control_GetState()==CAR_LINK_LOST);Step(30);
    before=Count(0xF6);ButtonFrame(1,0,0);assert(Car_Control_GetState()==CAR_LINK_LOST);
    ButtonFrame(0,0,1);Step(30);assert(Car_Control_GetState()==CAR_LINK_LOST && Count(0xF6)==before);
    ButtonFrame(0,0,0);Step(40);assert(Car_Control_GetState()==CAR_LINK_LOST);
    ButtonFrame(1,0,0);ButtonFrame(0,0,0);assert(Car_Control_GetState()==CAR_LINK_LOST);
    ButtonFrame(0,0,1);ButtonFrame(0,0,0);assert(Car_Control_GetState()==CAR_LINK_LOST);
    ButtonFrame(0,0,0);assert(Car_Control_GetState()==CAR_WAIT_CENTER);
    ButtonFrame(0,0,1);Step(40);
    assert(Car_Control_GetState()==CAR_WAIT_CENTER && Count(0xF6)==before);
    ButtonFrame(0,0,0);assert(Car_Control_GetState()==CAR_READY);
    /* RX transport corruption while running forces link loss immediately. */
    ButtonFrame(0,0,1);Step(30);HAL_UART_ErrorCallback(&huart6);Loop();assert(Car_Control_GetState()==CAR_LINK_LOST);
    Reset(0U);Ready();ButtonFrame(0,0,1);stall_motor=1U;Step(50);
    assert(Motor_HasFault() && Car_Control_GetState()==CAR_FAULT);
    stall_motor=0U;Step(40);ButtonFrame(0,0,0);Step(40);
    assert(Car_Control_GetState()==CAR_FAULT && Motor_HasFault());
    ButtonFrame(0,0,0);
    assert(Car_Control_GetState()==CAR_WAIT_CENTER);
    Step(40);ButtonFrame(0,0,0);assert(Car_Control_GetState()==CAR_READY && !Motor_HasFault());
    Reset(0U);Ready();next_tx_result=HAL_ERROR;ButtonFrame(0,0,1);Step(30);
    assert(Car_Control_GetState()==CAR_FAULT);
    puts("PASS failsafe: 500 ms boundary/tick wrap/reconnect rearm/brake cannot bypass/UART corruption/TX stall/error");
}
static void TestRepeatedManualSegmentsAndStop(void)
{
    BluetoothControlFrame c = {0};
    unsigned i, before;
    Reset(0U); Ready(); ImuSample(20,0); Loop();
    for (i = 0U; i < 4U; ++i) {
        c.forward = 1; ImuSample(20,0); ControlFrame(c); Step(30U);
        assert(Car_Control_GetState() == CAR_RUNNING);
        c.forward = 0; ControlFrame(c); Step(40U);
        assert(Car_Control_GetState() == CAR_READY);
        assert(fabsf(RemoteHeading_GetStatus()->reference_yaw - 20) < 0.01f);
    }
    ImuSample(24,0); Loop(); Step(40U);
    assert(RemoteHeading_GetStatus()->phase == REMOTE_ALIGNING);
    c.stop = 1; ControlFrame(c); Step(40U);
    before = Count(0xF6); ImuSample(24,0); Loop(); Step(40U);
    assert(Car_Control_GetState() == CAR_BRAKE_LOCK && Count(0xF6) == before);
    assert(RemoteHeading_GetStatus()->reference_valid);
    c.stop = 0; ControlFrame(c); Step(40U);
    assert(RemoteHeading_GetStatus()->phase == REMOTE_ALIGNING && Count(0xF6) > before);
    ImuSample(20,0); Loop(); Step(40U);
    assert(Car_Control_GetState() == CAR_READY);
    puts("PASS remote reference: button releases and STOP preserve origin; unlocked idle resumes alignment");
}

static void TestNeutralInputs(void)
{
    CarRemoteInput_t input = {0};
    unsigned before;
    Reset(0U);
    input.valid = 1U; input.sequence = 1U; input.received_tick = now;
    Car_Control_SubmitRemoteInput(&input); Loop();
    assert(Car_Control_GetState() == CAR_READY && Bluetooth_GetSequence() == 0U);
    input.command.forward = 1; ++input.sequence; input.received_tick = now;
    Car_Control_SubmitRemoteInput(&input); Loop(); Step(30U);
    assert(Car_Control_GetState() == CAR_RUNNING);
    input.command.stop = 1; ++input.sequence; input.received_tick = now;
    before = Count(0xF6);
    Car_Control_SubmitRemoteInput(&input); Loop(); Step(40U);
    assert(Car_Control_GetState() == CAR_BRAKE_LOCK && Count(0xF6) == before);
    input.command.forward = input.command.stop = 0; ++input.sequence; input.received_tick = now;
    Car_Control_SubmitRemoteInput(&input); Loop();
    assert(Car_Control_GetState() == CAR_READY);
    input.command.forward = 1; ++input.sequence; input.received_tick = now;
    Car_Control_SubmitRemoteInput(&input); Loop(); Step(30U);
    before = Count(0xFE);
    Car_Control_InvalidateRemoteInput();
    assert(Car_Control_GetState() == CAR_RUNNING && Count(0xFE) == before);
    Loop(); Step(40U);
    assert(Car_Control_GetState() == CAR_LINK_LOST && Count(0xFE) >= before + 4U);
    Reset(0U); input.command.forward = 0; input.sequence = 1U; input.received_tick = now;
    Car_Control_SubmitRemoteInput(&input); Loop();
    input.command.forward = 1; ++input.sequence; input.received_tick = now;
    Car_Control_SubmitRemoteInput(&input); Loop(); Step(500U);
    assert(Car_Control_GetState() == CAR_RUNNING);
    Step(1U); assert(Car_Control_GetState() == CAR_LINK_LOST);
    assert(Bluetooth_GetSequence() == 0U);
    puts("PASS neutral car inputs: no Bluetooth frames/getters, startup, STOP, ISR-only invalidation and exact 500ms lease");
}
static void TestBootEnable(void)
{
    unsigned before;
    Reset(0U);Step(1000U);
    assert(Car_Control_GetState()==CAR_WAIT_CENTER && sent_count==0U);
    ButtonFrame(0,0,1);Step(40U);
    assert(Car_Control_GetState()==CAR_WAIT_CENTER && Count(0xF6)==0U);
    ButtonFrame(0,0,0);assert(Car_Control_GetState()==CAR_READY);
    ButtonFrame(0,0,1);Step(40U);assert(Car_Control_GetState()==CAR_RUNNING);
    ButtonFrame(0,1,1);Step(40U);assert(Car_Control_GetState()==CAR_OFF);
    before=sent_count;ButtonFrame(0,0,0);Step(600U);
    assert(Car_Control_GetState()==CAR_OFF && sent_count==before);
    /* Legacy ENABLE toggles cannot bypass the center guard. */
    ButtonFrame(0,0,1);Step(40U);
    assert(Car_Control_GetState()==CAR_OFF && sent_count==before);
    ButtonFrame(0,0,0);Step(40U);
    assert(Car_Control_GetState()==CAR_OFF && sent_count==before);
    ButtonFrame(1,0,0);ButtonFrame(0,0,0);
    assert(Car_Control_GetState()==CAR_OFF && sent_count==before);
    ButtonFrame(0,1,0);Step(40U);
    before=sent_count;
    ButtonFrame(0,1,0);Step(40U);
    assert(Car_Control_GetState()==CAR_OFF && sent_count==before);
    ButtonFrame(0,0,0);assert(Car_Control_GetState()==CAR_OFF);
    ButtonFrame(0,0,0);assert(Car_Control_GetState()==CAR_WAIT_CENTER);
    ButtonFrame(0,0,0);assert(Car_Control_GetState()==CAR_WAIT_CENTER);
    Step(40U);assert(Car_Control_GetState()==CAR_WAIT_CENTER && sent_count==before+4U);
    ButtonFrame(0,0,0);assert(Car_Control_GetState()==CAR_READY);
    ButtonFrame(0,0,1);Step(40U);assert(Car_Control_GetState()==CAR_RUNNING);
    Reset(0U);HAL_UART_ErrorCallback(&huart5);Loop();
    assert(Car_Control_GetState()==CAR_FAULT);
    Step(40U);assert(Count(0xFE)==4U && Count(0xF6)==0U);
    puts("PASS boot enable: one-shot F3/no Bluetooth motion/no ENABLE/held DISABLE/centered packet recovery/brake resets recovery");
}
static void TestBootStopFrame(void)
{
    /* A STOP before enable can be queued must retain the boot request. The
     * first neutral packet queues enable; the second, after TX idle, is READY. */
    BluetoothControlFrame stop_frame = {0};
    stop_frame.stop = 1;
    Reset(0U); huart5.RxState = HAL_UART_STATE_READY; Motor_Init(); Car_Control_Init(); sent_count = 0U;
    ControlFrame(stop_frame); Step(40U);
    assert(Car_Control_GetState()==CAR_WAIT_CENTER && Count(0xF6)==0U);
    unsigned enabled = 0U;
    for (unsigned i=0U;i<sent_count;++i)
        if (sent[i].bytes[1]==0xF3U && sent[i].bytes[3]==1U) ++enabled;
    assert(enabled==0U);
    ButtonFrame(0,0,0); Step(40U);
    ButtonFrame(0,0,0);
    assert(Car_Control_GetState()==CAR_READY);
    enabled = 0U;
    for (unsigned i=0U;i<sent_count;++i)
        if (sent[i].bytes[1]==0xF3U && sent[i].bytes[3]==1U) ++enabled;
    assert(enabled==4U && Count(0xF6)==0U);
    /* After enabling, even the first STOP before READY immediately locks. */
    Reset(0U);Step(1000U);
    assert(Car_Control_GetState()==CAR_WAIT_CENTER);
    ControlFrame(stop_frame);Step(40U);
    assert(Car_Control_GetState()==CAR_BRAKE_LOCK && Count(0xF6)==0U);
    ButtonFrame(0,0,0);Step(40U);
    assert(Car_Control_GetState()==CAR_READY);
    ButtonFrame(0,0,1);Step(40U);
    assert(Car_Control_GetState()==CAR_RUNNING);
    /* Once the car has driven, STOP latches the brake lock as before. */
    ControlFrame(stop_frame);Step(40U);
    assert(Car_Control_GetState()==CAR_BRAKE_LOCK);
    ButtonFrame(0,0,0);Step(40U);
    assert(Car_Control_GetState()==CAR_READY);
    puts("PASS boot stop frame: powered STOP immediately locks, neutral unlocks within two frames");
}
static void TestPD10(void)
{
    unsigned i, before;
    Reset(0U);
    Step(600U);assert(Car_Control_GetState()==CAR_OFF && sent_count==0U);
    /* Short pulses do not start; long holds run without ANY Bluetooth packet. */
    pd10_low=1U;Step(10U);pd10_low=0U;Step(30U);assert(sent_count==0U);
    pd10_low=1U;Step(70U);assert(Car_Control_GetState()==CAR_RUNNING);
    assert(Count(0xF3)==4U && Count(0xFF)>0U);
    for(i=0U;i<sent_count;++i) {
        if(sent[i].bytes[1]==0xF6U) {
            assert(sent[i].bytes[2]==((sent[i].bytes[0]==1U || sent[i].bytes[0]==4U) ? 1U : 0U));
            assert(sent[i].bytes[3]==0U && sent[i].bytes[4]==100U && sent[i].bytes[6]==1U);
        }
    }
    Step(600U);assert(Car_Control_GetState()==CAR_RUNNING && Count(0xF3)==4U);
    before=sent_count;pd10_low=0U;Loop();assert(Car_Control_GetState()==CAR_OFF);Step(40U);
    assert(sent_count==before+8U);
    for(i=0U;i<4U;++i) assert(sent[before+i].bytes[1]==0xFEU);
    for(i=4U;i<8U;++i) assert(sent[before+i].bytes[1]==0xF3U && sent[before+i].bytes[3]==0U);
    /* Bluetooth commands cannot start movement in standalone mode. */
    before=sent_count;Frame(0,0,0,1000);Step(40U);assert(sent_count==before);
    /* Release during enable cancels all remaining enables and every speed. */
    Reset(0U);pd10_low=1U;Step(22U);assert(Car_Control_GetState()==CAR_LOCAL_STARTING);
    pd10_low=0U;Loop();Step(40U);assert(Car_Control_GetState()==CAR_OFF && Count(0xF6)==0U);
    /* Fault while held stays locked, and only a new press after release runs. */
    Reset(UINT32_MAX-30U);pd10_low=1U;Step(70U);
    HAL_UART_ErrorCallback(&huart5);Loop();Step(40U);assert(Car_Control_GetState()==CAR_FAULT);
    before=Count(0xF6);Step(100U);assert(Count(0xF6)==before);
    pd10_low=0U;Step(40U);assert(Car_Control_GetState()==CAR_OFF);
    pd10_low=1U;Step(70U);assert(Car_Control_GetState()==CAR_RUNNING);
    puts("PASS PD10: no Bluetooth/debounce/100 RPM/M1+M4 reversal/sync/hold/release/enable cancellation/fault rearm/tick wrap");
}
#include "mecanum_cases.inc"
#include "heading_cases.inc"
#include "pid_tuner_cases.inc"
#include "vision_follow_cases.inc"
#include "shot_cases.inc"
#include "left_turn_cases.inc"
#include "servo_cases.inc"
#include "remote_heading_cases.inc"
int main(void)
{
    setvbuf(stdout,NULL,_IONBF,0);
    TestParser();
    if (!CAR_PD10_STANDALONE_TEST && !CAR_MECANUM_TEST_MODE && !CAR_HEADING_TEST_MODE)
        { TestActualPhoneProfile(); TestPhone20Car(); TestPhone20Joystick(); TestRemoteMotionGate(); TestRemoteHeadingPreemption(); TestMovingDriftConfirmation(); }
    if (!CAR_PD10_STANDALONE_TEST && !CAR_MECANUM_TEST_MODE && !CAR_HEADING_TEST_MODE)
        { TestIndependentButtons(); TestIndependentTurns(); TestLeftPacket(); TestLeftRemoteTurn(); TestTranslationAfterTurn(); TestTurnWrongWayLocks();
          TestTurnSettledCorrection(); TestButtonRequiresReady(); }
    TestMotor();TestMecanumMath();TestDirectionWrappers();TestTunerCommands();TestTunerFraming();TestTunerTransport();
    if (CAR_HEADING_TEST_MODE) { TestHeadingIntegration();TestRemoteReferenceLifetime(); }
    else if (CAR_MECANUM_TEST_MODE) TestMecanumSafety();
    else if (CAR_PD10_STANDALONE_TEST) TestPD10();
    else { TestControl();TestFailsafe();TestRepeatedManualSegmentsAndStop();TestNeutralInputs();TestBootEnable();TestBootStopFrame();TestRemoteReferenceLifetime();TestHeadingIntegration();TestManualReleaseBrakesHeading();TestTunerSafety(); TestVisionPacket();TestVisionLocalFollow();TestVisionReversalAndFault();TestVisionJitterAndRecentering();TestVisionBluetooth();TestVisionAlignmentBounds();TestShotPacket();TestShotBluetoothSwitch();TestShotEntryAndModes();TestShotAimAndFire();TestShotJitterAndFreshFrames();TestShotSafety();TestShotAlignmentBounds();TestServoPacket();TestReservedFieldNoEffect();TestServoBoolPacket();TestBoolShotSwitch();TestBoolShotAimAndRelease();TestAimInterface();TestServoGapPacket();TestServoCarIndependence(); }
    puts("ALL CAR TESTS PASSED (host HAL simulation, not hardware)");return 0;
}
