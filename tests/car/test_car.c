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
#include "servo_remote.h"
#include "servo_pose.h"
#include "arm_tuner.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include <math.h>
#include <float.h>
UART_HandleTypeDef huart1, huart5, huart6;
I2C_HandleTypeDef hi2c1;
static uint32_t now;
static uint8_t stall_motor;
static uint8_t pd10_low;
static uint8_t pe0_low, pe4_low;
static uint8_t pc1_low, test_laser_on;
static unsigned test_laser_enable_count;
static unsigned test_servo_pose_calls;
static unsigned test_original_pose_calls, test_servo_reset_calls;
static ArmStep_t test_servo_last_step;
static ArmResult_t test_servo_result;
static ArmStatus_t test_arm_status;
static uint8_t test_arm_hold, test_tuner_session;
ArmResult_t Arm_SetMotionAllowed(bool allowed)
{
    test_arm_status.motion_allowed = allowed;
    if (!allowed && test_arm_status.state == ARM_RUNNING)
        test_arm_status.state = ARM_CANCELLED;
    return ARM_OK;
}
ArmResult_t Arm_StartSequence(const ArmStep_t *step, size_t count)
{
    assert(step != NULL && count == 1U && test_arm_status.motion_allowed);
    if (test_servo_result != ARM_OK) return test_servo_result;
    test_servo_last_step = *step;
    ++test_servo_pose_calls;
    test_arm_status.state = test_arm_hold ? ARM_RUNNING : ARM_COMPLETE_ESTIMATED;
    return ARM_OK;
}
ArmResult_t Arm_StartOriginalPreset(const ArmStep_t *step)
{
    ArmResult_t result = Arm_StartSequence(step, 1U);
    if (result == ARM_OK) ++test_original_pose_calls;
    return result;
}
ArmResult_t Arm_ResetController(void)
{
    if (test_arm_status.motion_allowed || test_arm_status.state == ARM_RUNNING)
        return ARM_BUSY;
    if (test_servo_result != ARM_OK) return test_servo_result;
    ++test_servo_reset_calls;
    test_arm_status.state = ARM_IDLE;
    return ARM_OK;
}
ArmResult_t Arm_SendImmediate(const ArmStep_t *step)
{
    assert(step != NULL);
    if (test_servo_result != ARM_OK) return test_servo_result;
    test_servo_last_step = *step;
    ++test_servo_pose_calls;
    test_arm_status.motion_allowed = true;
    test_arm_status.state = test_arm_hold ? ARM_RUNNING : ARM_COMPLETE_ESTIMATED;
    return ARM_OK;
}
ArmResult_t Arm_Stop(void)
{
    test_arm_status.motion_allowed = false;
    test_arm_status.state = ARM_CANCELLED;
    return ARM_OK;
}
ArmStatus_t Arm_GetStatus(void) { return test_arm_status; }
uint8_t ArmTuner_IsSessionActive(void) { return test_tuner_session; }
uint8_t Board_PD10IsLow(void) { return pd10_low; }
uint8_t Board_VisionButtonIsLow(void) { return pe0_low; }
uint8_t Board_ShotButtonIsLow(void) { return pc1_low; }
uint8_t Board_ServoButtonIsLow(void) { return pe4_low; }
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
    Bluetooth_Process(); Car_Control_Process(); Motor_Process(); Car_Control_Process(); ServoRemote_Process(); PID_Tuner_Process(); Debug_Process();
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
    ((BtControl_t *)Bluetooth_GetControl())->frame.brake = c.brake;
    ((BtControl_t *)Bluetooth_GetControl())->frame.disable = c.disable;
    Loop();
}
static void Reset(uint32_t tick)
{
    now = tick; stall_motor = 0U; next_tx_result = HAL_OK;
    stall_bt=0U; next_bt_tx_result=HAL_OK; bt_sent_count=0U;
    bt_inflight=NULL; bt_inflight_length=0U; bt_start_tick=now; bt_wire_ms=0U;
    pd10_low = pe0_low = pe4_low = pc1_low = test_laser_on = 0U;
    test_laser_enable_count = 0U; bt_frame_ok_count = 0U;
    test_servo_pose_calls = 0U; test_servo_result = ARM_OK;
    test_original_pose_calls = test_servo_reset_calls = 0U;
    test_arm_hold = test_tuner_session = 0U;
    memset(&test_servo_last_step, 0, sizeof(test_servo_last_step));
    memset(&test_arm_status, 0, sizeof(test_arm_status));
    test_arm_status.state = ARM_IDLE;
    bt_forward_frame_count = bt_control_forward_count = 0U;
    memset(&huart1,0,sizeof(huart1)); memset(&huart5,0,sizeof(huart5)); memset(&huart6,0,sizeof(huart6));
    huart1.RxState = huart5.RxState = huart6.RxState = HAL_UART_STATE_READY;
    Debug_TxCallback(&huart1);
    JY61_Init(); Bluetooth_Init(); Motor_Init(); TestMaxiCam_Reset(); Car_Control_Init();
    ServoRemote_Init();
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
    Frame(0,0,0,1000); assert(Car_Control_GetState()==CAR_WAIT_CENTER);
    Step(40U); Frame(0,0,0,1000); assert(Car_Control_GetState()==CAR_WAIT_CENTER);
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
    c.backward = 0; ControlFrame(c); Step(40U);
    c.strafe_left = 1; ControlFrame(c); Step(40U);
    AssertLatestPhysicalSpeed(1U, BLUETOOTH_TEST_MOVE_RPM);
    AssertLatestPhysicalSpeed(2U, BLUETOOTH_TEST_MOVE_RPM);
    c.strafe_left = 0; ControlFrame(c); Step(40U);
    c.strafe_right = 1; ControlFrame(c); Step(40U);
    AssertLatestPhysicalSpeed(1U, -BLUETOOTH_TEST_MOVE_RPM);
    AssertLatestPhysicalSpeed(2U, -BLUETOOTH_TEST_MOVE_RPM);
    c.strafe_right = 0; ControlFrame(c); Step(40U);
    c.forward = c.backward = 1; speed = Count(0xF6); ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_READY && Count(0xF6) == speed);
    c.forward = c.backward = 0; c.strafe_left = c.strafe_right = 1;
    ControlFrame(c); Step(40U); assert(Count(0xF6) == speed);
    c.strafe_left = c.strafe_right = 0; c.joy_x = 600;
    ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_RUNNING);
    AssertLatestPhysicalSpeed(1U, -300); AssertLatestPhysicalSpeed(2U, -300);
    c.joy_x = 0; c.joy_y = 600; ControlFrame(c); Step(40U);
    AssertLatestPhysicalSpeed(1U, -300); AssertLatestPhysicalSpeed(2U, 300);
    c.stop = 1; ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_BRAKE_LOCK);
    speed = Count(0xF6);
    c.stop = 0; ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_BRAKE_LOCK && Count(0xF6) == speed);
    c.joy_y = 0; ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_READY);
    c.joy_y = 600; ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_RUNNING);
    c.joy_y = 0; stopped = Count(0xFE); ControlFrame(c); Step(40U);
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

static void TestIndependentTurns(void)
{
    BluetoothControlFrame c = {0};
    unsigned speed, i;
    Reset(0U); Ready(); ImuSample(15.0f, 0.0f); Loop();
    c.right_90 = 1; ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_TURNING);
    AssertLatestPhysicalSpeed(1U, TURN_RIGHT_90_RPM);
    AssertLatestPhysicalSpeed(2U, TURN_RIGHT_90_RPM);
    AssertLatestPhysicalSpeed(3U, TURN_RIGHT_90_RPM);
    AssertLatestPhysicalSpeed(4U, TURN_RIGHT_90_RPM);
    speed = Count(0xF6);
    ImuSample(15.0f, 0.0f); ControlFrame(c); Step(40U); assert(Count(0xF6) == speed);
    c.right_90 = 0; ImuSample(15.0f, 0.0f); ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_TURNING);
    c.right_180 = 1; ImuSample(15.0f, 0.0f); ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_TURNING && Count(0xF6) == speed);
    c.right_90 = 1; ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_READY);
    c.right_90 = c.right_180 = 0; ControlFrame(c); Step(40U);
    ImuSample(25.0f, 0.0f); Loop();
    c.right_180 = 1; ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_TURNING);
    AssertLatestPhysicalSpeed(1U, TURN_RIGHT_180_RPM);
    AssertLatestPhysicalSpeed(2U, TURN_RIGHT_180_RPM);
    AssertLatestPhysicalSpeed(3U, TURN_RIGHT_180_RPM);
    AssertLatestPhysicalSpeed(4U, TURN_RIGHT_180_RPM);
    speed = Count(0xF6);
    ControlFrame(c); Step(40U); assert(Count(0xF6) == speed);
    c.stop = 1; ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_BRAKE_LOCK);
    c.stop = c.right_180 = 0; ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_READY);
    ImuSample(30.0f, 0.0f); ControlFrame(c);
    c.right_180 = 1; ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_TURNING && Count(0xF6) > speed);
    c.stop = 1; ControlFrame(c); Step(40U);
    c.stop = c.right_180 = 0; ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_READY);
    speed = Count(0xF6);
    ImuSample(35.0f, 0.0f); ControlFrame(c);
    c.right_90 = 1; ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_TURNING && Count(0xF6) > speed);
    speed = Count(0xF6);
    {
        unsigned stopped = Count(0xFE);
        ImuSample(-56.0f, -20.0f); Loop(); Step(40U);
        assert(TurnRight_GetStatus()->state == TURN_RIGHT_STOPPING);
        assert(Count(0xFE) >= stopped + 4U && Count(0xF6) == speed);
    }
    for (i = 0U; i < 14U; ++i) {
        Step(10U); ImuSample(-56.0f, 0.0f); Loop();
    }
    assert(TurnRight_GetStatus()->state == TURN_RIGHT_RESETTING);
    Step(10U); ImuSample(0.0f, 0.0f); Loop();
    assert(Car_Control_GetState() == CAR_READY);
    ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_READY && Count(0xF6) == speed);
    puts("PASS independent turns: 90/180 edge / held no retrigger / 0-1 rearm / completion / conflict / STOP cancel");
}

static void TestTranslationAfterTurn(void)
{
    BluetoothControlFrame c = {0};
    unsigned i;
    const int16_t move_rpm = BLUETOOTH_TEST_MOVE_RPM;
    const int16_t correction = move_rpm * MANUAL_HEADING_LIMIT_PERCENT / 100;
    Reset(0U); Ready(); ImuSample(0.0f, 0.0f); Loop();
    c.right_90 = 1; ControlFrame(c); Step(40U);
    c.right_90 = 0; ControlFrame(c); Step(40U);
    ImuSample(-86.0f, -20.0f); Loop(); Step(40U);
    assert(TurnRight_GetStatus()->state == TURN_RIGHT_STOPPING);
    for (i = 0U; i < 14U; ++i) {
        Step(10U); ImuSample(-90.0f, 0.0f); Loop();
    }
    assert(TurnRight_GetStatus()->state == TURN_RIGHT_RESETTING);
    Step(10U); ImuSample(0.0f, 0.0f); Loop();
    assert(Car_Control_GetState() == CAR_READY);
    assert(fabsf(Heading_GetStatus()->yaw_zero) < 2.0f);

    c.forward = 1; ControlFrame(c); Step(40U);
    AssertLatestPhysicalSpeed(1U, -move_rpm);
    AssertLatestPhysicalSpeed(2U, move_rpm);
    AssertLatestPhysicalSpeed(3U, move_rpm);
    AssertLatestPhysicalSpeed(4U, -move_rpm);
    AssertLatestSyncTransaction();

    ImuSample(100.0f, 0.0f); Loop(); Step(40U);
    assert(Heading_GetStatus()->yaw_error < -99.0f &&
           Heading_GetStatus()->omega_final >= 49);
    AssertLatestPhysicalSpeed(1U, -move_rpm + correction);
    AssertLatestPhysicalSpeed(2U, move_rpm + correction);
    AssertLatestPhysicalSpeed(3U, move_rpm + correction);
    AssertLatestPhysicalSpeed(4U, -move_rpm + correction);
    AssertLatestSyncTransaction();

    ImuSample(-100.0f, 0.0f); Loop(); Step(40U);
    assert(Heading_GetStatus()->yaw_error > 99.0f &&
           Heading_GetStatus()->omega_final <= -49);
    AssertLatestPhysicalSpeed(1U, -move_rpm - correction);
    AssertLatestPhysicalSpeed(2U, move_rpm - correction);
    AssertLatestPhysicalSpeed(3U, move_rpm - correction);
    AssertLatestPhysicalSpeed(4U, -move_rpm - correction);
    AssertLatestSyncTransaction();

    ImuSample(100.0f, 0.0f); Loop(); Step(40U);
    c.forward = 0; c.backward = 1; ControlFrame(c); Step(40U);
    AssertLatestPhysicalSpeed(1U, move_rpm + correction);
    AssertLatestPhysicalSpeed(2U, -move_rpm + correction);
    AssertLatestPhysicalSpeed(3U, -move_rpm + correction);
    AssertLatestPhysicalSpeed(4U, move_rpm + correction);
    AssertLatestSyncTransaction();

    ImuSample(100.0f, 0.0f); Loop();
    c.backward = 0; c.strafe_left = 1; ControlFrame(c); Step(40U);
    AssertLatestPhysicalSpeed(1U, move_rpm + correction);
    AssertLatestPhysicalSpeed(2U, move_rpm + correction);
    AssertLatestPhysicalSpeed(3U, -move_rpm + correction);
    AssertLatestPhysicalSpeed(4U, -move_rpm + correction);
    AssertLatestSyncTransaction();

    ImuSample(100.0f, 0.0f); Loop();
    c.strafe_left = 0; c.strafe_right = 1; ControlFrame(c); Step(40U);
    AssertLatestPhysicalSpeed(1U, -move_rpm + correction);
    AssertLatestPhysicalSpeed(2U, -move_rpm + correction);
    AssertLatestPhysicalSpeed(3U, move_rpm + correction);
    AssertLatestPhysicalSpeed(4U, move_rpm + correction);
    AssertLatestSyncTransaction();

    Reset(0U); Ready(); ImuSample(0.0f, 0.0f); Loop();
    memset(&c, 0, sizeof(c));
    c.right_180 = 1; ControlFrame(c); Step(40U);
    c.right_180 = 0; ControlFrame(c); Step(40U);
    ImuSample(-176.0f, -20.0f); Loop(); Step(40U);
    assert(TurnRight_GetStatus()->state == TURN_RIGHT_STOPPING);
    for (i = 0U; i < 14U; ++i) {
        Step(10U); ImuSample(-180.0f, 0.0f); Loop();
    }
    assert(TurnRight_GetStatus()->state == TURN_RIGHT_RESETTING);
    Step(10U); ImuSample(0.0f, 0.0f); Loop();
    assert(Car_Control_GetState() == CAR_READY);
    c.forward = 1; ControlFrame(c); Step(40U);
    AssertLatestPhysicalSpeed(1U, -move_rpm);
    AssertLatestPhysicalSpeed(2U, move_rpm);
    AssertLatestPhysicalSpeed(3U, move_rpm);
    AssertLatestPhysicalSpeed(4U, -move_rpm);
    AssertLatestSyncTransaction();
    puts("PASS translation after 90/180: heading zero / signed and capped correction / four nonzero speeds / sync");
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
    assert(TurnRight_GetStatus()->state == TURN_RIGHT_RESETTING);
    Step(10U); ImuSample(0.0f, 0.0f); Loop();
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
    unsigned before,i;
    uint8_t a[BT_CONTROL_FRAME_SIZE],b[BT_CONTROL_FRAME_SIZE],burst[2U*BT_CONTROL_FRAME_SIZE];
    Reset(0U); Frame(0,0,0,1000);Step(30);assert(sent_count==0U);
    Ready();assert(Count(0xF3)==0U);
    Frame(0,0,0,1000);Step(35);assert(Car_Control_GetState()==CAR_RUNNING);
    /* Straight: logical [500,500,500,500], then installation signs. */
    AssertLatestPhysicalSpeed(1U,-500);AssertLatestPhysicalSpeed(2U,500);
    AssertLatestPhysicalSpeed(3U,500);AssertLatestPhysicalSpeed(4U,-500);
    Frame(0,0,1000,0);Step(35);assert(Car_Control_GetState()==CAR_RUNNING);
    /* X is right translation, never continuous rotation. */
    AssertLatestPhysicalSpeed(1U,-500);AssertLatestPhysicalSpeed(2U,-500);
    AssertLatestPhysicalSpeed(3U,500);AssertLatestPhysicalSpeed(4U,500);
    Frame(0,0,500,1000);Step(35);assert(Car_Control_GetState()==CAR_RUNNING);
    /* Continuous 2-D translation; common scaling preserves direction. */
    AssertLatestPhysicalSpeed(1U,-500);AssertLatestPhysicalSpeed(2U,166);
    AssertLatestPhysicalSpeed(3U,500);AssertLatestPhysicalSpeed(4U,-166);
    Frame(0,0,1000,1000);Step(35);assert(Car_Control_GetState()==CAR_RUNNING);
    /* Right-forward 45 degrees: M1/M3 only for this X layout. */
    AssertLatestPhysicalSpeed(1U,-500);AssertLatestPhysicalSpeed(2U,0);
    AssertLatestPhysicalSpeed(3U,500);AssertLatestPhysicalSpeed(4U,0);
    Frame(0,0,-1000,1000);Step(35);assert(Car_Control_GetState()==CAR_RUNNING);
    AssertLatestPhysicalSpeed(1U,0);AssertLatestPhysicalSpeed(2U,500);
    AssertLatestPhysicalSpeed(3U,0);AssertLatestPhysicalSpeed(4U,-500);
    before=Count(0xFE);Frame(0,0,50,-50);Step(40);assert(Car_Control_GetState()==CAR_READY && Count(0xFE)==before+4U);
    Frame(0,0,0,-1000);Step(30);assert(Car_Control_GetState()==CAR_RUNNING);
    Pack(a,1,0,0,-1000);Pack(b,0,0,0,-1000);memcpy(burst,a,BT_CONTROL_FRAME_SIZE);memcpy(burst+BT_CONTROL_FRAME_SIZE,b,BT_CONTROL_FRAME_SIZE);
    Inject(&huart6,burst,sizeof(burst));Loop();assert(Car_Control_GetState()==CAR_BRAKE_LOCK);Step(40);
    assert(Car_Control_GetState()==CAR_BRAKE_LOCK && Count(0xFE)==before+8U);
    before=Count(0xF6);Step(40);assert(Count(0xF6)==before);
    Frame(0,0,0,0);assert(Car_Control_GetState()==CAR_READY);
    Frame(0,0,0,1000);Step(30);assert(Car_Control_GetState()==CAR_RUNNING);
    Frame(1,1,0,1000);assert(Car_Control_GetState()==CAR_OFF);Step(40);
    before=sent_count;Frame(0,1,0,1000);Step(40);assert(Car_Control_GetState()==CAR_OFF && sent_count==before);
    for(i=0;i<sent_count;++i) if(sent[i].bytes[1]==0xFF) assert(sent[i].length==4U);
    puts("PASS control: automatic enable/center guard/X-right translation/2-D mixing/brake burst/lock release/disable priority");
}
static void TestFailsafe(void)
{
    unsigned before;
    Reset(UINT32_MAX-100U);Ready();Frame(0,0,0,1000);Step(40);
    Step(460); assert(Bluetooth_IsConnected()); /* Exactly 500 ms remains connected. */
    Step(1);assert(Car_Control_GetState()==CAR_LINK_LOST);Step(30);
    before=Count(0xF6);Frame(1,0,0,0);assert(Car_Control_GetState()==CAR_LINK_LOST);
    Frame(0,0,0,1000);Step(30);assert(Car_Control_GetState()==CAR_LINK_LOST && Count(0xF6)==before);
    Frame(0,0,0,0);Step(40);assert(Car_Control_GetState()==CAR_LINK_LOST);
    Frame(1,0,0,0);Frame(0,0,0,0);assert(Car_Control_GetState()==CAR_LINK_LOST);
    Frame(0,0,0,1000);Frame(0,0,0,0);assert(Car_Control_GetState()==CAR_LINK_LOST);
    Frame(0,0,0,0);assert(Car_Control_GetState()==CAR_WAIT_CENTER);
    Frame(0,0,0,1000);Step(40);
    assert(Car_Control_GetState()==CAR_WAIT_CENTER && Count(0xF6)==before);
    Frame(0,0,0,0);assert(Car_Control_GetState()==CAR_READY);
    /* RX transport corruption while running forces link loss immediately. */
    Frame(0,0,0,1000);Step(30);HAL_UART_ErrorCallback(&huart6);Loop();assert(Car_Control_GetState()==CAR_LINK_LOST);
    Reset(0U);Ready();Frame(0,0,0,1000);stall_motor=1U;Step(50);
    assert(Motor_HasFault() && Car_Control_GetState()==CAR_FAULT);
    stall_motor=0U;Step(40);Frame(0,0,0,0);Step(40);
    assert(Car_Control_GetState()==CAR_FAULT && Motor_HasFault());
    Frame(0,0,0,0);
    assert(Car_Control_GetState()==CAR_WAIT_CENTER);
    Step(40);Frame(0,0,0,0);assert(Car_Control_GetState()==CAR_READY && !Motor_HasFault());
    Reset(0U);Ready();next_tx_result=HAL_ERROR;Frame(0,0,0,1000);Step(30);
    assert(Car_Control_GetState()==CAR_FAULT);
    puts("PASS failsafe: 500 ms boundary/tick wrap/reconnect rearm/brake cannot bypass/UART corruption/TX stall/error");
}
static void TestRepeatedManualSegmentsAndStop(void)
{
    BluetoothControlFrame c = {0};
    unsigned i, speeds, stops;
    Reset(0U); Ready();
    for (i = 0U; i < 4U; ++i) {
        float yaw = (float)(i * 40U);
        ImuSample(yaw, 0.0f);
        c.forward = 1;
        ControlFrame(c); Step(30U);
        assert(Car_Control_GetState() == CAR_RUNNING);
        assert(Heading_GetStatus()->reference_valid);
        assert(fabsf(Heading_GetStatus()->yaw_zero - yaw) < 0.1f);
        assert(fabsf(Heading_GetStatus()->yaw_error) < 0.1f);
        ImuSample(yaw + 15.0f, 0.0f); Loop(); Step(30U);
        assert(Heading_GetStatus()->omega_final != 0);
        assert(fabsf((float)Heading_GetStatus()->omega_final) >= 1.0f);
        c.forward = 0;
        stops = Count(0xFEU);
        ControlFrame(c); Step(40U);
        assert(Car_Control_GetState() == CAR_READY);
        assert(Count(0xFEU) == stops + 4U);
        assert(!Heading_GetStatus()->reference_valid);
        speeds = Count(0xF6U);
        Step(30U);
        assert(Count(0xF6U) == speeds);
    }
    ImuSample(160.0f, 0.0f);
    c.forward = 1; ControlFrame(c); Step(30U);
    speeds = Count(0xF6U); stops = Count(0xFEU);
    c.stop = 1; ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_BRAKE_LOCK);
    assert(Count(0xFEU) == stops + 4U && Count(0xF6U) == speeds);
    c.stop = 0; ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_BRAKE_LOCK && Count(0xF6U) == speeds);
    c.forward = 0; ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_READY && Count(0xF6U) == speeds);
    puts("PASS repeated manual segments: fresh heading reference / 20% correction / release and STOP preemption");
}
static void TestBootEnable(void)
{
    unsigned before;
    Reset(0U);Step(1000U);
    assert(Car_Control_GetState()==CAR_WAIT_CENTER && sent_count==0U);
    Frame(0,0,0,1000);Step(40U);
    assert(Car_Control_GetState()==CAR_WAIT_CENTER && Count(0xF6)==0U);
    Frame(0,0,0,0);assert(Car_Control_GetState()==CAR_READY);
    Frame(0,0,0,1000);Step(40U);assert(Car_Control_GetState()==CAR_RUNNING);
    Frame(0,1,0,1000);Step(40U);assert(Car_Control_GetState()==CAR_OFF);
    before=sent_count;Frame(0,0,0,0);Step(600U);
    assert(Car_Control_GetState()==CAR_OFF && sent_count==before);
    /* Legacy ENABLE toggles cannot bypass the center guard. */
    Frame(0,0,0,1000);Step(40U);
    assert(Car_Control_GetState()==CAR_OFF && sent_count==before);
    Frame(0,0,0,0);Step(40U);
    assert(Car_Control_GetState()==CAR_OFF && sent_count==before);
    Frame(1,0,0,0);Frame(0,0,0,0);
    assert(Car_Control_GetState()==CAR_OFF && sent_count==before);
    Frame(0,1,0,0);Step(40U);
    before=sent_count;
    Frame(0,1,0,0);Step(40U);
    assert(Car_Control_GetState()==CAR_OFF && sent_count==before);
    Frame(0,0,0,0);assert(Car_Control_GetState()==CAR_OFF);
    Frame(0,0,0,0);assert(Car_Control_GetState()==CAR_WAIT_CENTER);
    Frame(0,0,0,0);assert(Car_Control_GetState()==CAR_WAIT_CENTER);
    Step(40U);assert(Car_Control_GetState()==CAR_WAIT_CENTER && sent_count==before+4U);
    Frame(0,0,0,0);assert(Car_Control_GetState()==CAR_READY);
    Frame(0,0,0,1000);Step(40U);assert(Car_Control_GetState()==CAR_RUNNING);
    Reset(0U);HAL_UART_ErrorCallback(&huart5);Loop();
    assert(Car_Control_GetState()==CAR_FAULT);
    Step(40U);assert(Count(0xFE)==4U && Count(0xF6)==0U);
    puts("PASS boot enable: one-shot F3/no Bluetooth motion/no ENABLE/held DISABLE/centered packet recovery/brake resets recovery");
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
int main(void)
{
    setvbuf(stdout,NULL,_IONBF,0);
    TestParser();
    if (!CAR_PD10_STANDALONE_TEST && !CAR_MECANUM_TEST_MODE && !CAR_HEADING_TEST_MODE)
        TestActualPhoneProfile();
    if (!CAR_PD10_STANDALONE_TEST && !CAR_MECANUM_TEST_MODE && !CAR_HEADING_TEST_MODE)
        { TestIndependentButtons(); TestIndependentTurns(); TestLeftPacket(); TestLeftRemoteTurn(); TestTranslationAfterTurn(); TestTurnWrongWayLocks();
          TestTurnSettledCorrection(); TestButtonRequiresReady(); }
    TestMotor();TestMecanumMath();TestDirectionWrappers();TestTunerCommands();TestTunerFraming();TestTunerTransport();
    if (CAR_HEADING_TEST_MODE) { TestHeadingIntegration();TestHeadingMailbox(); }
    else if (CAR_MECANUM_TEST_MODE) TestMecanumSafety();
    else if (CAR_PD10_STANDALONE_TEST) TestPD10();
    else { TestControl();TestFailsafe();TestRepeatedManualSegmentsAndStop();TestBootEnable();TestHeadingIntegration();TestJoystickCenterBrakesHeading();TestTunerSafety(); TestVisionPacket();TestVisionLocalFollow();TestVisionReversalAndFault();TestVisionJitterAndRecentering();TestVisionBluetooth();TestVisionAlignmentBounds();TestShotPacket();TestShotBluetoothSwitch();TestShotEntryAndModes();TestShotAimAndFire();TestShotJitterAndFreshFrames();TestShotSafety();TestShotAlignmentBounds();TestPhone20Chassis();TestServoPacket();TestServoRemote();TestServoRemoteMotionAndReconnect();TestServoButton();TestServoButtonSafety();TestServoButtonOffline();TestServoOneButton();TestServoChassisInterlock();TestServoBoolPacket();TestServoBoolActions();TestServoBoolSafety();TestServoGapPacket();TestServoGapAction(); }
    puts("ALL CAR TESTS PASSED (host HAL simulation, not hardware)");return 0;
}
