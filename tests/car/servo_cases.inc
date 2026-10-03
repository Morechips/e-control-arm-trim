#include "../phone20_packet.h"

static void Phone20Frame(const int16_t values[16], uint16_t buttons, uint32_t gap)
{
    uint8_t p[41];
    PackPhone20(p, values, buttons, gap);
    Inject(&huart6, p, sizeof(p)); Bluetooth_Process(); Loop();
}

static void TestPhone20Car(void)
{
    int16_t values[16] = {0};
    const unsigned directions[] = {1U,2U,4U,5U};
    unsigned i, before;
    Reset(0U);
    Phone20Frame(values,0U,556U);
    assert(Car_Control_GetState()==CAR_READY && test_servo_pose_calls==0U);
    for(i=0U;i<4U;++i) {
        before=Count(0xF6);
        values[directions[i]]=1;
        Phone20Frame(values,0U,556U); Step(40U);
        assert(Car_Control_GetState()==CAR_RUNNING && Count(0xF6)>before);
        values[directions[i]]=0;
        Phone20Frame(values,0U,556U); Step(40U);
        assert(Car_Control_GetState()==CAR_READY);
    }
    values[1]=values[11]=1; values[13]=13; values[14]=-1000; values[15]=1000;
    Phone20Frame(values,BT_CONTROL_BOOL_AIM_BIT,556U); Step(40U);
    assert(Car_Control_GetState()==CAR_RUNNING && test_servo_aim_calls==1U);
    Phone20Frame(values,BT_CONTROL_BOOL_AIM_BIT,556U); Step(40U);
    assert(Car_Control_GetState()==CAR_RUNNING && test_servo_aim_calls==1U);
    assert(Bluetooth_GetArmSequence()==0U);
    values[3]=1;
    Phone20Frame(values,0U,556U); Step(40U);
    assert(Car_Control_GetState()==CAR_BRAKE_LOCK);
    memset(values,0,sizeof(values)); Phone20Frame(values,0U,556U); Step(40U);
    assert(Car_Control_GetState()==CAR_READY);
    values[1]=1; Phone20Frame(values,0U,556U); Step(40U);
    assert(Car_Control_GetState()==CAR_RUNNING);
    Step(BT_FAILSAFE_TIMEOUT_MS+1U);
    assert(Car_Control_GetState()==CAR_LINK_LOST);
    puts("PASS phone20 car: first GAP baseline, F/B/L/R, AIM while driving, STOP and disconnect");
}

static void TestPhone20Joystick(void)
{
    static const int16_t x_values[] = {0, 5, -5, 6, -6, 62, -62, 125, -125, 1000, -1000};
    int16_t values[16] = {0};
    uint8_t packet[41];
    unsigned i, before;
    uint32_t sequence;
    for (i = 0U; i < sizeof(x_values) / sizeof(x_values[0]); ++i) {
        Reset(0U);
        memset(values, 0, sizeof(values));
        /* Y is ignored even by the remote startup center gate. */
        values[0] = 1000;
        Phone20Frame(values, 0U, 556U);
        assert(Car_Control_GetState() == CAR_READY);
        sequence = Bluetooth_GetSequence();
        values[12] = x_values[i];
        values[11] = 1; values[13] = 13;
        values[14] = -1000; values[15] = 1000;
        PackPhone20(packet, values, 0U, 556U);
        Inject(&huart6, packet, 28U); Bluetooth_Process();
        assert(Bluetooth_GetSequence() == sequence);
        Inject(&huart6, packet + 28U, sizeof(packet) - 28U); Loop(); Step(40U);
        assert(Bluetooth_GetControl()->frame.joy_x == x_values[i] &&
               Bluetooth_GetControl()->frame.joy_y == 1000 && Bluetooth_GetArmSequence() == 0U);
        assert(Car_Control_GetState() == CAR_READY && Count(0xF6) == 0U && mock_reset_count == 0U);
    }
    Frame(0,0,0,0); Step(150U);
    sequence = Bluetooth_GetSequence(); before = Count(0xF6);
    values[0] = 0; values[12] = 1001; Phone20Frame(values, 0U, 556U);
    assert(Bluetooth_GetSequence() == sequence);
    values[0] = -1001; values[12] = 0; Phone20Frame(values, 0U, 556U);
    assert(Bluetooth_GetSequence() == sequence && Count(0xF6) == before);
    puts("PASS phone20 JOY: axes parsed but inactive / no unlock gate / split frame / no motor or reset TX / protocol limits");
}

static void PackServoControl(uint8_t *packet, const BluetoothControlFrame *c,
                             int16_t reserved)
{
    uint8_t left[BT_CONTROL_FRAME_LEFT_SIZE];
    uint8_t sum = 0U;
    unsigned i;
    PackLeftControl(left, c);
    memcpy(packet, left, 25U);
    packet[25] = (uint8_t)reserved;
    packet[26] = (uint8_t)((uint16_t)reserved >> 8);
    for (i = 1U; i <= 26U; ++i) sum = (uint8_t)(sum + packet[i]);
    packet[27] = sum;
    packet[28] = 0x5AU;
}

static void ServoControlFrameReserved(BluetoothControlFrame c, int16_t reserved)
{
    uint8_t packet[BT_CONTROL_FRAME_RESERVED_SIZE];
    PackServoControl(packet, &c, reserved);
    Inject(&huart6, packet, sizeof(packet));
    Bluetooth_Process();
    Loop();
}


static void PackServoBoolControl(uint8_t *packet, const BluetoothControlFrame *c,
                                 uint16_t buttons, int16_t reserved)
{
    uint8_t numeric[BT_CONTROL_FRAME_RESERVED_SIZE];
    uint8_t sum = 0U;
    unsigned i;
    PackServoControl(numeric, c, reserved);
    packet[0] = 0xA5U;
    packet[1] = (uint8_t)buttons;
    packet[2] = (uint8_t)(buttons >> 8);
    memcpy(&packet[3], &numeric[1], 26U);
    for (i = 1U; i <= 28U; ++i) sum = (uint8_t)(sum + packet[i]);
    packet[29] = sum;
    packet[30] = 0x5AU;
}

static void ServoBoolFrame(BluetoothControlFrame c, uint16_t buttons)
{
    uint8_t packet[BT_CONTROL_FRAME_SERVO_BOOL_SIZE];
    PackServoBoolControl(packet, &c, buttons, -8);
    Inject(&huart6, packet, sizeof(packet));
    Bluetooth_Process();
    Loop();
}

static void PackServoGapControl(uint8_t *packet, const BluetoothControlFrame *c,
                                uint16_t buttons, uint32_t gap, int16_t marker)
{
    uint8_t bool_packet[BT_CONTROL_FRAME_SERVO_BOOL_SIZE];
    uint8_t sum = 0U;
    unsigned i;
    PackServoBoolControl(bool_packet, c, buttons, marker);
    memcpy(packet, bool_packet, 29U);
    packet[29] = (uint8_t)gap;
    packet[30] = (uint8_t)(gap >> 8);
    packet[31] = (uint8_t)(gap >> 16);
    packet[32] = (uint8_t)(gap >> 24);
    for (i = 1U; i <= 32U; ++i) sum = (uint8_t)(sum + packet[i]);
    packet[33] = sum;
    packet[34] = 0x5AU;
}

static void ServoGapFrame(BluetoothControlFrame c, uint16_t buttons, uint32_t gap)
{
    uint8_t packet[BT_ARM_COMBINED_FRAME_SIZE];
    PackServoGapControl(packet, &c, buttons, gap, BT_GAP_FRAME_MARKER);
    Inject(&huart6, packet, sizeof(packet));
    Bluetooth_Process();
    Loop();
}

static void TestServoGapPacket(void)
{
    BluetoothControlFrame c = {0};
    uint8_t packet[BT_ARM_COMBINED_FRAME_SIZE];
    uint32_t sequence;
    unsigned i;
    Reset(0U);
    PackServoGapControl(packet, &c, BT_SERVO_BUTTON_TH_U, 2500U,
                        BT_GAP_FRAME_MARKER);
    sequence = Bluetooth_GetSequence();
    for (i = 0U; i + 1U < sizeof(packet); ++i) {
        Inject(&huart6, &packet[i], 1U);
        Bluetooth_Process();
        assert(Bluetooth_GetSequence() == sequence);
    }
    Inject(&huart6, &packet[sizeof(packet) - 1U], 1U);
    Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence + 1U &&
           Bluetooth_GetControl()->frame.servo_buttons == BT_SERVO_BUTTON_TH_U &&
           Bluetooth_GetControl()->frame.gap_pwm == 2500U &&
           Bluetooth_GetArmSequence() == 0U);
    sequence = Bluetooth_GetSequence();
    packet[33] ^= 1U;
    Inject(&huart6, packet, sizeof(packet)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence);
    PackServoGapControl(packet, &c, 0U, 499U, BT_GAP_FRAME_MARKER);
    Inject(&huart6, packet, sizeof(packet)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence);
    PackServoGapControl(packet, &c, 0U, 2501U, BT_GAP_FRAME_MARKER);
    Inject(&huart6, packet, sizeof(packet)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence);
    PackServoGapControl(packet, &c, 0U, UINT32_MAX, BT_GAP_FRAME_MARKER);
    Inject(&huart6, packet, sizeof(packet)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence);
    PackServoGapControl(packet, &c, 0U, 0U, BT_GAP_FRAME_MARKER);
    Inject(&huart6, packet, sizeof(packet)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence + 1U &&
           Bluetooth_GetControl()->frame.gap_pwm == 0U);
    PackServoGapControl(packet, &c, 0U, 500U, 0);
    Inject(&huart6, packet, sizeof(packet)); Bluetooth_Process();
    assert(Bluetooth_GetControl()->frame.gap_pwm == 0U &&
           Bluetooth_GetArmSequence() == 1U); /* Without -8 it is an arm frame. */
    puts("PASS servo GAP packet: 35-byte bool/short/int layout, split, marker, checksum and bounds");
}


static void TestServoBoolPacket(void)
{
    BluetoothControlFrame c = {0};
    uint8_t packet[BT_CONTROL_FRAME_SERVO_BOOL_SIZE];
    uint32_t sequence;
    unsigned i;
    Reset(0U);
    c.left_90 = 1;
    PackServoBoolControl(packet, &c, 1U << 8, -8);
    sequence = Bluetooth_GetSequence();
    for (i = 0U; i + 1U < sizeof(packet); ++i) {
        Inject(&huart6, &packet[i], 1U);
        Bluetooth_Process();
        assert(Bluetooth_GetSequence() == sequence);
    }
    Inject(&huart6, &packet[sizeof(packet) - 1U], 1U);
    Bluetooth_Process();
    assert(Bluetooth_GetSequence() == ++sequence);
    assert(Bluetooth_GetControl()->frame.servo_buttons == (1U << 8) &&
           Bluetooth_GetControl()->frame.left_90 == 1);
    packet[29] ^= 1U;
    Inject(&huart6, packet, sizeof(packet)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence);
    PackServoBoolControl(packet, &c, 0x1000U, -8);
    Inject(&huart6, packet, sizeof(packet)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence);
    c.forward = 2;
    PackServoBoolControl(packet, &c, 1U, -8);
    Inject(&huart6, packet, sizeof(packet)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence);
    c.forward = 0;
    PackServoBoolControl(packet, &c, 0U, -8);
    Inject(&huart6, packet, sizeof(packet)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence + 1U);
    assert(Bluetooth_GetControl()->frame.servo_buttons == 0U);
    puts("PASS servo bool packet: 31-byte packed bits, shifted shorts, split, checksum, range and unused numeric field");
}

static void TestBoolShotSwitch(void)
{
    BluetoothControlFrame c = {0};
    uint8_t packet[BT_ARM_COMBINED_FRAME_SIZE];
    uint32_t sequence;
    Reset(0U); Ready();
    sequence = Bluetooth_GetSequence();
    ServoGapFrame(c, BT_CONTROL_BOOL_SHOT_BIT, 0U);
    Step(100U);
    TestMaxiCam_ModeStep(); /* The loop's mode step runs, but no notice yet. */
    assert(Bluetooth_GetSequence() == sequence + 1U &&
           Bluetooth_GetControl()->frame.Shot == 1 &&
           Bluetooth_GetControl()->frame.servo_buttons == 0U &&
           test_servo_pose_calls == 0U &&
           Car_Control_GetState() == CAR_SHOT_ALIGNING &&
           TestMaxiCam_RequestedMode() == MAXICAM_MODE_AIM &&
           TestMaxiCam_RequestCount() == 1U && TestMaxiCam_ModePending() &&
           TestMaxiCam_ModeAttempts() == 0U && TestMaxiCam_ModeCount() == 0U);
    /* The 0x80 notice releases the deferred byte, and only once. */
    TestMaxiCam_PublishQrNotice();
    assert(TestMaxiCam_LastMode() == MODE_CMD_AIM &&
           TestMaxiCam_ModeCount() == 1U && TestMaxiCam_ModeAttempts() == 1U &&
           !TestMaxiCam_ModePending());
    ServoGapFrame(c, BT_CONTROL_BOOL_SHOT_BIT, 0U);
    assert(TestMaxiCam_ModeCount() == 1U);
    ServoGapFrame(c, BT_CONTROL_BOOL_SHOT_BIT, 0U);
    assert(TestMaxiCam_ModeCount() == 1U);
    TestMaxiCam_PublishQrNotice();
    assert(TestMaxiCam_ModeCount() == 1U && TestMaxiCam_ModeAttempts() == 1U);
    ServoGapFrame(c, 0U, 0U);
    assert(Car_Control_GetState() == CAR_WAIT_CENTER && !Laser_IsEnabled());
    Step(40U); ServoGapFrame(c, 0U, 0U);
    assert(Car_Control_GetState() == CAR_READY);

    ServoBoolFrame(c, BT_CONTROL_BOOL_SHOT_BIT);
    Step(100U);
    assert(Bluetooth_GetControl()->frame.Shot == 1 &&
           Bluetooth_GetControl()->frame.servo_buttons == 0U &&
           Car_Control_GetState() == CAR_SHOT_ALIGNING &&
           TestMaxiCam_RequestedMode() == MAXICAM_MODE_AIM &&
           TestMaxiCam_RequestCount() == 2U &&
           !TestMaxiCam_ModePending() && TestMaxiCam_ModeCount() == 1U);
    /* Re-requesting the mode the camera already runs does not retransmit. */
    TestMaxiCam_PublishQrNotice();
    assert(TestMaxiCam_ModeCount() == 1U && TestMaxiCam_ModeAttempts() == 1U);
    ServoBoolFrame(c, 0U);
    assert(Car_Control_GetState() == CAR_WAIT_CENTER);

    /* A failing byte keeps the request pending and the caller in shot mode. */
    Reset(0U); Ready();
    TestMaxiCam_SetSendResult(HAL_BUSY);
    ServoGapFrame(c, BT_CONTROL_BOOL_SHOT_BIT, 0U);
    Step(100U);
    TestMaxiCam_ModeStep(); /* Still deferred: no QR notice has arrived. */
    assert(Car_Control_GetState() == CAR_SHOT_ALIGNING &&
           TestMaxiCam_RequestCount() == 1U && TestMaxiCam_ModePending() &&
           TestMaxiCam_ModeAttempts() == 0U && TestMaxiCam_ModeCount() == 0U);
    TestMaxiCam_PublishQrNotice();
    assert(TestMaxiCam_ModeAttempts() == 1U && TestMaxiCam_ModeCount() == 0U &&
           TestMaxiCam_ModePending() && TestMaxiCam_LastMode() == UINT8_MAX &&
           Car_Control_GetState() == CAR_SHOT_ALIGNING);
    TestMaxiCam_SetSendResult(HAL_OK);
    TestMaxiCam_ModeStep();
    assert(TestMaxiCam_ModeAttempts() == 2U && TestMaxiCam_ModeCount() == 1U &&
           TestMaxiCam_LastMode() == MODE_CMD_AIM &&
           !TestMaxiCam_ModePending() &&
           Car_Control_GetState() == CAR_SHOT_ALIGNING);

    Reset(0U); Ready();
    c.Shot = 1;
    ServoGapFrame(c, 0U, 0U);
    Step(100U);
    assert(Bluetooth_GetControl()->frame.Shot == 1 &&
           Car_Control_GetState() == CAR_SHOT_ALIGNING);

    Reset(0U);
    c.Shot = 0;
    PackServoGapControl(packet, &c, 0x1000U, 0U, BT_GAP_FRAME_MARKER);
    sequence = Bluetooth_GetSequence();
    Inject(&huart6, packet, sizeof(packet)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence);
    puts("PASS bool SHOT: 35/31-byte entry, QR-released deferred mode, release, failed-retry, legacy short and reserved-bit rejection");
}

static void TestBoolShotAimAndRelease(void)
{
    BluetoothControlFrame c = {0};
    unsigned i;
    Reset(0U); Ready();
    ServoGapFrame(c, BT_CONTROL_BOOL_SHOT_BIT, 0U);
    Step(100U);
    assert(Car_Control_GetState() == CAR_SHOT_ALIGNING);
    for (i = 0U; i < 8U; ++i) {
        TestMaxiCam_Publish(DETECT_TARGET, 0, 1234, 1U);
        ServoGapFrame(c, BT_CONTROL_BOOL_SHOT_BIT, 0U);
        Step(VISION_SAMPLE_INTERVAL_MS);
    }
    assert(Car_Control_GetState() == CAR_SHOT_FIRING &&
           Laser_IsEnabled() && test_laser_enable_count == 1U);
    ServoGapFrame(c, 0U, 0U);
    assert(Car_Control_GetState() == CAR_WAIT_CENTER && !Laser_IsEnabled());
    puts("PASS bool SHOT aim: fresh camera frames fire once, release immediately disables laser");
}

static void TestAimInterface(void)
{
    BluetoothControlFrame c = {0};
    uint32_t sequence;
    Reset(0U); Ready();
    sequence = Bluetooth_GetAimSequence();
    ServoGapFrame(c, BT_CONTROL_BOOL_AIM_BIT, 0U);
    assert(Bluetooth_GetControl()->frame.aim == 1U &&
           Bluetooth_GetControl()->frame.Shot == 0 &&
           Bluetooth_GetControl()->frame.servo_buttons == 0U &&
           Bluetooth_GetAimSequence() == sequence + 1U &&
           Car_Control_GetState() == CAR_READY &&
           test_servo_aim_calls == 1U && TestMaxiCam_ModeCount() == 0U &&
           TestMaxiCam_RequestCount() == 0U && !TestMaxiCam_ModePending());
    ServoGapFrame(c, BT_CONTROL_BOOL_AIM_BIT, 0U);
    assert(Bluetooth_GetAimSequence() == sequence + 1U &&
           test_servo_aim_calls == 1U);
    ServoGapFrame(c, 0U, 0U);
    ServoGapFrame(c, BT_CONTROL_BOOL_AIM_BIT, 0U);
    assert(Bluetooth_GetAimSequence() == sequence + 2U &&
           test_servo_aim_calls == 2U);
    ServoBoolFrame(c, 0U);
    ServoBoolFrame(c, BT_CONTROL_BOOL_AIM_BIT);
    assert(Bluetooth_GetControl()->frame.aim == 1U &&
           Bluetooth_GetAimSequence() == sequence + 3U &&
           test_servo_aim_calls == 3U);
    ShotControlFrame(c);
    assert(Bluetooth_GetControl()->frame.aim == 0U &&
           Bluetooth_GetAimSequence() == sequence + 3U);
    puts("PASS AIM interface: 35/31-byte bool, press edge, release rearm, arm request and no shot output");
}




static void TestServoPacket(void)
{
    BluetoothControlFrame c = {0};
    uint8_t packet[BT_CONTROL_FRAME_RESERVED_SIZE];
    uint8_t legacy[BT_CONTROL_FRAME_LEFT_SIZE];
    uint8_t old_buttons[43] = {0};
    uint32_t sequence;
    unsigned i, j;
    Reset(0U);
    c.left_90 = 1;
    sequence = Bluetooth_GetSequence();
    for (i = 0U; i < 3U; ++i) {
        static const int16_t reserved_values[] = {0, 11, -1};
        PackServoControl(packet, &c, reserved_values[i]);
        for (j = 0U; j + 1U < sizeof(packet); ++j) {
            Inject(&huart6, &packet[j], 1U);
            Bluetooth_Process();
            assert(Bluetooth_GetSequence() == sequence);
        }
        Inject(&huart6, &packet[sizeof(packet) - 1U], 1U);
        Bluetooth_Process();
        assert(Bluetooth_GetSequence() == ++sequence);
        assert(Bluetooth_GetControl()->frame.left_90 == 1);
    }
    packet[27] ^= 1U;
    Inject(&huart6, packet, sizeof(packet)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence);
    PackServoControl(packet, &c, 1);
    packet[28] = 0U;
    Inject(&huart6, packet, sizeof(packet)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence);

    c.left_90 = 0;
    PackLeftControl(legacy, &c);
    Inject(&huart6, legacy, sizeof(legacy)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == ++sequence);

    /* An old 43-byte button frame must not be interpreted as a new mode. */
    memcpy(old_buttons, legacy, 25U);
    old_buttons[25] = 1U;
    for (i = 1U; i <= 40U; ++i) old_buttons[41] = (uint8_t)(old_buttons[41] + old_buttons[i]);
    old_buttons[42] = 0x5AU;
    Inject(&huart6, old_buttons, sizeof(old_buttons)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence);
    PackServoControl(packet, &c, 0);
    Inject(&huart6, packet, sizeof(packet)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence + 1U);
    puts("PASS reserved packet: 29-byte ignored field / split / checksum / tail / legacy / retired 43-byte frame");
}

static void TestReservedFieldNoEffect(void)
{
    BluetoothControlFrame c = {0};
    Reset(0U); Ready();
    ServoControlFrameReserved(c, 1);
    ServoControlFrameReserved(c, 10);
    ServoControlFrameReserved(c, -8);
    assert(test_servo_pose_calls == 0U && Car_Control_GetState() == CAR_READY);

    c.forward = 1;
    ServoControlFrameReserved(c, 1); Step(40U);
    assert(Car_Control_GetState() == CAR_RUNNING);
    ServoControlFrameReserved(c, 10); Step(40U);
    assert(Car_Control_GetState() == CAR_RUNNING &&
           test_servo_pose_calls == 0U);
    c.forward = 0;
    ServoControlFrameReserved(c, -1); Step(40U);
    assert(Car_Control_GetState() == CAR_READY);
    puts("PASS reserved field: arbitrary values neither move arm nor inhibit chassis");
}







static void TestServoCarIndependence(void)
{
    BluetoothControlFrame c = {0};
    const uint16_t events[] = {BT_SERVO_BUTTON_TB_B, BT_SERVO_BUTTON_TH_L,
                               BT_SERVO_BUTTON_RST, BT_CONTROL_BOOL_AIM_BIT};
    unsigned i, before;
    Reset(0U); Ready();
    c.forward = 1;
    ServoGapFrame(c, 0U, 500U); Step(40U);
    assert(Car_Control_GetState() == CAR_RUNNING);
    for (i = 0U; i < sizeof(events) / sizeof(events[0]); ++i) {
        before = test_servo_pose_calls;
        ServoGapFrame(c, events[i], 500U); Step(40U);
        assert(test_servo_pose_calls == before + 1U && Car_Control_GetState() == CAR_RUNNING);
    }
    before = test_servo_pose_calls;
    ServoGapFrame(c, 0U, 800U); Step(40U);
    assert(test_servo_pose_calls == before + 1U && Car_Control_GetState() == CAR_RUNNING);
    {
        const uint8_t reference[] = {0xA5U, 0x31U, 0U, 0x31U, 0x5AU};
        Inject(&huart6, reference, sizeof(reference)); Bluetooth_Process(); Loop(); Step(40U);
        assert(test_servo_pose_calls == before + 2U && Car_Control_GetState() == CAR_RUNNING);
    }
    pe4_low = 1U; Step(21U);
    assert(test_servo_pose_calls == before + 3U && Car_Control_GetState() == CAR_RUNNING);
    c.stop = 1; ServoGapFrame(c, 0U, 800U); Step(40U);
    assert(Car_Control_GetState() == CAR_BRAKE_LOCK);

    Reset(0U); Ready(); c = (BluetoothControlFrame){0};
    c.Cam_T = 1; ServoGapFrame(c, 0U, 500U); Step(100U);
    assert(Car_Control_GetState() == CAR_VISION_TRACKING);
    for (i = 0U; i < sizeof(events) / sizeof(events[0]); ++i) {
        ServoGapFrame(c, events[i], 500U);
        assert(Car_Control_GetState() == CAR_VISION_TRACKING);
    }
    ServoGapFrame(c, 0U, 900U);
    assert(Car_Control_GetState() == CAR_VISION_TRACKING);

    Reset(0U); Ready(); c = (BluetoothControlFrame){0};
    ServoGapFrame(c, BT_CONTROL_BOOL_SHOT_BIT | BT_CONTROL_BOOL_AIM_BIT, 500U);
    Step(100U);
    assert(test_servo_aim_calls == 1U && Car_Control_GetState() == CAR_SHOT_ALIGNING);
    for (i = 0U; i < sizeof(events) / sizeof(events[0]); ++i) {
        ServoGapFrame(c, BT_CONTROL_BOOL_SHOT_BIT | events[i], 500U);
        assert(Car_Control_GetState() == CAR_SHOT_ALIGNING);
    }
    before = test_servo_pose_calls;
    Step(BT_FAILSAFE_TIMEOUT_MS + 1U);
    assert(Car_Control_GetState() != CAR_SHOT_ALIGNING && !Laser_IsEnabled());
    assert(test_servo_pose_calls == before);
    puts("PASS servo independence: driving/vision/SHOT continue; chassis STOP/link-loss remain effective");
}
