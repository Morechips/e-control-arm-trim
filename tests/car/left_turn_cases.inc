static void PackLeftControl(uint8_t *packet, const BluetoothControlFrame *c)
{
    uint8_t shot[BT_CONTROL_FRAME_SHOT_SIZE];
    uint8_t sum = 0U;
    unsigned i;
    PackShotControl(shot, c);
    memcpy(packet, shot, 23U);
    packet[0] = 0xA5U;
    packet[23] = (uint8_t)c->left_90;
    packet[24] = (uint8_t)((uint16_t)c->left_90 >> 8);
    for (i = 1U; i <= 24U; ++i) sum = (uint8_t)(sum + packet[i]);
    packet[25] = sum;
    packet[26] = 0x5AU;
}

static void LeftControlFrame(BluetoothControlFrame c)
{
    uint8_t packet[BT_CONTROL_FRAME_LEFT_SIZE];
    PackLeftControl(packet, &c);
    Inject(&huart6, packet, sizeof(packet));
    Bluetooth_Process();
    Loop();
}

static void TestLeftPacket(void)
{
    BluetoothControlFrame c = {0};
    uint8_t packet[BT_CONTROL_FRAME_LEFT_SIZE];
    uint8_t legacy[BT_CONTROL_FRAME_SIZE];
    uint32_t sequence;
    unsigned i;
    Reset(0U);
    c.left_90 = 1;
    c.Cam_T = c.Shot = 1;
    PackLeftControl(packet, &c);
    sequence = Bluetooth_GetSequence();
    for (i = 0U; i < sizeof(packet) - 1U; ++i)
    {
        Inject(&huart6, &packet[i], 1U);
        Bluetooth_Process();
        assert(Bluetooth_GetSequence() == sequence);
    }
    Inject(&huart6, &packet[sizeof(packet) - 1U], 1U);
    Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence + 1U);
    assert(Bluetooth_GetControl()->frame.left_90 == 1 &&
           Bluetooth_GetControl()->frame.Cam_T == 1 &&
           Bluetooth_GetControl()->frame.Shot == 1);
    sequence = Bluetooth_GetSequence();
    packet[25] ^= 1U;
    Inject(&huart6, packet, sizeof(packet)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence);
    c.left_90 = 2; PackLeftControl(packet, &c);
    Inject(&huart6, packet, sizeof(packet)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence);
    c.left_90 = 1; PackLeftControl(packet, &c);
    packet[26] = 0U;
    Inject(&huart6, packet, sizeof(packet)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence);
    PackLeftControl(packet, &c);
    Inject(&huart6, packet, sizeof(packet)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence + 1U);
    c.left_90 = c.Cam_T = c.Shot = 0; PackControl(legacy, &c);
    Inject(&huart6, legacy, sizeof(legacy)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence + 2U);
    assert(Bluetooth_GetControl()->frame.left_90 == 0);
    {
        uint8_t old_vision[BT_CONTROL_FRAME_EXT_SIZE];
        uint8_t old_shot[BT_CONTROL_FRAME_SHOT_SIZE];
        c.Cam_T = 1; c.Shot = 1;
        PackVisionControl(old_vision, &c);
        Inject(&huart6, old_vision, sizeof(old_vision)); Bluetooth_Process();
        assert(Bluetooth_GetSequence() == sequence + 3U);
        assert(Bluetooth_GetControl()->frame.Cam_T == 1 &&
               Bluetooth_GetControl()->frame.Shot == 0 &&
               Bluetooth_GetControl()->frame.left_90 == 0);
        PackShotControl(old_shot, &c);
        Inject(&huart6, old_shot, sizeof(old_shot)); Bluetooth_Process();
        assert(Bluetooth_GetSequence() == sequence + 4U);
        assert(Bluetooth_GetControl()->frame.Shot == 1 &&
               Bluetooth_GetControl()->frame.left_90 == 0);
    }
    puts("PASS left packet: A5 27-byte split/checksum/domain/tail/recovery and shorter-frame compatibility");
}

static void TestLeftRemoteTurn(void)
{
    BluetoothControlFrame c = {0};
    unsigned speed, i;
    Reset(0U); Ready(); ImuSample(0.0f, 0.0f); Loop();
    c.left_90 = 1; LeftControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_TURNING);
    for (i = 1U; i <= 4U; ++i)
        AssertLatestPhysicalSpeed((uint8_t)i, -TURN_LEFT_90_RPM);
    speed = Count(0xF6);
    LeftControlFrame(c); Step(40U);
    assert(Count(0xF6) == speed);
    ImuSample(86.0f, 20.0f); Loop(); Step(40U);
    assert(TurnRight_GetStatus()->state == TURN_RIGHT_STOPPING);
    for (i = 0U; i < 14U; ++i)
    {
        Step(10U); ImuSample(90.0f, 0.0f); Loop();
    }
    assert(TurnRight_GetStatus()->state == TURN_RIGHT_DONE);
    Step(10U); ImuSample(90.0f, 0.0f); Loop();
    assert(Car_Control_GetState() == CAR_READY);
    LeftControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_READY && Count(0xF6) == speed);
    c.left_90 = 0; LeftControlFrame(c); Step(40U);
    c.left_90 = 1; LeftControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_TURNING);
    ImuSample(90.0f, 0.0f); Loop();
    c.left_90 = 0; LeftControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_TURNING);
    speed = Count(0xF6);
    c.right_90 = 1; LeftControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_READY && Count(0xF6) == speed);
    LeftControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_READY && Count(0xF6) == speed);
    c.right_90 = 0; LeftControlFrame(c); Step(40U);
    ImuSample(90.0f, 0.0f); Loop();
    c.left_90 = 1; LeftControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_TURNING && Count(0xF6) > speed);
    c.stop = 1; LeftControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_BRAKE_LOCK);
    c.left_90 = c.stop = 0; LeftControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_READY);

    Reset(0U); Ready(); ImuSample(0.0f, 0.0f); Loop();
    c.left_90 = c.right_90 = 1; LeftControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_READY && Count(0xF6) == 0U);
    c.right_90 = 0; LeftControlFrame(c); Step(40U);
    assert(Count(0xF6) == 0U);
    c.left_90 = 0; LeftControlFrame(c); Step(40U);
    ImuSample(0.0f, 0.0f); Loop();
    c.left_90 = 1; LeftControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_TURNING);
    now += BT_FAILSAFE_TIMEOUT_MS + 1U; Loop();
    assert(Car_Control_GetState() == CAR_LINK_LOST);

    Reset(0U); Ready(); ImuSample(0.0f, 0.0f); Loop();
    c.left_90 = 0; c.right_90 = 1; LeftControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_TURNING);
    speed = Count(0xF6);
    c.right_90 = 0; c.left_90 = 1; LeftControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_READY && Count(0xF6) == speed);
    LeftControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_READY && Count(0xF6) == speed);
    c.left_90 = 0; LeftControlFrame(c); Step(40U);
    ImuSample(0.0f, 0.0f); Loop();
    c.left_90 = 1; LeftControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_TURNING && Count(0xF6) > speed);
    c.stop = 1; LeftControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_BRAKE_LOCK);

    Reset(0U); Ready(); ImuSample(0.0f, 0.0f); Loop();
    c.stop = 0;
    c.left_90 = 1; LeftControlFrame(c); Step(40U);
    ImuSample(-11.0f, -20.0f); Loop(); Step(40U);
    assert(TurnRight_GetStatus()->state == TURN_RIGHT_FAULT);
    assert(Car_Control_GetState() == CAR_BRAKE_LOCK);
    puts("PASS left remote: 20 RPM direction/edge/completion/conflict/rearm/STOP/link loss/fault");
}
