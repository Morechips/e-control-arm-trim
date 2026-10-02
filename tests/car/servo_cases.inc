static void PackPhone20(uint8_t packet[BT_CONTROL_FRAME_PHONE20_SIZE],
                        const int16_t values[16], uint16_t buttons,
                        uint32_t gap)
{
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

static void Phone20Frame(const int16_t values[16])
{
    uint8_t packet[BT_CONTROL_FRAME_PHONE20_SIZE];
    PackPhone20(packet, values, 0U, 556U);
    Inject(&huart6, packet, sizeof(packet));
    Bluetooth_Process();
    Loop();
}

static void TestPhone20Chassis(void)
{
    int16_t values[16] = {0};
    const unsigned directions[] = {1U, 2U, 4U, 5U};
    unsigned i, before;
    Reset(0U);
    Phone20Frame(values);
    assert(Car_Control_GetState() == CAR_READY);
    for (i = 0U; i < sizeof(directions) / sizeof(directions[0]); ++i) {
        before = Count(0xF6U);
        values[directions[i]] = 1;
        Phone20Frame(values); Step(40U);
        assert(Car_Control_GetState() == CAR_RUNNING && Count(0xF6U) > before);
        values[directions[i]] = 0;
        Phone20Frame(values); Step(40U);
        assert(Car_Control_GetState() == CAR_READY);
    }
    values[12] = 600; /* JOY_X is after SERVO_MODE in teammate (20).pro. */
    Phone20Frame(values); Step(40U);
    assert(Car_Control_GetState() == CAR_RUNNING);
    values[12] = 0;
    Phone20Frame(values); Step(40U);
    assert(Car_Control_GetState() == CAR_READY);
    puts("PASS phone20 chassis: exact teammate 41-byte F/B/L/R and JOY_X layout");
}

static void PackServoControl(uint8_t *packet, const BluetoothControlFrame *c)
{
    uint8_t left[BT_CONTROL_FRAME_LEFT_SIZE];
    uint8_t sum = 0U;
    unsigned i;
    PackLeftControl(left, c);
    memcpy(packet, left, 25U);
    packet[25] = (uint8_t)c->servo_mode;
    packet[26] = (uint8_t)((uint16_t)c->servo_mode >> 8);
    for (i = 1U; i <= 26U; ++i) sum = (uint8_t)(sum + packet[i]);
    packet[27] = sum;
    packet[28] = 0x5AU;
}

static void ServoControlFrame(BluetoothControlFrame c)
{
    uint8_t packet[BT_CONTROL_FRAME_SERVO_MODE_SIZE];
    PackServoControl(packet, &c);
    Inject(&huart6, packet, sizeof(packet));
    Bluetooth_Process();
    Loop();
}

static void PackServoBoolControl(uint8_t *packet, const BluetoothControlFrame *c,
                                 uint16_t buttons, int16_t unused_mode)
{
    BluetoothControlFrame copy = *c;
    uint8_t numeric[BT_CONTROL_FRAME_SERVO_MODE_SIZE];
    uint8_t sum = 0U;
    unsigned i;
    copy.servo_mode = unused_mode;
    PackServoControl(numeric, &copy);
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
    PackServoGapControl(packet, &c, buttons, gap, BT_SERVO_GAP_MARKER);
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
    PackServoGapControl(packet, &c,
                        BT_SERVO_BUTTON_TH_U | BT_CONTROL_BUTTON_SHOT |
                        BT_CONTROL_BUTTON_AIM | BT_SERVO_BUTTON_TH_L,
                        2500U,
                        BT_SERVO_GAP_MARKER);
    sequence = Bluetooth_GetSequence();
    for (i = 0U; i + 1U < sizeof(packet); ++i) {
        Inject(&huart6, &packet[i], 1U);
        Bluetooth_Process();
        assert(Bluetooth_GetSequence() == sequence);
    }
    Inject(&huart6, &packet[sizeof(packet) - 1U], 1U);
    Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence + 1U &&
           Bluetooth_GetControl()->frame.servo_buttons ==
               (BT_SERVO_BUTTON_TH_U | BT_SERVO_BUTTON_TH_L) &&
           Bluetooth_GetControl()->frame.gap_pwm == 2500U &&
           Bluetooth_GetControl()->frame.servo_mode == 0 &&
           Bluetooth_GetControl()->frame.Shot == 1 &&
           Bluetooth_GetControl()->frame.aim == 1 &&
           Bluetooth_GetControl()->frame.Cam_T == 0 &&
           Bluetooth_GetControl()->frame.left_90 == 0 &&
           Bluetooth_GetArmSequence() == 0U);
    sequence = Bluetooth_GetSequence();
    packet[33] ^= 1U;
    Inject(&huart6, packet, sizeof(packet)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence);
    PackServoGapControl(packet, &c, 0U, 499U, BT_SERVO_GAP_MARKER);
    Inject(&huart6, packet, sizeof(packet)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence);
    PackServoGapControl(packet, &c, 0U, 2501U, BT_SERVO_GAP_MARKER);
    Inject(&huart6, packet, sizeof(packet)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence);
    PackServoGapControl(packet, &c, 0U, UINT32_MAX, BT_SERVO_GAP_MARKER);
    Inject(&huart6, packet, sizeof(packet)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence);
    PackServoGapControl(packet, &c, 0U, 0U, BT_SERVO_GAP_MARKER);
    Inject(&huart6, packet, sizeof(packet)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence + 1U &&
           Bluetooth_GetControl()->frame.gap_pwm == 0U);
    PackServoGapControl(packet, &c, 0U, 500U, 0);
    Inject(&huart6, packet, sizeof(packet)); Bluetooth_Process();
    assert(Bluetooth_GetControl()->frame.gap_pwm == 0U &&
           Bluetooth_GetArmSequence() == 1U); /* Without -8 it is an arm frame. */
    puts("PASS servo GAP packet: 35-byte bool/short/int layout, split, marker, checksum and bounds");
}

static void TestServoGapAction(void)
{
    BluetoothControlFrame c = {0};
    Reset(0U); Ready();
    ServoGapFrame(c, 0U, 500U);
    assert(test_original_pose_calls == 1U &&
           test_servo_last_step.joint_mask == (1U << ARM_JOINT_GRIPPER) &&
           test_servo_last_step.position[ARM_JOINT_GRIPPER] == 500U &&
           test_servo_last_step.move_ms == 1000U);
    ServoGapFrame(c, 0U, 500U);
    assert(test_original_pose_calls == 1U);
    ServoGapFrame(c, 0U, 2500U);
    assert(test_original_pose_calls == 2U &&
           test_servo_last_step.position[ARM_JOINT_GRIPPER] == 2500U &&
           test_servo_last_step.move_ms == 1000U);
    ServoGapFrame(c, 0U, 0U);
    assert(test_original_pose_calls == 2U);
    ServoGapFrame(c, 0U, 500U);
    assert(test_original_pose_calls == 3U);

    Reset(0U); Ready();
    c.joy_y = 1000; ServoGapFrame(c, 0U, 0U); Step(40U);
    assert(Car_Control_GetState() == CAR_RUNNING);
    ServoGapFrame(c, 0U, 800U);
    assert(test_original_pose_calls == 0U && Car_Control_GetState() == CAR_RUNNING);
    c.joy_y = 0; ServoGapFrame(c, 0U, 0U); Step(40U);
    ServoGapFrame(c, 0U, 800U);
    assert(test_original_pose_calls == 1U);
    puts("PASS servo GAP action: first-value baseline, ID 003 only, changed value and moving-car rejection");
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
           Bluetooth_GetControl()->frame.servo_mode == 0 &&
           Bluetooth_GetControl()->frame.left_90 == 1);
    packet[29] ^= 1U;
    Inject(&huart6, packet, sizeof(packet)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence);
    c.left_90 = 0;
    PackServoBoolControl(packet, &c,
                         BT_CONTROL_BUTTON_SHOT |
                         BT_CONTROL_BUTTON_AIM |
                         BT_SERVO_BUTTON_TH_L, -8);
    Inject(&huart6, packet, sizeof(packet)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == ++sequence &&
           Bluetooth_GetControl()->frame.servo_buttons == BT_SERVO_BUTTON_TH_L &&
           Bluetooth_GetControl()->frame.Shot == 1 &&
           Bluetooth_GetControl()->frame.aim == 1 &&
           Bluetooth_GetControl()->frame.Cam_T == 0 &&
           Bluetooth_GetControl()->frame.left_90 == 0);
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

static void TestServoBoolActions(void)
{
    static const uint16_t expected[8][3] = {
        {1532U, 1972U, 573U},
        {1421U, 2071U, 812U},
        {1421U, 2071U, 812U},
        {1717U, 2297U, 884U},
        {1717U, 2297U, 884U},
        {1499U, 1855U, 528U},
        {1499U, 1855U, 528U},
        {1800U, 1855U, 528U}
    };
    static const uint16_t expected_gripper[8] = {
        2192U, 2192U, 500U, 500U, 1800U, 1800U, 500U, 500U
    };
    BluetoothControlFrame c = {0};
    unsigned i, j;
    Reset(0U); Ready();
    for (i = 0U; i < 8U; ++i) {
        uint16_t button = (uint16_t)(1U << i);
        ServoBoolFrame(c, button);
        assert(test_original_pose_calls == i + 1U &&
               test_servo_pose_calls == i + 1U);
        assert(test_servo_last_step.joint_mask == 0x0FU &&
               test_servo_last_step.position[3] == expected_gripper[i]);
        for (j = 0U; j < 3U; ++j)
            assert(test_servo_last_step.position[j] == expected[i][j]);
        assert(test_servo_last_step.move_ms == 1000U);
        ServoBoolFrame(c, button);
        assert(test_original_pose_calls == i + 1U); /* Held button. */
        ServoBoolFrame(c, 0U);
    }
    ServoBoolFrame(c, 1U << 8);
    assert(test_servo_reset_calls == 1U && test_original_pose_calls == 8U);
    ServoBoolFrame(c, 1U << 8);
    assert(test_servo_reset_calls == 1U);
    ServoBoolFrame(c, 0U);
    ServoBoolFrame(c, (1U << 0) | (1U << 1));
    assert(test_original_pose_calls == 8U); /* Ambiguous simultaneous press. */
    ServoBoolFrame(c, 1U << 1);
    assert(test_original_pose_calls == 8U); /* Must release and press again. */
    ServoBoolFrame(c, 0U);
    ServoBoolFrame(c, 1U << 1);
    assert(test_original_pose_calls == 9U);
    ServoBoolFrame(c, 1U << 2);
    assert(test_original_pose_calls == 10U); /* Direct mode switch. */
    puts("PASS servo bool actions: eight four-joint poses, RST, edge, hold, conflicts and direct switch");
}

static void TestServoBoolSafety(void)
{
    BluetoothControlFrame c = {0};
    Reset(0U); Ready();
    c.joy_y = 1000;
    ServoBoolFrame(c, 0U); Step(40U);
    assert(Car_Control_GetState() == CAR_RUNNING);
    ServoBoolFrame(c, 1U);
    assert(test_original_pose_calls == 0U && Car_Control_GetState() == CAR_RUNNING);
    c.joy_y = 0;
    ServoBoolFrame(c, 0U); Step(40U);
    ServoBoolFrame(c, 1U);
    assert(test_original_pose_calls == 1U);
    ServoBoolFrame(c, 0U);
    Step(BT_FAILSAFE_TIMEOUT_MS + 1U);
    assert(!Bluetooth_IsConnected());
    ServoBoolFrame(c, 1U);
    assert(test_original_pose_calls == 1U); /* Reconnect needs car rearm. */
    ServoBoolFrame(c, 0U); Step(40U);
    ServoBoolFrame(c, 0U); Step(40U); ServoBoolFrame(c, 0U);
    assert(Car_Control_GetState() == CAR_READY);
    ServoBoolFrame(c, 1U);
    assert(test_original_pose_calls == 2U);
    puts("PASS servo bool safety: moving-car action rejected without blocking chassis, centered retry, disconnect and rearm");
}

static void TestServoPacket(void)
{
    BluetoothControlFrame c = {0};
    uint8_t packet[BT_CONTROL_FRAME_SERVO_MODE_SIZE];
    uint8_t legacy[BT_CONTROL_FRAME_LEFT_SIZE];
    uint8_t old_buttons[43] = {0};
    uint32_t sequence;
    unsigned i, j;
    Reset(0U);
    c.left_90 = 1;
    sequence = Bluetooth_GetSequence();
    for (i = 0U; i <= BT_SERVO_MODE_MAX; ++i) {
        c.servo_mode = (int16_t)i;
        PackServoControl(packet, &c);
        for (j = 0U; j + 1U < sizeof(packet); ++j) {
            Inject(&huart6, &packet[j], 1U);
            Bluetooth_Process();
            assert(Bluetooth_GetSequence() == sequence);
        }
        Inject(&huart6, &packet[sizeof(packet) - 1U], 1U);
        Bluetooth_Process();
        assert(Bluetooth_GetSequence() == ++sequence);
        assert(Bluetooth_GetControl()->frame.left_90 == 1 &&
               Bluetooth_GetControl()->frame.servo_mode == (int16_t)i);
    }
    packet[27] ^= 1U;
    Inject(&huart6, packet, sizeof(packet)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence);
    c.servo_mode = 11;
    PackServoControl(packet, &c);
    Inject(&huart6, packet, sizeof(packet)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence);
    c.servo_mode = -1;
    PackServoControl(packet, &c);
    Inject(&huart6, packet, sizeof(packet)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence);
    c.servo_mode = 1;
    PackServoControl(packet, &c);
    packet[28] = 0U;
    Inject(&huart6, packet, sizeof(packet)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence);

    c.left_90 = 0;
    PackLeftControl(legacy, &c);
    Inject(&huart6, legacy, sizeof(legacy)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == ++sequence);
    assert(Bluetooth_GetControl()->frame.servo_mode == 0);

    /* An old 43-byte button frame must not be interpreted as a new mode. */
    memcpy(old_buttons, legacy, 25U);
    old_buttons[25] = 1U;
    for (i = 1U; i <= 40U; ++i) old_buttons[41] = (uint8_t)(old_buttons[41] + old_buttons[i]);
    old_buttons[42] = 0x5AU;
    Inject(&huart6, old_buttons, sizeof(old_buttons)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence);
    PackServoControl(packet, &c);
    Inject(&huart6, packet, sizeof(packet)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence + 1U);
    puts("PASS servo packet: 29-byte modes 0..10 / split / checksum / range / tail / legacy / retired 43-byte frame");
}

static void TestServoRemote(void)
{
    BluetoothControlFrame c = {0};
    unsigned i;
    Reset(0U); Ready();
    for (i = 1U; i <= 10U; ++i) {
        unsigned before = test_servo_pose_calls;
        c.servo_mode = (int16_t)i;
        ServoControlFrame(c);
        assert(test_servo_pose_calls == before + (i != 7U));
        if (i == 1U || i == 6U)
            assert(test_servo_last_step.position[0] == 1532U &&
                   test_servo_last_step.position[1] == 2219U &&
                   test_servo_last_step.position[2] == 1202U &&
                   test_servo_last_step.joint_mask == 0x07U &&
                   test_servo_last_step.position[3] == 0U);
        if (i == 3U)
            assert(test_servo_last_step.position[0] == 1421U &&
                   test_servo_last_step.position[3] == 0U &&
                   test_servo_last_step.move_ms == 1000U);
        if (i == 9U)
            assert(test_servo_last_step.position[0] == 1499U &&
                   test_servo_last_step.position[3] == 0U);
        ServoControlFrame(c);
        assert(test_servo_pose_calls == before + (i != 7U));
    }
    c.servo_mode = 0; ServoControlFrame(c);
    c.servo_mode = 3; ServoControlFrame(c);
    assert(test_servo_pose_calls == 10U);
    c.servo_mode = 0; ServoControlFrame(c);
    test_servo_result = ARM_BUSY;
    c.servo_mode = 9; ServoControlFrame(c);
    assert(test_servo_pose_calls == 10U);
    ServoControlFrame(c);
    assert(test_servo_pose_calls == 10U);
    c.servo_mode = 0; ServoControlFrame(c);
    test_servo_result = ARM_OK;
    c.servo_mode = 9; ServoControlFrame(c);
    assert(test_servo_pose_calls == 11U);
    Reset(0U); Ready();
    test_arm_hold = 1U;
    c = (BluetoothControlFrame){0}; c.servo_mode = 3; ServoControlFrame(c);
    assert(test_servo_pose_calls == 1U);
    c.servo_mode = 0; ServoControlFrame(c);
    c.servo_mode = 9; ServoControlFrame(c);
    assert(test_servo_pose_calls == 1U); /* Running preset is exclusive. */
    test_arm_status.state = ARM_COMPLETE_ESTIMATED; Loop();
    c.servo_mode = 0; ServoControlFrame(c);
    test_arm_hold = 0U;
    c.servo_mode = 9; ServoControlFrame(c);
    assert(test_servo_pose_calls == 2U);
    puts("PASS servo modes: ID 003 omitted, calibrated modes, repetition and rearm");
}

static void TestServoRemoteMotionAndReconnect(void)
{
    BluetoothControlFrame c = {0};
    Reset(0U); Ready();
    c.joy_y = 1000;
    ServoControlFrame(c); Step(40U);
    assert(test_servo_pose_calls == 0U && Car_Control_GetState() == CAR_RUNNING);
    c.servo_mode = 3; ServoControlFrame(c); Step(40U);
    assert(test_servo_pose_calls == 0U && Car_Control_GetState() == CAR_RUNNING);
    c.joy_y = 0; c.servo_mode = 0;
    ServoControlFrame(c); Step(40U); ServoControlFrame(c);
    c.servo_mode = 3; ServoControlFrame(c);
    assert(test_servo_pose_calls == 1U);
    test_arm_hold = 1U;
    c.servo_mode = 0; ServoControlFrame(c);
    c.servo_mode = 9; ServoControlFrame(c);
    assert(test_servo_pose_calls == 2U);
    Step(BT_FAILSAFE_TIMEOUT_MS + 1U);
    assert(test_arm_status.state == ARM_CANCELLED);
    ServoControlFrame(c);
    assert(test_servo_pose_calls == 2U);
    c.servo_mode = 0; ServoControlFrame(c); Step(40U); ServoControlFrame(c);
    Step(40U); ServoControlFrame(c);
    assert(Car_Control_GetState() == CAR_READY);
    test_arm_hold = 0U;
    c.servo_mode = 3; ServoControlFrame(c);
    assert(test_servo_pose_calls == 3U);
    puts("PASS servo safety: driving action rejection, disconnect stop and reconnect rearm");
}

static void ServoButtonTap(void)
{
    pe4_low = 1U; Step(21U);
    pe4_low = 0U; Step(21U);
}

static void TestServoButton(void)
{
    BluetoothControlFrame c = {0};
    unsigned i;
    Reset(0U); Ready();
    pe4_low = 1U; Step(20U);
    assert(test_servo_pose_calls == 0U);
    Step(1U); Step(50U);
    assert(test_servo_pose_calls == 1U); /* Reference mode 1 fires once. */
    pe4_low = 0U; Step(21U);
    for (i = 2U; i <= 4U; ++i) {
        ServoControlFrame(c); /* Periodic zero does not affect PE4. */
        ServoButtonTap();
        assert(test_servo_pose_calls == i &&
               test_servo_last_step.position[0] == 1532U);
    }
    c.servo_mode = 2; ServoControlFrame(c);
    ServoButtonTap();
    assert(test_servo_pose_calls == 6U && test_servo_last_step.joint_mask == 0x07U &&
           test_servo_last_step.position[3] == 0U);
    c.servo_mode = 0; ServoControlFrame(c);
    ServoButtonTap();
    assert(test_servo_pose_calls == 7U); /* PE4 always selects mode 1. */
    puts("PASS servo PE4: pull-up, active-low debounce, hold, repeated mode 1 and Bluetooth handoff");
}

static void TestServoButtonSafety(void)
{
    BluetoothControlFrame c = {0};
    Reset(0U); Ready();
    c.servo_mode = 2; ServoControlFrame(c);
    test_tuner_session = 1U;
    ServoButtonTap();
    assert(test_servo_pose_calls == 1U); /* Dual-session ownership wins. */
    test_tuner_session = 0U;
    ServoButtonTap();
    assert(test_servo_pose_calls == 2U && test_servo_last_step.joint_mask == 0x07U &&
           test_servo_last_step.position[3] == 0U);

    Reset(0U); Ready();
    pe4_low = 1U; Step(20U); ++now;
    c = (BluetoothControlFrame){0}; c.servo_mode = 3;
    ServoControlFrame(c);
    assert(test_servo_pose_calls == 1U &&
           test_servo_last_step.position[0] == 1532U); /* PE4 wins. */
    pe4_low = 0U; Step(21U); ServoButtonTap();
    assert(test_servo_pose_calls == 2U && test_servo_last_step.position[0] == 1532U);

    Reset(0U); pe4_low = 1U; ServoRemote_Init(); Ready(); Step(30U);
    assert(test_servo_pose_calls == 0U);
    pe4_low = 0U; Step(21U);
    ServoButtonTap(); ServoButtonTap(); ServoButtonTap();
    assert(test_servo_pose_calls == 3U);
    puts("PASS servo PE4 direct: arm session, simultaneous Bluetooth frame and boot-held button");
}

static void TestServoButtonOffline(void)
{
    BluetoothControlFrame c = {0};
    Reset(0U);
    assert(!Bluetooth_IsConnected() && Car_Control_GetState() == CAR_WAIT_CENTER);
    ServoButtonTap();
    assert(test_servo_pose_calls == 1U &&
           test_servo_last_step.position[0] == 1532U);
    ServoButtonTap(); ServoButtonTap(); ServoButtonTap();
    assert(test_servo_pose_calls == 4U); /* Every new press sends mode 1. */
    assert(test_servo_last_step.joint_mask == 0x07U &&
           test_servo_last_step.position[3] == 0U);

    /* An offline PE4 preset remains active without Bluetooth keepalive. */
    Reset(0U);
    test_arm_hold = 1U;
    ServoButtonTap();
    assert(test_servo_pose_calls == 1U && test_arm_status.state == ARM_RUNNING);
    ServoButtonTap();
    assert(test_servo_pose_calls == 1U); /* Busy arm owns USART3 until completion. */
    Step(BT_FAILSAFE_TIMEOUT_MS + 1U);
    assert(test_arm_status.state == ARM_RUNNING);

    Reset(0U); Ready();
    Step(BT_FAILSAFE_TIMEOUT_MS + 41U);
    assert(!Bluetooth_IsConnected() && Car_Control_GetState() == CAR_LINK_LOST);
    ServoButtonTap();
    assert(test_servo_pose_calls == 1U);
    c.servo_mode = 0; ServoControlFrame(c); Step(40U);
    ServoControlFrame(c); Step(40U); ServoControlFrame(c);
    assert(Car_Control_GetState() == CAR_READY);
    ServoButtonTap();
    assert(test_servo_pose_calls == 2U);

    Reset(0U); Ready();
    c.joy_y = 1000; ServoControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_RUNNING);
    ServoButtonTap();
    assert(test_servo_pose_calls == 1U); /* PE4 bypasses chassis conditions. */
    c.joy_y = 0; ServoControlFrame(c); Step(40U);
    ServoButtonTap();
    assert(test_servo_pose_calls == 2U);
    puts("PASS servo PE4 offline: active low, boot/link-loss operation and no Bluetooth timeout cancellation");
}

static void ServoOneCommand(uint8_t command)
{
    const uint8_t packet[5] = {0xA5U, command, 0U, command, 0x5AU};
    const uint8_t release[7] = {0xA5U,0U,0U,0U,0U,0U,0x5AU};
    Inject(&huart6, packet, sizeof(packet));
    Inject(&huart6, release, sizeof(release));
    Bluetooth_Process(); Bluetooth_Process();
    Loop();
}

static void TestServoOneButton(void)
{
    BluetoothControlFrame c = {0};
    uint32_t control_sequence, rx_tick, short_sequence;
    unsigned before;
    Reset(0U); Ready();
    control_sequence = Bluetooth_GetSequence();
    rx_tick = Bluetooth_GetLastRxTick();
    short_sequence = Bluetooth_GetServoOneSequence();
    before = Count(0xF6U);
    ServoOneCommand(BT_SERVO_ONE_PRESS_CMD);
    assert(Bluetooth_GetServoOneSequence() == short_sequence + 1U &&
           Bluetooth_GetSequence() == control_sequence &&
           Bluetooth_GetLastRxTick() == rx_tick);
    assert(test_servo_pose_calls == 1U &&
           test_servo_last_step.position[0] == 1532U &&
           test_servo_last_step.position[1] == 2219U &&
           test_servo_last_step.position[2] == 1202U &&
           test_servo_last_step.joint_mask == 0x07U &&
           test_servo_last_step.position[3] == 0U);
    assert(Count(0xF6U) == before && Car_Control_GetState() == CAR_READY);
    ServoOneCommand(BT_SERVO_ONE_PRESS_CMD);
    assert(test_servo_pose_calls == 1U &&
           Bluetooth_GetServoOneSequence() == short_sequence + 1U);
    ServoOneCommand(BT_SERVO_ONE_RELEASE_CMD);
    ServoOneCommand(BT_SERVO_ONE_PRESS_CMD);
    assert(test_servo_pose_calls == 2U &&
           Bluetooth_GetServoOneSequence() == short_sequence + 2U);

    ServoOneCommand(BT_SERVO_ONE_RELEASE_CMD);
    ServoControlFrame(c); Step(40U); ServoControlFrame(c);
    c.strafe_right = 1; ServoControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_RUNNING);
    before = test_servo_pose_calls;
    ServoOneCommand(BT_SERVO_ONE_PRESS_CMD); Step(40U);
    assert(test_servo_pose_calls == before && Car_Control_GetState() == CAR_RUNNING);
    c.strafe_right = 0; ServoControlFrame(c); Step(40U); ServoControlFrame(c);
    ServoOneCommand(BT_SERVO_ONE_RELEASE_CMD);
    ServoOneCommand(BT_SERVO_ONE_PRESS_CMD);
    assert(test_servo_pose_calls == before + 1U);
    puts("PASS independent mode-1 A5 button: press/release edge, no car-field update, moving-car inhibit");
}

static void TestServoChassisInterlock(void)
{
    BluetoothControlFrame c = {0};
    unsigned before;
    Reset(0U); Ready();
    before = Count(0xF6U);
    c.servo_mode = 1; c.strafe_right = 1;
    ServoControlFrame(c); Step(50U);
    assert(Count(0xF6U) > before && Car_Control_GetState() == CAR_RUNNING);
    assert(test_servo_pose_calls == 0U); /* Pose needs neutral controls. */
    before = Count(0xF6U);
    c.servo_mode = 0; ServoControlFrame(c); Step(50U);
    assert(Count(0xF6U) > before && Car_Control_GetState() == CAR_RUNNING);
    c.strafe_right = 0; ServoControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_READY);
    before = Count(0xF6U);
    c.strafe_right = 1; ServoControlFrame(c); Step(40U);
    assert(Count(0xF6U) > before && Car_Control_GetState() == CAR_RUNNING);
    puts("PASS servo/chassis independence: unsafe pose is rejected while chassis input remains live");
}

