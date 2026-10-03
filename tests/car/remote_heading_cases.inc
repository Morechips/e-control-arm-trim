static void TestRemoteMotionGate(void)
{
    BluetoothControlFrame c = {0};
    unsigned before;
    Reset(0U); Ready(); ImuSample(0,0); Loop();
    ImuSample(3,0); c.right_90 = 1; ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_RUNNING);
    assert(RemoteHeading_GetStatus()->phase == REMOTE_ALIGNING);
    AssertLatestPhysicalSpeed(1U,1); AssertLatestPhysicalSpeed(2U,1);
    /* Releasing a requested relative turn retains its one-shot request. */
    c.right_90 = 0; ControlFrame(c);
    ImuSample(0,0); Loop(); Step(60U);
    assert(Car_Control_GetState() == CAR_TURNING);
    AssertLatestPhysicalSpeed(1U,TURN_RIGHT_90_RPM);
    assert(RemoteHeading_GetStatus()->phase == REMOTE_TURNING);
    c.stop = 1; ControlFrame(c); Step(40U);
    assert(RemoteHeading_GetStatus()->direction == REMOTE_FRONT && mock_reset_count == 0U);

    Reset(0U); Ready(); c = (BluetoothControlFrame){0};
    c.right_90 = 1; ControlFrame(c); Step(40U);
    assert(Count(0xF6) == 0U && RemoteHeading_GetStatus()->phase == REMOTE_NO_IMU);
    ImuSample(0,0); ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_READY); /* Rejected edge is not replayed. */
    c.right_90 = 0; c.forward = 1; ControlFrame(c); Step(40U);
    /* A small driving deviation stays mixed with translation; no FE stop. */
    before = Count(0xFE);
    ImuSample(4,0); Loop(); Step(40U);
    assert(RemoteHeading_GetStatus()->phase == REMOTE_DRIFTING);
    assert(Count(0xFE) == before);
    Step(30U); /* Let the already queued four-wheel update finish. */
    assert(RemoteHeading_GetStatus()->correction_rpm > 0);
    AssertLatestPhysicalSpeed(1U,
        (int16_t)(-BLUETOOTH_TEST_MOVE_RPM + RemoteHeading_GetStatus()->correction_rpm));
    AssertLatestPhysicalSpeed(2U,
        (int16_t)(BLUETOOTH_TEST_MOVE_RPM + RemoteHeading_GetStatus()->correction_rpm));
    c.forward = 0; ControlFrame(c);
    ImuSample(0,0); Loop(); Step(40U);
    before = Count(0xF6); Step(40U);
    assert(Car_Control_GetState() == CAR_READY && Count(0xF6) == before);
    puts("PASS remote motion gate: align before turn, no-IMU turn rejected, no stale movement replay");
}

static void TestRemoteHeadingPreemption(void)
{
    unsigned mode, before;
    for (mode = 0U; mode < 4U; ++mode) {
        BluetoothControlFrame c = {0};
        Reset(0U); Ready(); ImuSample(0,0); Loop();
        ImuSample(4,0); Loop(); Step(40U);
        assert(RemoteHeading_GetStatus()->phase == REMOTE_ALIGNING);
        if (mode == 0U) { c.stop = 1; ControlFrame(c); }
        if (mode == 1U) { c.disable = 1; ControlFrame(c); }
        if (mode == 2U) { now += BT_FAILSAFE_TIMEOUT_MS + 1U; Loop(); }
        if (mode == 3U) { HAL_UART_ErrorCallback(&huart5); Loop(); }
        Step(30U); before = Count(0xF6);
        ImuSample(8,0); Loop(); Step(50U);
        assert(Count(0xF6) == before && mock_reset_count == 0U);
    }
    {
        BluetoothControlFrame c = {0};
        Reset(0U); Ready(); ImuSample(0,0); Loop();
        ImuSample(4,0); Loop(); Step(30U);
        c.Cam_T = 1; VisionControlFrame(c); Step(100U);
        assert(Car_Control_GetState() == CAR_VISION_TRACKING);
        before = Count(0xF6); ImuSample(30,0); Loop(); Step(40U);
        assert(Count(0xF6) == before && !RemoteHeading_GetStatus()->reference_valid);
        c.Cam_T = 0; VisionControlFrame(c); Step(40U);
        c.joy_x = 1000; c.joy_y = -1000;
        ImuSample(30,0); VisionControlFrame(c); Step(40U);
        assert(Car_Control_GetState() == CAR_READY);
        assert(RemoteHeading_GetStatus()->direction == REMOTE_FRONT);
        assert(fabsf(RemoteHeading_GetStatus()->reference_yaw - 30) < 0.01f);
    }
    puts("PASS remote heading preemption: STOP, disable, disconnect, motor fault and vision reference rebuild");
}

static void TestMovingDriftConfirmation(void)
{
    BluetoothControlFrame c = {0};
    unsigned i, stops, speeds;
    Reset(0U); Ready(); ImuSample(0,0); Loop();
    c.forward = 1; ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_RUNNING);
    stops = Count(0xFE);
    for (i = 0U; i < 3U; ++i) {
        Step(100U); ImuSample(i == 1U ? 0.0f : 11.0f, 0.0f);
        ControlFrame(c); Step(10U);
        assert(Count(0xFE) == stops);
    }
    assert(RemoteHeading_GetStatus()->phase == REMOTE_DRIFTING);
    for (i = 0U; i < 3U; ++i) {
        Step(90U); ImuSample(11.0f, 0.0f);
        ControlFrame(c); Step(10U);
        if (i < 2U) assert(Count(0xFE) == stops);
    }
    Step(20U); /* Last stop frame is sent after the other three. */
    assert(Count(0xFE) == stops + 4U &&
           RemoteHeading_GetStatus()->phase == REMOTE_ALIGNING);
    speeds = Count(0xF6);
    Step(30U); ImuSample(0,0); ControlFrame(c); Step(100U);
    assert(RemoteHeading_GetStatus()->phase == REMOTE_ALIGNED &&
           Count(0xF6) > speeds);
    AssertLatestPhysicalSpeed(1U, -BLUETOOTH_TEST_MOVE_RPM);
    AssertLatestPhysicalSpeed(2U, BLUETOOTH_TEST_MOVE_RPM);
    puts("PASS moving drift: bump spike keeps translation; 300 ms severe drift stops; alignment resumes held direction");
}
