static void ImuSample(float yaw,float gz)
{
    uint8_t frames[22]={0x55,0x52};
    int16_t raw_gz=(int16_t)(gz*32768.0f/2000.0f),raw_yaw=(int16_t)(yaw*32768.0f/180.0f);
    unsigned i;
    frames[6]=(uint8_t)raw_gz;frames[7]=(uint8_t)((uint16_t)raw_gz>>8);
    frames[11]=0x55;frames[12]=0x53;frames[17]=(uint8_t)raw_yaw;frames[18]=(uint8_t)((uint16_t)raw_yaw>>8);
    for(i=0;i<10;i++){frames[10]=(uint8_t)(frames[10]+frames[i]);frames[21]=(uint8_t)(frames[21]+frames[11+i]);}
    JY61_Parse(frames,22,now);
}
static int16_t LatestSpeed(uint8_t address)
{
    unsigned i=sent_count;
    while(i--) if(sent[i].bytes[0]==address && sent[i].bytes[1]==0xF6) {
        int16_t v=(int16_t)((uint16_t)sent[i].bytes[3]*256U+sent[i].bytes[4]);
        return sent[i].bytes[2] ? (int16_t)-v : v;
    }
    assert(0);return 0;
}
static void TestHeadingIntegration(void)
{
    BluetoothControlFrame c = {0};
    unsigned before;
    float integral;
    Reset(0U); Ready(); ImuSample(20,0); Loop();
    ImuSample(22,0); c.forward = 1; ControlFrame(c); Step(40U);
    assert(RemoteHeading_GetStatus()->phase == REMOTE_ALIGNING);
    assert(LatestSpeed(1) == 1 && LatestSpeed(2) == 1);
    assert(Heading_SetPID(1,2,0));
    ImuSample(23,0); Loop(); integral = Heading_GetIntegralOutput();
    Step(40U); assert(Heading_GetIntegralOutput() == integral);
    ImuSample(20.4f,0); Loop(); Step(40U);
    assert(RemoteHeading_GetStatus()->phase == REMOTE_ALIGNED);
    AssertLatestPhysicalSpeed(1U,-BLUETOOTH_TEST_MOVE_RPM);
    AssertLatestPhysicalSpeed(2U,BLUETOOTH_TEST_MOVE_RPM);
    ImuSample(24,0); Loop(); Step(40U);
    assert(RemoteHeading_GetStatus()->phase == REMOTE_DRIFTING);
    before = Count(0xFE); Step(JY61_TIMEOUT_MS + 1U); Step(40U);
    assert(RemoteHeading_GetStatus()->phase == REMOTE_NO_IMU && Count(0xFE) == before);
    AssertLatestPhysicalSpeed(1U,-BLUETOOTH_TEST_MOVE_RPM);
    AssertLatestPhysicalSpeed(2U,BLUETOOTH_TEST_MOVE_RPM);
    ImuSample(24,0); ControlFrame(c); Step(40U);
    assert(RemoteHeading_GetStatus()->phase == REMOTE_DRIFTING);
    c.forward = 0; c.stop = 1; ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_BRAKE_LOCK);
    assert(Car_Control_AdjustTargetYaw(5) == HAL_ERROR);
    puts("PASS remote heading integration: pre-motion gate, mixed correction while driving, IMU fallback/recovery and STOP");
}

static void TestManualReleaseBrakesHeading(void)
{
    BluetoothControlFrame c = {0};
    unsigned before;
    Reset(0U); Ready(); ImuSample(0,0); Loop();
    c.forward = 1; ControlFrame(c); Step(30U);
    c.forward = 0; ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_READY);
    before = Count(0xF6); ImuSample(2,0); Loop(); Step(40U);
    assert(Car_Control_GetState() == CAR_RUNNING && Count(0xF6) > before);
    assert(RemoteHeading_GetStatus()->phase == REMOTE_ALIGNING);
    ImuSample(0.4f,0); Loop(); Step(40U);
    assert(Car_Control_GetState() == CAR_READY);
    puts("PASS idle heading: button release retains heading; disturbance aligns, deadband brakes");
}

static void TestRemoteReferenceLifetime(void)
{
    BluetoothControlFrame c = {0};
    unsigned i;
    Reset(0U); Ready(); ImuSample(0,0); Loop();
    for (i=1U; i<=4U; ++i) {
        c.right_90 = 1; ControlFrame(c); Step(30U);
        assert(Car_Control_GetState() == CAR_TURNING);
        FinishRemoteTurn(c, -90.0f * (float)i);
        assert(RemoteHeading_GetStatus()->direction == (RemoteDirection_t)(i % 4U));
        c.right_90 = 0; ControlFrame(c); Step(20U);
    }
    assert(mock_reset_count == 0U);
    Step(BT_FAILSAFE_TIMEOUT_MS + 1U);
    assert(Car_Control_GetState() == CAR_LINK_LOST && !RemoteHeading_GetStatus()->reference_valid);
    ImuSample(40,0); ControlFrame(c); Step(40U); ControlFrame(c); Step(40U); ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_READY && RemoteHeading_GetStatus()->direction == REMOTE_FRONT);
    assert(fabsf(RemoteHeading_GetStatus()->reference_yaw - 40) < 0.01f);
    puts("PASS remote direction cycle: four right turns return FRONT, no Z reset, reconnect creates a new reference");
}
