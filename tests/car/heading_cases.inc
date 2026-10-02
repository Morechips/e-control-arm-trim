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
    unsigned before;
    const int16_t vx=CAR_HEADING_TEST_MODE ? 3 : 50;
    Reset(0);Ready();ImuSample(20,0);Frame(0,0,0,100);
    assert(Heading_GetStatus()->reference_valid && Heading_GetStatus()->yaw_zero>19.9f);
    assert(Heading_GetStatus()->target==0.0f && Heading_GetStatus()->actual==0.0f);
    Frame(0,0,0,100);ImuSample(29.01f,0);Loop();Step(40);
    assert(Heading_GetStatus()->omega_final==4);
    {
        const int16_t applied = CAR_HEADING_TEST_MODE ? 1 : 4;
        assert(LatestSpeed(1)==-(vx-applied) && LatestSpeed(2)==vx+applied);
        assert(LatestSpeed(3)==vx+applied && LatestSpeed(4)==-(vx-applied));
    }
    /* Stale IMU removes only yaw correction; translation remains controllable. */
    before=Count(0xFE);Step(102);Step(30);
    assert(Heading_GetStatus()->state==HEADING_FAULT && Heading_GetStatus()->omega_final==0);
    assert(Count(0xFE)==before);
    assert(LatestSpeed(1)==-vx && LatestSpeed(2)==vx && LatestSpeed(3)==vx && LatestSpeed(4)==-vx);
    Frame(1,0,0,100);Step(30);ImuSample(29,0);Loop();
    assert(Car_Control_GetState()==CAR_BRAKE_LOCK && Heading_GetStatus()->omega_final==0);
    assert(Car_Control_AdjustTargetYaw(ANGLE_ADJUST_STEP_DEG)==HAL_ERROR && !Heading_RequestReference());
    before=Count(0xF6);Frame(0,0,0,100);Step(30);assert(Count(0xF6)==before);
    Frame(0,0,0,0);Step(30);assert(Car_Control_GetState()!=CAR_BRAKE_LOCK);
    puts("PASS heading integration: software zero / translation+bounded correction / IMU fallback / brake lock");
}
static void TestJoystickCenterBrakesHeading(void)
{
    unsigned speed_frames, brake_frames;
    Reset(0); Ready();
    ImuSample(20.0f,0.0f);
    Frame(0,0,0,600);
    assert(Heading_SetPID(1.0f,2.0f,0.0f));
    Step(20);
    ImuSample(30.0f,0.0f);
    Step(20);
    assert(Heading_GetStatus()->yaw_error < -9.0f);
    assert(Heading_GetIntegralOutput() < 0.0f);
    assert(Heading_GetStatus()->omega_final != 0);
    speed_frames=Count(0xF6);
    brake_frames=Count(0xFE);
    Frame(0,0,0,0);
    Step(40);
    assert(Car_Control_GetState()==CAR_READY);
    assert(Count(0xFE)==brake_frames+4U);
    assert(Count(0xF6)==speed_frames);
    assert(Heading_GetIntegralOutput()==0.0f);
    assert(Heading_GetStatus()->omega_final==0);
    assert(Heading_GetStatus()->omega_correction==0.0f);
    Step(80);
    assert(Count(0xF6)==speed_frames);
    puts("PASS joystick center: heading correction and integral cleared / immediate brake / no spin command");
}
static void TestHeadingMailbox(void)
{
    Reset(0);heading_test_action=1;++heading_test_request;Loop();Ready();ImuSample(20,0);Frame(0,0,0,100);
    assert(Heading_GetStatus()->target==0.0f); /* rejected request not replayed */
    heading_test_action=1;++heading_test_request;Loop();
    assert(Heading_GetStatus()->angle_adjusting && Heading_GetStatus()->target==ANGLE_ADJUST_STEP_DEG);
    assert(Heading_GetStatus()->omega_final==-2);
    Step(10);ImuSample(25,0);Loop();
    assert(Heading_GetStatus()->state==HEADING_HOLD && Heading_GetStatus()->target==ANGLE_ADJUST_STEP_DEG);
    Frame(0,0,0,100);Step(30);
    assert(Heading_GetStatus()->target==ANGLE_ADJUST_STEP_DEG); /* Translation preserves adjusted target. */
    heading_test_action=3;++heading_test_request;Loop();
    assert(Heading_GetStatus()->target==0.0f && Heading_GetStatus()->actual==0.0f);
    heading_test_action=0;++heading_test_request;Loop();
    assert(Car_Control_GetState()==CAR_BRAKE_LOCK && Heading_GetStatus()->omega_final==0);
    puts("PASS heading mailbox: denied request consumed / +/-5 target / target persistence / software re-zero / STOP lock");
}
