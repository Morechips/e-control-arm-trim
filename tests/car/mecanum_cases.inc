static void TestMecanumMath(void)
{
    /* Independent expected logical and physical signs in address order 1..4. */
    static const int16_t logical[MEC_ACTION_COUNT][4] = {
        {0,0,0,0}, {30,30,30,30}, {-30,-30,-30,-30},
        {-30,30,-30,30}, {30,-30,30,-30}, {0,30,0,30},
        {30,0,30,0}, {-30,0,-30,0}, {0,-30,0,-30}
    };
    static const int16_t physical[MEC_ACTION_COUNT][4] = {
        {0,0,0,0}, {-30,30,30,-30}, {30,-30,-30,30},
        {30,30,-30,-30}, {-30,-30,30,30}, {0,30,0,-30},
        {-30,0,30,0}, {30,0,-30,0}, {0,-30,0,30}
    };
    unsigned action, i;
    for (action=0;action<MEC_ACTION_COUNT;action++) {
        int16_t vx,vy,w;
        MecanumWheels_t wheels;
        int16_t actual[4];
        Mecanum_Test_Vector((MecanumTestAction_t)action,&vx,&vy,&w);
        Mecanum_Calculate(vx,vy,w,&wheels);
        actual[0]=wheels.m1;actual[1]=wheels.m2;actual[2]=wheels.m3;actual[3]=wheels.m4;
        for(i=0;i<4;i++) {
            assert(actual[i]==logical[action][i]);
            assert(Car_Motor_Physical((uint8_t)(i+1),actual[i])==physical[action][i]);
        }
    }
    {
        MecanumWheels_t w;
        assert(MOTOR_MAX_RPM==500 && MAX_YAW_CORRECTION_RPM==100.0f);
        Mecanum_Calculate(MOTOR_MAX_RPM,0,0,&w);
        assert(w.m1==500 && w.m2==500 && w.m3==500 && w.m4==500);
        Mecanum_Calculate(0,0,(int16_t)MAX_YAW_CORRECTION_RPM,&w);
        assert(w.m1==-100 && w.m2==100 && w.m3==100 && w.m4==-100);
        Mecanum_Calculate(MOTOR_MAX_RPM,MOTOR_MAX_RPM/2,0,&w);
        assert(w.m1==500 && w.m2==166 && w.m3==500 && w.m4==166);
        Mecanum_Calculate(MOTOR_MAX_RPM,MOTOR_MAX_RPM,0,&w);
        assert(w.m1==500 && w.m2==0 && w.m3==500 && w.m4==0);
        Mecanum_Calculate(480,0,(int16_t)MAX_YAW_CORRECTION_RPM,&w);
        assert(w.m1==327 && w.m2==500 && w.m3==500 && w.m4==327);
        Mecanum_Calculate(INT16_MIN,INT16_MAX,INT16_MIN,&w);
        assert(w.m1>=-MOTOR_MAX_RPM && w.m1<=MOTOR_MAX_RPM);
        assert(w.m2>=-MOTOR_MAX_RPM && w.m2<=MOTOR_MAX_RPM);
        assert(w.m3>=-MOTOR_MAX_RPM && w.m3<=MOTOR_MAX_RPM);
        assert(w.m4>=-MOTOR_MAX_RPM && w.m4<=MOTOR_MAX_RPM);
        assert(Car_Motor_Physical(1,INT16_MIN)==MOTOR_MAX_RPM);
        assert(Car_Motor_Physical(4,INT16_MAX)==-MOTOR_MAX_RPM);
        assert(car_motor_map[0].motor==MOTOR_RF && car_motor_map[0].address==1 && car_motor_map[0].roller=='A');
        assert(car_motor_map[1].motor==MOTOR_LF && car_motor_map[1].address==2 && car_motor_map[1].roller=='B');
        assert(car_motor_map[2].motor==MOTOR_LR && car_motor_map[2].address==3 && car_motor_map[2].roller=='A');
        assert(car_motor_map[3].motor==MOTOR_RR && car_motor_map[3].address==4 && car_motor_map[3].roller=='B');
        assert(Emm42_GetTelemetryCapability()==EMM42_TELEMETRY_UNSUPPORTED);
    }
    puts("PASS drive mix: 8-way mecanum vectors / heading mix / physical signs / common scaling / fixed mapping");
}
static void TestDirectionWrappers(void)
{
    HAL_StatusTypeDef (*const directions[])(int16_t) = {up, down, left, right};
    static const int8_t physical_signs[4][4] = {
        {-1, 1, 1, -1}, {1, -1, -1, 1},
        {1, 1, -1, -1}, {-1, -1, 1, 1}
    };
    const int16_t speeds[] = {0, CAR_PD10_FORWARD_RPM, INT16_MAX};
    unsigned direction, sample, wheel;

    for (direction = 0U; direction < 4U; ++direction)
    {
        for (sample = 0U; sample < sizeof(speeds) / sizeof(speeds[0]); ++sample)
        {
            int16_t expected = speeds[sample] > MOTOR_MAX_RPM ? MOTOR_MAX_RPM : speeds[sample];
            Reset(0U);
            assert(directions[direction](speeds[sample]) == HAL_OK);
            assert(directions[direction](speeds[sample]) == HAL_BUSY);
            Step(30U);
            assert(sent_count == 5U && Count(0xF6) == 4U && Count(0xFF) == 1U);
            for (wheel = 0U; wheel < 4U; ++wheel)
            {
                AssertLatestPhysicalSpeed((uint8_t)(wheel + 1U),
                                          (int16_t)(expected * physical_signs[direction][wheel]));
            }
        }
        Reset(0U);
        assert(directions[direction](-1) == HAL_ERROR);
        assert(directions[direction](INT16_MIN) == HAL_ERROR);
        assert(Motor_IsIdle() && sent_count == 0U);
        Motor_ErrorCallback(&huart5);
        assert(directions[direction](CAR_PD10_FORWARD_RPM) == HAL_ERROR);
    }

    Reset(0U);
    assert(up(CAR_PD10_FORWARD_RPM) == HAL_OK);
    assert(brake() == HAL_OK);
    Step(30U);
    assert(sent_count == 4U && Count(0xFE) == 4U);
    Reset(0U);
    assert(right(CAR_PD10_FORWARD_RPM) == HAL_OK);
    Motor_Process();
    assert(sent_count == 1U);
    assert(brake() == HAL_OK);
    Step(30U);
    assert(sent_count == 5U && Count(0xF6) == 1U && Count(0xFF) == 0U);
    for (wheel = 1U; wheel <= 4U; ++wheel)
    {
        const uint8_t stop[] = {(uint8_t)wheel, 0xFE, 0x98, 0, 0x6B};
        assert(sent[wheel].length == sizeof(stop));
        assert(memcmp(sent[wheel].bytes, stop, sizeof(stop)) == 0);
    }
    puts("PASS direction wrappers: physical directions / RPM limits / busy-fault / brake preemption");
}

static void RequestTest(unsigned action)
{
    mecanum_test_action=action; ++mecanum_test_request; Loop();
}
static void TestMecanumSafety(void)
{
    unsigned i, before;
    Reset(0);RequestTest(MEC_FORWARD);assert(!mecanum_test_active);Ready();Step(30);
    assert(!mecanum_test_active && Count(0xF6)==0); /* Denied request never replays. */
    RequestTest(MEC_FORWARD);assert(mecanum_test_active);Step(30);
    assert(Count(0xF6)==4 && Count(0xFF)==1);
    for(i=0;i<4;i++) assert(sent[i].bytes[4]==30 && sent[i].bytes[2]==(i==0 || i==3));
    /* Remain connected while checking the 1 second one-shot deadline. */
    for(i=0;i<9;i++) { Step(100);Frame(0,0,0,0); }
    Step(69);assert(mecanum_test_active);Step(1);assert(!mecanum_test_active);
    Step(30);assert(Count(0xFE)==4);
    RequestTest(MEC_RIGHT);assert(mecanum_test_active);Step(30);
    Frame(1,0,500,500);assert(Car_Control_GetState()==CAR_BRAKE_LOCK && !mecanum_test_active);
    Step(30);before=Count(0xF6);Frame(0,0,500,500);Step(30);
    assert(Car_Control_GetState()==CAR_READY && Count(0xF6)==before);
    RequestTest(MEC_FORWARD_RIGHT);Step(30);Frame(0,0,51,0);Step(30);
    assert(mecanum_test_active); /* JOY cannot cancel or trigger a diagnostic move. */
    RequestTest(MEC_STOP);Step(30);
    Frame(0,0,0,0);RequestTest(MEC_LEFT);Step(30);
    Frame(1,1,0,0);Step(40);assert(Car_Control_GetState()==CAR_OFF && !mecanum_test_active);
    assert(Count(0xF3)==4); /* Test cancellation must preserve queued disable frames. */
    Reset(0);Ready();RequestTest(MEC_BACKWARD);Step(501);Step(30);
    assert(Car_Control_GetState()==CAR_LINK_LOST && !mecanum_test_active);
    Reset(0);Ready();RequestTest(MEC_FORWARD_LEFT);Step(30);HAL_UART_ErrorCallback(&huart5);Loop();Step(30);
    assert(Car_Control_GetState()==CAR_FAULT && !mecanum_test_active);
    Reset(UINT32_MAX-400);Ready();RequestTest(MEC_FORWARD_RIGHT);Step(30);RequestTest(MEC_STOP);Step(30);
    assert(!mecanum_test_active && Count(0xFE)==4);
    before=Count(0xF6);RequestTest(99);Step(30);assert(Count(0xF6)==before && !mecanum_test_active);
    puts("PASS test mode: explicit request / 1s stop / brake lock / center required / disable preservation / link loss / fault / STOP");
}
