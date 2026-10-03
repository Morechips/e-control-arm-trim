static void FlushBT(void)
{
    unsigned i;
    for (i=0;i<PID_REPLY_QUEUE_SIZE+2U;i++) { Complete(&huart6); PID_Tuner_Process(); }
}
static void Tune(uint8_t cmd)
{
    const uint8_t packet[]={0xA5,cmd,0,cmd,0x5A};
    Inject(&huart6,packet,sizeof(packet)); Bluetooth_Process();
}
static void ExpectBT(const char *line)
{
    FlushBT(); assert(bt_sent_count && strcmp(bt_sent[bt_sent_count-1U],line)==0);
}
static void TestTunerCommands(void)
{
    unsigned n,i; uint32_t stamp,seq;
    /* Compare stored float defaults; x87 may retain literal excess precision. */
    const HeadingPIDParameters defaults = {HEADING_KP,HEADING_KI,HEADING_KD};
    Reset(0);assert(bt_sent_count==0 && !PID_Tuner_GetStatus()->debug_enabled);
    assert(Heading_GetPID()->kp==defaults.kp && Heading_GetPID()->ki==defaults.ki && Heading_GetPID()->kd==defaults.kd);
    assert(Heading_SetPID(1,0,0)); stamp=Bluetooth_GetLastRxTick();seq=Bluetooth_GetSequence();
    Tune(1);ExpectBT("PID P=1.100 I=0.000 D=0.000 STEP=COARSE\r\n");
    Tune(2);ExpectBT("PID P=1.000 I=0.000 D=0.000 STEP=COARSE\r\n");
    Tune(8);ExpectBT("PID P=1.000 I=0.000 D=0.000 STEP=FINE\r\n");
    Tune(1);ExpectBT("PID P=1.010 I=0.000 D=0.000 STEP=FINE\r\n");
    Tune(3);ExpectBT("PID P=1.010 I=0.001 D=0.000 STEP=FINE\r\n");
    Tune(5);ExpectBT("PID P=1.010 I=0.001 D=0.001 STEP=FINE\r\n");
    Tune(7);Tune(3);ExpectBT("PID P=1.010 I=0.011 D=0.001 STEP=COARSE\r\n");
    Tune(5);ExpectBT("PID P=1.010 I=0.011 D=0.011 STEP=COARSE\r\n");
    Tune(4);Tune(6);ExpectBT("PID P=1.010 I=0.001 D=0.001 STEP=COARSE\r\n");
    n=bt_sent_count;Tune(9);ExpectBT("PID P=1.010 I=0.001 D=0.001 STEP=COARSE\r\n");
    assert(bt_sent_count==n+1U && Bluetooth_GetSequence()==seq && Bluetooth_GetLastRxTick()==stamp);
    for(i=0;i<30;i++) {Tune(2);Tune(4);Tune(6);FlushBT();}
    assert(Heading_GetPID()->kp==0 && Heading_GetPID()->ki==0 && Heading_GetPID()->kd==0);
    assert(!Heading_SetPID(-1,0,0) && !Heading_SetPID(NAN,0,0) && !Heading_SetPID(0,INFINITY,0));
    assert(Heading_SetPID(FLT_MAX,10000000.0f,1));
    Tune(9);FlushBT(); assert(strstr(bt_sent[bt_sent_count-1U],"E+38")!=NULL);
    assert(Heading_GetPID()->kp==FLT_MAX); /* Formatting adds no gain limit. */
    n=bt_sent_count;Tune(0);Tune(12);PID_Tuner_HandleCommand(255);FlushBT();assert(bt_sent_count==n);
    Heading_Init();PID_Tuner_Init();
    assert(Heading_GetPID()->kp==defaults.kp && Heading_GetPID()->ki==defaults.ki && Heading_GetPID()->kd==defaults.kd);
    assert(PID_Tuner_GetStatus()->step_mode==PID_STEP_COARSE && !PID_Tuner_GetStatus()->debug_enabled);
    puts("PASS tuner: all 11 commands / coarse-fine steps / query / negative clamp / RAM defaults / finite gains / integer formatting");
}
static void TestTunerFraming(void)
{
    uint8_t tune[]={0xA5,1,0,1,0x5A},joy[BT_CONTROL_FRAME_SIZE],mixed[BT_CONTROL_FRAME_SIZE+10U],bad[5];
    unsigned i;uint32_t seq;
    Reset(0);assert(Heading_SetPID(1,0,0));seq=Bluetooth_GetSequence();
    for(i=0;i<4;i++){Inject(&huart6,tune+i,1);Bluetooth_Process();assert(Heading_GetPID()->kp==1);}
    Inject(&huart6,tune+4,1);Bluetooth_Process();assert(fabsf(Heading_GetPID()->kp-1.1f)<0.00001f);
    assert(Bluetooth_GetSequence()==seq && !Bluetooth_GetControl()->valid);
    Pack(joy,1,0,1000,-1000);
    memcpy(mixed,tune,5);memcpy(mixed+5,joy,sizeof(joy));memcpy(mixed+5+sizeof(joy),tune,5);
    Inject(&huart6,mixed,sizeof(mixed));Bluetooth_Process();assert(Bluetooth_GetSequence()==seq);
    Bluetooth_Process();assert(Bluetooth_GetSequence()==seq+1 && Bluetooth_GetControl()->frame.stop);
    Bluetooth_Process();assert(fabsf(Heading_GetPID()->kp-1.3f)<0.00001f);
    memcpy(bad,tune,5);bad[3]=2;Inject(&huart6,bad,5);Inject(&huart6,tune,5);Bluetooth_Process();
    assert(fabsf(Heading_GetPID()->kp-1.4f)<0.00001f);
    Inject(&huart6,tune,2);Bluetooth_Process();now+=BT_FRAME_GAP_TIMEOUT_MS+1;
    Inject(&huart6,tune+2,3);Bluetooth_Process();assert(fabsf(Heading_GetPID()->kp-1.4f)<0.00001f);
    Inject(&huart6,tune,5);now+=BT_FAILSAFE_TIMEOUT_MS+1;Bluetooth_Process();
    assert(fabsf(Heading_GetPID()->kp-1.4f)<0.00001f); /* Stale commands discarded. */
    /* Tail/high byte/checksum/unknown command noise cannot change gains. */
    for(i=0;i<3;i++) {
        memcpy(bad,tune,5);bad[i==0 ? 4 : i==1 ? 2 : 1]=0x20;
        Inject(&huart6,bad,5);Bluetooth_Process();now+=101;Bluetooth_Process();
    }
    assert(fabsf(Heading_GetPID()->kp-1.4f)<0.00001f);
    Inject(&huart6,tune,2);Bluetooth_Process();HAL_UART_ErrorCallback(&huart6);Bluetooth_Process();
    Inject(&huart6,tune+2,3);Bluetooth_Process();assert(fabsf(Heading_GetPID()->kp-1.4f)<0.00001f);
    Inject(&huart6,tune,5);Bluetooth_Process();assert(fabsf(Heading_GetPID()->kp-1.5f)<0.00001f);
    /* A full control frame never changes PID gains or command framing. */
    Pack(joy,0,0,1000,1);
    seq=Bluetooth_GetSequence();Inject(&huart6,joy,sizeof(joy));Bluetooth_Process();
    assert(Bluetooth_GetSequence()==seq+1 && fabsf(Heading_GetPID()->kp-1.5f)<0.00001f);
    puts("PASS tuner framing: short ValuePack / split-burst-mixed / checksum / stale-gap / UART error / no joystick refresh");
}
static unsigned CountCTRL(void)
{
    unsigned i,n=0;for(i=0;i<bt_sent_count;i++) if(strncmp(bt_sent[i],"CTRL ",5)==0) n++;
    return n;
}
static void TestTunerTransport(void)
{
    unsigned i,n;uint32_t start;
    Reset(0);assert(Heading_SetPID(0.5f,0,0.05f));Tune(9);next_bt_tx_result=HAL_BUSY;PID_Tuner_Process();
    assert(bt_sent_count==0 && PID_Tuner_GetStatus()->tx_errors==1);
    PID_Tuner_Process();assert(bt_sent_count==1);FlushBT();
    Tune(10);ExpectBT("PID DEBUG=ON\r\n");start=now;
    Step(99);assert(CountCTRL()==0);Step(1);assert(CountCTRL()==1);
    assert(strcmp(bt_sent[bt_sent_count-1],"CTRL T=0.0 A=0.0 E=0.0 O=0.0 P=0.500 I=0.000 D=0.050\r\n")==0);
    Step(400);assert(CountCTRL()==5);
    for(i=0;i<bt_sent_count;i++) if(strncmp(bt_sent[i],"CTRL ",5)==0)
        assert((uint32_t)(bt_ticks[i]-start)%100U==0);
    Tune(11);n=CountCTRL();Step(300);assert(CountCTRL()==n);
    /* Immutable in-flight buffer, bounded response queue, no wait on TX. */
    Reset(0);assert(Heading_SetPID(0.5f,0,0.05f));Tune(9);PID_Tuner_Process();stall_bt=1;
    for(i=0;i<20;i++) Tune(1);
    assert(PID_Tuner_GetStatus()->replies_dropped==12);
    assert(fabsf(Heading_GetPID()->kp-2.5f)<0.0001f);
    assert(memcmp(bt_inflight,bt_sent[0],bt_inflight_length)==0);
    n=bt_sent_count;Step(PID_TX_TIMEOUT_MS);assert(PID_Tuner_GetStatus()->tx_errors==1 && bt_sent_count==n+1);
    stall_bt=0;FlushBT();
    Tune(9);next_bt_tx_result=HAL_ERROR;PID_Tuner_Process();
    assert(PID_Tuner_GetStatus()->tx_errors==2);FlushBT();
    /* RX errors do not discard or corrupt an active TX buffer. */
    Tune(9);PID_Tuner_Process();HAL_UART_ErrorCallback(&huart6);Bluetooth_Process();Complete(&huart6);
    Reset(UINT32_MAX-90U);Tune(10);FlushBT();Step(200);assert(CountCTRL()==2);
    puts("PASS tuner TX: 100ms/debug-off/tick wrap / 9600 wire-time model / IRQ buffer lifetime / priority / busy-error-stall / queue bounds");
}
static void TestTunerSafety(void)
{
    unsigned i,n,before;uint32_t first=0,last=0;bool seen=false;
    Reset(0);Ready();Tune(10);Loop();ButtonFrame(0,0,1);
    for(i=0;i<20;i++){Step(20);ButtonFrame(0,0,1);}
    assert(CountCTRL()>0 && Car_Control_GetState()==CAR_RUNNING);
    for(i=0;i<sent_count;i++) if(sent[i].bytes[0]==1 && sent[i].bytes[1]==0xF6) {
        if(seen) assert((uint32_t)(sent[i].tick-last)>=CAR_CONTROL_PERIOD_MS &&
                       (uint32_t)(sent[i].tick-last)<=CAR_CONTROL_PERIOD_MS+2U);
        else first=sent[i].tick;
        last=sent[i].tick;seen=true;
    }
    assert(seen && (uint32_t)(last-first)>=300U);
    ButtonFrame(1,0,1);assert(Car_Control_GetState()==CAR_BRAKE_LOCK);Step(30);
    before=Count(0xF6);
    for(i=1;i<=11;i++){Tune((uint8_t)i);Loop();}
    ButtonFrame(0,0,1);Step(30);
    assert(Car_Control_GetState()==CAR_BRAKE_LOCK && Count(0xF6)==before);
    assert(!Heading_RequestReference() && Heading_GetStatus()->omega_final==0);
    ButtonFrame(0,0,0);Step(30);assert(Car_Control_GetState()==CAR_READY);
    /* Parameter traffic must not keep the motion link alive. */
    ButtonFrame(0,0,1);Step(30);n=Count(0xFE);
    for(i=0;i<6;i++){Tune(9);Step(100);}
    assert(Car_Control_GetState()==CAR_LINK_LOST && Count(0xFE)==n+4);
    /* Saturated Bluetooth TX never blocks an immediate brake packet. */
    Reset(0);Ready();ButtonFrame(0,0,1);Step(30);stall_bt=1;Tune(9);Loop();
    for(i=0;i<20;i++) Tune(9);
    ButtonFrame(1,0,1);assert(Car_Control_GetState()==CAR_BRAKE_LOCK);
    puts("PASS tuner safety: DEBUG with original 20ms motor period / brake-center latch / no keepalive / TX congestion brake");
}
