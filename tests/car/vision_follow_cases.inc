#include "vision_config.h"

static void PackVisionControl(uint8_t *packet, const BluetoothControlFrame *c)
{
    uint8_t legacy[BT_CONTROL_FRAME_SIZE];
    uint8_t sum = 0U;
    unsigned i;
    PackControl(legacy, c);
    memcpy(packet, legacy, 19U);
    packet[19] = (uint8_t)c->Cam_T;
    packet[20] = (uint8_t)((uint16_t)c->Cam_T >> 8);
    for (i = 1U; i <= 20U; ++i) sum = (uint8_t)(sum + packet[i]);
    packet[21] = sum;
    packet[22] = 0x5AU;
}

static void VisionControlFrame(BluetoothControlFrame c)
{
    uint8_t packet[BT_CONTROL_FRAME_EXT_SIZE];
    PackVisionControl(packet, &c);
    Inject(&huart6, packet, sizeof(packet));
    Bluetooth_Process();
    Loop();
}

static void VisionPress(void)
{
    pe0_low = 1U;
    Step(VISION_BUTTON_DEBOUNCE_MS + 1U);
}

static void VisionRelease(void)
{
    pe0_low = 0U;
    Step(VISION_BUTTON_DEBOUNCE_MS + 1U);
}

static void VisionFrames(uint8_t type, int16_t x, uint8_t valid, unsigned count)
{
    unsigned i;
    for (i = 0U; i < count; ++i) {
        TestMaxiCam_Publish(type, x, 0, valid);
        Loop();
        Step(VISION_SAMPLE_INTERVAL_MS);
    }
}

static void TestVisionPacket(void)
{
    BluetoothControlFrame c = {0};
    uint8_t packet[BT_CONTROL_FRAME_EXT_SIZE];
    uint8_t legacy[BT_CONTROL_FRAME_SIZE];
    uint32_t before;
    Reset(0U);
    c.Cam_T = 1;
    PackVisionControl(packet, &c);
    before = Bluetooth_GetSequence();
    Inject(&huart6, packet, sizeof(packet) - 1U); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == before);
    Inject(&huart6, &packet[22], 1U); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == before + 1U &&
           Bluetooth_GetControl()->frame.Cam_T == 1);
    packet[19] = 2U; packet[21] = 2U;
    Inject(&huart6, packet, sizeof(packet)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == before + 1U);
    PackVisionControl(packet, &c);
    packet[21] ^= 1U;
    Inject(&huart6, packet, sizeof(packet)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == before + 1U);
    PackVisionControl(packet, &c);
    Inject(&huart6, packet, sizeof(packet)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == before + 2U);
    c.Cam_T = 0; PackControl(legacy, &c);
    Inject(&huart6, legacy, sizeof(legacy)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == before + 3U &&
           Bluetooth_GetControl()->frame.Cam_T == 0);
    puts("PASS vision packet: 23-byte Cam_T / split / domain / checksum / legacy 21-byte compatibility");
}

static void TestVisionLocalFollow(void)
{
    unsigned speed, stops;
    Reset(0U);
    Step(600U);
    TestMaxiCam_Publish(DETECT_REDBALL, 30, 500, 1U);
    pe0_low = 1U; Step(10U); pe0_low = 0U; Step(30U);
    assert(Count(0xF6U) == 0U);
    VisionPress(); Step(100U);
    assert(Car_Control_GetState() == CAR_VISION_TRACKING && Count(0xF6U) == 0U);
    VisionFrames(DETECT_BLUEBALL, 21, 1U, 8U);
    assert(Count(0xF6U) == 4U);
    AssertLatestPhysicalSpeed(1U, -8);
    AssertLatestPhysicalSpeed(2U, VISION_FOLLOW_RPM);
    AssertLatestPhysicalSpeed(3U, VISION_FOLLOW_RPM);
    AssertLatestPhysicalSpeed(4U, -VISION_FOLLOW_RPM);
    speed = Count(0xF6U); stops = Count(0xFEU);
    Step(VISION_FRAME_TIMEOUT_MS - VISION_SAMPLE_INTERVAL_MS - 1U);
    assert(Count(0xF6U) == speed && Count(0xFEU) == stops);
    Step(1U); Step(40U);
    assert(Count(0xFEU) == stops + 4U);
    VisionFrames(DETECT_GREANBALL, -8, 1U, 8U);
    AssertLatestPhysicalSpeed(1U, VISION_FOLLOW_RPM);
    AssertLatestPhysicalSpeed(2U, -VISION_FOLLOW_RPM);
    stops = Count(0xFEU);
    TestMaxiCam_Publish(DETECT_UNKNOWN, 100, 0, 0U); Loop(); Step(40U);
    assert(Count(0xFEU) == stops + 4U);
    speed = Count(0xF6U);
    VisionFrames(DETECT_REDBALL, -8, 1U, 8U);
    assert(Count(0xF6U) == speed + 4U);
    stops = Count(0xFEU);
    TestMaxiCam_Publish(DETECT_GREANBALL, 0, 1, 1U); Loop(); Step(40U);
    assert(Count(0xFEU) == stops + 4U);
    speed = Count(0xF6U); Step(40U); assert(Count(0xF6U) == speed);
    TestMaxiCam_Publish(DETECT_UNKNOWN, 100, 0, 0U); Loop();
    assert(Count(0xF6U) == speed);
    TestMaxiCam_Publish(DETECT_REDBALL, 5, 0, 1U); Loop(); Step(20U);
    assert(Count(0xF6U) == speed);
    VisionRelease(); VisionPress();
    assert(Car_Control_GetState() == CAR_OFF);
    puts("PASS vision PE0: debounce / post-entry frames / 8 RPM / deadzone / timeout / auto recovery / second-press exit");
}

static void TestVisionReversalAndFault(void)
{
    unsigned stops, speed, start, i, first_reverse = 0U, last_stop = 0U;
    Reset(0U); Step(600U);
    VisionPress(); Step(100U);
    VisionFrames(DETECT_REDBALL, 21, 1U, 8U);
    stops = Count(0xFEU); speed = Count(0xF6U); start = sent_count;
    TestMaxiCam_Publish(DETECT_REDBALL, -20, 0, 1U); Loop();
    assert(Count(0xF6U) == speed);
    Step(40U);
    assert(Count(0xFEU) == stops + 4U && Count(0xF6U) == speed);
    VisionFrames(DETECT_REDBALL, -20, 1U, 8U);
    assert(Count(0xF6U) == speed + 4U);
    for (i = start; i < sent_count; ++i) {
        if (sent[i].bytes[1] == 0xFEU) last_stop = i;
        if (sent[i].bytes[1] == 0xF6U && first_reverse == 0U) first_reverse = i;
    }
    assert(first_reverse > last_stop);
    AssertLatestPhysicalSpeed(1U, VISION_FOLLOW_RPM);
    HAL_UART_ErrorCallback(&huart5); Loop();
    assert(Car_Control_GetState() == CAR_FAULT);
    speed = Count(0xF6U);
    Step(50U); assert(Count(0xF6U) == speed);
    VisionRelease(); Step(50U);
    assert(Car_Control_GetState() == CAR_OFF && !Motor_HasFault());
    VisionPress(); Step(100U);
    assert(Car_Control_GetState() == CAR_VISION_TRACKING);
    puts("PASS vision safety: reversal stops first / motor fault exits / release and new press rearm");
}

static void TestVisionJitterAndRecentering(void)
{
    unsigned i, speed, stops;
    Reset(0U); Step(600U); VisionPress(); Step(100U);
    for (i = 0U; i < 8U; ++i)
        VisionFrames(DETECT_REDBALL, (i & 1U) ? 21 : -1, 1U, 1U);
    assert(Count(0xF6U) == 0U);
    VisionFrames(DETECT_REDBALL, 21, 1U, 8U);
    assert(Count(0xF6U) == 4U);
    AssertLatestPhysicalSpeed(1U, -8);
    speed = Count(0xF6U); stops = Count(0xFEU);
    TestMaxiCam_Publish(DETECT_REDBALL, 0, 0, 1U); Loop(); Step(40U);
    assert(Count(0xFEU) == stops + 4U);
    VisionFrames(DETECT_REDBALL, 21, 1U, 2U);
    assert(Count(0xF6U) == speed); /* Old center and pre-settle frames cannot decide. */
    VisionFrames(DETECT_REDBALL, 21, 1U, 8U);
    assert(Count(0xF6U) == speed + 4U);
    puts("PASS vision jitter: alternating offsets stay stopped / post-stop off-center frames restart at 8 RPM");
}

static void TestVisionBluetooth(void)
{
    BluetoothControlFrame c = {0};
    unsigned speed;
    Reset(0U); Ready();
    c.forward = 1; ControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_RUNNING);
    c.Cam_T = 1; VisionControlFrame(c); Step(100U);
    assert(Car_Control_GetState() == CAR_VISION_TRACKING);
    TestMaxiCam_Publish(DETECT_REDBALL, 15, 0, 1U); Loop(); Step(40U);
    speed = Count(0xF6U);
    c.forward = 1; VisionControlFrame(c); Step(40U);
    assert(Count(0xF6U) == speed);
    c.forward = 0; c.Cam_T = 0; VisionControlFrame(c);
    assert(Car_Control_GetState() == CAR_WAIT_CENTER);
    Step(40U); assert(Car_Control_GetState() == CAR_WAIT_CENTER);
    VisionControlFrame(c); assert(Car_Control_GetState() == CAR_READY);
    c.Cam_T = 1; VisionControlFrame(c); Step(100U);
    c.stop = 1; VisionControlFrame(c);
    assert(Car_Control_GetState() == CAR_BRAKE_LOCK);
    c.stop = 0; c.Cam_T = 0; VisionControlFrame(c); Step(40U);
    VisionControlFrame(c);
    assert(Car_Control_GetState() == CAR_READY);
    c.Cam_T = 1; VisionControlFrame(c); Step(100U);
    Step(BT_FAILSAFE_TIMEOUT_MS + 1U);
    assert(Car_Control_GetState() == CAR_LINK_LOST);
    Reset(0U); Ready();
    VisionPress(); Step(100U);
    Step(BT_FAILSAFE_TIMEOUT_MS + 1U);
    assert(Car_Control_GetState() == CAR_VISION_TRACKING);
    c.Cam_T = 1; VisionControlFrame(c);
    VisionRelease(); VisionPress();
    assert(Car_Control_GetState() == CAR_WAIT_CENTER);
    VisionControlFrame(c);
    assert(Car_Control_GetState() != CAR_VISION_TRACKING);
    Reset(0U); Ready();
    VisionPress(); Step(100U);
    c.Cam_T = 0; c.stop = 1; VisionControlFrame(c);
    assert(Car_Control_GetState() == CAR_BRAKE_LOCK);
    puts("PASS vision Bluetooth: Cam_T edges / manual suppression / centered rearm / STOP / link loss / latest entry wins");
}

static void TestVisionAlignmentBounds(void)
{
    static const int16_t centered[] = {0, 10, 20};
    static const int16_t outside[] = {-1, 21};
    unsigned i;
    for (i = 0U; i < sizeof(centered) / sizeof(centered[0]); ++i) {
        Reset(0U); Step(600U); VisionPress(); Step(100U);
        VisionFrames(DETECT_REDBALL, centered[i], 1U, 8U);
        assert(Car_Control_GetState() == CAR_VISION_TRACKING);
        assert(Count(0xF6U) == 0U);
    }
    for (i = 0U; i < sizeof(outside) / sizeof(outside[0]); ++i) {
        Reset(0U); Step(600U); VisionPress(); Step(100U);
        VisionFrames(DETECT_REDBALL, outside[i], 1U, 8U);
        assert(Count(0xF6U) == 4U);
        AssertLatestPhysicalSpeed(1U, i == 0U ? VISION_FOLLOW_RPM : -VISION_FOLLOW_RPM);
    }
    puts("PASS vision bounds: x=-1/0/10/20/21 around shifted inclusive deadzone");
}
