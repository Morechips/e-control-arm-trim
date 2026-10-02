#include "laser_config.h"
#include "vision_config.h"

static void PackShotControl(uint8_t *packet, const BluetoothControlFrame *c)
{
    uint8_t vision[BT_CONTROL_FRAME_EXT_SIZE];
    uint8_t sum = 0U;
    unsigned i;
    PackVisionControl(vision, c);
    memcpy(packet, vision, 21U);
    packet[21] = (uint8_t)c->Shot;
    packet[22] = (uint8_t)((uint16_t)c->Shot >> 8);
    for (i = 1U; i <= 22U; ++i) sum = (uint8_t)(sum + packet[i]);
    packet[23] = sum;
    packet[24] = 0x5AU;
}

static void ShotControlFrame(BluetoothControlFrame c)
{
    uint8_t packet[BT_CONTROL_FRAME_SHOT_SIZE];
    PackShotControl(packet, &c);
    Inject(&huart6, packet, sizeof(packet));
    Bluetooth_Process();
    Loop();
}

static void ShotPress(void)
{
    pc1_low = 1U;
    Step(SHOT_BUTTON_DEBOUNCE_MS + 1U);
}

static void ShotRelease(void)
{
    pc1_low = 0U;
    Step(SHOT_BUTTON_DEBOUNCE_MS + 1U);
}

static void ShotFrame(uint8_t type, int16_t x, uint8_t valid)
{
    TestMaxiCam_Publish(type, x, 1234, valid);
    Loop();
}

static void ShotFrames(uint8_t type, int16_t x, uint8_t valid, unsigned count)
{
    unsigned i;
    for (i = 0U; i < count; ++i) {
        ShotFrame(type, x, valid);
        Step(VISION_SAMPLE_INTERVAL_MS);
    }
}

static void TestShotPacket(void)
{
    BluetoothControlFrame c = {0};
    uint8_t packet[BT_CONTROL_FRAME_SHOT_SIZE];
    uint8_t old_vision[BT_CONTROL_FRAME_EXT_SIZE];
    uint8_t legacy[BT_CONTROL_FRAME_SIZE];
    uint32_t sequence;
    Reset(0U);
    c.Shot = 1;
    PackShotControl(packet, &c);
    sequence = Bluetooth_GetSequence();
    Inject(&huart6, packet, sizeof(packet) - 1U); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence);
    Inject(&huart6, &packet[24], 1U); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence + 1U &&
           Bluetooth_GetControl()->frame.Shot == 1);
    c.Shot = 2; PackShotControl(packet, &c);
    Inject(&huart6, packet, sizeof(packet)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence + 1U);
    c.Shot = 1; PackShotControl(packet, &c); packet[23] ^= 1U;
    Inject(&huart6, packet, sizeof(packet)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence + 1U);
    c.Shot = 0; c.Cam_T = 1; PackVisionControl(old_vision, &c);
    Inject(&huart6, old_vision, sizeof(old_vision)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence + 2U &&
           Bluetooth_GetControl()->frame.Shot == 0 &&
           Bluetooth_GetControl()->frame.Cam_T == 1);
    c.Cam_T = 0; PackControl(legacy, &c);
    Inject(&huart6, legacy, sizeof(legacy)); Bluetooth_Process();
    assert(Bluetooth_GetSequence() == sequence + 3U &&
           Bluetooth_GetControl()->frame.Shot == 0 &&
           Bluetooth_GetControl()->frame.Cam_T == 0);
    puts("PASS Shot packet: 25-byte short / split / domain / 23-byte and 21-byte compatibility");
}

static void TestShotBluetoothSwitch(void)
{
    BluetoothControlFrame c = {0};
    Reset(0U); Ready();
    c.Shot = 1;
    ShotControlFrame(c); Step(100U);
    assert(Car_Control_GetState() == CAR_SHOT_ALIGNING &&
           TestMaxiCam_LastMode() == MODE_CMD_AIM &&
           TestMaxiCam_ModeCount() == 1U);
    ShotControlFrame(c);
    assert(TestMaxiCam_ModeCount() == 1U);
    c.Cam_T = 1; ShotControlFrame(c);
    c.Cam_T = 0; ShotControlFrame(c);
    assert(Car_Control_GetState() == CAR_SHOT_ALIGNING &&
           TestMaxiCam_ModeCount() == 1U);

    c.Cam_T = 1; c.Shot = 0; ShotControlFrame(c);
    assert(Car_Control_GetState() == CAR_WAIT_CENTER &&
           !Laser_IsEnabled() && TestMaxiCam_ModeCount() == 1U);
    c.Cam_T = 0; Step(40U); ShotControlFrame(c);
    assert(Car_Control_GetState() == CAR_READY);
    c.forward = 1; ShotControlFrame(c); Step(40U);
    assert(Car_Control_GetState() == CAR_RUNNING);
    c.forward = 0; c.Shot = 1; ShotControlFrame(c); Step(100U);
    assert(Car_Control_GetState() == CAR_SHOT_ALIGNING &&
           TestMaxiCam_ModeCount() == 2U);
    c.Shot = 0; ShotControlFrame(c);
    assert(Car_Control_GetState() == CAR_WAIT_CENTER);

    Reset(0U); Ready();
    ShotPress(); Step(100U);
    assert(Car_Control_GetState() == CAR_SHOT_ALIGNING);
    c = (BluetoothControlFrame){0}; ShotControlFrame(c);
    assert(Car_Control_GetState() == CAR_SHOT_ALIGNING);
    puts("PASS SHOT switch: 1 enters, 0 exits to neutral remote, mode priority and PC1 independence");
}

static void TestShotEntryAndModes(void)
{
    Reset(0U); Step(600U);
    ShotFrame(DETECT_TARGET, 0, 1U);
    pc1_low = 1U; Step(10U); pc1_low = 0U; Step(30U);
    assert(TestMaxiCam_ModeAttempts() == 0U);
    ShotPress(); Step(100U);
    assert(TestMaxiCam_LastMode() == MODE_CMD_AIM &&
           TestMaxiCam_ModeCount() == 1U &&
           Car_Control_GetState() == CAR_SHOT_ALIGNING &&
           !Laser_IsEnabled());
    Step(100U);
    assert(test_laser_enable_count == 0U); /* Pre-entry frame is discarded. */
    ShotRelease(); ShotPress();
    assert(Car_Control_GetState() == CAR_OFF && !Laser_IsEnabled());
    Reset(0U); Step(600U);
    TestMaxiCam_SetSendResult(HAL_BUSY);
    ShotPress(); Step(100U);
    assert(TestMaxiCam_ModeAttempts() == 1U && TestMaxiCam_ModeCount() == 0U &&
           Car_Control_GetState() == CAR_OFF);
    ShotRelease(); TestMaxiCam_SetSendResult(HAL_OK); ShotPress(); Step(100U);
    assert(TestMaxiCam_ModeAttempts() == 2U &&
           Car_Control_GetState() == CAR_SHOT_ALIGNING);
    Reset(0U); Step(600U);
    TestMaxiCam_SetSendResult(HAL_ERROR);
    VisionPress(); Step(40U);
    assert(TestMaxiCam_ModeAttempts() == 1U &&
           TestMaxiCam_ModeCount() == 0U && Car_Control_GetState() == CAR_OFF);
    VisionRelease(); TestMaxiCam_SetSendResult(HAL_OK);
    VisionPress(); Step(100U);
    assert(TestMaxiCam_LastMode() == MODE_CMD_OBJECT &&
           TestMaxiCam_ModeCount() == 1U);
    ShotPress(); Step(100U);
    assert(TestMaxiCam_LastMode() == MODE_CMD_AIM &&
           TestMaxiCam_ModeCount() == 2U &&
           Car_Control_GetState() == CAR_SHOT_ALIGNING);
    VisionRelease(); VisionPress(); Step(100U);
    assert(TestMaxiCam_LastMode() == MODE_CMD_OBJECT &&
           TestMaxiCam_ModeCount() == 3U &&
           Car_Control_GetState() == CAR_VISION_TRACKING);
    puts("PASS Shot entry: PC1 debounce / camera commands / failed send / post-entry frames / mode switch");
}

static void StartShotFiring(void)
{
    Reset(0U); Step(600U);
    ShotPress(); Step(100U);
    ShotFrame(DETECT_TARGET, 0, 1U); Step(10U);
    ShotFrame(DETECT_TARGET, 0, 1U); Step(10U);
    assert(!Laser_IsEnabled());
    ShotFrames(DETECT_TARGET, 0, 1U, 8U);
    assert(Car_Control_GetState() == CAR_SHOT_FIRING &&
           Laser_IsEnabled() && test_laser_enable_count == 1U);
}

static void TestShotAimAndFire(void)
{
    unsigned i, speed, stops;
    Reset(0U); Step(600U); ShotPress(); Step(100U);
    ShotFrame(DETECT_REDBALL, 0, 1U); Step(10U);
    assert(Car_Control_GetState() == CAR_SHOT_ALIGNING && !Laser_IsEnabled());
    ShotFrames(DETECT_TARGET, 11, 1U, 8U);
    AssertLatestPhysicalSpeed(1U, -8);
    speed = Count(0xF6U); stops = Count(0xFEU);
    ShotFrame(DETECT_TARGET, -11, 1U);
    assert(Count(0xF6U) == speed);
    Step(40U);
    assert(Count(0xFEU) == stops + 4U);
    ShotFrames(DETECT_TARGET, -11, 1U, 8U);
    AssertLatestPhysicalSpeed(1U, VISION_FOLLOW_RPM);
    ShotFrame(DETECT_TARGET, 0, 1U); Step(20U);
    ShotFrame(DETECT_TARGET, 0, 1U); Step(20U);
    assert(!Laser_IsEnabled());
    ShotFrames(DETECT_TARGET, 0, 1U, 8U);
    assert(Car_Control_GetState() == CAR_SHOT_FIRING && Laser_IsEnabled());
    for (i = 0U; i < 20U; ++i) {
        ShotFrame(DETECT_TARGET, 0, 1U);
        Step(100U);
    }
    assert(Car_Control_GetState() == CAR_OFF && !Laser_IsEnabled() &&
           test_laser_enable_count == 1U);
    Step(200U);
    assert(test_laser_enable_count == 1U && TestMaxiCam_ModeCount() == 1U);
    puts("PASS Shot aim/fire: target-only / forward-reverse / fresh stable frames / single 2s pulse / auto exit");
}

static void TestShotJitterAndFreshFrames(void)
{
    unsigned i, speed;
    Reset(0U); Step(600U); ShotPress(); Step(100U);
    ShotFrame(DETECT_TARGET, 0, 1U);
    Step(VISION_SETTLE_MS + VISION_FRAME_TIMEOUT_MS);
    assert(Car_Control_GetState() == CAR_SHOT_ALIGNING && !Laser_IsEnabled());
    for (i = 0U; i < 8U; ++i)
        ShotFrames(DETECT_TARGET, (i & 1U) ? 10 : -10, 1U, 1U);
    assert(Car_Control_GetState() == CAR_SHOT_ALIGNING && !Laser_IsEnabled());
    ShotFrames(DETECT_TARGET, 0, 1U, 8U);
    assert(Car_Control_GetState() == CAR_SHOT_FIRING && Laser_IsEnabled());

    Reset(0U); Step(600U); ShotPress(); Step(100U);
    ShotFrames(DETECT_TARGET, 21, 1U, 8U);
    speed = Count(0xF6U);
    assert(speed == 4U && !Laser_IsEnabled());
    ShotFrame(DETECT_TARGET, 0, 1U); Step(40U);
    ShotFrames(DETECT_TARGET, 21, 1U, 8U);
    assert(Count(0xF6U) == speed + 4U && !Laser_IsEnabled());
    puts("PASS Shot stability: no stale or swinging-frame fire / stopped off-center restarts correction");
}

static void TestShotSafety(void)
{
    BluetoothControlFrame c = {0};
    Reset(0U); Step(600U); ShotPress(); Step(100U);
    ShotFrames(DETECT_TARGET, 21, 1U, 8U);
    ShotFrame(DETECT_UNKNOWN, 0, 0U); Step(40U);
    assert(Car_Control_GetState() == CAR_SHOT_ALIGNING && !Laser_IsEnabled());
    ShotFrames(DETECT_TARGET, 0, 1U, 8U);
    assert(Car_Control_GetState() == CAR_SHOT_FIRING && Laser_IsEnabled());
    StartShotFiring();
    ShotFrame(DETECT_TARGET, 21, 1U);
    assert(Car_Control_GetState() == CAR_OFF && !Laser_IsEnabled());
    StartShotFiring();
    ShotFrame(DETECT_UNKNOWN, 0, 0U);
    assert(Car_Control_GetState() == CAR_OFF && !Laser_IsEnabled());
    StartShotFiring();
    Step(VISION_FRAME_TIMEOUT_MS);
    assert(Car_Control_GetState() == CAR_OFF && !Laser_IsEnabled());
    StartShotFiring();
    ShotRelease(); ShotPress();
    assert(Car_Control_GetState() == CAR_OFF && !Laser_IsEnabled());
    StartShotFiring();
    HAL_UART_ErrorCallback(&huart5); Loop();
    assert(Car_Control_GetState() == CAR_FAULT && !Laser_IsEnabled());
    Reset(0U); Ready();
    c.Shot = 1; ShotControlFrame(c); Step(100U);
    ShotFrames(DETECT_TARGET, 0, 1U, 3U);
    ShotControlFrame(c);
    ShotFrames(DETECT_TARGET, 0, 1U, 3U);
    ShotControlFrame(c);
    ShotFrames(DETECT_TARGET, 0, 1U, 2U);
    assert(Laser_IsEnabled());
    c.stop = 1; ShotControlFrame(c);
    assert(Car_Control_GetState() == CAR_BRAKE_LOCK && !Laser_IsEnabled());
    Reset(0U); Ready();
    c.stop = 0; c.Shot = 1; ShotControlFrame(c); Step(100U);
    c.Shot = 0; ShotControlFrame(c);
    assert(Car_Control_GetState() == CAR_WAIT_CENTER && !Laser_IsEnabled());
    Reset(0U); Ready();
    c.stop = 0; c.Shot = 1; ShotControlFrame(c); Step(100U);
    Step(BT_FAILSAFE_TIMEOUT_MS + 1U);
    assert(Car_Control_GetState() == CAR_LINK_LOST && !Laser_IsEnabled());
    puts("PASS Shot safety: deviation / loss / 200ms timeout / PC1 exit / fault / STOP / Bluetooth loss");
}

static void TestShotAlignmentBounds(void)
{
    static const int16_t centered[] = {-10, 0, 10};
    static const int16_t outside[] = {-11, 11};
    unsigned i;
    for (i = 0U; i < sizeof(centered) / sizeof(centered[0]); ++i) {
        Reset(0U); Step(600U); ShotPress(); Step(100U);
        ShotFrames(DETECT_TARGET, centered[i], 1U, 8U);
        assert(Car_Control_GetState() == CAR_SHOT_FIRING && Laser_IsEnabled());
        assert(Count(0xF6U) == 0U);
    }
    for (i = 0U; i < sizeof(outside) / sizeof(outside[0]); ++i) {
        Reset(0U); Step(600U); ShotPress(); Step(100U);
        ShotFrames(DETECT_TARGET, outside[i], 1U, 8U);
        assert(Car_Control_GetState() == CAR_SHOT_ALIGNING && !Laser_IsEnabled());
        assert(Count(0xF6U) == 4U);
        AssertLatestPhysicalSpeed(1U, i == 0U ? VISION_FOLLOW_RPM : -VISION_FOLLOW_RPM);
    }
    puts("PASS shot bounds: center x=0, inclusive -10..10 and stable-frame firing gate");
}
