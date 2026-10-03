#include "board_inputs.h"
#include "board_input_config.h"
#include "car_control.h"
#include "uart_driver.h"
#include "route_fsm.h"
#include "turn_right.h"
#include "car_config.h"
#include "laser_config.h"
#include "maxicam.h"
#include "vision_config.h"
#include "mission_fsm.h"
#include "serial_io.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t now;
static uint8_t motor_idle = 1U, motor_fault;
static HAL_StatusTypeDef next_motion_result = HAL_OK;
static TurnRightStatus_t turn_status;
static char events[64];
static int16_t event_rpms[64];
static size_t event_count;
static char logs[160][80];
static size_t log_count;
UART_HandleTypeDef huart4;
GPIO_TypeDef mock_gpioc;
GPIO_TypeDef mock_gpiob, mock_gpiod, mock_gpioe;
static uint32_t laser_pin_mode;
static unsigned laser_low_count;
static unsigned laser_float_count;
static unsigned button_config_count;
static uint8_t mode_bytes[8];
static size_t mode_count, mode_attempts;
static HAL_StatusTypeDef next_mode_result = HAL_OK;

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *gpio)
{
    if (port == GPIOD || port == GPIOE || (port == GPIOC && gpio->Pin == GPIO_PIN_1)) return;
    if (port == GPIOB)
    {
        assert(gpio->Pin == GPIO_PIN_8 && gpio->Mode == GPIO_MODE_INPUT &&
               gpio->Pull == GPIO_PULLUP);
        ++button_config_count;
        return;
    }
    assert(port == GPIOC && gpio->Pin == GPIO_PIN_3 && gpio->Pull == GPIO_NOPULL);
    laser_pin_mode = gpio->Mode;
    if (gpio->Mode == GPIO_MODE_INPUT) ++laser_float_count;
    else assert(gpio->Mode == GPIO_MODE_OUTPUT_OD);
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    assert(port == GPIOC && pin == GPIO_PIN_3 && state == GPIO_PIN_RESET);
    ++laser_low_count;
}

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, const uint8_t *data,
                                    uint16_t size, uint32_t timeout)
{
    HAL_StatusTypeDef result = next_mode_result;
    assert(uart == &huart4 && size == 1U && timeout == MAXICAM_MODE_TX_TIMEOUT_MS);
    ++mode_attempts;
    next_mode_result = HAL_OK;
    if (result == HAL_OK)
    {
        assert(mode_count < sizeof(mode_bytes));
        mode_bytes[mode_count++] = data[0];
    }
    return result;
}

static void ReceiveQRByte(uint8_t byte)
{
    size_t before = event_count;
    bool was_success = qr_success;
    ExecState state = ActionFSM_GetState();
    *huart4.rx = byte;
    MaxiCam_RxCallback(&huart4);
    assert(qr_success == was_success && event_count == before);
    MaxiCam_Process();
    /* Neither the UART callback nor foreground parsing may issue motion. */
    assert(event_count == before && ActionFSM_GetState() == state);
}

void Car_Control_SubmitLocalInput(const CarLocalInput_t *input) { (void)input; }
uint32_t HAL_GetTick(void) { return now; }
uint8_t Debug_CanLog(uint8_t count) { (void)count; return 1U; }
uint8_t Motor_IsIdle(void) { return motor_idle; }
uint8_t Motor_HasFault(void) { return motor_fault; }
void Debug_Log(const char *text)
{
    assert(log_count < sizeof(logs) / sizeof(logs[0]));
    (void)snprintf(logs[log_count++], sizeof(logs[0]), "%s", text);
}
static HAL_StatusTypeDef Motion(char event, int16_t rpm)
{
    HAL_StatusTypeDef result = next_motion_result;
    /* Any speed command during BRAKE / WAIT_STOP / DONE / FINISH is a bug. */
    assert(ActionFSM_GetState() == EXEC_RUN);
    assert(ActionFSM_GetAction() != ACTION_FINISH);
    next_motion_result = HAL_OK;
    if (result == HAL_OK)
    {
        assert(event_count < sizeof(events));
        event_rpms[event_count] = rpm;
        events[event_count++] = event;
    }
    return result;
}
HAL_StatusTypeDef up(int16_t rpm)
{
    assert(rpm == SPEED_SLOW_RPM || rpm == SPEED_MEDIUM_RPM || rpm == SPEED_FULL_RPM);
    return Motion('U', rpm);
}
HAL_StatusTypeDef down(int16_t rpm)
{
    assert(rpm == SPEED_SLOW_RPM);
    return Motion('D', rpm);
}
HAL_StatusTypeDef left(int16_t rpm)
{
    assert(rpm == SPEED_SLOW_RPM || rpm == SPEED_MEDIUM_RPM || rpm == SPEED_FULL_RPM);
    return Motion('L', rpm);
}
HAL_StatusTypeDef right(int16_t rpm)
{
    assert(rpm == SPEED_SLOW_RPM || rpm == SPEED_MEDIUM_RPM || rpm == SPEED_FULL_RPM);
    return Motion('R', rpm);
}
HAL_StatusTypeDef right90(int16_t rpm)
{
    assert(rpm == SPEED_NORMAL_RPM);
    turn_status.state = TURN_RIGHT_TURNING;
    return Motion('T', rpm);
}
HAL_StatusTypeDef left90(int16_t rpm)
{
    assert(rpm == SPEED_NORMAL_RPM);
    turn_status.state = TURN_RIGHT_TURNING;
    return Motion('t', rpm);
}
HAL_StatusTypeDef right180(int16_t rpm)
{
    assert(rpm == SPEED_FULL_RPM);
    turn_status.state = TURN_RIGHT_DONE;
    return HAL_OK;
}
HAL_StatusTypeDef brake(void)
{
    assert(event_count < sizeof(events));
    event_rpms[event_count] = 0;
    events[event_count++] = 'B';
    motor_idle = 0U;
    return HAL_OK;
}
void TurnRight_Process(bool motion_allowed) { assert(motion_allowed); }
void TurnRight_Cancel(void) { turn_status.state = TURN_RIGHT_CANCELLED; }
const TurnRightStatus_t *TurnRight_GetStatus(void) { return &turn_status; }

static void Reset(void)
{
    ActionFSM_Abort();
    now = 0U;
    motor_idle = 1U;
    motor_fault = 0U;
    next_motion_result = HAL_OK;
    memset(&turn_status, 0, sizeof(turn_status));
    memset(events, 0, sizeof(events));
    memset(event_rpms, 0, sizeof(event_rpms));
    event_count = log_count = 0U;
    qr_success = task1_done = task2_done = turn_left_done = false;
    task3_done = task3_object_acquired = final_route_done = false;
    laser_low_count = laser_float_count = 0U;
    button_config_count = 0U;
    laser_pin_mode = UINT32_MAX;
    mode_count = mode_attempts = 0U;
    next_mode_result = HAL_OK;
    mock_gpiob.IDR = GPIO_PIN_8;
    mock_gpioc.IDR = mock_gpiod.IDR = mock_gpioe.IDR = UINT16_MAX;
    Laser_Init(); BoardInputs_Init();
    MissionFSM_Init();
    MaxiCam_Init();
    assert(laser_pin_mode == GPIO_MODE_INPUT && laser_low_count == 0U);
    assert(button_config_count == 1U);
}

static void ReceiveTarget(uint8_t type, int16_t offset_x, int16_t offset_y)
{
    const uint8_t bytes[] = {
        type,
        (uint8_t)((uint16_t)offset_x & 0xFFU),
        (uint8_t)((uint16_t)offset_x >> 8),
        (uint8_t)((uint16_t)offset_y & 0xFFU),
        (uint8_t)((uint16_t)offset_y >> 8)
    };
    size_t i, before = event_count;
    ExecState state = ActionFSM_GetState();
    for (i = 0U; i < sizeof(bytes); ++i)
    {
        *huart4.rx = bytes[i];
        MaxiCam_RxCallback(&huart4);
        MaxiCam_Process();
    }
    assert(event_count == before && ActionFSM_GetState() == state);
}

static size_t CountLog(const char *expected)
{
    size_t i, count = 0U;
    for (i = 0U; i < log_count; ++i)
        if (strcmp(logs[i], expected) == 0) ++count;
    return count;
}

static void StartMotion(ActionType expected, char event)
{
    size_t before = event_count;
    assert(route[route_index].action == expected && ActionFSM_GetState() == EXEC_ENTER);
    RouteFSM_Update();
    assert(ActionFSM_GetState() == EXEC_RUN);
    RouteFSM_Update();
    assert(ActionFSM_GetState() == EXEC_RUN);
    assert(event_count == before + 1U && events[before] == event);
}

static void SettleBrake(ExecState expected_after)
{
    size_t before = event_count;
    assert(ActionFSM_GetState() == EXEC_BRAKE);
    RouteFSM_Update();
    assert(ActionFSM_GetState() == EXEC_WAIT_STOP);
    RouteFSM_Update();
    assert(ActionFSM_GetState() == EXEC_WAIT_STOP);
    motor_idle = 1U;
    RouteFSM_Update();
    now += ACTION_STOP_SETTLE_MS - 1U;
    RouteFSM_Update();
    assert(ActionFSM_GetState() == EXEC_WAIT_STOP);
    now += 1U;
    RouteFSM_Update();
    if (expected_after == EXEC_DONE)
        assert(ActionFSM_GetState() == EXEC_ENTER || RouteFSM_GetState() == ROUTE_FINISHED);
    else assert(ActionFSM_GetState() == expected_after);
    assert(event_count == before + 1U && events[before] == 'B');
}

static void FinishPathStep(size_t index)
{
    size_t before = event_count;
    RouteFSM_NotifyActionEnd();
    RouteFSM_Update();
    assert(route_index == index && ActionFSM_GetState() == EXEC_BRAKE);
    assert(event_count == before);
    SettleBrake(EXEC_DONE);
    assert(route_index == index + 1U && events[before] == 'B');
}

static void TestCompleteRoute(void)
{
    static const ActionType expected_route[] = {
        ACTION_FORWARD, ACTION_TURN_RIGHT, ACTION_WAIT_QR, ACTION_QR_LEFT_TURN,
        ACTION_TRANSLATE_LEFT, ACTION_FORWARD,
        ACTION_TRANSLATE_LEFT, ACTION_TURN_RIGHT, ACTION_TASK_1,
        ACTION_TURN_LEFT, ACTION_FORWARD_TO_TASK_2, ACTION_TASK_2,
        ACTION_FORWARD_AFTER_TASK_2, ACTION_TURN_RIGHT, ACTION_TASK_3,
        ACTION_RETURN_TO_FINISH, ACTION_FINISH
    };
    VisionTask_t task = {VISION_COLOR_RED, VISION_COLOR_GREEN, VISION_SHAPE_CONE};
    size_t i, before;

    Reset();
    assert(route_length == sizeof(expected_route) / sizeof(expected_route[0]));
    for (i = 0U; i < route_length; ++i) assert(route[i].action == expected_route[i]);
    RouteFSM_Init(); RouteFSM_Update();
    assert(RouteFSM_GetState() == ROUTE_RUNNING && route_index == 0U);
    MissionFSM_SetTaskData(&task);

    StartMotion(ACTION_FORWARD, 'U');
    RouteFSM_Update(); RouteFSM_Update();
    assert(!qr_success && route_index == 0U && ActionFSM_GetState() == EXEC_RUN);
    assert(event_count == 1U && events[0] == 'U');
    assert(event_rpms[0] == SPEED_NORMAL_RPM);
    assert(!RouteFSM_QRStageDone());
    FinishPathStep(0U);
    assert(route_index == 1U && !RouteFSM_QRStageDone());
    StartMotion(ACTION_TURN_RIGHT, 'T');
    assert(event_rpms[event_count - 1U] == SPEED_FULL_RPM);
    RouteFSM_NotifyActionEnd(); RouteFSM_Update();
    assert(route_index == 1U && ActionFSM_GetState() == EXEC_RUN);
    turn_status.state = TURN_RIGHT_DONE;
    RouteFSM_Update();
    assert(ActionFSM_GetState() == EXEC_BRAKE && route_index == 1U);
    SettleBrake(EXEC_DONE);
    assert(route_index == 2U && !RouteFSM_QRStageDone());
    RouteFSM_Update();
    assert(ActionFSM_GetState() == EXEC_RUN && motor_idle);
    before = event_count;
    ReceiveQRByte(0x80U);
    RouteFSM_Update();
    assert(ActionFSM_GetState() == EXEC_BRAKE && event_count == before);
    SettleBrake(EXEC_DONE);
    assert(route_index == 3U && !RouteFSM_QRStageDone());
    StartMotion(ACTION_QR_LEFT_TURN, 't');
    RouteFSM_Update();
    assert(route_index == 3U && !RouteFSM_QRStageDone());
    turn_status.state = TURN_RIGHT_DONE;
    RouteFSM_Update();
    assert(ActionFSM_GetState() == EXEC_BRAKE && !RouteFSM_QRStageDone());
    SettleBrake(EXEC_DONE);
    assert(route_index == 4U && RouteFSM_QRStageDone());

    StartMotion(ACTION_TRANSLATE_LEFT, 'L'); FinishPathStep(4U);
    StartMotion(ACTION_FORWARD, 'U'); FinishPathStep(5U);
    StartMotion(ACTION_TRANSLATE_LEFT, 'L'); FinishPathStep(6U);
    StartMotion(ACTION_TURN_RIGHT, 'T');
    assert(event_rpms[event_count - 1U] == SPEED_FULL_RPM);
    assert(route_index == 7U && MissionFSM_GetState() != TASK1_SEARCH_OBJECT);
    RouteFSM_NotifyActionEnd(); RouteFSM_Update();
    assert(route_index == 7U && ActionFSM_GetState() == EXEC_RUN);
    turn_status.state = TURN_RIGHT_DONE;
    RouteFSM_Update();
    assert(route_index == 7U && ActionFSM_GetState() == EXEC_BRAKE &&
           MissionFSM_GetState() != TASK1_SEARCH_OBJECT);
    SettleBrake(EXEC_DONE);

    assert(route_index == 8U && route[route_index].action == ACTION_TASK_1);
    assert(MissionFSM_GetState() == TASK1_SEARCH_OBJECT);
    RouteFSM_Update(); /* Enter task search. */
    RouteFSM_Update(); /* First search command. */
    assert(!task1_done && MissionFSM_GetState() == TASK1_SEARCH_OBJECT);
    assert(events[event_count - 1U] == 'R' && event_rpms[event_count - 1U] == SPEED_SLOW_RPM);
    assert(CountLog("[ROUTE] FINISHED\r\n") == 0U);
    assert(CountLog("[QR] stationary scan\r\n") == 1U);
    assert(mode_count == 0U);
}

static void FeedMissionDeadzone(uint8_t type);

static void CompleteRouteTask1(void)
{
    assert(route_index == 8U && ActionFSM_GetState() == EXEC_RUN);
    ReceiveTarget(DETECT_REDBALL, 0, 0);
    RouteFSM_Update();
    assert(MissionFSM_GetState() == TASK1_ALIGN_OBJECT);
    motor_idle = 1U;
    FeedMissionDeadzone(DETECT_REDBALL);
    assert(MissionFSM_GetState() == TASK1_PICK);
    MissionFSM_Update();
    Servo_NotifyDone();
    MissionFSM_Update();
    MissionFSM_Update();
    assert(MissionFSM_GetState() == TASK1_SEARCH_BUCKET);
    MissionFSM_Update();
    ReceiveTarget(DETECT_BARREL, 0, 0);
    MissionFSM_Update();
    assert(MissionFSM_GetState() == TASK1_ALIGN_BUCKET);
    motor_idle = 1U;
    FeedMissionDeadzone(DETECT_BARREL);
    MissionFSM_Update();
    Servo_NotifyDone();
    MissionFSM_Update();
    assert(MissionFSM_GetState() == TASK1_DONE && task1_done);
}

static void TestRouteModeSwitches(void)
{
    unsigned i;
    TestCompleteRoute();
    CompleteRouteTask1();
    /* Task 1 records an AIM request and advances the route immediately: the
     * camera only accepts the byte after a QR notification, so nothing is
     * transmitted yet and no error can be latched. Clear the latch that the
     * route's own QR stage already set, to exercise the deferral branch. */
    MaxiCam_ResetQrNotice();
    RouteFSM_Update();
    assert(route_index == 9U && mode_attempts == 0U && mode_count == 0U);
    assert(MaxiCam_ModePending() && RouteFSM_GetState() != ROUTE_ERROR);
    MaxiCam_Process();
    assert(mode_attempts == 0U && mode_count == 0U && MaxiCam_ModePending());

    /* One QR notice releases the request without waiting for the retry gap. */
    ReceiveQRByte(QR_SUCCESS_CODE);
    assert(MaxiCam_QrNotified());
    MaxiCam_Process();
    assert(mode_count == 1U && mode_bytes[0] == MODE_CMD_AIM);
    assert(!MaxiCam_ModePending());

    /* ACTION_TURN_LEFT is the brake-and-wait placeholder: it reaches EXEC_BRAKE
     * and stays there until the completion event, because the motor is busy. */
    RouteFSM_Update();
    RouteFSM_Update();
    ActionFSM_SetTurnLeftDone(true);
    RouteFSM_Update();
    SettleBrake(EXEC_DONE);
    assert(route_index == 10U && mode_count == 1U);
    StartMotion(ACTION_FORWARD_TO_TASK_2, 'U');
    FinishPathStep(10U);
    assert(route_index == 11U && mode_count == 1U);
    RouteFSM_Update();
    RouteFSM_Update();
    ReceiveTarget(DETECT_TARGET, 0, 0);
    RouteFSM_Update();
    assert(MissionFSM_GetState() == TASK2_ALIGN);
    motor_idle = 1U;
    ReceiveTarget(DETECT_TARGET, 0, 0);
    RouteFSM_Update();
    assert(MissionFSM_GetState() == TASK2_ALIGN && !Laser_IsEnabled());
    motor_idle = 1U;
    RouteFSM_Update();
    now += ACTION_STOP_SETTLE_MS;
    RouteFSM_Update();
    assert(MissionFSM_GetState() == TASK2_CONFIRM);
    for (i = 0U; i < MAXICAM_ALIGN_CONFIRM_FRAMES; ++i)
    {
        ReceiveTarget(DETECT_TARGET, 0, (int16_t)i);
        RouteFSM_Update();
    }
    assert(MissionFSM_GetState() == TASK2_LASER_ON && Laser_IsEnabled());
    next_mode_result = HAL_BUSY;
    now += LASER_HOLD_MS;
    RouteFSM_Update(); /* ACTION_TASK_2 completes and requests OBJECT mode */
    assert(route_index == 12U && task2_done && !Laser_IsEnabled());
    MaxiCam_Process(); /* first attempt fails */
    assert(mode_count == 1U && MaxiCam_ModePending());
    assert(RouteFSM_GetState() != ROUTE_ERROR);
    next_mode_result = HAL_OK;
    now += MAXICAM_MODE_RETRY_GAP_MS;
    MaxiCam_Process(); /* retry succeeds */
    assert(mode_count == 2U && mode_bytes[1] == MODE_CMD_OBJECT && !MaxiCam_ModePending());
    assert(laser_pin_mode == GPIO_MODE_INPUT);

    /* A transmit failure must never latch ROUTE_ERROR: the request simply stays
     * pending for a later attempt. */
    for (i = 0U; i < 2U; ++i)
    {
        TestCompleteRoute();
        CompleteRouteTask1();
        next_mode_result = i == 0U ? HAL_ERROR : HAL_TIMEOUT;
        MaxiCam_ResetQrNotice(); /* no notice yet: still deferred */
        RouteFSM_Update();
        assert(RouteFSM_GetState() != ROUTE_ERROR && mode_count == 0U);
        assert(MaxiCam_ModePending());
        assert(laser_pin_mode == GPIO_MODE_INPUT);
    }
}

/* The deferral/retry timing is exercised on its own so that the extra elapsed
 * time cannot disturb the mission FSM's settle timers. */
static void TestCameraModeRetry(void)
{
    TestCompleteRoute();
    CompleteRouteTask1();
    MaxiCam_ResetQrNotice();
    next_mode_result = HAL_BUSY;
    RouteFSM_Update(); /* task 1 completes and requests AIM */
    assert(route_index == 9U && mode_attempts == 0U);
    MaxiCam_Process();
    assert(mode_attempts == 0U && MaxiCam_ModePending()); /* deferred */
    ReceiveQRByte(QR_SUCCESS_CODE);
    MaxiCam_Process(); /* first attempt fails */
    assert(mode_attempts == 1U && mode_count == 0U && MaxiCam_ModePending());
    assert(RouteFSM_GetState() != ROUTE_ERROR);
    next_mode_result = HAL_OK;
    now += MAXICAM_MODE_RETRY_GAP_MS;
    MaxiCam_Process(); /* retry succeeds */
    assert(mode_attempts == 2U && mode_count == 1U && mode_bytes[0] == MODE_CMD_AIM);
    assert(!MaxiCam_ModePending() && RouteFSM_GetState() != ROUTE_ERROR);
}

static void TestBusyAndError(void)
{
    size_t before;
    Reset(); RouteFSM_Init(); RouteFSM_Update(); RouteFSM_Update();
    next_motion_result = HAL_BUSY;
    RouteFSM_Update();
    assert(ActionFSM_GetState() == EXEC_RUN && event_count == 0U);
    RouteFSM_Update(); assert(events[0] == 'U');
    motor_fault = 1U; RouteFSM_Update();
    assert(RouteFSM_GetState() == ROUTE_ERROR && route_index == 0U);
    assert(events[event_count - 1U] == 'B');
    before = event_count;
    RouteFSM_Update(); RouteFSM_Update();
    assert(event_count == before && route_index == 0U);
}

static void ReachQRRightTurn(void)
{
    Reset(); RouteFSM_Init(); RouteFSM_Update();
    StartMotion(ACTION_FORWARD, 'U');
    RouteFSM_Update(); RouteFSM_Update();
    assert(route_index == 0U && event_count == 1U);
    FinishPathStep(0U);
    assert(route_index == 1U && !qr_success);
}

static void ReachQRScan(void)
{
    ReachQRRightTurn();
    StartMotion(ACTION_TURN_RIGHT, 'T');
    RouteFSM_Update();
    assert(route_index == 1U && ActionFSM_GetState() == EXEC_RUN);
    turn_status.state = TURN_RIGHT_DONE;
    RouteFSM_Update();
    SettleBrake(EXEC_DONE);
    assert(route_index == 2U && !RouteFSM_QRStageDone());
    RouteFSM_Update();
    assert(ActionFSM_GetState() == EXEC_RUN && motor_idle);
}

static void TestQRSearch(void)
{
    size_t before, i;
    ReachQRScan();
    before = event_count;
    ReceiveQRByte('8'); ReceiveQRByte('0'); ReceiveQRByte(0x81U);
    ReceiveTarget(DETECT_REDBALL, 0x0180, -10);
    ReceiveTarget(DETECT_UNKNOWN, 0, 0);
    for (i = 0U; i < 20U; ++i)
    {
        RouteFSM_NotifyActionEnd();
        RouteFSM_Update();
    }
    assert(!qr_success && event_count == before && route_index == 2U);
    assert(ActionFSM_GetState() == EXEC_RUN);
    assert(CountLog("[QR] stationary scan\r\n") == 1U);
    ReceiveQRByte(0x80U); ReceiveQRByte(0x80U);
    assert(qr_success && event_count == before);
    RouteFSM_Update();
    assert(ActionFSM_GetState() == EXEC_BRAKE && route_index == 2U);
    assert(event_count == before);
    assert(CountLog("[QR] success -> BRAKE\r\n") == 1U);
    now = UINT32_MAX - 50U; /* Stop hold must survive tick wrap. */
    SettleBrake(EXEC_DONE);
    assert(route_index == 3U && qr_success && !RouteFSM_QRStageDone());
    assert(event_count == before + 1U && events[event_count - 1U] == 'B');
    assert(CountLog("[QR] BRAKE -> WAIT_STOP\r\n") == 1U);
    assert(CountLog("[QR] DONE\r\n") == 1U);
    StartMotion(ACTION_QR_LEFT_TURN, 't');
    assert(!RouteFSM_QRStageDone());
    RouteFSM_Update();
    assert(route_index == 3U && ActionFSM_GetState() == EXEC_RUN);
    turn_status.state = TURN_RIGHT_DONE;
    RouteFSM_Update();
    assert(ActionFSM_GetState() == EXEC_BRAKE && !RouteFSM_QRStageDone());
    SettleBrake(EXEC_DONE);
    assert(route_index == 4U && RouteFSM_QRStageDone());
    StartMotion(ACTION_TRANSLATE_LEFT, 'L');
    assert(event_rpms[event_count - 1U] == SPEED_NORMAL_RPM);
}

static void TestQRBeforeSearch(void)
{
    Reset(); RouteFSM_Init(); RouteFSM_Update();
    StartMotion(ACTION_FORWARD, 'U');
    ReceiveQRByte(0x80U);
    assert(qr_success);
    RouteFSM_Update();
    assert(route_index == 0U && ActionFSM_GetState() == EXEC_RUN);
    FinishPathStep(0U);
    StartMotion(ACTION_TURN_RIGHT, 'T');
    turn_status.state = TURN_RIGHT_DONE;
    RouteFSM_Update();
    SettleBrake(EXEC_DONE);
    assert(route_index == 2U && !qr_success); /* Pre-scan notice is stale. */
    RouteFSM_Update(); RouteFSM_Update();
    assert(route_index == 2U && ActionFSM_GetState() == EXEC_RUN);
    ReceiveQRByte(0x80U);
    RouteFSM_Update();
    assert(ActionFSM_GetState() == EXEC_BRAKE);
}

static void TestQRSearchBusyFaultAndRestart(void)
{
    size_t before;
    ReachQRRightTurn(); RouteFSM_Update();
    next_motion_result = HAL_BUSY; RouteFSM_Update();
    assert(event_count == 2U && route_index == 1U);
    RouteFSM_Update();
    assert(event_count == 3U && events[2] == 'T');
    RouteFSM_Update();
    assert(event_count == 3U); /* Accepted turn command is never resent. */
    RouteFSM_Init();
    assert(!qr_success && !RouteFSM_QRStageDone() &&
           RouteFSM_GetState() == ROUTE_INIT && route_index == 0U);
    assert(event_count == 4U && events[3] == 'B');
    RouteFSM_Update();
    assert(!qr_success && ActionFSM_GetAction() == ACTION_FORWARD);

    ReachQRRightTurn(); RouteFSM_Update();
    next_motion_result = HAL_ERROR;
    RouteFSM_Update();
    assert(RouteFSM_GetState() == ROUTE_ERROR && route_index == 1U);
    assert(events[event_count - 1U] == 'B');
    before = event_count;
    RouteFSM_Update(); RouteFSM_Update();
    assert(event_count == before);

    ReachQRScan();
    motor_fault = 1U; RouteFSM_Update();
    assert(RouteFSM_GetState() == ROUTE_ERROR && route_index == 2U);
}

static void TestQRSearchStopIsolation(void)
{
    size_t before;
    ReachQRScan();
    ReceiveQRByte(0x80U); RouteFSM_Update(); RouteFSM_Update();
    assert(ActionFSM_GetState() == EXEC_WAIT_STOP);
    before = event_count;
    now += 1000U; RouteFSM_Update(); /* Busy queue is never considered stopped. */
    assert(route_index == 2U && event_count == before);
    motor_idle = 1U; RouteFSM_Update();
    now += ACTION_STOP_SETTLE_MS - 1U; RouteFSM_Update();
    motor_idle = 0U; RouteFSM_Update(); /* Queue busy again resets the hold. */
    motor_idle = 1U; RouteFSM_Update();
    now += ACTION_STOP_SETTLE_MS - 1U; RouteFSM_Update();
    assert(route_index == 2U && ActionFSM_GetState() == EXEC_WAIT_STOP);
    ++now; RouteFSM_Update();
    assert(route_index == 3U && event_count == before);
}

static void BeginAlignment(void)
{
    Reset();
    ActionFSM_Init(ACTION_ALIGN_TARGET);
    assert(ActionFSM_GetState() == EXEC_ENTER);
    assert(ActionFSM_GetAlignState() == ALIGN_IDLE);
    ActionFSM_Update();
    assert(ActionFSM_GetState() == EXEC_RUN);
    assert(ActionFSM_GetAlignState() == ALIGN_TRACKING);
}

static void SettleAlignmentBrake(ExecState expected_after)
{
    size_t before = event_count;
    assert(ActionFSM_GetState() == EXEC_BRAKE);
    ActionFSM_Update();
    assert(ActionFSM_GetState() == EXEC_WAIT_STOP);
    assert(ActionFSM_GetAlignState() == ALIGN_WAIT_STOP);
    assert(event_count == before + 1U && events[before] == 'B');
    ActionFSM_Update();
    assert(ActionFSM_GetState() == EXEC_WAIT_STOP);
    motor_idle = 1U;
    ActionFSM_Update();
    now += ACTION_STOP_SETTLE_MS - 1U;
    ActionFSM_Update();
    assert(ActionFSM_GetState() == EXEC_WAIT_STOP);
    ++now;
    ActionFSM_Update();
    assert(ActionFSM_GetState() == expected_after);
}

static void TestMaxiCamAlignmentDirections(void)
{
    size_t before;
    BeginAlignment();

    next_motion_result = HAL_BUSY;
    ReceiveTarget(DETECT_REDBALL, 21, -300);
    ActionFSM_Update();
    assert(event_count == 0U);
    ActionFSM_Update(); /* Retry the same unconsumed frame after transport busy. */
    assert(event_count == 1U && events[0] == 'U');
    assert(event_rpms[0] == SPEED_PRECISE_RPM);

    /* offset_y changes cannot change or repeat the X-selected direction. */
    before = event_count;
    ReceiveTarget(DETECT_REDBALL, 21, 300);
    ActionFSM_Update();
    assert(event_count == before);

    ReceiveTarget(DETECT_REDBALL, -1, 32767);
    ActionFSM_Update();
    assert(event_count == 2U && events[1] == 'D');
    assert(event_rpms[1] == SPEED_PRECISE_RPM);
}

static void TestMaxiCamAlignmentConfirmation(void)
{
    static const int16_t first_run[] = {0, 10, 21};
    static const int16_t confirmed[] = {20, 10, 0};
    size_t i, before;

    BeginAlignment();
    for (i = 0U; i < sizeof(first_run) / sizeof(first_run[0]); ++i)
    {
        ReceiveTarget(DETECT_BLUEBALL, first_run[i], (int16_t)(100 + (int)i));
        ActionFSM_Update();
    }
    assert(ActionFSM_GetState() == EXEC_RUN); /* x=21 reset the first 3 frames. */
    assert(event_count == 1U && events[0] == 'U');

    for (i = 0U; i < sizeof(confirmed) / sizeof(confirmed[0]); ++i)
    {
        before = event_count;
        ReceiveTarget(DETECT_BLUEBALL, confirmed[i], (int16_t)(-200 - (int)i));
        ActionFSM_Update();
        assert(event_count == before);
        if (i + 1U < sizeof(confirmed) / sizeof(confirmed[0]))
            assert(ActionFSM_GetState() == EXEC_RUN);
    }
    assert(ActionFSM_GetState() == EXEC_BRAKE);
    assert(ActionFSM_GetAlignState() == ALIGN_BRAKE);

    /* New frames after alignment cannot restart motion before or during stop. */
    before = event_count;
    ReceiveTarget(DETECT_HOSTAGE_1, -100, 1234);
    assert(event_count == before && ActionFSM_GetState() == EXEC_BRAKE);
    ActionFSM_Update();
    assert(event_count == before + 1U && events[before] == 'B');
    ReceiveTarget(DETECT_HOSTAGE_1, 100, -1234);
    ActionFSM_Update();
    assert(event_count == before + 1U && ActionFSM_GetState() == EXEC_WAIT_STOP);
    motor_idle = 1U;
    ActionFSM_Update();
    now += ACTION_STOP_SETTLE_MS;
    ActionFSM_Update();
    assert(ActionFSM_GetState() == EXEC_DONE);
    assert(ActionFSM_GetAlignState() == ALIGN_DONE);
    assert(event_count == before + 1U);
}

static void TestMaxiCamAlignmentTargetLoss(void)
{
    size_t i, before;
    BeginAlignment();

    ReceiveTarget(DETECT_HOSTAGE_2, 5, 10);
    ActionFSM_Update();
    ReceiveTarget(DETECT_HOSTAGE_2, 10, -10);
    ActionFSM_Update();
    ReceiveTarget(DETECT_UNKNOWN, 0, 999);
    ActionFSM_Update();
    assert(ActionFSM_GetState() == EXEC_RUN && event_count == 0U);

    /* Invalid target reset the first two confirmations: two fresh frames are not done. */
    for (i = 0U; i < 2U; ++i)
    {
        ReceiveTarget(DETECT_HOSTAGE_2, 0, (int16_t)i);
        ActionFSM_Update();
        assert(ActionFSM_GetState() == EXEC_RUN);
    }
    ReceiveTarget(DETECT_HOSTAGE_2, 0, 44);
    ActionFSM_Update();
    assert(ActionFSM_GetState() == EXEC_BRAKE);

    BeginAlignment();
    ReceiveTarget(DETECT_REDBALL, 21, 0);
    ActionFSM_Update();
    assert(event_count == 1U && events[0] == 'U');
    ReceiveTarget(DETECT_UNKNOWN, -123, 456);
    ActionFSM_Update();
    assert(ActionFSM_GetState() == EXEC_BRAKE);
    assert(ActionFSM_GetAlignState() == ALIGN_BRAKE);

    /* A valid frame arriving during the loss stop is saved but cannot move. */
    before = event_count;
    ReceiveTarget(DETECT_REDBALL, -10, -777);
    assert(event_count == before);
    SettleAlignmentBrake(EXEC_RUN);
    assert(ActionFSM_GetAlignState() == ALIGN_TRACKING);
    before = event_count;
    ActionFSM_Update();
    assert(event_count == before + 1U && events[before] == 'D');
    assert(event_rpms[before] == SPEED_PRECISE_RPM);
}

static void FeedMissionDeadzone(uint8_t type)
{
    static const int16_t offsets[] = {0, 10, 20};
    unsigned i;
    size_t before = event_count;
    assert(MAXICAM_ALIGN_CONFIRM_FRAMES == 3U);
    for (i = 0U; i < MAXICAM_ALIGN_CONFIRM_FRAMES; ++i)
    {
        ReceiveTarget(type, offsets[i], (int16_t)(300 - (int)i));
        MissionFSM_Update();
        if (i + 1U < MAXICAM_ALIGN_CONFIRM_FRAMES) assert(event_count == before);
    }
    assert(event_count == before + 1U && events[event_count - 1U] == 'B');
    assert(MissionFSM_GetState() == TASK1_ALIGN_OBJECT ||
           MissionFSM_GetState() == TASK1_ALIGN_BUCKET ||
           MissionFSM_GetState() == TASK2_ALIGN ||
           MissionFSM_GetState() == TASK3_ALIGN);
    motor_idle = 1U;
    MissionFSM_Update();
    now += ACTION_STOP_SETTLE_MS;
    MissionFSM_Update();
}

static void TestMissionUnknownPolicy(ActionType action, MissionState search,
                                     MissionState align, uint8_t expected, uint16_t rpm)
{
    VisionTask_t task = {VISION_COLOR_RED, VISION_COLOR_GREEN, VISION_SHAPE_CONE};
    size_t before;
    unsigned i;
    Reset();
    MissionFSM_SetTaskData(&task);
    ActionFSM_Init(action);
    ActionFSM_Update();
    MissionFSM_Update();
    assert(MissionFSM_GetState() == search && events[event_count - 1U] == 'R');
    assert(event_rpms[event_count - 1U] == rpm);
    before = event_count;
    ReceiveTarget(DETECT_BLUEBALL, 0, 0);
    MissionFSM_Update();
    for (i = 0U; i < 9U; ++i)
    {
        ReceiveTarget(DETECT_UNKNOWN, 0, 0);
        MissionFSM_Update();
    }
    ReceiveTarget(DETECT_BLUEBALL, 0, 0);
    MissionFSM_Update();
    assert(MissionFSM_GetState() == search && event_count == before);

    ReceiveTarget(expected, 21, 0);
    MissionFSM_Update();
    assert(MissionFSM_GetState() == align && events[event_count - 1U] == 'B');
    motor_idle = 1U;
    ReceiveTarget(expected, 21, 0);
    MissionFSM_Update();
    assert(events[event_count - 1U] == 'U' && event_rpms[event_count - 1U] == SPEED_SLOW_RPM);
    ReceiveTarget(expected, -1, 0);
    MissionFSM_Update();
    assert(events[event_count - 1U] == 'D' && event_rpms[event_count - 1U] == SPEED_SLOW_RPM);
    for (i = 0U; i < 4U; ++i)
    {
        ReceiveTarget(DETECT_UNKNOWN, 0, 0);
        MissionFSM_Update();
        assert(MissionFSM_GetState() == align);
    }
    MaxiCam_ErrorCallback(&huart4);
    MaxiCam_Process();
    MissionFSM_Update();
    assert(MissionFSM_GetState() == align); /* UART failure is not UNKNOWN. */
    ReceiveTarget(DETECT_BLUEBALL, 0, 0); /* Wrong type breaks UNKNOWN streak. */
    MissionFSM_Update();
    for (i = 0U; i < 4U; ++i)
    {
        ReceiveTarget(DETECT_UNKNOWN, 0, 0);
        MissionFSM_Update();
        assert(MissionFSM_GetState() == align);
    }
    ReceiveTarget(expected, 20, 0); /* Matching target also resets the streak. */
    MissionFSM_Update();
    for (i = 0U; i < 4U; ++i)
    {
        ReceiveTarget(DETECT_UNKNOWN, 0, 0);
        MissionFSM_Update();
        assert(MissionFSM_GetState() == align);
    }
    ReceiveTarget(DETECT_UNKNOWN, 0, 0);
    MissionFSM_Update();
    assert(MissionFSM_GetState() == align && events[event_count - 1U] == 'B');
    motor_idle = 1U;
    MissionFSM_Update();
    now += ACTION_STOP_SETTLE_MS;
    MissionFSM_Update();
    assert(MissionFSM_GetState() == search);
    MissionFSM_Update();
    assert(events[event_count - 1U] == 'L' && event_rpms[event_count - 1U] == rpm);
    before = event_count;
    for (i = 0U; i < 8U; ++i)
    {
        ReceiveTarget(DETECT_UNKNOWN, 0, 0);
        MissionFSM_Update();
    }
    ReceiveTarget(DETECT_BLUEBALL, 0, 0);
    MissionFSM_Update();
    assert(MissionFSM_GetState() == search && event_count == before);
    ReceiveTarget(expected, 0, 0);
    MissionFSM_Update();
    assert(MissionFSM_GetState() == align && events[event_count - 1U] == 'B');
    motor_idle = 1U;
    ReceiveTarget(expected, 21, 0);
    MissionFSM_Update();
    assert(events[event_count - 1U] == 'U' && event_rpms[event_count - 1U] == SPEED_SLOW_RPM);
    ReceiveTarget(expected, -20, 0);
    MissionFSM_Update();
    assert(events[event_count - 1U] == 'D' && event_rpms[event_count - 1U] == SPEED_SLOW_RPM);
    FeedMissionDeadzone(expected);
    assert(MissionFSM_GetState() != align);
}

static void TestMissionTasks(void)
{
    VisionTask_t task = {VISION_COLOR_RED, VISION_COLOR_GREEN, VISION_SHAPE_CONE};
    Reset();
    MissionFSM_SetTaskData(&task);
    MissionFSM_SetBucketDetectType(DETECT_HOSTAGE_3); /* External confirmed-type adapter. */
    ActionFSM_Init(ACTION_TASK_1);
    ActionFSM_Update();
    assert(ActionFSM_GetState() == EXEC_RUN);
    MissionFSM_Update();
    assert(events[event_count - 1U] == 'R' && event_rpms[event_count - 1U] == SPEED_SLOW_RPM);
    assert(!task1_done);
    ReceiveTarget(DETECT_REDBALL, 30, 99);
    MissionFSM_Update();
    assert(MissionFSM_GetState() == TASK1_ALIGN_OBJECT && events[event_count - 1U] == 'B');
    motor_idle = 1U;
    {
        unsigned i;
        for (i = 0U; i + 1U < MAXICAM_TARGET_LOST_FRAMES; ++i)
        {
            ReceiveTarget(DETECT_UNKNOWN, 0, (int16_t)i);
            MissionFSM_Update();
            assert(MissionFSM_GetState() == TASK1_ALIGN_OBJECT && !task1_done);
        }
        ReceiveTarget(DETECT_UNKNOWN, 0, 99);
        MissionFSM_Update();
        motor_idle = 1U;
        MissionFSM_Update();
        now += ACTION_STOP_SETTLE_MS;
        MissionFSM_Update();
        assert(MissionFSM_GetState() == TASK1_SEARCH_OBJECT && !task1_done);
    }
    MissionFSM_Update();
    assert(events[event_count - 1U] == 'L' && event_rpms[event_count - 1U] == SPEED_SLOW_RPM);
    ReceiveTarget(DETECT_REDBALL, 20, 0);
    MissionFSM_Update();
    assert(MissionFSM_GetState() == TASK1_ALIGN_OBJECT);
    motor_idle = 1U;
    FeedMissionDeadzone(DETECT_REDBALL);
    assert(MissionFSM_GetState() == TASK1_PICK);
    MissionFSM_Update();
    assert(Servo_GetRequestedAction() == SERVO_ACTION_PICK && !Servo_IsDone());
    MissionFSM_Update();
    assert(MissionFSM_GetState() == TASK1_PICK && !task1_done);
    Servo_NotifyDone(); MissionFSM_Update();
    assert(MissionFSM_GetState() == TASK1_ROTATE_180 && !task1_done);
    MissionFSM_Update();
    assert(MissionFSM_GetState() == TASK1_SEARCH_BUCKET);
    ReceiveTarget(DETECT_UNKNOWN, 0, 0);
    MissionFSM_Update();
    MissionFSM_Update();
    assert(events[event_count - 1U] == 'R' && event_rpms[event_count - 1U] == SPEED_SLOW_RPM);
    ReceiveTarget(DETECT_HOSTAGE_3, 1, -300);
    MissionFSM_Update();
    assert(MissionFSM_GetState() == TASK1_ALIGN_BUCKET);
    motor_idle = 1U;
    ReceiveTarget(DETECT_HOSTAGE_3, 21, 0);
    MissionFSM_Update();
    assert(events[event_count - 1U] == 'U' && event_rpms[event_count - 1U] == SPEED_SLOW_RPM);
    ReceiveTarget(DETECT_HOSTAGE_3, -20, 0);
    MissionFSM_Update();
    assert(events[event_count - 1U] == 'D' && event_rpms[event_count - 1U] == SPEED_SLOW_RPM);
    {
        unsigned i;
        size_t before = event_count;
        for (i = 0U; i < MAXICAM_TARGET_LOST_FRAMES; ++i)
        {
            ReceiveTarget(DETECT_UNKNOWN, 0, 0);
            MissionFSM_Update();
            if (i + 1U < MAXICAM_TARGET_LOST_FRAMES)
                assert(MissionFSM_GetState() == TASK1_ALIGN_BUCKET && event_count == before);
        }
        assert(event_count == before + 1U && events[event_count - 1U] == 'B');
        motor_idle = 1U;
        MissionFSM_Update();
        now += ACTION_STOP_SETTLE_MS;
        MissionFSM_Update();
        assert(MissionFSM_GetState() == TASK1_SEARCH_BUCKET);
        MissionFSM_Update();
        assert(events[event_count - 1U] == 'L' && event_rpms[event_count - 1U] == SPEED_SLOW_RPM);
        ReceiveTarget(DETECT_HOSTAGE_3, 0, 0);
        MissionFSM_Update();
        assert(MissionFSM_GetState() == TASK1_ALIGN_BUCKET);
        motor_idle = 1U;
    }
    FeedMissionDeadzone(DETECT_HOSTAGE_3);
    assert(MissionFSM_GetState() == TASK1_RELEASE);
    MissionFSM_Update();
    assert(Servo_GetRequestedAction() == SERVO_ACTION_RELEASE && !task1_done);
    Servo_NotifyDone(); MissionFSM_Update();
    assert(MissionFSM_GetState() == TASK1_DONE && task1_done);

    ActionFSM_Init(ACTION_TASK_2); ActionFSM_Update();
    MissionFSM_Update();
    assert(MissionFSM_GetState() == TASK2_SEARCH &&
           events[event_count - 1U] == 'R' && event_rpms[event_count - 1U] == SPEED_MEDIUM_RPM);
    ReceiveTarget(DETECT_GREANBALL, -40, 32767);
    MissionFSM_Update();
    assert(MissionFSM_GetState() == TASK2_SEARCH); /* Old color target is ignored. */
    ReceiveTarget(DETECT_TARGET, -40, 32767);
    MissionFSM_Update();
    assert(MissionFSM_GetState() == TASK2_ALIGN);
    motor_idle = 1U;
    ReceiveTarget(DETECT_TARGET, 11, -32768);
    MissionFSM_Update();
    assert(events[event_count - 1U] == 'U');
    ReceiveTarget(DETECT_TARGET, -11, 32767);
    MissionFSM_Update();
    assert(events[event_count - 1U] == 'D');
    ReceiveTarget(DETECT_TARGET, 0, -32768);
    MissionFSM_Update();
    assert(MissionFSM_GetState() == TASK2_ALIGN && events[event_count - 1U] == 'B');
    assert(!Laser_IsEnabled() && laser_low_count == 0U);
    now += ACTION_STOP_SETTLE_MS * 2U;
    MissionFSM_Update();
    assert(MissionFSM_GetState() == TASK2_ALIGN); /* Queue is still busy. */
    motor_idle = 1U;
    MissionFSM_Update();
    now += ACTION_STOP_SETTLE_MS;
    MissionFSM_Update();
    assert(MissionFSM_GetState() == TASK2_CONFIRM);
    ReceiveTarget(DETECT_TARGET, 0, 100);
    MissionFSM_Update();
    ReceiveTarget(DETECT_TARGET, 10, -100);
    MissionFSM_Update();
    assert(MissionFSM_GetState() == TASK2_CONFIRM && !Laser_IsEnabled());
    ReceiveTarget(DETECT_TARGET, 11, 0); /* Drift resets confirmation. */
    MissionFSM_Update();
    assert(MissionFSM_GetState() == TASK2_ALIGN);
    ReceiveTarget(DETECT_TARGET, 0, 0);
    MissionFSM_Update();
    assert(events[event_count - 1U] == 'B');
    motor_idle = 1U;
    MissionFSM_Update();
    now += ACTION_STOP_SETTLE_MS;
    MissionFSM_Update();
    assert(MissionFSM_GetState() == TASK2_CONFIRM);
    ReceiveTarget(DETECT_TARGET, 0, 300);
    MissionFSM_Update();
    ReceiveTarget(DETECT_TARGET, -10, -300);
    MissionFSM_Update();
    assert(!Laser_IsEnabled() && laser_low_count == 0U);
    ReceiveTarget(DETECT_TARGET, 10, -32768);
    MissionFSM_Update();
    assert(MissionFSM_GetState() == TASK2_LASER_ON && Laser_IsEnabled() && !task2_done);
    assert(laser_pin_mode == GPIO_MODE_OUTPUT_OD && laser_low_count == 1U);
    now += LASER_HOLD_MS - 1U; MissionFSM_Update();
    assert(Laser_IsEnabled() && !task2_done);
    ++now; MissionFSM_Update();
    assert(MissionFSM_GetState() == TASK2_DONE && !Laser_IsEnabled() && task2_done);
    assert(laser_pin_mode == GPIO_MODE_INPUT);

    ActionFSM_Init(ACTION_TASK_3); ActionFSM_Update();
    MissionFSM_Update();
    assert(MissionFSM_GetState() == TASK3_SEARCH && events[event_count - 1U] == 'R' &&
           event_rpms[event_count - 1U] == SPEED_MEDIUM_RPM);
    ReceiveTarget(DETECT_HOSTAGE_2, -200, 42);
    MissionFSM_Update(); assert(MissionFSM_GetState() == TASK3_ALIGN);
    motor_idle = 1U;
    FeedMissionDeadzone(DETECT_HOSTAGE_2);
    assert(MissionFSM_GetState() == TASK3_CONFIRM);
    ReceiveTarget(DETECT_HOSTAGE_2, 0, 1234);
    MissionFSM_Update(); assert(MissionFSM_GetState() == TASK3_PICK);
    MissionFSM_Update();
    assert(Servo_GetRequestedAction() == SERVO_ACTION_RESCUE_PICK && !task3_object_acquired);
    Servo_NotifyDone(); MissionFSM_Update();
    assert(MissionFSM_GetState() == TASK3_RETURN && task3_object_acquired && !task3_done);
    {
        size_t before = event_count;
    MissionFSM_NotifyFinalRouteDone(); MissionFSM_Update();
    assert(MissionFSM_GetState() == MISSION_DONE && task3_done && final_route_done);
    assert(!Laser_IsEnabled());
        assert(event_count == before + 1U && events[event_count - 1U] == 'B');
        before = event_count;
        MissionFSM_Update(); MissionFSM_Update();
        assert(event_count == before);
    }
    puts("PASS mission: task1 search/align/servo/180/bucket/release; task2 confirm/nonblocking laser; task3 acquire/return gate/terminal brake");
}

static void TestTask2LossAndLaserAbort(void)
{
    VisionTask_t task = {VISION_COLOR_RED, VISION_COLOR_GREEN, VISION_SHAPE_CONE};
    unsigned i;
    Reset();
    MissionFSM_SetTaskData(&task);
    ActionFSM_Init(ACTION_TASK_2);
    ActionFSM_Update();
    MissionFSM_Update();
    ReceiveTarget(DETECT_TARGET, 21, 0);
    MissionFSM_Update();
    motor_idle = 1U;
    ReceiveTarget(DETECT_TARGET, 21, 0);
    MissionFSM_Update();
    assert(events[event_count - 1U] == 'U');
    ReceiveTarget(DETECT_UNKNOWN, 0, 0);
    MissionFSM_Update();
    assert(MissionFSM_GetState() == TASK2_ALIGN && events[event_count - 1U] == 'B');
    assert(!Laser_IsEnabled() && laser_low_count == 0U);
    motor_idle = 1U;
    MissionFSM_Update();
    now += ACTION_STOP_SETTLE_MS;
    MissionFSM_Update();
    assert(MissionFSM_GetState() == TASK2_SEARCH);

    Reset();
    MissionFSM_SetTaskData(&task);
    ActionFSM_Init(ACTION_TASK_2);
    ActionFSM_Update();
    MissionFSM_Update();
    ReceiveTarget(DETECT_TARGET, 0, 0);
    MissionFSM_Update();
    motor_idle = 1U;
    ReceiveTarget(DETECT_TARGET, 0, 0);
    MissionFSM_Update();
    motor_idle = 1U;
    MissionFSM_Update();
    now += ACTION_STOP_SETTLE_MS;
    MissionFSM_Update();
    assert(MissionFSM_GetState() == TASK2_CONFIRM);
    for (i = 0U; i < MAXICAM_ALIGN_CONFIRM_FRAMES; ++i)
    {
        ReceiveTarget(DETECT_TARGET, 0, 0);
        MissionFSM_Update();
    }
    assert(Laser_IsEnabled() && laser_pin_mode == GPIO_MODE_OUTPUT_OD);
    ActionFSM_Abort();
    assert(!Laser_IsEnabled() && laser_pin_mode == GPIO_MODE_INPUT);
    assert(!task2_done);

    Reset();
    MissionFSM_SetTaskData(&task);
    ActionFSM_Init(ACTION_TASK_2);
    ActionFSM_Update();
    MissionFSM_Update();
    ReceiveTarget(DETECT_TARGET, 0, 0);
    MissionFSM_Update();
    motor_idle = 1U;
    ReceiveTarget(DETECT_TARGET, 0, 0);
    MissionFSM_Update();
    motor_idle = 1U;
    MissionFSM_Update();
    now += ACTION_STOP_SETTLE_MS;
    MissionFSM_Update();
    for (i = 0U; i < MAXICAM_ALIGN_CONFIRM_FRAMES; ++i)
    {
        ReceiveTarget(DETECT_TARGET, 0, 0);
        MissionFSM_Update();
    }
    assert(Laser_IsEnabled());
    ReceiveTarget(DETECT_UNKNOWN, 0, 0);
    MissionFSM_Update();
    assert(MissionFSM_GetState() == MISSION_ERROR && !Laser_IsEnabled());
    assert(laser_pin_mode == GPIO_MODE_INPUT && !task2_done);
}

static void TestManualLaserButton(void)
{
    Reset();
    assert(!Laser_IsEnabled() && laser_pin_mode == GPIO_MODE_INPUT);
    mock_gpiob.IDR = 0U;
    BoardInputs_ProcessAux();
    now += LASER_BUTTON_DEBOUNCE_MS - 1U;
    BoardInputs_ProcessAux();
    assert(!Laser_IsEnabled() && laser_low_count == 0U);
    mock_gpiob.IDR = GPIO_PIN_8;
    BoardInputs_ProcessAux(); /* A short press must not trigger PC3. */
    assert(!Laser_IsEnabled());
    mock_gpiob.IDR = 0U;
    BoardInputs_ProcessAux();
    now += LASER_BUTTON_DEBOUNCE_MS;
    BoardInputs_ProcessAux();
    assert(Laser_IsEnabled() && laser_pin_mode == GPIO_MODE_OUTPUT_OD);
    assert(laser_low_count == 1U);
    BoardInputs_ProcessAux();
    assert(laser_low_count == 1U); /* Held button causes no repeated GPIO writes. */
    mock_gpiob.IDR = GPIO_PIN_8;
    BoardInputs_ProcessAux();
    assert(!Laser_IsEnabled() && laser_pin_mode == GPIO_MODE_INPUT);

    Reset();
    now = UINT32_MAX - 10U;
    mock_gpiob.IDR = 0U;
    BoardInputs_ProcessAux();
    now += LASER_BUTTON_DEBOUNCE_MS - 1U;
    BoardInputs_ProcessAux();
    assert(!Laser_IsEnabled());
    ++now;
    BoardInputs_ProcessAux();
    assert(Laser_IsEnabled()); /* Debounce works across HAL tick wrap. */
    mock_gpiob.IDR = GPIO_PIN_8;
    BoardInputs_ProcessAux();
    assert(!Laser_IsEnabled());

    Reset();
    Laser_Enable();
    assert(Laser_IsEnabled() && laser_low_count == 1U);
    mock_gpiob.IDR = 0U;
    BoardInputs_ProcessAux();
    now += LASER_BUTTON_DEBOUNCE_MS;
    BoardInputs_ProcessAux();
    Laser_Disable();
    assert(Laser_IsEnabled() && laser_pin_mode == GPIO_MODE_OUTPUT_OD);
    assert(laser_low_count == 1U);
    mock_gpiob.IDR = GPIO_PIN_8;
    BoardInputs_ProcessAux();
    assert(!Laser_IsEnabled() && laser_pin_mode == GPIO_MODE_INPUT);

    Reset();
    mock_gpiob.IDR = 0U;
    BoardInputs_ProcessAux();
    now += LASER_BUTTON_DEBOUNCE_MS;
    BoardInputs_ProcessAux();
    assert(Laser_IsEnabled() && laser_low_count == 1U);
    Laser_Enable();
    mock_gpiob.IDR = GPIO_PIN_8;
    BoardInputs_ProcessAux();
    assert(Laser_IsEnabled() && laser_pin_mode == GPIO_MODE_OUTPUT_OD);
    assert(laser_low_count == 1U);
    Laser_Disable();
    assert(!Laser_IsEnabled() && laser_pin_mode == GPIO_MODE_INPUT);
    puts("PASS PB8 manual trigger: debounce, immediate release, tick wrap, auto/manual overlap");
}

int main(void)
{
    assert(SpeedMode_GetRPM(SPEED_SLOW) == 100U);
    assert(SpeedMode_GetRPM(SPEED_MEDIUM) == 133U);
    assert(SpeedMode_GetRPM(SPEED_FULL) == 167U);
    assert(SpeedMode_GetRPM((SpeedMode)99) == SPEED_PRECISE_RPM);
    TestRouteModeSwitches(); TestCameraModeRetry(); TestBusyAndError();
    TestQRSearch(); TestQRBeforeSearch(); TestQRSearchBusyFaultAndRestart();
    TestQRSearchStopIsolation();
    TestMaxiCamAlignmentDirections(); TestMaxiCamAlignmentConfirmation();
    TestMaxiCamAlignmentTargetLoss();
    TestMissionUnknownPolicy(ACTION_TASK_1, TASK1_SEARCH_OBJECT, TASK1_ALIGN_OBJECT,
                             DETECT_REDBALL, SPEED_SLOW_RPM);
    TestMissionUnknownPolicy(ACTION_TASK_3, TASK3_SEARCH, TASK3_ALIGN,
                             DETECT_HOSTAGE_2, SPEED_MEDIUM_RPM);
    TestMissionTasks();
    TestTask2LossAndLaserAbort();
    TestManualLaserButton();
    puts("PASS FIRST_UNKNOWN_SEARCH_RIGHT");
    puts("PASS LOST_5_FRAMES_REACQUIRE_LEFT");
    puts("PASS REACQUIRE_TARGET_ALIGN");
    puts("PASS ALIGN_3_FRAMES_DEADZONE");
    puts("PASS WRONG_TYPE_NOT_MARKED_SEEN");
    puts("PASS NEW_TARGET_RESET");
    puts("PASS REGRESSION");
    puts("PASS QR_FORWARD_UNTIL_ROAD_END");
    puts("PASS QR_RIGHT_TURN_90");
    puts("PASS QR_SCAN_STATIONARY");
    puts("PASS QR_0X80_SUCCESS_ONLY");
    puts("PASS QR_LEFT_TURN_90");
    puts("PASS QR_STAGE_DONE_GATE");
    puts("PASS LEFT_TRANSLATE_PRESERVED");
    puts("PASS TASK_FLOW_UNCHANGED");
    puts("PASS route: QR turn/scan/turn gate and path to task1");
    puts("PASS action: every motion END -> BRAKE -> WAIT_STOP -> DONE; right uses existing wrapper");
    puts("PASS QR: fresh binary 0x80 after stationary scan; target and UNKNOWN packets ignored");
    puts("PASS QR safety: turn busy-retry / no repeated motion / faults / restart reset / stop isolation / tick wrap");
    puts("PASS speed: route requests 167 RPM / turn controller caps 90/180 separately");
    puts("PASS speed isolation: no motion in BRAKE, WAIT_STOP, DONE or FINISH / no repeated speed logs");
    puts("PASS align: shifted X=10 / inclusive 0..20 deadzone / 100 RPM / Y ignored / 3 fresh frames");
    puts("PASS align safety: target loss reset+brake / no restart before WAIT_STOP / DONE after stop");
    puts("PASS route placeholder: left turn waits for its explicit completion event and sends no turn command");
    return 0;
}
