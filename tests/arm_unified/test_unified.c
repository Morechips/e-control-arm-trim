#include "arm_trim_input.h"
#include "arm_trim_service.h"
#include "bluetooth_driver.h"
#include "car_control.h"
#include "motor_driver.h"
#include "pid_tuner.h"
#include "serial_io.h"
#include "uart_driver.h"
#include "board_inputs.h"
#include "../phone20_packet.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(c) do { ++checks; if (!(c)) { fprintf(stderr, "FAIL %u: %s\n", (unsigned)__LINE__, #c); exit(1); } } while (0)
static int bt_peripheral, servo_peripheral;
UART_HandleTypeDef huart6;
static UART_HandleTypeDef servo_uart;
static uint32_t tick, tx_tick;
static unsigned checks, tx_count, car_count, pid_count;
static bool active;
static CarState_t car_state = CAR_READY;
static char history[2048][SERVO_MAX_TX_LENGTH + 1U], replies[16000];
static CarRemoteInput_t car_input;
static uint16_t held;
static int16_t direction = -1;

uint32_t HAL_GetTick(void) { return tick; }
CarState_t Car_Control_GetState(void) { return car_state; }
uint8_t Motor_IsIdle(void) { return 1U; }
uint8_t Motor_HasFault(void) { return 0U; }
void Debug_Log(const char *line) { (void)line; }
uint8_t Debug_CanLog(uint8_t count) { (void)count; return 0U; }
bool PID_Tuner_QueueReply(const char *line)
{
    if (strlen(replies) + strlen(line) < sizeof(replies)) strcat(replies, line);
    return true;
}
void PID_Tuner_HandleCommand(uint8_t command) { CHECK(command >= 1U && command <= 11U); ++pid_count; }
void BoardInputs_TraceServo(uint32_t count, unsigned result) { (void)count; (void)result; }
void Car_Control_InvalidateRemoteInput(void) { }
void Car_Control_SubmitRemoteInput(const CarRemoteInput_t *input)
{
    CHECK(input->valid); ++car_count; car_input = *input;
    car_state = input->command.forward ? CAR_RUNNING : CAR_READY;
}
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *byte, uint16_t count)
{
    CHECK(uart == &huart6 && count == 1U);
    uart->rx = byte; uart->RxState = 1U; return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, const uint8_t *data, uint16_t length)
{
    CHECK(uart == &servo_uart && !active && tx_count < 2048U);
    CHECK(length > 0U && length <= SERVO_MAX_TX_LENGTH);
    memcpy(history[tx_count], data, length); history[tx_count++][length] = '\0';
    active = true; tx_tick = tick; uart->gState = HAL_UART_STATE_BUSY_TX; return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *uart)
{
    CHECK(uart == &servo_uart); active = false;
    uart->gState = HAL_UART_STATE_READY; return HAL_OK;
}
static void Feed(const uint8_t *bytes, size_t size)
{
    for (size_t i = 0U; i < size; ++i) {
        *huart6.rx = bytes[i]; huart6.RxState = HAL_UART_STATE_READY;
        Bluetooth_RxCallback(&huart6);
    }
    Bluetooth_Process();
    Bluetooth_DispatchServoActions(0U);
    ArmTrimInput_Process();
}
static void PhoneValues(uint16_t buttons, const int16_t values[16], uint32_t gap)
{
    uint8_t packet[41]; PackPhone20(packet, values, buttons, gap); Feed(packet, sizeof(packet));
}
static void Phone(uint16_t buttons)
{
    int16_t values[16] = {0}; values[14] = direction; held = buttons;
    PhoneValues(buttons, values, 0U);
}
static void Tick(void)
{
    tick += 5U;
    if (active && (uint32_t)(tick - tx_tick) >= 3U) {
        active = false; servo_uart.gState = HAL_UART_STATE_READY;
        UART_TxCallback(&servo_uart);
    }
    if (tick % 50U == 0U) Phone(held);
    else ArmTrimInput_Process();
}
static void Finish(void)
{
    for (unsigned i = 0U; i < 10000U && ArmTrimService_IsBusy(); ++i) Tick();
    CHECK(!ArmTrimService_IsBusy());
}
static void Reset(void)
{
    held = 0U;
    (void)ArmTrimService_End(); Finish();
    if (ArmTrimService_GetStatus().state == ARM_TRIM_SERVICE_FAULT)
        CHECK(ArmTrimService_ClearFault() == ARM_TRIM_OK);
    CHECK(!active && !ArmTrimService_OwnsMotion());
    tick += 2000U; tx_count = car_count = pid_count = 0U; replies[0] = '\0';
    car_state = CAR_READY; direction = -1;
    huart6.Instance = &bt_peripheral; huart6.gState = huart6.RxState = HAL_UART_STATE_READY;
    servo_uart.Instance = &servo_peripheral;
    servo_uart.Init = (UART_InitTypeDef){115200U, UART_WORDLENGTH_8B, UART_STOPBITS_1,
        UART_PARITY_NONE, UART_MODE_TX_RX, UART_HWCONTROL_NONE};
    servo_uart.gState = servo_uart.RxState = HAL_UART_STATE_READY;
    CHECK(Servo_Init(&servo_uart) == SERVO_OK);
    Bluetooth_Init(); ArmTrimInput_Init(); Phone(0U);
    CHECK(tx_count == 0U);
}
static void CheckPlanar(unsigned from)
{
    for (unsigned i = from; i < tx_count; ++i) CHECK(strstr(history[i], "#003") == NULL);
}
static void ReadyBall(void)
{
    Phone(BT_SERVO_BUTTON_TB_M); Phone(0U); Finish();
    CHECK(ArmTrimService_GetStatus().core.reference_valid);
    CHECK(ArmTrimService_OwnsMotion());
}
static void TestHandoffAndIndependentGrip(void)
{
    unsigned before;
    Reset(); ReadyBall();
    Phone(BT_CONTROL_BOOL_JOG_BIT);
    for (unsigned i = 0U; i < 30U; ++i) Tick();
    CHECK(ArmTrimService_GetStatus().core.jogging);
    before = tx_count;
    Phone(BT_CONTROL_BOOL_JOG_BIT | BT_SERVO_BUTTON_RST);
    CHECK(ArmTrimService_GetStatus().pose_pending && tx_count == before);
    Finish(); CheckPlanar(0U);
    CHECK(strstr(history[tx_count - 1U], "#000P1524T2000!") != NULL);
    CHECK(!ArmTrimService_GetStatus().core.jogging);
    before = tx_count;
    for (unsigned i = 0U; i < 100U; ++i) Tick();
    CHECK(tx_count == before); /* Holding WT never resumes after switching. */
    Phone(0U);
    Phone(BT_CONTROL_BOOL_CLOSE_BIT); Phone(0U); Finish();
    CHECK(strstr(history[tx_count - 1U], "#003P0500T1500!") != NULL);
    CHECK(strstr(history[tx_count - 1U], "#000") == NULL);
    before = tx_count; ReadyBall(); CheckPlanar(before);
    CHECK(ArmTrimService_GetStatus().core.reference_valid);
}
static void TestBusyAndStop(void)
{
    unsigned before;
    Reset(); Phone(BT_SERVO_BUTTON_TB_M);
    Phone(BT_SERVO_BUTTON_RST); CHECK(tx_count == 1U);
    Phone(0U); Finish(); CHECK(tx_count == 1U);
    CHECK(strstr(replies, "TRIM POSE RESULT=2") != NULL);
    Phone(BT_SERVO_BUTTON_RST); Phone(0U); Finish(); CHECK(tx_count == 2U);
    Reset(); ReadyBall(); Phone(BT_CONTROL_BOOL_JOG_BIT);
    for (unsigned i = 0U; i < 20U; ++i) Tick();
    Phone(BT_SERVO_BUTTON_RST | BT_CONTROL_BOOL_JOG_BIT);
    CHECK(ArmTrimService_GetStatus().pose_pending);
    Phone(BT_CONTROL_BOOL_ARM_STOP_BIT); held = 0U; Finish();
    CHECK(!ArmTrimService_GetStatus().pose_pending && !ArmTrimService_GetStatus().core.reference_valid);
    for (unsigned i = 0U; i < tx_count; ++i) CHECK(strstr(history[i], "#000P1524T2000!") == NULL);
    Reset(); before = tx_count;
    Phone(BT_SERVO_BUTTON_TB_M | BT_SERVO_BUTTON_TH_C); Phone(0U); Finish();
    CHECK(tx_count == before && strstr(replies, "ONE_ACTION") != NULL);
    Phone(BT_CONTROL_BOOL_AIM_BIT); Phone(0U);
    CHECK(tx_count == before + 1U && strstr(replies, "TRIM POSE RESULT=0") != NULL);
    CHECK(strcmp(history[before], "{#000P1058T2000!#001P0821T2000!#002P0554T2000!}") == 0);
    Finish(); CheckPlanar(before);
    CHECK(ArmTrimService_GetStatus().state != ARM_TRIM_SERVICE_FAULT);
}
static void TestBothPagesAndCarFields(void)
{
    uint8_t packet[7] = {0xA5U, ARM_TRIM_BUTTON_BALL, 0U, 0U, 0U, ARM_TRIM_BUTTON_BALL, 0x5AU};
    int16_t values[16] = {0};
    Reset(); Feed(packet, sizeof(packet)); Finish();
    CHECK(ArmTrimService_GetStatus().core.reference_valid);
    Phone(BT_SERVO_BUTTON_RST); Phone(0U); Finish(); CheckPlanar(0U);
    ReadyBall();
    packet[1] = packet[5] = ARM_TRIM_BUTTON_BUCKET; Feed(packet, sizeof(packet)); Finish();
    CHECK(ArmTrimService_GetStatus().profile == ARM_TRIM_PROFILE_BUCKET);
    CHECK(strstr(history[tx_count - 1U], "#000P1566T2000!") != NULL);
    values[13] = 9; values[11] = 1;
    PhoneValues(0U, values, 0U); PhoneValues(0U, values, 0U);
    CHECK(pid_count == 1U && strstr(replies, "TRIM v4.7") != NULL);
    values[13] = 0; PhoneValues(0U, values, 0U);
    values[13] = 9; PhoneValues(0U, values, 0U); CHECK(pid_count == 2U);
    memset(values, 0, sizeof(values)); values[1] = 1;
    unsigned before = tx_count;
    PhoneValues(BT_SERVO_BUTTON_TB_M, values, 0U);
    CHECK(car_input.command.forward == 1 && car_count > 0U && tx_count == before);
    CHECK(strstr(replies, "PARKED_IDLE_REQUIRED") != NULL);
    values[1] = 0; PhoneValues(0U, values, 0U);
    /* Standalone gripper requires no profile/BEGIN. */
    Reset(); Phone(BT_CONTROL_BOOL_OPEN_BIT); Phone(0U); Finish();
    CHECK(tx_count == 1U && strstr(history[0], "#003P1800T1500!") != NULL);
    CHECK(!ArmTrimService_GetStatus().core.reference_valid);
    (void)ArmTrimService_End(); Finish();
}
static void TestLegacyPageReleasesJog(void)
{
    const unsigned lengths[] = {21U, 31U};
    for (unsigned schema = 0U; schema < sizeof(lengths) / sizeof(lengths[0]); ++schema) {
        uint8_t packet[31] = {0};
        Reset(); ReadyBall(); Phone(BT_CONTROL_BOOL_JOG_BIT);
        for (unsigned i = 0U; i < 20U; ++i) Tick();
        CHECK(ArmTrimService_GetStatus().core.jogging);
        packet[0] = 0xA5U; packet[lengths[schema] - 1U] = 0x5AU;
        Feed(packet, lengths[schema]);
        /* Keep legacy neutral packets arriving, rather than using Tick()'s
         * unified page. They must actively release instead of renewing WT. */
        for (unsigned i = 0U; i < 400U; ++i) {
            tick += 5U;
            if (active && (uint32_t)(tick - tx_tick) >= 3U) {
                active = false; servo_uart.gState = HAL_UART_STATE_READY;
                UART_TxCallback(&servo_uart);
            }
            if (i % 10U == 0U) Feed(packet, lengths[schema]);
            else ArmTrimInput_Process();
        }
        CHECK(!ArmTrimService_IsBusy() && !ArmTrimService_GetStatus().core.jogging);
        CHECK(ArmTrimService_GetStatus().core.reference_valid);
        CHECK(ArmTrimService_GetStatus().core.offset_mm > -10.0f);
        CheckPlanar(0U); held = 0U;
    }
}
int main(void)
{
    TestHandoffAndIndependentGrip(); TestBusyAndStop(); TestBothPagesAndCarFields();
    TestLegacyPageReleasesJog();
    printf("PASS unified BT -> input -> service -> real Servo/UART: %u checks\n", checks);
    return 0;
}
