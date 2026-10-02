#include "servo_remote.h"
#include "bluetooth_driver.h"
#include "board_app.h"
#include "car_control.h"
#include "motor_driver.h"
#include "serial_io.h"
#include "servo_pose.h"
#include "arm_control.h"
#include "arm_tuner.h"
#include "servo_remote_config.h"
#include "servo_pose_config.h"
#include <stdio.h>

/* Legacy Bluetooth modes 6 and 7 retain their existing meanings. */
static const ServoMode_t pose_by_mode[BT_SERVO_MODE_MAX + 1U] = {
    SERVO_MODE_NONE,
    SERVO_MODE_BALL_PREPARE,
    SERVO_MODE_BALL_REACH,
    SERVO_MODE_BALL_GRIP,
    SERVO_MODE_ABOVE_BUCKET,
    SERVO_MODE_BALL_RELEASE,
    SERVO_MODE_BALL_PREPARE,
    SERVO_MODE_NONE,
    SERVO_MODE_HOSTAGE_REACH,
    SERVO_MODE_HOSTAGE_GRIP,
    SERVO_MODE_HOSTAGE_LIFT
};
static const ServoMode_t pose_by_button[8] = {
    SERVO_MODE_BALL_PREPARE, SERVO_MODE_BALL_REACH,
    SERVO_MODE_BALL_GRIP, SERVO_MODE_ABOVE_BUCKET,
    SERVO_MODE_BALL_RELEASE, SERVO_MODE_HOSTAGE_REACH,
    SERVO_MODE_HOSTAGE_GRIP, SERVO_MODE_HOSTAGE_LIFT
};

static uint32_t last_sequence;
static uint32_t last_servo_one_sequence;
static int16_t previous_mode;
static int16_t selected_mode;
static uint16_t previous_buttons;
static uint16_t previous_gap_pwm;
static uint32_t previous_rx_tick;
static uint8_t armed;
static uint8_t preset_active;
static uint8_t preset_requires_bt;
static uint8_t gap_initialized;
static uint8_t button_raw, button_stable;
static uint32_t button_tick;

void ServoRemote_Init(void)
{
    last_sequence = Bluetooth_GetSequence();
    last_servo_one_sequence = Bluetooth_GetServoOneSequence();
    previous_mode = 0;
    selected_mode = 0;
    previous_buttons = 0U;
    previous_gap_pwm = 0U;
    previous_rx_tick = 0U;
    armed = 0U;
    preset_active = 0U;
    preset_requires_bt = 0U;
    gap_initialized = 0U;
    button_raw = button_stable = Board_ServoButtonIsLow();
    button_tick = HAL_GetTick();
}

static uint8_t ControlsNeutral(const BluetoothControlFrame *c)
{
    return (uint8_t)(c->joy_x >= -JOY_DEADZONE && c->joy_x <= JOY_DEADZONE &&
        c->joy_y >= -JOY_DEADZONE && c->joy_y <= JOY_DEADZONE &&
        !c->forward && !c->backward && !c->stop && !c->strafe_left &&
        !c->strafe_right && !c->right_90 && !c->right_180 && !c->left_90 &&
        !c->Cam_T && !c->Shot && !c->aim && !c->brake && !c->disable);
}

static uint8_t ButtonPressed(void)
{
    uint8_t low = Board_ServoButtonIsLow();
    uint32_t now = HAL_GetTick();
    if (low != button_raw) {
        button_raw = low;
        button_tick = now;
    }
    if (low == button_stable ||
        (uint32_t)(now - button_tick) < SERVO_BUTTON_DEBOUNCE_MS) return 0U;
    button_stable = low;
    return low;
}

static uint8_t CanSendBluetooth(const BluetoothControlFrame *c)
{
    return (uint8_t)(armed && Bluetooth_IsConnected() &&
                     Car_Control_GetState() == CAR_READY &&
                     Motor_IsIdle() && ControlsNeutral(c) &&
                     !ArmTuner_IsSessionActive());
}

static void SendMode(int16_t mode, uint8_t requires_bt)
{
    ServoMode_t pose_mode = pose_by_mode[mode];
    ArmStep_t step;
    ServoPoseResult_t pose_result;
    ArmResult_t result;
    ArmStatus_t status;
    char message[64];
    status = Arm_GetStatus();
    if (ArmTuner_IsSessionActive() || preset_active || status.motion_allowed ||
        status.state == ARM_RUNNING ||
        status.state == ARM_STOPPING || status.state == ARM_FAULT) {
        Debug_Log("[SERVO] rejected: arm busy\r\n");
        return;
    }
    if (pose_mode == SERVO_MODE_NONE) {
        selected_mode = mode;
        Debug_Log("[SERVO] mode 7 no action\r\n");
        return;
    }
    pose_result = ServoPose_BuildMode(pose_mode, &step);
    if (pose_result == SERVO_POSE_LIMIT) {
        selected_mode = mode; /* Keep the Bluetooth selection for mode changes. */
        (void)snprintf(message, sizeof(message),
                       "[SERVO] mode %u rejected: calibrated limit\r\n",
                       (unsigned)mode);
        Debug_Log(message);
        return;
    }
    if (pose_result != SERVO_POSE_OK) return;
    result = Arm_SetMotionAllowed(true);
    if (result == ARM_OK) {
        result = Arm_StartSequence(&step, 1U);
        if (result != ARM_OK) (void)Arm_SetMotionAllowed(false);
    }
    if (result == ARM_OK) {
        selected_mode = mode;
        preset_active = 1U;
        preset_requires_bt = requires_bt;
    }
    (void)snprintf(message, sizeof(message), "[SERVO] mode %u request %s\r\n",
                   (unsigned)mode, result == ARM_OK ? "accepted" : "rejected");
    Debug_Log(message);
}

static void SendPe4ModeOne(void)
{
    /* PE4 remains available without Bluetooth, but never steals ownership
     * from a running preset or an @ARM/dual-joystick session. */
    SendMode(1, 0U);
}

static void SendOriginalButton(uint16_t button)
{
    ArmStep_t step;
    ArmResult_t result;
    ArmStatus_t status = Arm_GetStatus();
    unsigned index = 0U;
    char message[64];
    if (preset_active || status.motion_allowed || status.state == ARM_RUNNING ||
        status.state == ARM_STOPPING || status.state == ARM_FAULT) {
        Debug_Log("[SERVO] bool rejected: arm busy\r\n");
        return;
    }
    if (button == BT_SERVO_BUTTON_RST) {
        result = Arm_ResetController();
    } else {
        /* TH_L belongs to the teammate's direct-servo pose table.  It has no
         * calibrated equivalent in the local safe pose table, so consume it
         * without indexing past pose_by_button or transmitting a guessed pose. */
        if ((button & 0x00FFU) == 0U) {
            Debug_Log("[SERVO] bool rejected: no local calibrated pose\r\n");
            return;
        }
        while ((button & (1U << index)) == 0U) ++index;
        if (ServoPose_BuildOriginalMode(pose_by_button[index], &step) !=
            SERVO_POSE_OK) return;
        result = Arm_SetMotionAllowed(true);
        if (result == ARM_OK) {
            result = Arm_StartOriginalPreset(&step);
            if (result != ARM_OK) (void)Arm_SetMotionAllowed(false);
        }
        if (result == ARM_OK) {
            preset_active = 1U;
            preset_requires_bt = 1U;
        }
    }
    (void)snprintf(message, sizeof(message), "[SERVO] bool %u request %s\r\n",
                   button == BT_SERVO_BUTTON_RST ? 9U : index + 1U,
                   result == ARM_OK ? "accepted" : "rejected");
    Debug_Log(message);
}

static void SendGap(uint16_t pwm)
{
    ArmStep_t step = {0};
    ArmStatus_t status = Arm_GetStatus();
    ArmResult_t result;
    char message[64];
    if (preset_active || status.motion_allowed || status.state == ARM_RUNNING ||
        status.state == ARM_STOPPING || status.state == ARM_FAULT) {
        Debug_Log("[SERVO] GAP rejected: arm busy\r\n");
        return;
    }
    step.joint_mask = (uint8_t)(1U << ARM_JOINT_GRIPPER);
    step.position[ARM_JOINT_GRIPPER] = pwm;
    step.move_ms = SERVO_POSE_MOVE_MS;
    result = Arm_SetMotionAllowed(true);
    if (result == ARM_OK) {
        result = Arm_StartOriginalPreset(&step);
        if (result != ARM_OK) (void)Arm_SetMotionAllowed(false);
    }
    if (result == ARM_OK) {
        preset_active = 1U;
        preset_requires_bt = 1U;
    }
    (void)snprintf(message, sizeof(message), "[SERVO] GAP %u request %s\r\n",
                   (unsigned)pwm, result == ARM_OK ? "accepted" : "rejected");
    Debug_Log(message);
}

void ServoRemote_Process(void)
{
    const BluetoothControlFrame *c = &Bluetooth_GetControl()->frame;
    uint32_t sequence = Bluetooth_GetSequence();
    uint32_t servo_one_sequence = Bluetooth_GetServoOneSequence();
    uint8_t pressed = ButtonPressed();
    uint8_t servo_one_pressed = (uint8_t)(servo_one_sequence != last_servo_one_sequence);
    uint16_t new_buttons;
    uint8_t gap_changed;
    int16_t mode;
    last_servo_one_sequence = servo_one_sequence;

    if (preset_active) {
        ArmState_t state = Arm_GetStatus().state;
        if (preset_requires_bt && !Bluetooth_IsConnected() && state == ARM_RUNNING)
            (void)Arm_Stop();
        if (state == ARM_COMPLETE_ESTIMATED) {
            (void)Arm_SetMotionAllowed(false);
            preset_active = 0U;
        } else if (state == ARM_CANCELLED || state == ARM_STOPPING ||
                   state == ARM_FAULT) {
            if (state == ARM_FAULT &&
                Arm_GetStatus().error == ARM_ERROR_TRANSPORT)
                Debug_Log("[SERVO] USART3 TX failed\r\n");
            preset_active = 0U;
        }
    }

    if (!Bluetooth_IsConnected()) {
        if (preset_active && preset_requires_bt) {
            (void)Arm_Stop();
            preset_active = 0U;
        }
        armed = 0U;
        previous_mode = 0;
        previous_buttons = 0U;
        previous_gap_pwm = 0U;
        previous_rx_tick = 0U;
        gap_initialized = 0U;
        last_sequence = sequence;
        /* PE4 keeps its press/release state while Bluetooth is away. */
    }
    if (pressed) {
        if (Bluetooth_IsConnected() && sequence != last_sequence) {
            last_sequence = sequence;
            previous_mode = c->servo_mode;
            previous_buttons = c->servo_buttons;
            previous_gap_pwm = c->gap_pwm;
            previous_rx_tick = Bluetooth_GetLastRxTick();
            gap_initialized = 1U;
        }
        SendPe4ModeOne();
        return; /* The physical button takes priority over Bluetooth modes. */
    }
    if (Bluetooth_IsConnected() && sequence != last_sequence) {
        uint32_t rx_tick = Bluetooth_GetLastRxTick();
        if (gap_initialized &&
            (uint32_t)(rx_tick - previous_rx_tick) > BT_FAILSAFE_TIMEOUT_MS) {
            previous_buttons = 0U;
            previous_gap_pwm = 0U;
            gap_initialized = 0U;
        }
        last_sequence = sequence;
        new_buttons = (uint16_t)(c->servo_buttons & (uint16_t)~previous_buttons);
        previous_buttons = c->servo_buttons;
        gap_changed = (uint8_t)(gap_initialized &&
                                c->gap_pwm != previous_gap_pwm);
        previous_gap_pwm = c->gap_pwm;
        previous_rx_tick = rx_tick;
        gap_initialized = 1U;
        mode = c->servo_mode;
        if (mode == 0) {
            uint8_t changed = (uint8_t)(previous_mode != 0);
            previous_mode = 0;
            armed = 1U;
            if (new_buttons != 0U) {
                if (c->servo_buttons == new_buttons &&
                    (new_buttons & (uint16_t)(new_buttons - 1U)) == 0U &&
                    CanSendBluetooth(c)) SendOriginalButton(new_buttons);
                return;
            }
            if (gap_changed && c->gap_pwm != 0U) {
                if (CanSendBluetooth(c)) SendGap(c->gap_pwm);
                return;
            }
            if (changed) {
                selected_mode = 0;
                return; /* Bluetooth transition wins over a simultaneous press. */
            }
        } else if (mode > 0 && mode <= BT_SERVO_MODE_MAX &&
                   mode != previous_mode) {
            previous_mode = mode;
            if (CanSendBluetooth(c) && mode != selected_mode) SendMode(mode, 1U);
            return; /* Consume the press even when the Bluetooth mode is denied. */
        }
    }
    if (servo_one_pressed) {
        if (CanSendBluetooth(c)) SendMode(1, 1U);
        return; /* One press is consumed even when the safety gate denies it. */
    }
}
