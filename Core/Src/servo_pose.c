#include "servo_pose.h"
#include "servo_pose_config.h"
#include <string.h>

/* Numeric presets use IDs 000..002; bool presets also use ID 003.
 * Each row keeps its former ID 003 PWM in a comment for recalibration. */
static const uint16_t pose_pwm[SERVO_POSE_COUNT][3] = {
    {1532U, 2219U, 1202U}, /* Reference; old ID 003: 1200 */
    {1421U, 2071U,  812U}, /* Ball reach; old ID 003: 2192 */
    {1421U, 2071U,  812U}, /* Ball grip; old ID 003: 1016 */
    {1717U, 2297U,  884U}, /* Above bucket; old ID 003: 1482 */
    {1717U, 2297U,  884U}, /* Ball release; old ID 003: 1800 */
    {1499U, 1855U,  528U}, /* Hostage reach; old ID 003: 1800 */
    {1499U, 1855U,  528U}, /* Hostage grip; old ID 003: 1400 */
    {1800U, 1855U,  528U}  /* Hostage lift; old ID 003: 1400 */
};

/* Former ID 003 PWM for this original pose: 2192. */
static const uint16_t original_ball_prepare[3] = {
    1532U, 1972U, 573U
};

static const uint16_t original_gripper_pwm[SERVO_POSE_COUNT] = {
    SERVO_POSE_TB_B_GRIPPER_PWM, SERVO_POSE_TB_M_GRIPPER_PWM,
    SERVO_POSE_TB_G_GRIPPER_PWM, SERVO_POSE_BD_U_GRIPPER_PWM,
    SERVO_POSE_BD_D_GRIPPER_PWM, SERVO_POSE_TH_C_GRIPPER_PWM,
    SERVO_POSE_TH_G_GRIPPER_PWM, SERVO_POSE_TH_U_GRIPPER_PWM
};

static const char *const pose_name[SERVO_POSE_COUNT] = {
    "BALL_APPROACH", "BALL_REACH", "BALL_GRIP", "BALL_ABOVE_BIN",
    "BALL_RELEASE", "HOSTAGE_REACH", "HOSTAGE_GRIP", "HOSTAGE_LIFT"
};

ServoPoseResult_t ServoPose_BuildStep(uint8_t pose_index, ArmStep_t *step)
{
    static const uint16_t min_p[3] = {
        ARM_P0_MIN, ARM_P1_MIN, ARM_P2_MIN
    };
    static const uint16_t max_p[3] = {
        ARM_P0_MAX, ARM_P1_MAX, ARM_P2_MAX
    };
    uint8_t i;
    if (step == NULL || pose_index >= SERVO_POSE_COUNT) return SERVO_POSE_INVALID;
    for (i = 0U; i < 3U; ++i)
        if (pose_pwm[pose_index][i] < min_p[i] ||
            pose_pwm[pose_index][i] > max_p[i]) return SERVO_POSE_LIMIT;
    memset(step, 0, sizeof(*step));
    step->joint_mask = 0x07U;
    for (i = 0U; i < 3U; ++i) step->position[i] = pose_pwm[pose_index][i];
    step->move_ms = SERVO_POSE_MOVE_MS;
    return SERVO_POSE_OK;
}

ServoPoseResult_t ServoPose_BuildMode(ServoMode_t mode, ArmStep_t *step)
{
    if (mode < SERVO_MODE_BALL_PREPARE || mode > SERVO_MODE_HOSTAGE_LIFT)
        return SERVO_POSE_INVALID;
    return ServoPose_BuildStep((uint8_t)(mode - SERVO_MODE_BALL_PREPARE), step);
}

ServoPoseResult_t ServoPose_BuildOriginalMode(ServoMode_t mode, ArmStep_t *step)
{
    const uint16_t *pwm;
    uint16_t gripper_pwm;
    uint8_t i;
    if (step == NULL || mode < SERVO_MODE_BALL_PREPARE ||
        mode > SERVO_MODE_HOSTAGE_LIFT) return SERVO_POSE_INVALID;
    pwm = mode == SERVO_MODE_BALL_PREPARE ? original_ball_prepare :
          pose_pwm[(unsigned)mode - SERVO_MODE_BALL_PREPARE];
    gripper_pwm = original_gripper_pwm[(unsigned)mode - SERVO_MODE_BALL_PREPARE];
    for (i = 0U; i < 3U; ++i)
        if (pwm[i] < 500U || pwm[i] > 2500U) return SERVO_POSE_INVALID;
    if (gripper_pwm < 500U || gripper_pwm > 2500U) return SERVO_POSE_INVALID;
    memset(step, 0, sizeof(*step));
    step->joint_mask = 0x0FU;
    for (i = 0U; i < 3U; ++i) step->position[i] = pwm[i];
    step->position[ARM_JOINT_GRIPPER] = gripper_pwm;
    step->move_ms = SERVO_POSE_MOVE_MS;
    return SERVO_POSE_OK;
}

const char *ServoPose_Name(ServoMode_t mode)
{
    if (mode < SERVO_MODE_BALL_PREPARE || mode > SERVO_MODE_HOSTAGE_LIFT)
        return "UNKNOWN";
    return pose_name[(unsigned)mode - SERVO_MODE_BALL_PREPARE];
}
