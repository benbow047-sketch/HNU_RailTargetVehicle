#include "TargetMotor/target_motor.h"

#include <string.h>

#include "Bluetooth/bluetooth.h"
#include "PID.h"
#include "can.h"
#include "dji_motor.h"
#include "ranging_task.h"

#define MOTOR_FEEDBACK_TIMEOUT_MS       100U
#define MODE_TRANSITION_STOP_MS         200U
#define ENDPOINT_REVERSAL_STOP_MS       200U
#define TRACK_COMMAND_DEADZONE          150
#define TRACK_REMOTE_RAMP_RPM_PER_SEC   900.0f
#define TRACK_AUTO_RAMP_RPM_PER_SEC     1500.0f
#define TRACK_BRAKE_STOP_RPM             30.0f
#define ARMOR_COMMAND_DEADZONE          150
#define ARMOR_RAMP_RPM_PER_SEC          200.0f
#define ARMOR_HOLD_KP_RPM_PER_DEG         1.5f
#define ARMOR_HOLD_MAX_RPM               60.0f
#define ARMOR_HOLD_DEADBAND_DEG           0.25f

/* If the physical direction is opposite, change this value to 0. */
#define TRACK_POSITIVE_DIRECTION_IS_RIGHT 1
#define AUTO_MOVE_INITIAL_DIRECTION       (-1)

static motor_config_t track_motor_config =
{
    .motor_type = M3508,
    .hcan = &hcan1,
    .rx_id = 0x202U,
    .controller = NULL,
};

static motor_config_t armor_motor_config =
{
    .motor_type = GM6020,
    .hcan = &hcan1,
    .rx_id = 0x208U,
    .controller = NULL,
};

static pid_config_t track_pid_config =
{
    .Kp = 8.0f,
    .Ki = 0.0f,
    .Kd = 0.0f,
    .dt = 1.0f,
    .MaxOut = 2000.0f,
    .DeadBand = 1.0f,
    .IntegralLimit = 0.0f,
    .Improve = PID_IMPROVE_NONE,
};

static pid_config_t armor_pid_config =
{
    .Kp = 50.0f,
    .Ki = 0.0f,
    .Kd = 0.0f,
    .dt = 1.0f,
    .MaxOut = 6000.0f,
    .DeadBand = 0.5f,
    .IntegralLimit = 0.0f,
    .Improve = PID_IMPROVE_NONE,
};

/* Dedicated zero-speed controller used only after the remote stick is released. */
static pid_config_t track_brake_pid_config =
{
    .Kp = 12.0f,
    .Ki = 0.0f,
    .Kd = 0.0f,
    .dt = 1.0f,
    .MaxOut = 4000.0f,
    .DeadBand = 0.0f,
    .IntegralLimit = 0.0f,
    .Improve = PID_IMPROVE_NONE,
};

static dji_motor_object_t *track_motor;
static dji_motor_object_t *armor_motor;
static pid_obj_t *track_pid;
static pid_obj_t *armor_pid;
static pid_obj_t *track_brake_pid;
static float track_ramped_target_rpm;
static float armor_ramped_target_rpm;
static float armor_hold_target_angle;
static uint8_t armor_hold_active;
static uint32_t last_process_tick;
static uint32_t mode_transition_tick;
static uint32_t endpoint_stop_tick;
static uint8_t endpoint_stop_active;
static uint8_t last_mode;
static int8_t auto_move_direction;

volatile target_motor_test_state_t g_target_motor_test_state;

static uint8_t motor_is_online(const dji_motor_object_t *motor, uint32_t now)
{
    return (uint8_t)((motor != NULL) &&
                     (motor->feedback_count > 0U) &&
                     ((now - motor->last_feedback_tick) <= MOTOR_FEEDBACK_TIMEOUT_MS));
}

static int16_t track_speed_control(dji_motor_measure_t measure)
{
    if ((!g_target_motor_test_state.track_control_enabled) || (track_pid == NULL))
    {
        g_target_motor_test_state.track_control_output = 0;
        return 0;
    }
    pid_obj_t *active_pid = g_target_motor_test_state.track_braking ?
                            track_brake_pid : track_pid;
    if (active_pid == NULL)
    {
        g_target_motor_test_state.track_control_output = 0;
        return 0;
    }
    const float output = pid_calculate(active_pid, (float)measure.speed_rpm,
                                       track_ramped_target_rpm);
    g_target_motor_test_state.track_control_output = (int16_t)output;
    return (int16_t)output;
}

static int16_t armor_speed_control(dji_motor_measure_t measure)
{
    if ((!g_target_motor_test_state.armor_control_enabled) || (armor_pid == NULL))
    {
        g_target_motor_test_state.armor_control_output = 0;
        return 0;
    }
    const float output = pid_calculate(armor_pid, (float)measure.speed_rpm,
                                       armor_ramped_target_rpm);
    g_target_motor_test_state.armor_control_output = (int16_t)output;
    return (int16_t)output;
}

static float ramp_towards(float current, float target, float max_step)
{
    if (current < target)
    {
        current += max_step;
        return (current > target) ? target : current;
    }
    if (current > target)
    {
        current -= max_step;
        return (current < target) ? target : current;
    }
    return current;
}

static uint8_t positive_endpoint_active(void)
{
#if TRACK_POSITIVE_DIRECTION_IS_RIGHT
    return g_ranging_state.right_endpoint;
#else
    return g_ranging_state.left_endpoint;
#endif
}

static uint8_t negative_endpoint_active(void)
{
#if TRACK_POSITIVE_DIRECTION_IS_RIGHT
    return g_ranging_state.left_endpoint;
#else
    return g_ranging_state.right_endpoint;
#endif
}

static void stop_track_immediately(void)
{
    track_ramped_target_rpm = 0.0f;
    g_target_motor_test_state.track_control_enabled = 0U;
    g_target_motor_test_state.track_control_output = 0;
    if (track_pid != NULL) pid_clear(track_pid);
}

static void handle_mode_change(uint8_t mode, uint32_t now)
{
    if (mode == last_mode) return;

    last_mode = mode;
    mode_transition_tick = now;
    endpoint_stop_active = 0U;
    auto_move_direction = AUTO_MOVE_INITIAL_DIRECTION;
    track_ramped_target_rpm = 0.0f;
    armor_ramped_target_rpm = 0.0f;
    armor_hold_active = 0U;
    g_target_motor_test_state.track_control_enabled = 0U;
    g_target_motor_test_state.armor_control_enabled = 0U;
    g_target_motor_test_state.track_control_output = 0;
    g_target_motor_test_state.armor_control_output = 0;
    if (track_pid != NULL) pid_clear(track_pid);
    if (armor_pid != NULL) pid_clear(armor_pid);
}

void TargetMotor_Init(void)
{
    memset((void *)&g_target_motor_test_state, 0, sizeof(g_target_motor_test_state));
    track_pid = pid_register(&track_pid_config);
    armor_pid = pid_register(&armor_pid_config);
    track_brake_pid = pid_register(&track_brake_pid_config);
    track_motor = dji_motor_register(&track_motor_config, track_speed_control);
    armor_motor = dji_motor_register(&armor_motor_config, armor_speed_control);

    last_process_tick = HAL_GetTick();
    mode_transition_tick = last_process_tick;
    endpoint_stop_tick = last_process_tick;
    endpoint_stop_active = 0U;
    last_mode = BT_MODE_STOP;
    auto_move_direction = AUTO_MOVE_INITIAL_DIRECTION;
    armor_hold_target_angle = 0.0f;
    armor_hold_active = 0U;
    g_target_motor_test_state.auto_move_direction = auto_move_direction;
    g_target_motor_test_state.init_ok =
        (uint8_t)((track_pid != NULL) && (track_brake_pid != NULL) &&
                  (armor_pid != NULL) &&
                  (track_motor != NULL) && (armor_motor != NULL));
}

void TargetMotor_Process(void)
{
    const uint32_t now = HAL_GetTick();
    uint32_t elapsed_ms = now - last_process_tick;
    last_process_tick = now;
    if (elapsed_ms > 20U) elapsed_ms = 20U;

    g_target_motor_test_state.track_online = motor_is_online(track_motor, now);
    g_target_motor_test_state.armor_online = motor_is_online(armor_motor, now);

    const uint8_t mode = g_bt_test_state.mode;
    handle_mode_change(mode, now);

    const uint8_t transition_complete =
        (uint8_t)((now - mode_transition_tick) >= MODE_TRANSITION_STOP_MS);
    const uint8_t common_safe =
        (uint8_t)(g_target_motor_test_state.init_ok &&
                  g_bt_test_state.link_online &&
                  (!g_bt_test_state.emergency_stop) &&
                  transition_complete);
    const uint8_t automatic_move_mode =
        (uint8_t)((mode == BT_MODE_AUTO_MOVE) || (mode == BT_MODE_AUTO_BOTH));
    const uint8_t automatic_rotate_mode =
        (uint8_t)((mode == BT_MODE_AUTO_ROTATE) || (mode == BT_MODE_AUTO_BOTH));

    if (endpoint_stop_active &&
        ((now - endpoint_stop_tick) >= ENDPOINT_REVERSAL_STOP_MS))
    {
        endpoint_stop_active = 0U;
    }

    const uint8_t ranging_ready = Ranging_AutomaticReady(now);
    if (common_safe && automatic_move_mode && ranging_ready &&
        g_target_motor_test_state.track_online && (!endpoint_stop_active))
    {
        const uint8_t reached_endpoint =
            (uint8_t)(((auto_move_direction > 0) && positive_endpoint_active()) ||
                      ((auto_move_direction < 0) && negative_endpoint_active()));
        if (reached_endpoint)
        {
            auto_move_direction = (int8_t)-auto_move_direction;
            endpoint_stop_active = 1U;
            endpoint_stop_tick = now;
            stop_track_immediately();
        }
    }
    g_target_motor_test_state.auto_move_direction = auto_move_direction;

    const uint8_t both_endpoints =
        (uint8_t)(g_ranging_state.left_endpoint && g_ranging_state.right_endpoint);
    const uint8_t automatic_move_allowed =
        (uint8_t)(automatic_move_mode && ranging_ready &&
                  (!endpoint_stop_active) && (!both_endpoints));
    const uint8_t safe_to_translate =
        (uint8_t)(common_safe && g_target_motor_test_state.track_online &&
                  ((mode == BT_MODE_REMOTE) || automatic_move_allowed));

    float track_requested_rpm = 0.0f;
    if (safe_to_translate)
    {
        if (automatic_move_allowed)
        {
            track_requested_rpm = (float)auto_move_direction *
                                  (float)g_bt_test_state.auto_track_rpm;
        }
        else if ((g_bt_test_state.move_command > TRACK_COMMAND_DEADZONE) ||
                 (g_bt_test_state.move_command < -TRACK_COMMAND_DEADZONE))
        {
            const int32_t command = g_bt_test_state.move_command;
            const int32_t magnitude = (command < 0) ? -command : command;
            const float normalized =
                (float)(magnitude - TRACK_COMMAND_DEADZONE) /
                (float)(1000 - TRACK_COMMAND_DEADZONE);
            track_requested_rpm = normalized *
                (float)g_bt_test_state.remote_track_full_scale_rpm;
            if (command < 0) track_requested_rpm = -track_requested_rpm;
        }
    }

    const uint8_t requested_endpoint_blocked =
        (uint8_t)(((track_requested_rpm > 0.0f) && positive_endpoint_active()) ||
                  ((track_requested_rpm < 0.0f) && negative_endpoint_active()));
    const uint8_t moving_into_endpoint =
        (uint8_t)(((track_ramped_target_rpm > 0.0f) && positive_endpoint_active()) ||
                  ((track_ramped_target_rpm < 0.0f) && negative_endpoint_active()));
    if (requested_endpoint_blocked || moving_into_endpoint ||
        (automatic_move_mode && (!ranging_ready || both_endpoints || endpoint_stop_active)))
    {
        track_requested_rpm = 0.0f;
        stop_track_immediately();
    }

    float track_ramp_goal = track_requested_rpm;
    if ((mode == BT_MODE_REMOTE) && (track_requested_rpm == 0.0f))
    {
        /* Joystick released: request zero speed immediately and keep PID braking. */
        track_ramped_target_rpm = 0.0f;
        track_ramp_goal = 0.0f;
    }
    else if (((track_ramped_target_rpm > 0.0f) && (track_requested_rpm < 0.0f)) ||
             ((track_ramped_target_rpm < 0.0f) && (track_requested_rpm > 0.0f)))
    {
        track_ramp_goal = 0.0f;
    }
    const float track_ramp_rate = automatic_move_mode ?
                                  TRACK_AUTO_RAMP_RPM_PER_SEC :
                                  TRACK_REMOTE_RAMP_RPM_PER_SEC;
    track_ramped_target_rpm = ramp_towards(
        track_ramped_target_rpm, track_ramp_goal,
        track_ramp_rate * ((float)elapsed_ms / 1000.0f));
    g_target_motor_test_state.track_braking =
        (uint8_t)((mode == BT_MODE_REMOTE) &&
                  (track_requested_rpm == 0.0f) &&
                  (((float)g_target_motor_test_state.track_speed_rpm > TRACK_BRAKE_STOP_RPM) ||
                   ((float)g_target_motor_test_state.track_speed_rpm < -TRACK_BRAKE_STOP_RPM)));
    if (!g_target_motor_test_state.track_braking && (track_brake_pid != NULL))
    {
        pid_clear(track_brake_pid);
    }
    g_target_motor_test_state.track_control_enabled =
        (uint8_t)(safe_to_translate && (!requested_endpoint_blocked) &&
                  (!endpoint_stop_active) &&
                  ((track_requested_rpm > 0.5f) ||
                   (track_requested_rpm < -0.5f) ||
                   g_target_motor_test_state.track_braking ||
                   (track_ramped_target_rpm > 0.5f) ||
                   (track_ramped_target_rpm < -0.5f)));
    g_target_motor_test_state.track_target_rpm = (int16_t)track_ramped_target_rpm;
    if (!g_target_motor_test_state.track_control_enabled)
    {
        g_target_motor_test_state.track_control_output = 0;
        if (track_pid != NULL) pid_clear(track_pid);
    }

    const uint8_t armor_hold_mode =
        (uint8_t)((mode == BT_MODE_REMOTE) || (mode == BT_MODE_AUTO_MOVE));
    const uint8_t safe_to_rotate =
        (uint8_t)(common_safe && g_target_motor_test_state.armor_online &&
                  (armor_hold_mode || automatic_rotate_mode));
    float armor_requested_rpm = 0.0f;
    uint8_t armor_rotation_commanded = 0U;
    if (safe_to_rotate)
    {
        if (automatic_rotate_mode)
        {
            armor_requested_rpm = (float)g_bt_test_state.auto_armor_rpm;
            armor_rotation_commanded = 1U;
        }
        else if ((g_bt_test_state.rotate_command > ARMOR_COMMAND_DEADZONE) ||
                 (g_bt_test_state.rotate_command < -ARMOR_COMMAND_DEADZONE))
        {
            const int32_t command = g_bt_test_state.rotate_command;
            const int32_t magnitude = (command < 0) ? -command : command;
            const float normalized =
                (float)(magnitude - ARMOR_COMMAND_DEADZONE) /
                (float)(1000 - ARMOR_COMMAND_DEADZONE);
            /* Quadratic response keeps small stick motions gentle. */
            armor_requested_rpm =
                (float)g_bt_test_state.remote_armor_full_scale_rpm *
                normalized * normalized;
            if (command < 0) armor_requested_rpm = -armor_requested_rpm;
            armor_rotation_commanded = 1U;
        }
    }

    if (!safe_to_rotate || armor_rotation_commanded)
    {
        armor_hold_active = 0U;
    }
    else if (armor_hold_mode && (armor_motor != NULL))
    {
        if (!armor_hold_active)
        {
            /* Capture the exact multi-turn encoder angle when rotation is released. */
            armor_hold_target_angle = armor_motor->measure.total_angle;
            armor_hold_active = 1U;
        }
        const float angle_error =
            armor_hold_target_angle - armor_motor->measure.total_angle;
        if ((angle_error > ARMOR_HOLD_DEADBAND_DEG) ||
            (angle_error < -ARMOR_HOLD_DEADBAND_DEG))
        {
            armor_requested_rpm = angle_error * ARMOR_HOLD_KP_RPM_PER_DEG;
            if (armor_requested_rpm > ARMOR_HOLD_MAX_RPM)
                armor_requested_rpm = ARMOR_HOLD_MAX_RPM;
            else if (armor_requested_rpm < -ARMOR_HOLD_MAX_RPM)
                armor_requested_rpm = -ARMOR_HOLD_MAX_RPM;
        }
    }

    float armor_ramp_goal = armor_requested_rpm;
    if (armor_hold_active)
    {
        /* Position outer loop already limits correction speed; do not delay it with the ramp. */
        armor_ramped_target_rpm = armor_requested_rpm;
    }
    else if (((armor_ramped_target_rpm > 0.0f) && (armor_requested_rpm < 0.0f)) ||
             ((armor_ramped_target_rpm < 0.0f) && (armor_requested_rpm > 0.0f)))
    {
        armor_ramp_goal = 0.0f;
    }
    if (!armor_hold_active)
    {
        armor_ramped_target_rpm = ramp_towards(
            armor_ramped_target_rpm, armor_ramp_goal,
            ARMOR_RAMP_RPM_PER_SEC * ((float)elapsed_ms / 1000.0f));
    }
    g_target_motor_test_state.armor_control_enabled =
        (uint8_t)(safe_to_rotate &&
                  (armor_hold_active || automatic_rotate_mode ||
                   (armor_ramped_target_rpm > 0.2f) ||
                   (armor_ramped_target_rpm < -0.2f)));
    g_target_motor_test_state.armor_target_rpm = (int16_t)armor_ramped_target_rpm;
    if (!g_target_motor_test_state.armor_control_enabled)
    {
        g_target_motor_test_state.armor_control_output = 0;
        if (armor_pid != NULL) pid_clear(armor_pid);
    }

    dji_motor_control();

    if (track_motor != NULL)
    {
        g_target_motor_test_state.track_speed_rpm = track_motor->measure.speed_rpm;
        g_target_motor_test_state.track_current = track_motor->measure.real_current;
        g_target_motor_test_state.track_ecd = track_motor->measure.ecd;
        g_target_motor_test_state.track_temperature = track_motor->measure.temperature;
        g_target_motor_test_state.track_feedback_count = track_motor->feedback_count;
    }
    if (armor_motor != NULL)
    {
        g_target_motor_test_state.armor_speed_rpm = armor_motor->measure.speed_rpm;
        g_target_motor_test_state.armor_current = armor_motor->measure.real_current;
        g_target_motor_test_state.armor_ecd = armor_motor->measure.ecd;
        g_target_motor_test_state.armor_temperature = armor_motor->measure.temperature;
        g_target_motor_test_state.armor_feedback_count = armor_motor->feedback_count;
    }
}
