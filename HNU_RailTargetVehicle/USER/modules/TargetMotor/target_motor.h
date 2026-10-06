#ifndef RAIL_TARGET_MOTOR_H
#define RAIL_TARGET_MOTOR_H

#include <stdint.h>

typedef struct
{
    uint8_t init_ok;
    uint8_t track_online;
    uint8_t armor_online;
    uint8_t track_control_enabled;
    uint8_t armor_control_enabled;
    uint8_t track_braking;
    int8_t auto_move_direction;
    int16_t track_speed_rpm;
    int16_t armor_speed_rpm;
    int16_t track_target_rpm;
    int16_t track_control_output;
    int16_t armor_target_rpm;
    int16_t armor_control_output;
    int16_t track_current;
    int16_t armor_current;
    uint16_t track_ecd;
    uint16_t armor_ecd;
    uint8_t track_temperature;
    uint8_t armor_temperature;
    uint32_t track_feedback_count;
    uint32_t armor_feedback_count;
} target_motor_test_state_t;

extern volatile target_motor_test_state_t g_target_motor_test_state;

void TargetMotor_Init(void);
void TargetMotor_Process(void);

#endif
