//
// Created by halo on 2026/7/25.
//

#ifndef HNU_RAILTARGETVEHICLE_MOTOR_PID_CFG_H
#define HNU_RAILTARGETVEHICLE_MOTOR_PID_CFG_H

#define CHASSIS_KP_V_MOTOR         15.0f
#define CHASSIS_KI_V_MOTOR         0.0098f
#define CHASSIS_KD_V_MOTOR         0.0f
#define PID_DT                     1000.0f
#define CHASSIS_INTEGRAL_V_MOTOR   300.0f
#define CHASSIS_MAX_V_MOTOR        16384.0f

#define GIMBAL_KP_V_MOTOR          0.0f
#define GIMBAL_KI_V_MOTOR          0.0f
#define GIMBAL_KD_V_MOTOR          0.0f
#define GIMBAL_INTEGRAL_V_MOTOR    0.0f
#define GIMBAL_MAX_V_MOTOR         0.0f

#endif //HNU_RAILTARGETVEHICLE_MOTOR_PID_CFG_H