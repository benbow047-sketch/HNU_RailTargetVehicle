/**
 * @file motor_def.h
 * @author neozng
 * @brief  电机通用的数据结构定义
 * @version beta
 * @date 2022-11-01
 *
 * @copyright Copyright (c) 2022 HNU YueLu EC all rights reserved
 *
 */

#ifndef _MOTOR_DEF_H
#define _MOTOR_DEF_H

#include <stdint.h>
#include "main.h"

#ifndef PI
#define PI 3.1415926535f
#endif
#define PI2 (PI * 2.0f) // 2 pi

#define RAD_2_DEGREE 57.2957795f    // 180/pi
#define DEGREE_2_RAD 0.01745329252f // pi/180

#define RPM_2_ANGLE_PER_SEC 6.0f       // ×360°/60sec
#define RPM_2_RAD_PER_SEC 0.104719755f // ×2pi/60sec

#define LIMIT_MIN_MAX(x, min, max) (x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))

#define CAN_CHASSIS &hcan1
#define CAN_GIMBAL  &hcan2


/* 超电CAN发送帧ID */
#ifndef CAN_ID_BUS_TO_SUPERCAP
#define CAN_ID_BUS_TO_SUPERCAP 0x210
#endif

typedef enum
{
    MOTOR_STOP = 0,
    MOTOR_ENALBED = 1,
} motor_working_type_e;

/* 电机类型枚举 */
typedef enum
{
    MOTOR_TYPE_NONE = 0,
    GM6020,
    M3508,
    M2006,
    HT04,
    MF9025,
    GM6623,
    //SUPERCAP
} motor_type_e;

/* 电机配置结构体 */
typedef struct
{
    motor_type_e motor_type;
    CAN_HandleTypeDef *hcan;         // CAN外设句柄 (&hcan1 或 &hcan2)
    uint32_t tx_id;                  // 发送id(主发)
    uint32_t rx_id;                  // 接收id(主收)
    void *controller;                // 电机控制器(预留)
} motor_config_t;

#endif // _MOTOR_DEF_H