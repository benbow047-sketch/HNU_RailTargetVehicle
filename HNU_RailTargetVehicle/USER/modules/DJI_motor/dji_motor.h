/**
 * @file dji_motor.h
 * @author neozng
 * @version 0.3
 * @date 2022-11-01
 * @copyright Copyright (c) 2022 HNU YueLu EC all rights reserved
 *
 */

 /*
 * Change Logs:
 * Date            Author          Notes
 * 2023-08-23      ChuShicheng     first version
 * 2026-07-25      halo            适配裸机 BSP_CAN 层, 移除 RT-Thread 依赖
 */
#ifndef _DJI_MOTOR_H
#define _DJI_MOTOR_H

#include "motor_def.h"
#include <stdint.h>

/**
 * @brief DJI motor feedback
 */
typedef struct
{
    /* 以下是处理得出的数据 */
    float angle_single_round; // 单圈角度
    float speed_aps;          // 角速度,单位为:度/秒
    float total_angle;        // 总角度,注意方向
    int32_t total_round;      // 总圈数,注意方向
    float  target;            // 目标值(输出轴扭矩/速度/角度(单位度))

    /* 以下是电调直接回传的数据 */
    uint16_t ecd;             // 0-8191
    uint16_t last_ecd;        // 上一次读取的编码器值
    int16_t  speed_rpm;       // 电机的转速值
    int16_t  speed_rad;       // rpm = 角速度（rad/s） ×60/2π
    int16_t real_current;     // 实际转矩电流
    uint8_t temperature;      // Celsius
} dji_motor_measure_t;

/**
 * @brief DJI intelligent motor typedef
 */
typedef struct dji_motor_object
{
    CAN_HandleTypeDef *hcan;                // 电机挂载的CAN外设句柄
    dji_motor_measure_t measure;            // 电机测量值

    uint32_t tx_id;                         // 发送id(主发)
    uint32_t rx_id;                         // 接收id(主收)
    /* 分组发送设置 */
    uint8_t send_group;                     // 同一帧报文分组
    uint8_t message_num;                    // 一帧报文中位置

    motor_type_e motor_type;                // 电机类型
    motor_working_type_e stop_flag;         // 启停标志

    /* 电机离线检测 */
    uint32_t last_feedback_tick;            // 上次收到反馈的时刻 (HAL_GetTick())
    uint32_t feedback_count;                 // 成功解析的反馈帧计数

    /* 电机控制相关 */
    void *controller;                       // 电机控制器
    int16_t (*control)(dji_motor_measure_t measure);   // 控制电机的接口 用户可以自定义,返回值为16位的电压或电流值
} dji_motor_object_t;

/**
 * @brief 调用此函数注册一个DJI智能电机,需要传递较多的初始化参数,请在application初始化的时候调用此函数
 *        推荐传参时像标准库一样构造initStructure然后传入此函数.
 *        recommend: type xxxinitStructure = {.member1=xx,
 *                                            .member2=xx,
 *                                             ....};
 *        请注意不要在一条总线上挂载过多的电机(超过6个),若一定要这么做,请降低每个电机的反馈频率(设为500Hz),
 *        并减小dji_motor_control()任务的运行频率.
 *
 * @attention M3508和M2006的反馈报文都是0x200+id,而GM6020的反馈是0x204+id,请注意前两者和后者的id不要冲突.
 *            如果产生冲突,在初始化电机的时候会进入IDcrash_Handler(),可以通过debug来判断是否出现冲突.
 *
 * @param config 电机初始化结构体,包含了电机控制设置,电机PID参数设置,电机类型以及电机挂载的CAN设置
 * @param control 电机控制函数指针, 输入 dji_motor_measure_t, 返回 int16_t 电流/电压值
 *
 * @return dji_motor_object_t*
 */
dji_motor_object_t *dji_motor_register(motor_config_t *config, int16_t (*control)(dji_motor_measure_t measure));

/**
 * @brief 该函数需要被周期性调用 (1kHz), 运行所有已注册电机的控制算法并发送CAN报文
 */
void dji_motor_control(void);

/**
 * @brief 停止电机,注意不是将设定值设为零,而是直接给电机发送的电流值置零
 */
void dji_motor_relax(dji_motor_object_t *motor);

/**
 * @brief 启动电机,此时电机会响应设定值
 *        初始化时不需要此函数,因为stop_flag的默认值为0
 */
void dji_motor_enable(dji_motor_object_t *motor);

/**
 * @brief 电机反馈报文接收回调函数, 在 can1_rx_callback / can2_rx_callback 中调用
 *
 * @param hcan 接收到报文的CAN外设句柄
 * @param id 接收到的报文的id
 * @param data 接收到的报文的数据
 */
void dji_motot_rx_callback(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data);

/**
 * @brief 更新超电CAN发送帧数据
 * @note  字节顺序必须与超电板接收顺序一致:
 *        d0: Enable, d1: Charge, d2: PowerLimit, d3: ChargePower
 */
void dji_supercap_update_frame(uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3,
                               uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7);

float decode_6623_aps(float inputs);

#endif /* _DJI_MOTOR_H */
