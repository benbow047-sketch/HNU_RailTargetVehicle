/*
* Change Logs:
* Date            Author          Notes
* 2023-08-23      ChuShicheng     first version (RT-Thread)
* 2026-07-25      halo            适配裸机 BSP_CAN 层, 移除 RT-Thread 依赖
*/
#include "dji_motor.h"
#include "BSP_CAN.h"
#include "can.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ======================== 简易日志宏 (裸机替代 RT-Thread ulog) ======================== */
/* USART1 is reserved for the binary Bluetooth protocol. */
#define LOG_E(fmt, ...)  ((void)0)
#define LOG_W(fmt, ...)  ((void)0)

/* ======================== 常量与宏 ======================== */
#define DJI_MOTOR_CNT 14               // 默认波特率下，实测挂载电机极限数量
/* 滤波系数设置为1的时候即关闭滤波 */
#define SPEED_SMOOTH_COEF   0.85f      // 最好大于0.85
#define CURRENT_SMOOTH_COEF 0.9f       // 必须大于0.9
#define ECD_ANGLE_COEF_DJI  0.043945f  // (360/8192),将编码器值转化为角度制
#define OFFLINE_TIMEOUT_MS  50         // 电机离线判定超时 (ms)

/* ======================== CAN 报文结构体 (裸机替代 rt_can_msg) ======================== */
typedef struct
{
    uint32_t id;
    uint8_t  data[8];
} can_msg_t;

/* ======================== 全局变量 ======================== */
float     power_all        = 0;
uint16_t  power_limit_out  = 60;
uint16_t  power_limit_test = 0;

#ifdef BSP_USING_POWER_LIMIT
extern uint8_t powerOverloadFlag;       // 超功率标志位
#endif

static uint8_t idx = 0;                  // register idx, 该文件的全局电机索引, 在注册时使用

/* DJI电机的实例, 此处仅保存指针 */
static dji_motor_object_t *dji_motor_obj[DJI_MOTOR_CNT] = {NULL};

/**
 * @brief 由于DJI电机发送以四个一组的形式进行,故对其进行特殊处理,用6个(2can×3group)can_msg专门负责发送
 *        该变量将在 dji_motor_control() 中使用,分组在 motor_send_grouping()中进行
 *
 * C610(m2006)/C620(m3508):0x1ff,0x200;
 * GM6020:0x1ff,0x2ff
 * 反馈(rx_id): GM6020: 0x204+id ; C610/C620: 0x200+id
 * can1: [0]:0x1FF,[1]:0x200,[2]:0x2FF
 * can2: [3]:0x1FF,[4]:0x200,[5]:0x2FF
 * [6]: 超电报文，通过CAN1发送
 */
static can_msg_t send_msg[7] = {
    [0] = {.id = 0x1FF, .data = {0}},
    [1] = {.id = 0x200, .data = {0}},
    [2] = {.id = 0x2FF, .data = {0}},
    [3] = {.id = 0x1FF, .data = {0}},
    [4] = {.id = 0x200, .data = {0}},
    [5] = {.id = 0x2FF, .data = {0}},
    [6] = {.id = CAN_ID_BUS_TO_SUPERCAP, .data = {0}},
};

/**
 * @brief 6个用于确认是否有电机注册到send_msg中的标志位,防止发送空帧,此变量将在dji_motor_control()使用
 *        flag的初始化在 motor_send_grouping()中进行, 0~2为CAN1, 3~5为CAN2
 */
static uint8_t sender_enable_flag[7] = {0};

/* ======================== 超电帧更新 ======================== */

/**
 * @brief 更新超电CAN发送帧数据
 * @note  字节顺序必须与超电板接收顺序一致:
 *        d0: Enable, d1: Charge, d2: PowerLimit, d3: ChargePower
 */
void dji_supercap_update_frame(uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3,
                               uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7)
{
    __disable_irq();   // 进入临界区
    send_msg[6].data[0] = d0;
    send_msg[6].data[1] = d1;
    send_msg[6].data[2] = d2;
    send_msg[6].data[3] = d3;
    send_msg[6].data[4] = d4;
    send_msg[6].data[5] = d5;
    send_msg[6].data[6] = d6;
    send_msg[6].data[7] = d7;
    sender_enable_flag[6] = 1;
    __enable_irq();    // 退出临界区
}

/* ======================== 电机分组 ======================== */

/**
 * @brief 根据电调/拨码开关上的ID,根据说明书的默认id分配方式计算发送ID和接收ID,
 *        并对电机进行分组以便处理多电机控制命令
 */
static void motor_send_grouping(dji_motor_object_t *motor)
{
    uint8_t motor_id       = 0;
    uint8_t motor_send_num = 0;
    uint8_t motor_group    = 0;

    switch (motor->motor_type)
    {
        case M2006:
        case M3508:
            motor_id = motor->rx_id - 0x200; // 下标从零开始,先减一方便赋值
            if (motor_id <= 4) // 根据ID分组: id 1~4 → 0x200 组
            {
                motor_send_num = motor_id - 1;
                motor_group    = (motor->hcan == &hcan2) ? 4 : 1;
            }
            else               // id 5~8 → 0x1FF 组
            {
                motor_send_num = motor_id - 5;
                motor_group    = (motor->hcan == &hcan2) ? 3 : 0;
            }

            // 计算接收id并设置分组发送id
            sender_enable_flag[motor_group] = 1;
            motor->message_num = motor_send_num;
            motor->send_group  = motor_group;

            // 检查是否发生id冲突
            for (size_t i = 0; i < idx; ++i)
            {
                if (dji_motor_obj[i]->hcan == motor->hcan
                    && dji_motor_obj[i]->rx_id == motor->rx_id)
                {
                    LOG_E("ID crash! rx_id=0x%03X, hcan=%s",
                          motor->rx_id,
                          (motor->hcan == &hcan1) ? "can1" : "can2");
                    while (1); // 6020的id 1-4和2006/3508的id 5-8会发生冲突
                }
            }
            break;

        case GM6020:
            motor_id = motor->rx_id - 0x204;
            if (motor_id <= 4) // id 1~4 → 0x1FF 组
            {
                motor_send_num = motor_id - 1;
                motor_group    = (motor->hcan == &hcan2) ? 3 : 0;
            }
            else               // id 5~8 → 0x2FF 组
            {
                motor_send_num = motor_id - 5;
                motor_group    = (motor->hcan == &hcan2) ? 5 : 2;
            }

            sender_enable_flag[motor_group] = 1;
            motor->message_num = motor_send_num;
            motor->send_group  = motor_group;

            for (size_t i = 0; i < idx; ++i)
            {
                if (dji_motor_obj[i]->hcan == motor->hcan
                    && dji_motor_obj[i]->rx_id == motor->rx_id)
                {
                    LOG_E("ID crash! rx_id=0x%03X, hcan=%s",
                          motor->rx_id,
                          (motor->hcan == &hcan1) ? "can1" : "can2");
                    while (1);
                }
            }
            break;

        default:
            while (1)
                LOG_E("You must not register other motors using the API of DJI motor.");
    }
}

/* ======================== 反馈解码 ======================== */

/**
 * @brief 根据返回的CAN数据对反馈报文进行解析
 *
 * @param motor 电机实例指针
 * @param data  8字节CAN报文数据
 */
static void decode_dji_motor(dji_motor_object_t *motor, uint8_t *data)
{
    uint8_t *rxbuff = data;
    dji_motor_measure_t *measure = &motor->measure;

    // 判定电机未离线，更新最后收到反馈的时间戳
    motor->last_feedback_tick = HAL_GetTick();
    motor->feedback_count++;

    // 解析数据并对电流和速度进行滤波,电机的反馈报文具体格式见电机说明手册
    measure->last_ecd = measure->ecd;
    measure->ecd = ((uint16_t)rxbuff[0]) << 8 | rxbuff[1];
    measure->angle_single_round = ECD_ANGLE_COEF_DJI * (float)measure->ecd;
    measure->speed_rpm =
        (1.0f - SPEED_SMOOTH_COEF) * measure->speed_rpm
        + SPEED_SMOOTH_COEF * (float)((int16_t)(rxbuff[2] << 8 | rxbuff[3]));
    measure->speed_aps =
        (1.0f - SPEED_SMOOTH_COEF) * measure->speed_aps
        + RPM_2_ANGLE_PER_SEC * SPEED_SMOOTH_COEF * (float)((int16_t)(rxbuff[2] << 8 | rxbuff[3]));
    measure->real_current =
        (1.0f - CURRENT_SMOOTH_COEF) * measure->real_current
        + CURRENT_SMOOTH_COEF * (float)((int16_t)(rxbuff[4] << 8 | rxbuff[5]));
    measure->temperature = rxbuff[6];
    measure->speed_rad = measure->speed_rpm * 0.10472f;  // 转子转速 rad/s

    // 多圈角度计算,前提是假设两次采样间电机转过的角度小于180°
    if (measure->ecd - measure->last_ecd > 4096)
        measure->total_round--;
    else if (measure->ecd - measure->last_ecd < -4096)
        measure->total_round++;
    measure->total_angle = measure->total_round * 360 + measure->angle_single_round;
}

/* ======================== BSP_CAN 接收回调重写 ======================== */

/**
 * @brief CAN1 接收回调 — 重写 BSP_CAN 中的 __weak 定义
 * @note  在 HAL_CAN_RxFifo0MsgPendingCallback 中断上下文中被调用
 */
void can1_rx_callback(uint32_t id, uint8_t *data)
{
    dji_motot_rx_callback(&hcan1, id, data);
}

/**
 * @brief CAN2 接收回调 — 重写 BSP_CAN 中的 __weak 定义
 * @note  在 HAL_CAN_RxFifo0MsgPendingCallback 中断上下文中被调用
 */
void can2_rx_callback(uint32_t id, uint8_t *data)
{
    dji_motot_rx_callback(&hcan2, id, data);
}

/**
 * @brief 电机反馈报文接收回调函数, 在 can1_rx_callback / can2_rx_callback 中调用
 *
 * @param hcan 接收到报文的CAN外设句柄
 * @param id   接收到的报文的id
 * @param data 接收到的报文的数据
 */
void dji_motot_rx_callback(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data)
{
    // 找到对应的实例后再调用decode_dji_motor进行解析
    for (size_t i = 0; i < idx; ++i)
    {
        if (dji_motor_obj[i]->hcan == hcan && dji_motor_obj[i]->rx_id == id)
        {
            decode_dji_motor(dji_motor_obj[i], data);
        }
    }
}

/* ======================== 电机启停控制 ======================== */

void dji_motor_relax(dji_motor_object_t *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

void dji_motor_enable(dji_motor_object_t *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

/* ======================== 电机控制主循环 ======================== */

/**
 * @brief 运算所有电机实例的控制器, 发送控制报文
 * @note  需以 1kHz 频率周期性调用
 */
void dji_motor_control(void)
{
#if !defined(BSP_USING_DEFAULT_MODE) && !defined(BSP_USING_POWER_LIMIT)
#define BSP_USING_DEFAULT_MODE
#endif

#ifdef BSP_USING_DEFAULT_MODE
    dji_motor_object_t *motor;
    dji_motor_measure_t measure;
    uint8_t  group, num;    // 电机组号和组内编号
    int16_t  set = 0;       // 电机控制器计算得到的输出值
    uint32_t now = HAL_GetTick();

    // 遍历所有电机实例,运行控制算法并填入报文
    for (size_t i = 0; i < idx; ++i)
    {
        motor   = dji_motor_obj[i];
        measure = motor->measure;

        // 离线检测
        if (now - motor->last_feedback_tick > OFFLINE_TIMEOUT_MS)
        {
            LOG_W("Motor lost! hcan=%s, rx_id=0x%03X",
                  (motor->hcan == &hcan1) ? "can1" : "can2",
                  motor->rx_id);
        }

        set = motor->control(measure); // 调用对接的电机控制器计算

        // 分组填入发送数据
        group = motor->send_group;
        num   = motor->message_num;
        send_msg[group].data[2 * num]     = (uint8_t)(set >> 8);
        send_msg[group].data[2 * num + 1] = (uint8_t)(set & 0x00FF);

        // 若该电机处于停止状态,直接将buff置零
        if (motor->stop_flag == MOTOR_STOP)
            memset(send_msg[group].data + 2 * num, 0, 2);
    }

    // 遍历flag,检查是否要发送这一帧报文
    for (size_t i = 0; i < 7; ++i)
    {
        if (sender_enable_flag[i])
        {
            // 0~2: CAN1, 3~5: CAN2, 6: CAN1 (超电)
            CAN_HandleTypeDef *hcan = (i < 3 || i == 6) ? &hcan1 : &hcan2;

            if (CANSendMessage(hcan, send_msg[i].id, send_msg[i].data, 8) != 0)
            {
                LOG_W("CAN send failed! hcan=%s, id=0x%03X",
                      (hcan == &hcan1) ? "can1" : "can2",
                      send_msg[i].id);
            }
        }
    }
#endif /* BSP_USING_DEFAULT_MODE */

#ifdef BSP_USING_POWER_LIMIT
    dji_motor_object_t *motor;
    dji_motor_measure_t measure;
    uint8_t  group, num;    // 电机组号和组内编号
    int16_t  set = 0;       // 电机控制器计算得到的输出值
    int16_t  motor_current_set[4] = {0};
    uint8_t  id;
    uint8_t  group1[4] = {0}, num1[4] = {0};
    uint32_t now = HAL_GetTick();

    // 遍历所有电机实例,运行控制算法并填入报文
    for (size_t i = 0; i < idx; ++i)
    {
        motor   = dji_motor_obj[i];
        measure = motor->measure;

        // 离线检测
        if (now - motor->last_feedback_tick > OFFLINE_TIMEOUT_MS)
        {
            LOG_W("Motor lost! hcan=%s, rx_id=0x%03X",
                  (motor->hcan == &hcan1) ? "can1" : "can2",
                  motor->rx_id);
        }

        // 当电机为底盘电机，即挂载在CAN1总线且为3508电机，使用底盘功率限制，其余正常填入报文
        if ((motor->motor_type == M3508) && (motor->hcan == &hcan1))
        {
            switch (motor->rx_id)
            {
                case 0x201:
                case 0x202:
                case 0x203:
                case 0x204:
                {
                    int index = motor->rx_id - 0x201; // 计算索引0-3
                    if (powerOverloadFlag == 1)        // 超过功率，发送计算出来的转子电流目标值
                    {
                        motor_current_set[index] = (int16_t)PowerCtrl_Info.Output[index];
                    }
                    else
                    {
                        motor_current_set[index] = motor->control(measure);
                    }
                    LIMIT_MIN_MAX(motor_current_set[index], -7000, 7000); // 限幅

                    group1[index] = motor->send_group;
                    num1[index]   = motor->message_num;

                    for (int j = 0; j < 4; ++j)
                    {
                        send_msg[group1[j]].data[2 * num1[j]]     = (uint8_t)(motor_current_set[j] >> 8);
                        send_msg[group1[j]].data[2 * num1[j] + 1] = (uint8_t)(motor_current_set[j] & 0x00FF);
                    }

                    if (motor->stop_flag == MOTOR_STOP)
                    {
                        for (int j = 0; j < 4; ++j)
                        {
                            memset(send_msg[group1[j]].data + 2 * num1[j], 0, 2);
                        }
                    }
                }
                break;
            }
        }
        else
        {
            set = motor->control(measure);
            group = motor->send_group;
            num   = motor->message_num;
            send_msg[group].data[2 * num]     = (uint8_t)(set >> 8);
            send_msg[group].data[2 * num + 1] = (uint8_t)(set & 0x00FF);

            if (motor->stop_flag == MOTOR_STOP)
                memset(send_msg[group].data + 2 * num, 0, 2);
        }
    }

    // 遍历flag,检查是否要发送这一帧报文
    for (size_t i = 0; i < 7; ++i)
    {
        if (sender_enable_flag[i])
        {
            CAN_HandleTypeDef *hcan = (i < 3 || i == 6) ? &hcan1 : &hcan2;

            if (CANSendMessage(hcan, send_msg[i].id, send_msg[i].data, 8) != 0)
            {
                LOG_W("CAN send failed! hcan=%s, id=0x%03X",
                      (hcan == &hcan1) ? "can1" : "can2",
                      send_msg[i].id);
            }
        }
    }
#endif /* BSP_USING_POWER_LIMIT */
}

/* ======================== 电机注册 ======================== */

/**
 * @brief 电机初始化,返回一个电机实例
 * @param config  电机配置
 * @param control 电机控制函数指针
 * @return dji_motor_object_t* 电机实例指针
 */
dji_motor_object_t *dji_motor_register(motor_config_t *config,
                                       int16_t (*control)(dji_motor_measure_t measure))
{
    dji_motor_object_t *object = (dji_motor_object_t *)malloc(sizeof(dji_motor_object_t));
    if (object == NULL)
    {
        LOG_E("malloc failed for motor object!");
        while (1);
    }
    memset(object, 0, sizeof(dji_motor_object_t));

    // 对接用户配置的 motor_config
    object->motor_type = config->motor_type;     // 6020 or 2006 or 3508
    object->rx_id      = config->rx_id;          // 电机接收报文的ID
    object->hcan       = config->hcan;           // 挂载的CAN外设
    object->control    = control;                // 电机控制器

    // 电机分组,因为至多4个电机可以共用一帧CAN控制报文
    motor_send_grouping(object);

    // 初始化离线检测时间戳
    object->last_feedback_tick = HAL_GetTick();

    dji_motor_enable(object);
    dji_motor_obj[idx++] = object;
    return object;
}

/* ======================== GM6623 辅助函数 ======================== */

/**
 * @brief GM6623 角度解码 (占位实现)
 */
float decode_6623_aps(float inputs)
{
    // TODO: 根据实际 GM6623 协议实现解码
    (void)inputs;
    return 0.0f;
}
