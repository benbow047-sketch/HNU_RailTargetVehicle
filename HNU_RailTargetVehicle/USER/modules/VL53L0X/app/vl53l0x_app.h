/**
 * @file    vl53l0x_app.h
 * @brief   VL53L0X 应用层头文件
 *          提供测量模式定义、初始化与测距 API
 */

#ifndef __VL53L0X_APP_H
#define __VL53L0X_APP_H

#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================*/
/* 测量模式宏                                                                */
/*===========================================================================*/
#define VL53L0X_MODE_DEFAULT        0   /* 默认模式       */
#define VL53L0X_MODE_HIGH_ACCURACY  1   /* 高精度模式     */
#define VL53L0X_MODE_LONG_RANGE     2   /* 长距离模式     */
#define VL53L0X_MODE_HIGH_SPEED     3   /* 高速模式       */

/*===========================================================================*/
/* 模式配置结构体                                                            */
/*===========================================================================*/
typedef struct {
    FixPoint1616_t signalLimit;          /* 信号阈值                    */
    FixPoint1616_t sigmaLimit;           /* Sigma 阈值                  */
    uint32_t       timingBudget;         /* 测量时间预算 (微秒)          */
    uint8_t        preRangeVcselPeriod;  /* 预范围 VCSEL 周期           */
    uint8_t        finalRangeVcselPeriod;/* 最终范围 VCSEL 周期          */
} VL53L0X_ModeConfig_t;

/*===========================================================================*/
/* 全局变量                                                                  */
/*===========================================================================*/
extern VL53L0X_Dev_t                   vl53l0x_dev;    /* 设备句柄       */
extern VL53L0X_RangingMeasurementData_t vl53l0x_data;  /* 测量结果       */
extern volatile uint16_t               Distance_data;  /* 距离值 (mm)    */
extern volatile uint8_t                g_tof_ready;    /* 数据就绪标志   */
extern const VL53L0X_ModeConfig_t      Mode_data[];    /* 4 种模式参数   */

/*===========================================================================*/
/* API 函数                                                                  */
/*===========================================================================*/

/** @brief 初始化 VL53L0X 传感器 */
VL53L0X_Error vl53l0x_init(VL53L0X_Dev_t *dev);

/** @brief 设置测量模式（需在测距前调用） */
VL53L0X_Error vl53l0x_set_mode(VL53L0X_Dev_t *dev, uint8_t mode);

/** @brief 执行一次单次测距，结果存入 pdata 和 Distance_data */
VL53L0X_Error vl53l0x_start_single_test(VL53L0X_Dev_t *dev,
    VL53L0X_RangingMeasurementData_t *pdata, char *buf);

/** @brief 打印 PAL 错误信息 */
void print_pal_error(VL53L0X_Error Status);

/** @brief 获取模式名称字符串 */
void mode_string(uint8_t mode, char *buf);

/** @brief 打印设备信息 */
void vl53l0x_info(void);

/** @brief 执行一次测距并打印结果（用户最常用） */
void One_measurement(uint8_t mode);

/** @brief 启动连续测距 + 中断通知模式 */
VL53L0X_Error vl53l0x_interrupt_start(VL53L0X_Dev_t *dev, uint8_t mode);

/** @brief 中断回调中读取测距数据（由 HAL_GPIO_EXTI_Callback 调用） */
void vl53l0x_interrupt_handler(VL53L0X_Dev_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* __VL53L0X_APP_H */