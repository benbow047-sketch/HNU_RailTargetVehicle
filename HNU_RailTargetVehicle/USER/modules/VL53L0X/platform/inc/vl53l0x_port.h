/**
 * @file    vl53l0x_port.h
 * @brief   VL53L0X 平台移植配置文件
 *          将此文件中的宏与 CubeMX 生成的配置对齐即可完成移植
 *
 * @note    用户需根据实际 CubeMX 配置修改此文件中的宏定义
 */

#ifndef __VL53L0X_PORT_H
#define __VL53L0X_PORT_H

#include "stm32f4xx_hal.h"
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================*/
/* I2C 配置                                                                  */
/*===========================================================================*/

/** @brief I2C 句柄（CubeMX 生成的 I2C 外设实例） */
#define VL53L0X_I2C_HANDLE       hi2c2

/** @brief I2C 操作超时 (ms) */
#define VL53L0X_I2C_TIMEOUT      100

/*===========================================================================*/
/* 传感器1 (TOF1) —— XSHUT + INT                                             */
/*===========================================================================*/

#define VL53L0X_XSHUT_PORT       XSHUT1_GPIO_Port
#define VL53L0X_XSHUT_PIN        XSHUT1_Pin
#define VL53L0X_XSHUT_LOW()      HAL_GPIO_WritePin(VL53L0X_XSHUT_PORT, VL53L0X_XSHUT_PIN, GPIO_PIN_RESET)
#define VL53L0X_XSHUT_HIGH()     HAL_GPIO_WritePin(VL53L0X_XSHUT_PORT, VL53L0X_XSHUT_PIN, GPIO_PIN_SET)

#define VL53L0X_INT_PORT         HALL_LEFT_GPIO_Port
#define VL53L0X_INT_PIN          HALL_LEFT_Pin

/*===========================================================================*/
/* 延时宏                                                                    */
/*===========================================================================*/

/** @brief 毫秒延时，直接使用 HAL_Delay */
#define VL53L0X_DELAY_MS(ms)     HAL_Delay(ms)

/*===========================================================================*/
/* VL53L0X 默认地址                                                          */
/*===========================================================================*/

/** @brief VL53L0X 默认 I2C 8-bit 地址 (0x29 << 1) */
#define VL53L0X_ADDR_DEFAULT     0x52

#ifdef __cplusplus
}
#endif

#endif /* __VL53L0X_PORT_H */
