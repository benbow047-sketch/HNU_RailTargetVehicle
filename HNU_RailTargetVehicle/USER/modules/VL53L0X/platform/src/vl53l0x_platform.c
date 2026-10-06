/**
 * @file    vl53l0x_platform.c
 * @brief   VL53L0X 平台抽象层 —— HAL I2C 实现
 *
 *          所有 VL53L0X 寄存器读写通过 HAL I2C 外设完成。
 *          移植到其他 MCU 时只需重写此文件中的 I2C 调用。
 */

#include "vl53l0x_platform.h"

#include "i2c.h"

/*===========================================================================*/
/* 内部辅助：I2C 地址格式转换                                               */
/*===========================================================================*/

/**
 * @brief  将 ST API 的 8-bit I2C 地址转为 HAL 所需格式
 * @note   ST API 中使用 (7-bit addr << 1) 格式，如 0x29 << 1 = 0x52
 *         HAL_I2C_Mem_Write 的 DevAddress 参数同样使用此格式，直接透传
 */
#define VL53L0X_I2C_ADDR(dev)  ((uint16_t)((dev)->I2cDevAddr))

/*===========================================================================*/
/* 多字节读写（核心函数，其他函数均基于此实现）                               */
/*===========================================================================*/

/**
 * @brief  向 VL53L0X 寄存器连续写入多字节
 * @param  Dev     设备句柄
 * @param  index   寄存器地址 (8-bit)
 * @param  pdata   数据缓冲区
 * @param  count   写入字节数
 * @retval VL53L0X_ERROR_NONE              成功
 * @retval VL53L0X_ERROR_INVALID_PARAMS    count 超过限制
 * @retval VL53L0X_ERROR_CONTROL_INTERFACE I2C 通信失败
 */
VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index,
                                  uint8_t *pdata, uint32_t count)
{
    if (count >= VL53L0X_MAX_I2C_XFER_SIZE)
        return VL53L0X_ERROR_INVALID_PARAMS;

    if (HAL_I2C_Mem_Write(&VL53L0X_I2C_HANDLE,
                          VL53L0X_I2C_ADDR(Dev),
                          index,
                          I2C_MEMADD_SIZE_8BIT,
                          pdata,
                          (uint16_t)count,
                          VL53L0X_I2C_TIMEOUT) != HAL_OK)
        return VL53L0X_ERROR_CONTROL_INTERFACE;

    return VL53L0X_ERROR_NONE;
}

/**
 * @brief  从 VL53L0X 寄存器连续读取多字节
 * @param  Dev     设备句柄
 * @param  index   寄存器地址 (8-bit)
 * @param  pdata   数据缓冲区（输出）
 * @param  count   读取字节数
 * @retval VL53L0X_ERROR_NONE              成功
 * @retval VL53L0X_ERROR_INVALID_PARAMS    count 超过限制
 * @retval VL53L0X_ERROR_CONTROL_INTERFACE I2C 通信失败
 */
VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index,
                                 uint8_t *pdata, uint32_t count)
{
    if (count >= VL53L0X_MAX_I2C_XFER_SIZE)
        return VL53L0X_ERROR_INVALID_PARAMS;

    if (HAL_I2C_Mem_Read(&VL53L0X_I2C_HANDLE,
                         VL53L0X_I2C_ADDR(Dev),
                         index,
                         I2C_MEMADD_SIZE_8BIT,
                         pdata,
                         (uint16_t)count,
                         VL53L0X_I2C_TIMEOUT) != HAL_OK)
        return VL53L0X_ERROR_CONTROL_INTERFACE;

    return VL53L0X_ERROR_NONE;
}

/*===========================================================================*/
/* 单字节 / 双字节 / 四字节 写入                                                  */
/*===========================================================================*/

/**
 * @brief  向 VL53L0X 寄存器写入 1 个字节
 */
VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data)
{
    return VL53L0X_WriteMulti(Dev, index, &data, 1);
}

/**
 * @brief  向 VL53L0X 寄存器写入 2 个字节（大端，MSB 在前）
 */
VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)(data >> 8);       /* 高字节在前 */
    buf[1] = (uint8_t)(data & 0xFF);

    /* 寄存器地址非 2 字节对齐时需要分两次写入 */
    if (index % 2 == 1) {
        VL53L0X_Error st;
        st = VL53L0X_WriteMulti(Dev, index,     &buf[0], 1);
        if (st != VL53L0X_ERROR_NONE) return st;
        st = VL53L0X_WriteMulti(Dev, index + 1, &buf[1], 1);
        return st;
    }

    return VL53L0X_WriteMulti(Dev, index, buf, 2);
}

/**
 * @brief  向 VL53L0X 寄存器写入 4 个字节（大端，MSB 在前）
 */
VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data)
{
    uint8_t buf[4];
    buf[0] = (uint8_t)(data >> 24);
    buf[1] = (uint8_t)((data & 0x00FF0000) >> 16);
    buf[2] = (uint8_t)((data & 0x0000FF00) >> 8);
    buf[3] = (uint8_t)(data & 0x000000FF);

    return VL53L0X_WriteMulti(Dev, index, buf, 4);
}

/*===========================================================================*/
/* 读-改-写                                                                 */
/*===========================================================================*/

/**
 * @brief  对 VL53L0X 寄存器进行读-改-写操作
 * @param  Dev      设备句柄
 * @param  index    寄存器地址
 * @param  AndData  与掩码（清除指定位）
 * @param  OrData   或掩码（置位指定位）
 *         结果: data = (data & AndData) | OrData
 */
VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index,
                                  uint8_t AndData, uint8_t OrData)
{
    VL53L0X_Error Status;
    uint8_t data;

    Status = VL53L0X_RdByte(Dev, index, &data);
    if (Status != VL53L0X_ERROR_NONE)
        return Status;

    data = (data & AndData) | OrData;

    return VL53L0X_WrByte(Dev, index, data);
}

/*===========================================================================*/
/* 单字节 / 双字节 / 四字节 读取                                           */
/*===========================================================================*/

/**
 * @brief  从 VL53L0X 寄存器读取 1 个字节
 */
VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data)
{
    return VL53L0X_ReadMulti(Dev, index, data, 1);
}

/**
 * @brief  从 VL53L0X 寄存器读取 2 个字节（大端，MSB 在前）
 */
VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data)
{
    VL53L0X_Error Status;
    uint8_t buf[2];

    Status = VL53L0X_ReadMulti(Dev, index, buf, 2);
    if (Status != VL53L0X_ERROR_NONE)
        return Status;

    *data = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
    return VL53L0X_ERROR_NONE;
}

/**
 * @brief  从 VL53L0X 寄存器读取 4 个字节（大端，MSB 在前）
 */
VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data)
{
    VL53L0X_Error Status;
    uint8_t buf[4];

    Status = VL53L0X_ReadMulti(Dev, index, buf, 4);
    if (Status != VL53L0X_ERROR_NONE)
        return Status;

    *data = ((uint32_t)buf[0] << 24) |
            ((uint32_t)buf[1] << 16) |
            ((uint32_t)buf[2] << 8)  |
            ((uint32_t)buf[3]);
    return VL53L0X_ERROR_NONE;
}

/*===========================================================================*/
/* 轮询延时                                                                  */
/*===========================================================================*/

/**
 * @brief  平台轮询延时
 * @note   VL53L0X API 内部使用，原软件 I2C 中用于产生微秒级间隔。
 */
#define VL53L0X_POLLINGDELAY_LOOPNB  250

VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev)
{
    (void)Dev;
    volatile uint32_t i;
    for (i = 0; i < VL53L0X_POLLINGDELAY_LOOPNB; i++) {
        __NOP();
    }
    return VL53L0X_ERROR_NONE;
}