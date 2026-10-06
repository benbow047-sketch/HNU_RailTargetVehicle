/**
 * @file    vl53l0x_app.c
 * @brief   VL53L0X 应用层实现
 *          初始化 / 模式配置 / 单次测距 / 连续测距
 */

#include "vl53l0x_app.h"
#include <stdio.h>
#include <string.h>

/*===========================================================================*/
/* 全局变量                                                                  */
/*===========================================================================*/
VL53L0X_Dev_t                    vl53l0x_dev;      /* 设备句柄              */
VL53L0X_DeviceInfo_t             vl53l0x_dev_info;  /* 设备信息              */
VL53L0X_RangingMeasurementData_t vl53l0x_data;      /* 测距数据              */
volatile uint16_t                Distance_data;      /* 距离值 (mm)           */
volatile uint8_t                 g_tof_ready;        /* 数据就绪标志          */

/*===========================================================================*/
/* 4 种预设模式参数表                                                        */
/*===========================================================================*/
const VL53L0X_ModeConfig_t Mode_data[] = {
    /* signalLimit          sigmaLimit           timingBudget  preVcsel  finalVcsel */
    { (FixPoint1616_t)(0.25 * 65536), (FixPoint1616_t)(18 * 65536),  33000, 14, 10 }, /* 默认     */
    { (FixPoint1616_t)(0.25 * 65536), (FixPoint1616_t)(18 * 65536), 200000, 14, 10 }, /* 高精度   */
    { (FixPoint1616_t)(0.1  * 65536), (FixPoint1616_t)(60 * 65536),  33000, 18, 14 }, /* 长距离   */
    { (FixPoint1616_t)(0.25 * 65536), (FixPoint1616_t)(32 * 65536),  20000, 14, 10 }, /* 高速     */
};

/*===========================================================================*/
/* 工具函数                                                                  */
/*===========================================================================*/

/**
 * @brief  打印 PAL 错误信息
 */
void print_pal_error(VL53L0X_Error Status)
{
    char buf[VL53L0X_MAX_STRING_LENGTH];
    VL53L0X_GetPalErrorString(Status, buf);
    printf("API Status: %i : %s\r\n", Status, buf);
}

/**
 * @brief  获取模式名称字符串
 */
void mode_string(uint8_t mode, char *buf)
{
    switch (mode) {
        case VL53L0X_MODE_DEFAULT:       strcpy(buf, "Default");       break;
        case VL53L0X_MODE_HIGH_ACCURACY: strcpy(buf, "High Accuracy"); break;
        case VL53L0X_MODE_LONG_RANGE:    strcpy(buf, "Long Range");    break;
        case VL53L0X_MODE_HIGH_SPEED:    strcpy(buf, "High Speed");    break;
        default:                         strcpy(buf, "Unknown");       break;
    }
}

/*===========================================================================*/
/* 传感器复位                                                                */
/*===========================================================================*/

/**
 * @brief  硬件复位 VL53L0X（通过 XSHUT 引脚断电再上电）
 */
void vl53l0x_reset(VL53L0X_Dev_t *dev)
{
    uint8_t addr = dev->I2cDevAddr;

    VL53L0X_XSHUT_LOW();
    VL53L0X_DELAY_MS(30);
    VL53L0X_XSHUT_HIGH();
    VL53L0X_DELAY_MS(30);

    dev->I2cDevAddr = VL53L0X_ADDR_DEFAULT;   /* 上电后恢复默认地址 */
    VL53L0X_DataInit(dev);                     /* 重新数据初始化     */
    dev->I2cDevAddr = addr;
}

/*===========================================================================*/
/* 初始化                                                                    */
/*===========================================================================*/

/**
 * @brief  初始化 VL53L0X 传感器
 * @note   调用前需确保 CubeMX 已完成 I2C 和 GPIO 初始化
 * @retval VL53L0X_ERROR_NONE  成功
 *         其他值               失败（通过 print_pal_error 输出原因）
 */
VL53L0X_Error vl53l0x_init(VL53L0X_Dev_t *dev)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    /* 填充设备信息 */
    dev->I2cDevAddr      = VL53L0X_ADDR_DEFAULT;  /* 8-bit 地址 0x52    */
    dev->comms_type      = 1;                      /* I2C 通信            */
    dev->comms_speed_khz = 400;                    /* 400 kHz             */

    /* 硬件复位序列 */
    VL53L0X_XSHUT_LOW();
    VL53L0X_DELAY_MS(30);
    VL53L0X_XSHUT_HIGH();
    VL53L0X_DELAY_MS(30);

    /* 数据初始化 */
    Status = VL53L0X_DataInit(dev);
    if (Status != VL53L0X_ERROR_NONE) goto error;

    /* 读取设备信息确认通信正常 */
    Status = VL53L0X_GetDeviceInfo(dev, &vl53l0x_dev_info);
    if (Status != VL53L0X_ERROR_NONE) goto error;

    return VL53L0X_ERROR_NONE;

error:
    print_pal_error(Status);
    return Status;
}

/*===========================================================================*/
/* 模式设置                                                                  */
/*===========================================================================*/

/**
 * @brief  设置 VL53L0X 测量模式
 * @param  mode  0=默认, 1=高精度, 2=长距离, 3=高速
 */
VL53L0X_Error vl53l0x_set_mode(VL53L0X_Dev_t *dev, uint8_t mode)
{
    VL53L0X_Error Status;
    uint8_t  VhvSettings;
    uint8_t  PhaseCal;
    uint32_t refSpadCount;
    uint8_t  isApertureSpads;

    if (mode > VL53L0X_MODE_HIGH_SPEED) mode = VL53L0X_MODE_DEFAULT;

    /* 复位并静态初始化 */
    vl53l0x_reset(dev);
    Status = VL53L0X_StaticInit(dev);
    if (Status != VL53L0X_ERROR_NONE) goto error;

    /* 参考校准 */
    Status = VL53L0X_PerformRefCalibration(dev, &VhvSettings, &PhaseCal);
    if (Status != VL53L0X_ERROR_NONE) goto error;
    VL53L0X_DELAY_MS(2);

    /* SPAD 管理 */
    Status = VL53L0X_PerformRefSpadManagement(dev, &refSpadCount, &isApertureSpads);
    if (Status != VL53L0X_ERROR_NONE) goto error;
    VL53L0X_DELAY_MS(2);

    /* 配置单次测距模式 */
    Status = VL53L0X_SetDeviceMode(dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);
    if (Status != VL53L0X_ERROR_NONE) goto error;
    VL53L0X_DELAY_MS(2);

    /* 启用和配置 Limit Check */
    Status = VL53L0X_SetLimitCheckEnable(dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    if (Status != VL53L0X_ERROR_NONE) goto error;
    VL53L0X_DELAY_MS(2);

    Status = VL53L0X_SetLimitCheckEnable(dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    if (Status != VL53L0X_ERROR_NONE) goto error;
    VL53L0X_DELAY_MS(2);

    Status = VL53L0X_SetLimitCheckValue(dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                                        Mode_data[mode].sigmaLimit);
    if (Status != VL53L0X_ERROR_NONE) goto error;
    VL53L0X_DELAY_MS(2);

    Status = VL53L0X_SetLimitCheckValue(dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                                        Mode_data[mode].signalLimit);
    if (Status != VL53L0X_ERROR_NONE) goto error;
    VL53L0X_DELAY_MS(2);

    /* 测量时间预算 */
    Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(dev,
                                        Mode_data[mode].timingBudget);
    if (Status != VL53L0X_ERROR_NONE) goto error;
    VL53L0X_DELAY_MS(2);

    /* VCSEL 脉冲周期 */
    Status = VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE,
                                        Mode_data[mode].preRangeVcselPeriod);
    if (Status != VL53L0X_ERROR_NONE) goto error;
    VL53L0X_DELAY_MS(2);

    Status = VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE,
                                        Mode_data[mode].finalRangeVcselPeriod);
    if (Status != VL53L0X_ERROR_NONE) goto error;

    return VL53L0X_ERROR_NONE;

error:
    print_pal_error(Status);
    return Status;
}

/*===========================================================================*/
/* 单次测距                                                                  */
/*===========================================================================*/

/**
 * @brief  执行一次完整的单次测距
 * @param  pdata  输出：测距数据
 * @param  buf    输出：RangeStatus 字符串（至少 VL53L0X_MAX_STRING_LENGTH 字节）
 */
VL53L0X_Error vl53l0x_start_single_test(VL53L0X_Dev_t *dev,
    VL53L0X_RangingMeasurementData_t *pdata, char *buf)
{
    VL53L0X_Error Status;

    Status = VL53L0X_PerformSingleRangingMeasurement(dev, pdata);
    if (Status != VL53L0X_ERROR_NONE)
        return Status;

    memset(buf, 0x00, VL53L0X_MAX_STRING_LENGTH);
    VL53L0X_GetRangeStatusString(pdata->RangeStatus, buf);
    Distance_data = pdata->RangeMilliMeter;

    return VL53L0X_ERROR_NONE;
}

/*===========================================================================*/
/* 便捷函数                                                                  */
/*===========================================================================*/

/**
 * @brief  打印设备信息
 */
void vl53l0x_info(void)
{
    printf("\r\n------- VL53L0X Device Info -------\r\n\r\n");
    printf("  Name:       %s\r\n", vl53l0x_dev_info.Name);
    printf("  Addr:       0x%02x\r\n", vl53l0x_dev.I2cDevAddr);
    printf("  ProductId:  %s\r\n", vl53l0x_dev_info.ProductId);
    printf("  Revision:   %d.%d\r\n",
           vl53l0x_dev_info.ProductRevisionMajor,
           vl53l0x_dev_info.ProductRevisionMinor);
    printf("\r\n-----------------------------------\r\n");
}

/**
 * @brief  执行一次测距并打印距离值
 * @param  mode  测量模式 (0~3)
 */
void One_measurement(uint8_t mode)
{
    VL53L0X_Error Status;
    char buf[VL53L0X_MAX_STRING_LENGTH];

    Status = vl53l0x_set_mode(&vl53l0x_dev, mode);
    if (Status != VL53L0X_ERROR_NONE) return;

    Status = vl53l0x_start_single_test(&vl53l0x_dev, &vl53l0x_data, buf);
    if (Status != VL53L0X_ERROR_NONE) return;

    printf("  d: %4d mm  [%s]\r\n", vl53l0x_data.RangeMilliMeter, buf);
}

/*===========================================================================*/
/* 中断模式 —— 连续测距 + INT 通知                                          */
/*===========================================================================*/

/**
 * @brief  启动连续测距中断模式
 * @note   每次测距完成后 TOF_INT 引脚产生上升沿，触发 HAL_GPIO_EXTI_Callback
 *         回调中调用 vl53l0x_interrupt_handler() 读取数据
 */
VL53L0X_Error vl53l0x_interrupt_start(VL53L0X_Dev_t *dev, uint8_t mode)
{
    VL53L0X_Error Status;
    uint8_t  VhvSettings;
    uint8_t  PhaseCal;
    uint32_t refSpadCount;
    uint8_t  isApertureSpads;

    if (mode > VL53L0X_MODE_HIGH_SPEED) mode = VL53L0X_MODE_DEFAULT;

    /* 复位并静态初始化 */
    vl53l0x_reset(dev);
    Status = VL53L0X_StaticInit(dev);
    if (Status != VL53L0X_ERROR_NONE) goto error;

    /* 参考校准 */
    Status = VL53L0X_PerformRefCalibration(dev, &VhvSettings, &PhaseCal);
    if (Status != VL53L0X_ERROR_NONE) goto error;
    VL53L0X_DELAY_MS(2);

    /* SPAD 管理 */
    Status = VL53L0X_PerformRefSpadManagement(dev, &refSpadCount, &isApertureSpads);
    if (Status != VL53L0X_ERROR_NONE) goto error;
    VL53L0X_DELAY_MS(2);

    /* 连续测距模式 */
    Status = VL53L0X_SetDeviceMode(dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
    if (Status != VL53L0X_ERROR_NONE) goto error;
    VL53L0X_DELAY_MS(2);

    /* 测量间隔 */
    Status = VL53L0X_SetInterMeasurementPeriodMilliSeconds(dev,
                                        Mode_data[mode].timingBudget);
    if (Status != VL53L0X_ERROR_NONE) goto error;
    VL53L0X_DELAY_MS(2);

    /* Limit Check */
    Status = VL53L0X_SetLimitCheckEnable(dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    if (Status != VL53L0X_ERROR_NONE) goto error;
    VL53L0X_DELAY_MS(2);

    Status = VL53L0X_SetLimitCheckEnable(dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    if (Status != VL53L0X_ERROR_NONE) goto error;
    VL53L0X_DELAY_MS(2);

    Status = VL53L0X_SetLimitCheckValue(dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                                        Mode_data[mode].sigmaLimit);
    if (Status != VL53L0X_ERROR_NONE) goto error;
    VL53L0X_DELAY_MS(2);

    Status = VL53L0X_SetLimitCheckValue(dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                                        Mode_data[mode].signalLimit);
    if (Status != VL53L0X_ERROR_NONE) goto error;
    VL53L0X_DELAY_MS(2);

    /* 测量时间预算 */
    Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(dev,
                                        Mode_data[mode].timingBudget);
    if (Status != VL53L0X_ERROR_NONE) goto error;
    VL53L0X_DELAY_MS(2);

    /* VCSEL 脉冲周期 */
    Status = VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE,
                                        Mode_data[mode].preRangeVcselPeriod);
    if (Status != VL53L0X_ERROR_NONE) goto error;
    VL53L0X_DELAY_MS(2);

    Status = VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE,
                                        Mode_data[mode].finalRangeVcselPeriod);
    if (Status != VL53L0X_ERROR_NONE) goto error;
    VL53L0X_DELAY_MS(2);

    /* 先停止再配置中断 */
    Status = VL53L0X_StopMeasurement(dev);
    if (Status != VL53L0X_ERROR_NONE) goto error;
    VL53L0X_DELAY_MS(2);

    /* GPIO 中断：新测量就绪 → 上升沿触发 → EXTI_Callback */
    Status = VL53L0X_SetGpioConfig(dev, 0,
                VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
                VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY,
                VL53L0X_INTERRUPTPOLARITY_HIGH);  /* HIGH = Rising, 匹配 CubeMX 配置 */
    if (Status != VL53L0X_ERROR_NONE) goto error;
    VL53L0X_DELAY_MS(2);

    /* 清除中断标志 */
    Status = VL53L0X_ClearInterruptMask(dev, 0);
    if (Status != VL53L0X_ERROR_NONE) goto error;

    g_tof_ready = 0;

    printf("Interrupt Mode Started!\r\n");

    /* 启动连续测距（此调用后传感器立即开始测量，中断很快就会来） */
    Status = VL53L0X_StartMeasurement(dev);
    if (Status != VL53L0X_ERROR_NONE) goto error;

error:
    print_pal_error(Status);
    return Status;
}

/**
 * @brief  中断回调中读取测距数据
 * @note   在 HAL_GPIO_EXTI_Callback 中调用此函数
 *         读取完成后置位 g_tof_ready，主循环检测此标志即可
 */
void vl53l0x_interrupt_handler(VL53L0X_Dev_t *dev)
{
    VL53L0X_Error Status;

    Status = VL53L0X_GetRangingMeasurementData(dev, &vl53l0x_data);
    if (Status == VL53L0X_ERROR_NONE) {
        Distance_data = vl53l0x_data.RangeMilliMeter;
        g_tof_ready = 1;
    }

    /* 清除本次中断，等待下一次测距完成 */
    VL53L0X_ClearInterruptMask(dev, 0);
}