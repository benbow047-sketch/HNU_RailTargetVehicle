#ifndef __VL53L0X_PLATFORM_H
#define __VL53L0X_PLATFORM_H

#include "vl53l0x_def.h"
#include "vl53l0x_port.h"
#include <stdio.h>
#include <string.h>

/* Log macros (no-ops, disable VL53L0X API internal logging) */
#define VL53L0X_ErrLog(...)                 (void)0
#define _LOG_FUNCTION_START(module, fmt, ...)   (void)0
#define _LOG_FUNCTION_END(module, status, ...)   (void)0
#define _LOG_FUNCTION_END_FMT(module, status, fmt, ...) (void)0
#define VL53L0X_COPYSTRING(str, ...)        strcpy(str, __VA_ARGS__)

//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
//ALIENTEK MiniV3 STM32������
//VL53L0X �ײ����� ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2017/7/1
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) �������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////

//vl53l0x�豸I2C��Ϣ
typedef struct {
    VL53L0X_DevData_t Data;              /*!< embed ST Ewok Dev  data as "Data"*/
    /*!< user specific field */
    uint8_t   I2cDevAddr;                /*!< i2c device address user specific field */
    uint8_t   comms_type;                /*!< Type of comms : VL53L0X_COMMS_I2C or VL53L0X_COMMS_SPI */
    uint16_t  comms_speed_khz;           /*!< Comms speed [kHz] : typically 400kHz for I2C           */

} VL53L0X_Dev_t;


typedef VL53L0X_Dev_t* VL53L0X_DEV;

#define VL53L0X_MAX_I2C_XFER_SIZE  64 //����I2Cд������ֽ���
#define PALDevDataGet(Dev, field) (Dev->Data.field)
#define PALDevDataSet(Dev, field, data) (Dev->Data.field)=(data)


VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata,uint32_t count);
VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata,uint32_t count);
VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data);
VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data);
VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data);
VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData);
VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data);
VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data);
VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data);
VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev);


#endif 

