/**
 * @file    BSP_CAN.h
 * @brief   CAN总线BSP驱动接口 (STM32F4 bxCAN)
 * @author  halo
 * @date    2026/6/8
 */

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "main.h"

void CAN_Start(void);
uint8_t CANSendMessage(CAN_HandleTypeDef *hcan, uint32_t id, const uint8_t *data, uint8_t len);
uint8_t CANReceiveMessage(CAN_HandleTypeDef *hcan, uint32_t *id, uint8_t *data);
void can1_rx_callback(uint32_t id, uint8_t *data);
void can2_rx_callback(uint32_t id, uint8_t *data);

#endif /* BSP_CAN_H */