/**
 * @file    BSP_CAN.c
 * @brief   CAN总线BSP驱动实现 (STM32F4 bxCAN)
 * @author  halo
 * @date    2026/6/8
 */

#include "BSP_CAN.h"

#include "can.h"

/* ======================== 内部函数 ======================== */

/**
 * @brief   配置CAN1滤波器 — 接收所有标准ID消息到FIFO0
 */
static void CAN1_Filter_Init(void)
{
    CAN_FilterTypeDef sFilterConfig;

    /* 滤波器0: 32位ID掩码模式, 全部放行至FIFO0 */
    sFilterConfig.FilterBank           = 0;
    sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh         = 0x0000;
    sFilterConfig.FilterIdLow          = 0x0000;
    sFilterConfig.FilterMaskIdHigh     = 0x0000;
    sFilterConfig.FilterMaskIdLow      = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation     = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
}

/**
 * @brief   配置CAN2滤波器 — 接收所有标准ID消息到FIFO0
 */
static void CAN2_Filter_Init(void)
{
    CAN_FilterTypeDef sFilterConfig;

    /* 滤波器14: 32位ID掩码模式, 全部放行至FIFO0 */
    sFilterConfig.FilterBank           = 14;
    sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh         = 0x0000;
    sFilterConfig.FilterIdLow          = 0x0000;
    sFilterConfig.FilterMaskIdHigh     = 0x0000;
    sFilterConfig.FilterMaskIdLow      = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation     = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
}

/* ======================== 公共接口 ======================== */

/**
 * @brief   启动CAN1和CAN2并使能FIFO0接收中断
 */
void CAN_Start(void)
{
    CAN1_Filter_Init();
    CAN2_Filter_Init();
    HAL_CAN_Start(&hcan1);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief   通过指定CAN外设发送一条标准数据帧
 * @param   hcan   CAN外设句柄 (&hcan1 或 &hcan2)
 * @param   id     11位标准帧ID (0x000 ~ 0x7FF)
 * @param   data   待发送数据指针
 * @param   len    数据长度 (0~8 字节)
 * @return  0: 发送成功
 * @return  1: 发送失败
 */
uint8_t CANSendMessage(CAN_HandleTypeDef *hcan, const uint32_t id, const uint8_t *data, const uint8_t len)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;

    if (data == NULL || len > 8)
        return 1;

    TxHeader.StdId            = id;
    TxHeader.ExtId            = 0;
    TxHeader.IDE              = CAN_ID_STD;
    TxHeader.RTR              = CAN_RTR_DATA;
    TxHeader.DLC              = len;
    TxHeader.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(hcan, &TxHeader, data, &TxMailbox) != HAL_OK)
        return 1;

    return 0;
}

/**
 * @brief   从指定CAN外设FIFO0读取一条接收到的消息
 * @param   hcan   CAN外设句柄 (&hcan1 或 &hcan2)
 * @param   id     [输出] 接收到的CAN消息ID
 * @param   data   [输出] 数据缓冲区指针 (至少8字节)
 * @return  接收到的数据长度(字节), 0表示FIFO空
 */
uint8_t CANReceiveMessage(CAN_HandleTypeDef *hcan, uint32_t *id, uint8_t *data)
{
    CAN_RxHeaderTypeDef RxHeader;

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, data) != HAL_OK)
        return 0;

    if (id != NULL)
    {
        *id = RxHeader.StdId;
    }

    return (uint8_t)RxHeader.DLC;
}

/* ======================== HAL 中断回调 ======================== */

/**
 * @brief   HAL CAN FIFO0消息挂起回调 (CAN1 & CAN2)
 * @note    由 HAL_CAN_IRQHandler 在中断上下文中调用
 * @note    此函数读取消息后将数据分发给 can1_rx_callback() / can2_rx_callback()
 * @warning 请删除 main.c 中原有的 HAL_CAN_RxFifo0MsgPendingCallback,
 *          改为重写 __weak can1_rx_callback() / can2_rx_callback() 实现接收处理
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, data) != HAL_OK)
        return;

    if (hcan == &hcan1)
    {
        can1_rx_callback(RxHeader.StdId, data);
    }
    else if (hcan == &hcan2)
    {
        can2_rx_callback(RxHeader.StdId, data);
    }
}

/**
 * @brief   CAN1接收回调 (用户重写)
 * @note    弱定义, 用户可在其他文件中重写 — 例如在 dji_motor.c 中实现
 * @param   id    接收到的CAN标准ID
 * @param   data  8字节数据缓冲区
 */
__weak void can1_rx_callback(uint32_t id, uint8_t *data)
{
    (void)id;
    (void)data;
    /* 空实现, 用户重写 */
}

/**
 * @brief   CAN2接收回调 (用户重写)
 * @note    弱定义, 用户可在其他文件中重写 — 例如在 dji_motor.c 中实现
 * @param   id    接收到的CAN标准ID
 * @param   data  8字节数据缓冲区
 */
__weak void can2_rx_callback(uint32_t id, uint8_t *data)
{
    (void)id;
    (void)data;
    /* 空实现, 用户重写 */
}