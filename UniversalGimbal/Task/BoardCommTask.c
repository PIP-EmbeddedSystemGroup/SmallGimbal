/**
 * @attention   采用UTF-8字符集编码
 * @brief       上下板通信任务
 * @authors     北工大PIP战队 樊郡捷
 * @version     v1.1
 * @date        2025-03-14
 * @details     采用RTOS的邮箱机制 扩展了CAN外设原本只有3个的硬件发送缓冲区
 */

#include "header.h"

osMailQId BoardCommTxMailQueue = NULL;
osMailQDef(BoardCommTx, 32, BoardCommCanMail_t);

/// @brief CAN的发送邮箱只有三个缓冲 在这个任务中通过消息队列的方式增加一个外部缓冲
///        COMM. -> communication
/// @param argument 
void BoardCommTask(void const* argument)
{
    BoardCommTxMailQueue = osMailCreate(osMailQ(BoardCommTx), NULL);

    osPriority Priority = osThreadGetPriority(osThreadGetId());

    while (1)
    {
        osEvent MailEvent;
        BoardCommCanMail_t* TxMail;

        MailEvent = osMailGet(BoardCommTxMailQueue, osWaitForever);    //如果队列为空 则调用此函数的任务将被挂起 直到队列中有消息可用
        TxMail = (BoardCommCanMail_t*)MailEvent.value.p;

        CAN_TxHeaderTypeDef TxMessage = { 0 };
        TxMessage.StdId = TxMail->ID;
        TxMessage.DLC = TxMail->DLC;
        if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0)
        {
            osThreadSetPriority(osThreadGetId(), osPriorityLow);
            while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0);   //设置为低优先级后直接忙等
            osThreadSetPriority(osThreadGetId(), Priority);
        }
        uint32_t TxFifoUsed;
        HAL_CAN_AddTxMessage(&hcan2, &TxMessage, TxMail->Data, &TxFifoUsed);

        osMailFree(BoardCommTxMailQueue, TxMail);
    }
}
