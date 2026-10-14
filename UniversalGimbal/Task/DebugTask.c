/**
 * @attention   采用UTF-8字符集编码
 * @brief       调试任务
 * @authors     北工大PIP战队 樊郡捷
 * @version     v1.1
 * @date        2025-02-27
 * @details     发送数据供上位机绘制曲线
 */

#include "header.h"

#define DEBUG_TASK_UPDATE_TICK 2

volatile DebugData_t DebugData;

void DebugTask(void const *argument)
{
    uint32_t PreviousWakeTime = osKernelSysTick();

    DebugData.Head = 0x55AA55AA;
    DebugData.Tail = 0xAA55AA55;

    while (1)
    {
        uint32_t NowTick = HAL_GetTick();
        osDelayUntil(&PreviousWakeTime, DEBUG_TASK_UPDATE_TICK);	
                           
        HAL_UART_Transmit_DMA(&huart6, (uint8_t *)&DebugData, sizeof(DebugData));
    }
}
