/**
 * @attention   采用UTF-8字符集编码
 * @brief       调试任务
 * @authors     北工大PIP战队 樊郡捷
 * @version     v1.1
 * @date        2025-02-27
 * @details     发送数据供上位机绘制曲线
 */

#include "header.h"

#define DEBUG_TASK_UPDATE_TICK 10

struct
{
    uint32_t Head;
    float Value[6];
    uint32_t Tail;
} DebugData = {0};

extern float temph;
float Vision_YawPathPreview(void);
float Vision_YawSpinAiming(void);
float Vision_YawSpinAimingFunction(float phase);
float Vision_YawSpinTargetPhase(void);
void DebugTask(void const *argument)
{
    uint32_t PreviousWakeTime = osKernelSysTick();

    DebugData.Head = 0x55AA55AA;
    DebugData.Tail = 0xAA55AA55;

#if DEBUG_MODE == DEBUG_MODE_NORMAL
    while (1)
    {
        osDelayUntil(&PreviousWakeTime, DEBUG_TASK_UPDATE_TICK);

        DebugData.Value[0] = Gimbal.Pitch.Motor.Current.Set;
        DebugData.Value[1] = Inertial.Gyro[AX];
        DebugData.Value[2] = 0;
        DebugData.Value[3] = 0;
        DebugData.Value[4] = 0;
        DebugData.Value[5] = 0;

        HAL_UART_Transmit_DMA(&huart6, (uint8_t *)&DebugData, sizeof(DebugData));

        // CDC_Transmit_FS((uint8_t*)&DebugData, sizeof(DebugData));
    }
#elif DEBUG_MODE == DEBUG_MODE_FFT
    while (1)
    {
        // osDelayUntil(&PreviousWakeTime, DEBUG_TASK_UPDATE_TICK);
        osThreadSuspend(osThreadGetId());

        struct
        {
            uint32_t Head;
            float Value[48];
            uint32_t Tail;
        } DebugDataFFT;

        DebugDataFFT.Head = 0x12345678;
        DebugDataFFT.Tail = 0x87654321;

        for (int i = 0; i < 48; i++)
            DebugDataFFT.Value[i] = Vision.Yaw.FreqMag[i];
        HAL_UART_Transmit_DMA(&huart6, (uint8_t *)&DebugDataFFT, sizeof(DebugDataFFT));

        // CDC_Transmit_FS((uint8_t*)&DebugData, sizeof(DebugData));
    }
#endif
}
