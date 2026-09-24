/**
 * @attention   采用UTF-8字符集编码
 * @brief       2026赛季自瞄任务
 * @authors     北工大PIP战队 刘诗厚
 * @version     v2.1
 * @date        2025-11-4
 * @details     实现了mbl的视觉下位机，把陀螺仪数据上发，接受目标角度即可
 */

#include "header.h"

Vision_t Vision;

void VisionTask(void const *argument)
{
    uint32_t PrevWakeTick = HAL_GetTick();

   // NUC_Init();初始化放在main中，提前
    Vision.NUC = &NUC;

    Vision.VisionMode = VISION_MODE_ARMOR;
    Vision.DecisionMode = DECISION_MODE_AOTU;

 while (1)
{
    osSignalWait(0x0001, osWaitForever);

    if (Controller.AimingOn && NUC.Pitch_ControlMode == ANGLE_CONTROL)
    {
        Controller.View.Pitch = Vision.NUC->PitchDataRecieve;
    }
    if (Controller.AimingOn && NUC.Yaw_ControlMode == ANGLE_CONTROL)
    {
        Controller.View.Yaw = Vision.NUC->YawDataRecieve;
    }
//    if (Controller.AimingOn && NUC.Pitch_ControlMode == ANGLE_INCRESEMENT_CONTROL)
//    {
//        Controller.View.Pitch = Vision.NUC->PitchDataRecieve + Inertial.Pitch;
//    }
//    if (Controller.AimingOn && NUC.Yaw_ControlMode == ANGLE_INCRESEMENT_CONTROL)
//    {
//        Controller.View.Yaw = Vision.NUC->YawDataRecieve + Inertial.Yaw;
//    }
}
}
