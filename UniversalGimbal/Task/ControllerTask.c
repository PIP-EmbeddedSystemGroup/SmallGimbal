/**
 * @attention   采用UTF-8字符集编码
 * @brief       机器人遥控任务
 * @authors     北工大PIP战队 樊郡捷
 * @version     v1.0
 * @date        2024-10-14
 * @details     采用统一的更新频率 统一了不同接收机的不同回传频率
 *              实现了整车使能和失能的归一化
 *              实现了不同控制模式的输出量的统一
 *              实现了控制量单位的统一
 */

#include "header.h"

#define CONTROLLER_TASK_UPDATE_TICK 10

Controller_t Controller = {0};

void Controller_Pad_DBUS(void);
void Controller_Pad_VTM(void);
void Controller_MouseKeyboard(void);
Key_t tempAimon;

void ControllerTask(void const *argument)
{
    uint32_t PreviousWakeTime = osKernelSysTick();

    Mouse_Init();
    Keyboard_Init();
    RC_DBUS_Init();
    RC_VTM_Init();

    Controller.isEnable = RESET;
    Daemon.ErrorFlag = 0;
    Controller.FrictionOn = RESET;
    Controller.ShootingOn = RESET;
    Controller.isBoxOn = RESET;
    Controller.AimingOn = RESET;
    Controller.VisionMode = VISION_MODE_ARMOR;
    Controller.AimingInvRotateEnable = SET;
    Controller.isBossMode = RESET;
    Controller.Move.FB.Set = 0;
    Controller.Move.LR.Set = 0;
    Controller.Move.FB.Real = 0;
    Controller.Move.LR.Real = 0;
    Controller.View.Pitch = 0;
    Controller.View.Yaw = 0;
    Controller.View.YawVisionOffset = 0;
    Controller.View.PitchVisionOffset = 0;
    Controller.View.YawVisionOffset_LittlePowerRune = 0.969999552;
    Controller.View.PitchVisionOffset_LittlePowerRune = 1.51999843 - 0.00499986345;

    Key_Init(&Controller.PadS1.Up, 0);
    Key_Init(&Controller.PadS1.Down, 0);
		
		
    HAL_Delay(1000);
    while (1)
    {
        osDelayUntil(&PreviousWakeTime, CONTROLLER_TASK_UPDATE_TICK);

        uint32_t NowTick = HAL_GetTick();

        Controller.isEnable = SET;
    
  


        if (NowTick - RC_DBUS.LastOnlineTick < 500) // 如果DBUS在线 采用DBUS控制
        {
            Controller.AimingOn = 0;
            Controller_Pad_DBUS();
        }
        else
        {
            Controller.AimingOn = 1;
        }

        // RC_DBUS.Pad.S2 = 1;

        // if (NowTick - RC_DBUS.LastOnlineTick < 100) // 如果DBUS在线 采用DBUS控制
        // {
        //     switch (RC_DBUS.Pad.S2)
        //     {
        //     case RC_DBUS_S2_PAD_CTRL: // 上
        //         Controller_Pad_DBUS();
        //         break;
        //     case RC_DBUS_S2_COMPUTER_CTRL: // 中
        //         Controller_MouseKeyboard();
        //         break;
        //     case RC_DBUS_S2_DISABLE_CTRL: // 下
        //     default:
        //         Controller.Move.FB.Set = 0;
        //         Controller.Move.LR.Set = 0;
        //         Controller.View.Pitch = 0;
        //         Controller.View.Yaw = Inertial.Yaw;
        //         Controller.FrictionOn = RESET;
        //         Controller.ShootingOn = RESET;
        //         Controller.AimingOn = RESET;
        //         Controller.isBossMode = RESET;
        //         break;
        //     }
        // }
        // else // 如果DUBS掉线 VTM在线 采用VTM控制
        // {
        //     switch (RC_VTM.Pad.Switch)
        //     {
        //     case RC_VTM_Switch_PAD_CTRL: // 左
        //         Controller_Pad_VTM();
        //         break;
        //     case RC_VTM_Switch_COMPUTER_CTRL: // 中
        //         Controller_MouseKeyboard();
        //         break;
        //     case RC_VTM_Switch_DISABLE_CTRL: // 右
        //     default:
        //         Controller.Move.FB.Set = 0;
        //         Controller.Move.LR.Set = 0;
        //         Controller.View.Pitch = 0;
        //         Controller.View.Yaw = Inertial.Yaw;
        //         Controller.FrictionOn = RESET;
        //         Controller.ShootingOn = RESET;
        //         Controller.AimingOn = RESET;
        //         Controller.isBossMode = RESET;
        //         break;
        //     }
        // }
    }
}

void Controller_Pad_DBUS(void)
{
    Controller.View.Yaw += -(RC_DBUS.Pad.CH2 - 1024) / 660.f * YAW_SENSITY_PAD * (CONTROLLER_TASK_UPDATE_TICK / 1000.f);
    Controller.View.Pitch =
        PITCH_INIT_DEGREE +
        (((RC_DBUS.Pad.CH3 - 1024) > 0) ? ((RC_DBUS.Pad.CH3 - 1024) / 660.f * PITCH_MAX_DEGREE) : (-(RC_DBUS.Pad.CH3 - 1024) / 660.f * PITCH_MIN_DEGREE));
}

void Controller_Pad_VTM(void)
{
    // Controller.Move.FB.Set = (RC_VTM.Pad.CH1 - 1024) / 660.f * CHASSIS_SPEED; // m/s
    // Controller.Move.LR.Set = -(RC_VTM.Pad.CH0 - 1024) / 660.f * CHASSIS_SPEED;

    // if (!Controller.AimingOn)
    // {
    //     Controller.View.Yaw += -(RC_VTM.Pad.CH3 - 1024) / 660.f * YAW_SENSITY_PAD * (CONTROLLER_TASK_UPDATE_TICK / 1000.f);
    //     Controller.View.Pitch =
    //         PITCH_INIT_DEGREE +
    //         (((RC_VTM.Pad.CH2 - 1024) > 0) ? ((RC_VTM.Pad.CH2 - 1024) / 660.f * PITCH_MAX_DEGREE) : (-(RC_VTM.Pad.CH2 - 1024) / 660.f * PITCH_MIN_DEGREE));
    // }

    // Controller.isBoxOn = ((int)RC_VTM.Pad.CH4 - 1024) < -500;
    // if (Controller.isBoxOn)
    //     Shooting_BoxOpen();
    // else
    //     Shooting_BoxClose();

    // tempAimon.Value = RC_VTM.Pad.S2 == 1;
    // if (Key_Update(&tempAimon) == KEY_LONG_PRESS)
    //     Controller.AimingOn = !Controller.AimingOn;

    // Controller.PadS1.Up.Value = RC_VTM.Pad.S1 == 1;
    // if (Key_Update(&Controller.PadS1.Up) == KEY_LONG_PRESS)
    //     Controller.FrictionOn = !Controller.FrictionOn;

    // Controller.PadS1.Down.Value = RC_VTM.Pad.Trigger == 1;
    // if (Key_Update(&Controller.PadS1.Down) == KEY_PRESSED && Controller.FrictionOn == SET)
    //     Controller.ShootingOn = SET;
    // else
    //     Controller.ShootingOn = RESET;
}
void Controller_MouseKeyboard(void)
{
    // if (Key_Update(&Mouse.R) == KEY_PRESSED)
    // {
    //     if (!Controller.AimingOn)
    //     {
    //         Controller.View.YawVisionOffset = 0;
    //         Controller.View.PitchVisionOffset = 0;
    //     }
    //     Controller.AimingOn = SET;
    // }
    // else
    // {
    //     Controller.AimingOn = RESET;
    // }

    // if (!Controller.AimingOn || !Vision.isDetected)
    // {
    //     Controller.View.Yaw += -Mouse.X * YAW_SENSITY_MOUSE;
    //     Controller.View.Pitch += -Mouse.Y * PITCH_SENSITY_MOUSE;
    // }
    // else
    // // else if (Key_Update(&Keyboard.Q) == KEY_PRESSED && Vision.isDetected)
    // {
    //     Controller.View.YawVisionOffset += -Mouse.X * YAW_SENSITY_MOUSE * 0.1f;
    //     Controller.View.PitchVisionOffset += -Mouse.Y * PITCH_SENSITY_MOUSE * 0.1f;
    // }

    // float MoveFB = 0, MoveLR = 0;
    // if (Key_Update(&Keyboard.W) == KEY_PRESSED && Key_Update(&Keyboard.C) != KEY_PRESSED)
    //     MoveFB += CHASSIS_SPEED * 0.5f;
    // else
    //     MoveFB += CHASSIS_SPEED * 0.1f;
    // if (Key_Update(&Keyboard.S) == KEY_PRESSED && Key_Update(&Keyboard.C) != KEY_PRESSED)
    //     MoveFB += -CHASSIS_SPEED * 0.5f;
    // else
    //     MoveFB += -CHASSIS_SPEED * 0.1f;
    // if (Key_Update(&Keyboard.A) == KEY_PRESSED && Key_Update(&Keyboard.C) != KEY_PRESSED)
    //     MoveLR += CHASSIS_SPEED * 0.5f;
    // else
    //     MoveLR += CHASSIS_SPEED * 0.1f;
    // if (Key_Update(&Keyboard.D) == KEY_PRESSED && Key_Update(&Keyboard.C) != KEY_PRESSED)
    //     MoveLR += -CHASSIS_SPEED * 0.5f;
    // else
    //     MoveLR += -CHASSIS_SPEED * 0.1f;
    // if (MoveFB != 0 && MoveLR != 0)
    // {
    //     MoveFB /= SQRT2;
    //     MoveLR /= SQRT2;
    // }
    // if (Key_Update(&Keyboard.Shift) == KEY_PRESSED && Key_Update(&Keyboard.C) != KEY_PRESSED)
    // {
    //     MoveFB *= 2;
    //     MoveLR *= 2;
    // }
    // Controller.Move.FB.Set = MoveFB;
    // Controller.Move.LR.Set = MoveLR;

    // if (Key_Update(&Keyboard.F) == KEY_LONG_PRESS)
    //     Controller.FrictionOn = !Controller.FrictionOn;

    // if (Key_Update(&Mouse.L) == KEY_PRESSED && Controller.FrictionOn == SET)
    //     Controller.ShootingOn = SET;
    // else
    //     Controller.ShootingOn = RESET;

    // if (Key_Update(&Keyboard.V) == KEY_LONG_PRESS)
    // {
    //     if (Controller.VisionMode == VISION_MODE_ARMOR)
    //         Controller.VisionMode = VISION_MODE_POWER_RUNE_LITTLE;
    //     else if (Controller.VisionMode == VISION_MODE_POWER_RUNE_LITTLE)
    //         Controller.VisionMode = VISION_MODE_POWER_RUNE_BIG;
    //     else
    //         Controller.VisionMode = VISION_MODE_ARMOR;
    // }

    // if (Key_Update(&Keyboard.B) == KEY_LONG_PRESS)
    //     Controller.isBossMode = !Controller.isBossMode;

    // if (Key_Update(&Keyboard.E) == KEY_LONG_PRESS)
    //     HAL_NVIC_SystemReset();

    // if (Key_Update(&Keyboard.R) == KEY_LONG_PRESS)
    // {
    //     Controller.isBoxOn = !Controller.isBoxOn;
    //     if (Controller.isBoxOn)
    //         Shooting_BoxOpen();
    //     else
    //         Shooting_BoxClose();
    // }

    // if (Key_Update(&Keyboard.X) == KEY_LONG_PRESS)
    //     Controller.AimingInvRotateEnable = !Controller.AimingInvRotateEnable;
}
