#ifndef _CONTROLLER_TASK_H_
#define _CONTROLLER_TASK_H_

/**
 * @attention   采用UTF-8字符集编码
 */

#include "stdint.h"
#include "cmsis_os.h"

#include "MouseKeyboard.h"
#include "VisionTask.h"

typedef struct
{
    int isEnable;

    struct
    {
        Key_t Up;
        Key_t Down;
    } PadS1;
    int FrictionOn;
    int ShootingOn;

    int isBoxOn;

    int AimingOn;
    VisionMode_t VisionMode;
    int AimingInvRotateEnable;

    int isBossMode;

    struct
    {
        struct
        {
            float Set;
            float Real;
        } FB;   //Front Back
        struct
        {
            float Set;
            float Real;
        } LR;   //Left Right
    } Move; // m/s

    struct
    {
        float Yaw;
        float Pitch;
        float YawVisionOffset;
        float PitchVisionOffset;
        float YawVisionOffset_LittlePowerRune;
        float PitchVisionOffset_LittlePowerRune;
    } View; // degree
} Controller_t;

void ControllerTask(void const* argument);

extern Controller_t Controller;

#endif
