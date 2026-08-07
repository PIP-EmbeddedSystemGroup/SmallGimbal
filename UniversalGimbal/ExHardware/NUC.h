#ifndef _NUC_H_
#define _NUC_H_

/**
 * @attention   采用UTF-8字符集编码
 */

#include "stm32f4xx_hal.h"
#include "stdint.h"

typedef enum
{
    ANGLE_CONTROL = 0xB4,
    SPEED_CONTROL = 0xF3,
    ANGLE_INCRESEMENT_CONTROL = 0xA8,
    FORCE_CONTROL = 0xC5
} Yaw_PITCH_ControlMode_t;

typedef enum
{
    NO_DETECTED = 0xB4,
    DETECTED_NO_LOCKED = 0xA8,
    DETECTED_AND_LOCKED = 0xB0
} VisonResult_t;

typedef struct
{
    uint8_t Yaw_ControlMode;
    float YawDataRecieve; // 接受的是degree，会转换成rad

    uint8_t Pitch_ControlMode;
    float PitchDataRecieve; // 接受的是degree，会转换成rad

    uint8_t VisonResult;
    uint8_t ShootingPremision;


    struct
    {
        union
        {
            struct
            {
                int W : 1;
                int S : 1;
                int A : 1;
                int D : 1;
                int Shift : 1;
                int Ctrl : 1;
                int Q : 1;
                int E : 1;
                int R : 1;
                int F : 1;
                int G : 1;
                int Z : 1;
                int X : 1;
                int C : 1;
                int V : 1;
                int B : 1;
            } Code;
            uint16_t Raw;
        } Key;

        __packed struct
        {
            int16_t X;
            int16_t Y;
            int16_t Z; // 鼠标滚轮 十分不灵敏
            uint8_t L;
            uint8_t R;
        } Mouse;

    } RC;

    uint8_t EnableKeyBoardCtrl;

    int UpdateCounter;
    uint32_t LastFreqUpdateTick;
    int Freq;

    int ErrorCount;
    uint32_t LastOnlineTick;
} NUC_t;

void NUC_Init(void);
void NUC_SendConfig(float Q[4]);
void NUC_RxCallback(int isFromUart);

extern uint8_t NucRxBuf[64];
extern NUC_t NUC;

#endif
