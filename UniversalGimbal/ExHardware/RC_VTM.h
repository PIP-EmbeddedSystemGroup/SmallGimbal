#ifndef _RC_VTM_H_
#define _RC_VTM_H_

/**
 * @attention   采用UTF-8字符集编码
 */

#include "stm32f4xx_hal.h"
#include "stdint.h"

typedef struct
{
     struct
    {
        uint16_t CH0;
        uint16_t CH1;
        uint16_t CH2;
        uint16_t CH3;
        uint16_t CH4;
        uint8_t Switch;
        uint8_t Stop;
        uint8_t Trigger;
        uint8_t S1;
        uint8_t S2;
    } Pad;
    struct
    {
        int16_t X;
        int16_t Y;
        int16_t Z; //鼠标滚轮 十分不灵敏
        uint8_t M;
        uint8_t L;
        uint8_t R;
    } Mouse;
    union   //联合体: 使Code结构体和Raw变量共用同一内存空间
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
        } Code; //位域结构体: 其成员所占用的空间按位计算 (引用时由编译器自动生成位运算操作)
        uint16_t Raw;
    } Key;

    uint32_t LastOnlineTick;

    int PackageCount;
    int StickyCount;
    int ErrorCount;
    int UpdateCounter;
    uint32_t LastFreqUpdateTick;
    int Freq;
} RC_VTM_t;

void RC_VTM_Init(void);
void RC_VTM_RxCallback(uint8_t* rxBuf, int rxSize);

extern uint8_t VtmBuf[21];
extern RC_VTM_t RC_VTM;

#endif
