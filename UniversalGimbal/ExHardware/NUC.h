#ifndef _NUC_H_
#define _NUC_H_

/**
 * @attention   采用UTF-8字符集编码
 */

#include "stm32f4xx_hal.h"
#include "stdint.h"

extern DMA_HandleTypeDef hdma_usart1_rx;

typedef enum
{
    ANGLE_CONTROL = 0xB4,
    SPEED_CONTROL = 0xF3,
    FORCE_CONTROL = 0xC5
} Yaw_PITCH_ControlMode_t;

typedef enum
{
    NO_DETECTED = 0xB4,
    DETECTED_NO_LOCKED = 0xA8,
    DETECTED_AND_LOCKED = 0xB0
} VisonResult_t;


typedef enum {
    NUC_STATE_UNSYNC = 0,
    NUC_STATE_SYNCING,
    NUC_STATE_RUNNING
} NUC_State_t;

typedef struct
{
    uint8_t Yaw_ControlMode;
    float YawDataRecieve; // 接受的应该是度

    uint8_t Pitch_ControlMode;
    float PitchDataRecieve; 

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

	NUC_State_t State;          // 当前状态
    uint64_t LastRxTimeUs;      // 上次收到任意有效包时的本地微秒时间
	
	 uint64_t NUC_RXNUM;
	uint64_t NUC_TXNUM;

	
} NUC_t;

uint64_t NUC_GetClockUs(void);
void NUC_Init(void);
void NUC_SendConfig(float Q[4]);
void NUC_RxCallback(uint8_t *rxBuf, int rxSize);
void NUC_TimeoutCheck(void);
	
extern uint8_t NucRxBuf[72];
extern  uint8_t NucDmaBuf[128];
extern NUC_t NUC;

#endif
