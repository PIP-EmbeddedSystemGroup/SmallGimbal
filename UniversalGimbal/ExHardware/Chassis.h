#ifndef _CHASSIS_H_
#define _CHASSIS_H_

/**
 * @attention   采用UTF-8字符集编码
 */

#include "stm32f4xx_hal.h"
#include "stdint.h"

typedef struct
{
    uint32_t LastOnlineTick;

    int isSpinMode;
} Chassis_t;

void ChassisBoard_ReceiveError(CAN_RxHeaderTypeDef* canHeader, uint8_t data[8]);
void ChassisBoard_ReceiveState(CAN_RxHeaderTypeDef* canHeader, uint8_t data[8]);
void ChassisBoard_ReceiveSpeed(CAN_RxHeaderTypeDef* canHeader, uint8_t data[8]);
void GimbalBoard_ReceiveRefereeBuf1(CAN_RxHeaderTypeDef *canHeader, uint8_t data[8]);
void GimbalBoard_ReceiveRefereeBuf2(CAN_RxHeaderTypeDef *canHeader, uint8_t data[8]);
void ChassisBoard_SendDBUS(void);
void ChassisBoard_SendVTM(void);
void ChassisBoard_SendRcVision(void);
void ChassisBoard_SendYawOutput(float YawCurrentSet);
void ChassisBoard_SendError(void);
void ChassisBoard_SendState(void);
void ChassisBoard_SendFusionInsSpeed(void);

extern Chassis_t Chassis;

#endif
