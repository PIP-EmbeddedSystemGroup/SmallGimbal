/**
 * @attention 采用UTF-8字符集编码
 * @brief    瓴控电机接口
 * @author  北工大PIP战队 刘诗厚
 * @version v1.0
 * @date 2025-10-19
 * @details  瓴控电机的维护函数，后续的瓴控电机都加在这，瓴控的MS系列没有电流环，所以只能设置电压的大小，同时
 *           MS系列不会返回电流值的大小，而是返回功率的大小
 *
 */

#include "LKMotor.h"

typedef struct
{
   LKMotor_t *parent;

   int RxID;
   int TxID;

   int16_t PowerReal;
   int16_t VoltageSet;

   int isMSSerie;

   int isEnable;
} LKMotor_Special_t;

#define LKMotor_LIST_MAX_LENGTH 4
int LKMotor_ListCounter = 0;
LKMotor_Special_t LKMotor_List[LKMotor_LIST_MAX_LENGTH] = {0};

void MF7015_Init(LKMotor_t *motor, int ID)
{
   motor->Type = MOTOR_TYPE_MF7015;
   motor->EncoderResolution = 65536;
   motor->GearRatio = 1;

   motor->Current.Set = 0;
   motor->Current.Real = 0;
   motor->Speed.Set = 0;
   motor->Speed.Real = 0;
   motor->Position.Set = 0;
   motor->Position.Real = 0;
   motor->Position.PrevReal = 0;
   motor->Position.Unwraped = 0;
   motor->Position.UnwrapedAfterGear.Set = 0;
   motor->Position.UnwrapedAfterGear.Real = 0;
   motor->Temperature = 0;

   motor->LastOnlineTick = 0;

   if (LKMotor_ListCounter < LKMotor_LIST_MAX_LENGTH)
   {
      LKMotor_List[LKMotor_ListCounter].parent = motor;
      LKMotor_List[LKMotor_ListCounter].RxID = 0x180 + ID;
      LKMotor_List[LKMotor_ListCounter].TxID = 0x140 + ID;
      LKMotor_List[LKMotor_ListCounter].isEnable = SET;
      LKMotor_List[LKMotor_ListCounter].isMSSerie = 0;

      motor->SpecialData = &LKMotor_List[LKMotor_ListCounter];
      LKMotor_ListCounter++;
   }
}

/// @brief 由于LKMotor的ID需要手动设置 不具有一致性 因此调用此函数来查找有无对应ID的DM4310 再进一步进行更新
/// @param ID   报文的CANID
/// @param data 报文数据
void LKMotor_TryUpdate(int ID, uint8_t data[8])
{
   for (int i = 0; i < LKMotor_LIST_MAX_LENGTH; i++)
   {
      if (LKMotor_List[i].parent != NULL && LKMotor_List[i].RxID == ID)
      {
         LKMotor_Update(LKMotor_List[i].parent, data);
         return;
      }
   }
}

void LKMotor_Update(LKMotor_t *motor, uint8_t data[8])
{
   if (data[0] != 0x9C)
      return;
   motor->Temperature = data[1];
   if (((LKMotor_Special_t *)(motor->SpecialData))->isMSSerie)
      ((LKMotor_Special_t *)(motor->SpecialData))->PowerReal = data[3] << 8 | data[2];
   else
      motor->Current.Real = data[3] << 8 | data[2];
   motor->Speed.Real = data[5] << 8 | data[4];
   motor->Position.Real = data[7] << 8 | data[6];

   motor->LastOnlineTick = HAL_GetTick();
}

void LKMotor_Output(LKMotor_t *motor)
{
   CAN_TxHeaderTypeDef TxMessage = {0};
   uint8_t Data[8] = {0};

   TxMessage.StdId = ((LKMotor_Special_t *)(motor->SpecialData))->TxID;
   TxMessage.DLC = 8;

   if (((LKMotor_Special_t *)(motor->SpecialData))->isMSSerie)
   {
      Data[0] = 0xA0;
      Data[4] = ((LKMotor_Special_t *)(motor->SpecialData))->VoltageSet;
      Data[5] = ((LKMotor_Special_t *)(motor->SpecialData))->VoltageSet >> 8;
   }
   else
   {
      Data[0] = 0xA1;
      Data[4] = motor->Current.Set;
      Data[5] = motor->Current.Set >> 8;
   }

   uint32_t TxFifoUsed;
   HAL_CAN_AddTxMessage(&hcan1, &TxMessage, Data, &TxFifoUsed);
}

void LKMotor_Enable(LKMotor_t *motor)
{
   LKMotor_Special_t *  LKMotor_Special = motor->SpecialData;

   CAN_TxHeaderTypeDef TxMessage = {0};
   uint8_t Data[] = {0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

   TxMessage.StdId = LKMotor_Special->TxID;
   TxMessage.DLC = 8;

   uint32_t TxFifoUsed;
   HAL_CAN_AddTxMessage(&hcan1, &TxMessage, Data, &TxFifoUsed);

   LKMotor_Special->isEnable = SET;
}

void LKMotor_Disable(LKMotor_t *motor)
{
   LKMotor_Special_t *  LKMotor_Special = motor->SpecialData;

   CAN_TxHeaderTypeDef TxMessage = {0};
   uint8_t Data[] = {0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

   TxMessage.StdId = LKMotor_Special->TxID;
   TxMessage.DLC = 8;

   uint32_t TxFifoUsed;
   HAL_CAN_AddTxMessage(&hcan1, &TxMessage, Data, &TxFifoUsed);

   LKMotor_Special->isEnable = RESET;
}
