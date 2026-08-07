/**
 * @attention   采用UTF-8字符集编码
 * @brief       RM图传链路控制器(VTM) 接收函数
 * @authors     北工大PIP战队 樊郡捷
 * @version     v1.0
 * @date        2024-10-14
 * @details
 */

#include "header.h"
#include "RC_VTM.h"

uint8_t VtmBuf[21];
RC_VTM_t RC_VTM = {0};
uint8_t DataLength = 21;
uint16_t CRC16 = 0;

extern DMA_HandleTypeDef hdma_usart6_rx;

void RC_VTM_Init(void)
{

    RC_VTM.Pad.CH0 = 1024;
    RC_VTM.Pad.CH1 = 1024;
    RC_VTM.Pad.CH2 = 1024;
    RC_VTM.Pad.CH3 = 1024;
    RC_VTM.Pad.CH4 = 1024;
    RC_VTM.Pad.S1 = 0;
    RC_VTM.Pad.S2 = 0;
    RC_VTM.Pad.Switch = 0;
    RC_VTM.Pad.Stop = 0;
    RC_VTM.Pad.Trigger = 0;

    RC_VTM.Key.Raw = 0;
    RC_VTM.Mouse.L = 0;
    RC_VTM.Mouse.R = 0;
    RC_VTM.Mouse.M = 0;
    RC_VTM.Mouse.X = 0;
    RC_VTM.Mouse.Y = 0;
    RC_VTM.Mouse.Z = 0;

    RC_VTM.PackageCount = 0;
    RC_VTM.StickyCount = 0;
    RC_VTM.ErrorCount = 0;
    RC_VTM.LastOnlineTick = 0;

    // HAL_UART_Receive_DMA(&huart6, VtmBuf, 21);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, VtmBuf, 21);
    __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
}

void RC_VTM_RxCallback(uint8_t *rxBuf, int rxSize)
{
    if (rxBuf[0] != 0xA9 || rxBuf[1] != 0x53 || !CRC16_VerifyChecksum(rxBuf, 21))
    {
        RC_VTM.ErrorCount++;
        // HAL_UART_Receive_DMA(&huart6, rxBuf, 21);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rxBuf, 21);
        __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
        return;
    }
    RC_VTM.Pad.CH0 = (rxBuf[2] | rxBuf[3] << 8) & 0x07ff;
    RC_VTM.Pad.CH1 = (rxBuf[3] >> 3 | rxBuf[4] << 5) & 0x07ff;
    RC_VTM.Pad.CH2 = (rxBuf[4] >> 6 | rxBuf[5] << 2 | rxBuf[6] << 10) & 0x07ff;
    RC_VTM.Pad.CH3 = ((rxBuf[6] >> 1) | (rxBuf[7] << 7)) & 0x07ff;
    RC_VTM.Pad.Switch = (rxBuf[7] >> 4) & 0x0003;
    RC_VTM.Pad.Stop = (rxBuf[7] >> 6) & 0x0001;
    RC_VTM.Pad.S1 = (rxBuf[7] >> 7) & 0x0001 == 1;
    RC_VTM.Pad.S2 = rxBuf[8] & 0x0001;
    RC_VTM.Pad.CH4 = (rxBuf[8] >> 1 | rxBuf[9] << 7) & 0x07ff;
    RC_VTM.Pad.Trigger = (rxBuf[9] >> 4) & 0x0001;

    RC_VTM.Mouse.X = (rxBuf[10] | rxBuf[11] << 8);
    RC_VTM.Mouse.Y = (rxBuf[12] | rxBuf[13] << 8);
    RC_VTM.Mouse.Z = (rxBuf[14] | rxBuf[15] << 8);
    RC_VTM.Mouse.L = (rxBuf[16]) & 0x0003;
    RC_VTM.Mouse.R = (rxBuf[16] >> 2) & 0x0003;
    RC_VTM.Mouse.M = (rxBuf[16] >> 4) & 0x0003;
    RC_VTM.Key.Raw = (rxBuf[17] | rxBuf[18] << 8) & 0xffff;
    CRC16 = (rxBuf[19] | rxBuf[20]) & 0xffff;
    RC_VTM.PackageCount++;

    uint32_t NowTick = HAL_GetTick();

     if (NowTick - NUC.LastOnlineTick > 100)
        {
    Mouse.X = RC_VTM.Mouse.X;
    Mouse.Y = - RC_VTM.Mouse.Y;
    Mouse.Z = RC_VTM.Mouse.Z;
    Mouse.L.Value = RC_VTM.Mouse.L;
    Mouse.R.Value = RC_VTM.Mouse.R;

    Keyboard.W.Value = RC_VTM.Key.Code.W;
    Keyboard.S.Value = RC_VTM.Key.Code.S;
    Keyboard.A.Value = RC_VTM.Key.Code.A;
    Keyboard.D.Value = RC_VTM.Key.Code.D;
    Keyboard.Shift.Value = RC_VTM.Key.Code.Shift;
    Keyboard.Ctrl.Value = RC_VTM.Key.Code.Ctrl;
    Keyboard.Q.Value = RC_VTM.Key.Code.Q;
    Keyboard.E.Value = RC_VTM.Key.Code.E;
    Keyboard.R.Value = RC_VTM.Key.Code.R;
    Keyboard.F.Value = RC_VTM.Key.Code.F;
    Keyboard.G.Value = RC_VTM.Key.Code.G;
    Keyboard.Z.Value = RC_VTM.Key.Code.Z;
    Keyboard.X.Value = RC_VTM.Key.Code.X;
    Keyboard.C.Value = RC_VTM.Key.Code.C;
    Keyboard.V.Value = RC_VTM.Key.Code.V;
    Keyboard.B.Value = RC_VTM.Key.Code.B;

    }
    RC_VTM.LastOnlineTick = NowTick;
    RC_VTM.UpdateCounter++;

    if (NowTick - RC_VTM.LastFreqUpdateTick > 2000)
    {
        RC_VTM.Freq = RC_VTM.UpdateCounter * 1000 / (NowTick - RC_VTM.LastFreqUpdateTick);
        RC_VTM.UpdateCounter = 0;
        RC_VTM.LastFreqUpdateTick = NowTick;
    }
    // HAL_UART_Receive_DMA(&huart6, VtmBuf, 21);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, VtmBuf, 21);
    __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
}
