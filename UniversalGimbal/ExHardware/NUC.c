/**
 * @attention   采用UTF-8字符集编码
 * @brief       NUC数据收发
 * @authors     北工大PIP战队 刘诗厚
 * @version     v2.0
 * @date        2025-12-26
 * @details     键位的提取上做了修改，上位机把所有的键位发送出来了,留出调试空间？而且上位机把鼠标的控制和云台自瞄耦合在一起了，所以下位机还需要做特化处理
 */

#include "header.h"

#define KEY_IS_PRESSED(buf, key) (((buf)[(key) >> 3] >> ((key) & 0x07)) & 0x01)

#define NUC_PACKAGE_LENGTH 64

enum
{
    KEY_Q = 28,
    KEY_W = 29,
    KEY_E = 30,
    KEY_R = 31,

    KEY_A = 42,
    KEY_S = 43,
    KEY_D = 44,

    KEY_F = 45,
    KEY_G = 46,

    KEY_Z = 55,
    KEY_X = 56,
    KEY_C = 57,
    KEY_V = 58,
    KEY_B = 59,

    KEY_LSHIFT = 54,
    KEY_RSHIFT = 65,
    KEY_LCTRL = 66,
    KEY_RCTRL = 70,

    MOUSE_LEFT = 72,
    MOUSE_RIGHT = 73
};

uint8_t NucRxBuf[NUC_PACKAGE_LENGTH] = {0};
uint8_t NucTxBuf[80] = {0};
NUC_t NUC;

float tempy = 1.5f; // 4.f;
float tempp = 1.2f;

extern osThreadId VisionHandle;

void NUC_Init(void)
{
    NUC.UpdateCounter = 0;
    NUC.LastFreqUpdateTick = 0;
    NUC.Freq = 100;

    NUC.ErrorCount = 0;
    NUC.LastOnlineTick = 0;

    HAL_UART_Receive_DMA(&huart1, NucRxBuf, NUC_PACKAGE_LENGTH);
}

void Float32ToUint8Array(float data, uint8_t *buf)
{
    memcpy(buf, &data, sizeof(float));
}

void Uint8ArrayToFloat32(float *data, uint8_t *buf)
{
    memcpy(data, buf, sizeof(float));
}

void NUC_SendConfig(float Q[4])
{
    NucTxBuf[0] = 0xA9;
    NucTxBuf[1] = 0xC8;
    NucTxBuf[2] = 0xB3;
    NucTxBuf[3] = 0xE5;
    NucTxBuf[7] = 0x55;
    NucTxBuf[24] = 0x55;
    NucTxBuf[41] = 0x55;
    NucTxBuf[58] = 0x55;
    NucTxBuf[64] = 0x55;

    NucTxBuf[4] = Vision.VisionMode;
    NucTxBuf[5] = Vision.DecisionMode;
    NucTxBuf[6] = Referee.RobotStatus.RobotId < 100 ? 0xA8 : 0xB0;

    Float32ToUint8Array(Q[0], &NucTxBuf[8]);
    Float32ToUint8Array(Q[1], &NucTxBuf[12]);
    Float32ToUint8Array(Q[2], &NucTxBuf[16]);
    Float32ToUint8Array(Q[3], &NucTxBuf[20]);

    NucTxBuf[34] = ((uint16_t)Referee.ShootData.InitialSpeed) & 0xFF;
    NucTxBuf[35] = (((uint16_t)Referee.ShootData.InitialSpeed) >> 8) & 0xFF;

    CRC16_Append(NucTxBuf, 78, CRC16_CCITT);

    HAL_UART_Transmit_IT(&huart1, NucTxBuf, 80);
    CDC_Transmit_FS(NucTxBuf, 80);

}

void NUC_RC_RxCallback(void)
{
    uint32_t NowTick = HAL_GetTick();

    NUC.RC.Mouse.X = 0;
    NUC.RC.Mouse.Y = 0;
    NUC.RC.Mouse.Z = 0;
    NUC.RC.Mouse.L = KEY_IS_PRESSED(&NucRxBuf[19], MOUSE_LEFT);
    NUC.RC.Mouse.R = KEY_IS_PRESSED(&NucRxBuf[19], MOUSE_RIGHT);

    NUC.RC.Key.Code.W = KEY_IS_PRESSED(&NucRxBuf[19], KEY_W);
    NUC.RC.Key.Code.A = KEY_IS_PRESSED(&NucRxBuf[19], KEY_A);
    NUC.RC.Key.Code.S = KEY_IS_PRESSED(&NucRxBuf[19], KEY_S);
    NUC.RC.Key.Code.D = KEY_IS_PRESSED(&NucRxBuf[19], KEY_D);
    NUC.RC.Key.Code.Ctrl = KEY_IS_PRESSED(&NucRxBuf[19], KEY_LCTRL);
    NUC.RC.Key.Code.Shift = KEY_IS_PRESSED(&NucRxBuf[19], KEY_LSHIFT);
    NUC.RC.Key.Code.Q = KEY_IS_PRESSED(&NucRxBuf[19], KEY_Q);
    NUC.RC.Key.Code.E = KEY_IS_PRESSED(&NucRxBuf[19], KEY_E);
    NUC.RC.Key.Code.R = KEY_IS_PRESSED(&NucRxBuf[19], KEY_R);
    NUC.RC.Key.Code.F = KEY_IS_PRESSED(&NucRxBuf[19], KEY_F);
    NUC.RC.Key.Code.G = KEY_IS_PRESSED(&NucRxBuf[19], KEY_G);
    NUC.RC.Key.Code.Z = KEY_IS_PRESSED(&NucRxBuf[19], KEY_Z);
    NUC.RC.Key.Code.X = KEY_IS_PRESSED(&NucRxBuf[19], KEY_X);
    NUC.RC.Key.Code.C = KEY_IS_PRESSED(&NucRxBuf[19], KEY_C);
    NUC.RC.Key.Code.V = KEY_IS_PRESSED(&NucRxBuf[19], KEY_V);
    NUC.RC.Key.Code.B = KEY_IS_PRESSED(&NucRxBuf[19], KEY_B);

    if (NUC.EnableKeyBoardCtrl)
    {
        Mouse.X = NUC.RC.Mouse.X;
        Mouse.Y = NUC.RC.Mouse.Y;
        Mouse.Z = NUC.RC.Mouse.Z;
        Mouse.L.Value = NUC.RC.Mouse.L;
        Mouse.R.Value = NUC.RC.Mouse.R;

        Keyboard.W.Value = NUC.RC.Key.Code.W;
        Keyboard.S.Value = NUC.RC.Key.Code.S;
        Keyboard.A.Value = NUC.RC.Key.Code.A;
        Keyboard.D.Value = NUC.RC.Key.Code.D;
        Keyboard.Shift.Value = NUC.RC.Key.Code.Shift;
        Keyboard.Ctrl.Value = NUC.RC.Key.Code.Ctrl;
        Keyboard.Q.Value = NUC.RC.Key.Code.Q;
        Keyboard.E.Value = NUC.RC.Key.Code.E;
        Keyboard.R.Value = NUC.RC.Key.Code.R;
        Keyboard.F.Value = NUC.RC.Key.Code.F;
        Keyboard.G.Value = NUC.RC.Key.Code.G;
        Keyboard.Z.Value = NUC.RC.Key.Code.Z;
        Keyboard.X.Value = NUC.RC.Key.Code.X;
        Keyboard.C.Value = NUC.RC.Key.Code.C;
        Keyboard.V.Value = NUC.RC.Key.Code.V;
        Keyboard.B.Value = NUC.RC.Key.Code.B;
    }
}

void NUC_RxCallback(int isFromUart)
{
    int FrameHeaderOffset = 0;
    uint32_t NowTick = HAL_GetTick();

    for (FrameHeaderOffset = 0; FrameHeaderOffset < NUC_PACKAGE_LENGTH; FrameHeaderOffset++)
        if (NucRxBuf[FrameHeaderOffset] == 0xA9)
            break;

    if (FrameHeaderOffset != 0) // 出现数据包错位
    {
        NUC.ErrorCount++;

        if (FrameHeaderOffset != 0)
        {
            if (isFromUart)
            {
                HAL_UART_DMAStop(&huart1);
                HAL_UART_Receive(&huart1, NucRxBuf, FrameHeaderOffset, 1);
                HAL_UART_Receive_DMA(&huart1, NucRxBuf, NUC_PACKAGE_LENGTH);
            }
            return;
        }
    }

    if (!(NucRxBuf[0] == 0xA9 && NucRxBuf[1] == 0xC8 &&
          NucRxBuf[2] == 0xB3 && NucRxBuf[3] == 0xE5 &&
          NucRxBuf[9] == 0x55 && NucRxBuf[15] == 0x55 &&
          NucRxBuf[18] == 0x55 && NucRxBuf[29] == 0x55 &&
          CRC16_Verify(NucRxBuf, 64, CRC16_CCITT) == HAL_OK))

    {
        NUC.ErrorCount++;
        return;
    }

    NUC.Yaw_ControlMode = NucRxBuf[4];
    Uint8ArrayToFloat32(&NUC.YawDataRecieve, &NucRxBuf[5]);

    NUC.Pitch_ControlMode = NucRxBuf[10];
    Uint8ArrayToFloat32(&NUC.PitchDataRecieve, &NucRxBuf[11]);

    NUC.VisonResult = NucRxBuf[16];
    NUC.ShootingPremision = NucRxBuf[17] == 0xA8 ? 1 : 0;

    NUC_RC_RxCallback();

    NUC.UpdateCounter++;
    if (NowTick - NUC.LastFreqUpdateTick > 500)
    {
        NUC.Freq = NUC.UpdateCounter * 1000 / (NowTick - NUC.LastFreqUpdateTick);
        NUC.UpdateCounter = 0;
        NUC.LastFreqUpdateTick = NowTick;
    }

    NUC.LastOnlineTick = NowTick;

    if (VisionHandle)
        osThreadResume(VisionHandle);
}
