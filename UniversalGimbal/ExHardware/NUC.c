/**
 * @attention   采用UTF-8字符集编码
 * @brief       NUC数据收发
 * @authors     北工大PIP战队 刘诗厚
 * @version     v5.1
 * @date        2026-9-20
 * @details     键位的提取上做了修改，上位机把所有的键位发送出来了,留出调试空间？而且上位机把鼠标的控制和云台自瞄耦合在一起了，所以下位机还需要做特化处理
 */

#include "header.h"

#define KEY_IS_PRESSED(buf, key) (((buf)[(key) >> 3] >> ((key) & 0x07)) & 0x01)

#define NUC_PACKAGE_LENGTH 72
#define NUC_SYNC_PACKAGE_LENGTH 20

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
uint8_t NucSyncRxBuf[NUC_SYNC_PACKAGE_LENGTH] = {0};
uint8_t NucTxBuf[88] = {0};

//发送缓冲区定义在全局变量，若在函数内部定义则会出现函数退出栈变量生命周期结束，但DMA还没搬运的情况
uint8_t tx_buf[NUC_SYNC_PACKAGE_LENGTH];

//DMA接收临时缓冲区
uint8_t NucDmaBuf[128] = {0};

NUC_t NUC;

float tempy = 1.5f; // 4.f;
float tempp = 1.2f;

extern osThreadId VisionHandle;
void NUC_Fixed_frame_processing(void);


void Float32ToUint8Array(float data, uint8_t *buf)
{
    memcpy(buf, &data, sizeof(float));
}

void Uint8ArrayToFloat32(float *data, uint8_t *buf)
{
    memcpy(data, buf, sizeof(float));
}



void NUC_Init(void)
{
    NUC.UpdateCounter = 0;
    NUC.LastFreqUpdateTick = 0;
    NUC.Freq = 100;

    NUC.ErrorCount = 0;
    NUC.LastOnlineTick = 0;
	
	  /* 初始化微秒时钟 */
	DWT_Init(SystemCoreClock / 1000000);
	
    /* 状态机初始化 */
    NUC.State = NUC_STATE_UNSYNC;
    NUC.LastRxTimeUs = 0;
	
		NUC.NUC_RXNUM=0;
		NUC.NUC_TXNUM=0;

	 // 改用不定长 DMA 接收	
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, NucDmaBuf, sizeof(NucDmaBuf));
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

}


/**
 * @brief 处理时间同步包 Package-A，并发送回包 Package-B
 */
static void NUC_SyncPacketHandler(void)
{

    /* 获取本地微秒时间 */
    uint64_t mcu_clock_us =DWT_GetTimeline_us();

    // 回包
    memcpy(tx_buf, NucSyncRxBuf, NUC_SYNC_PACKAGE_LENGTH);
    memcpy(&tx_buf[10], &mcu_clock_us, sizeof(uint64_t));
  
    CRC16_Append(tx_buf, NUC_SYNC_PACKAGE_LENGTH-2, CRC16_CCITT);
	
		HAL_UART_Transmit_DMA(&huart1, tx_buf, NUC_SYNC_PACKAGE_LENGTH);

    /* 记录本次接收时间，用于 100ms 无数据检测 */
    NUC.LastRxTimeUs = mcu_clock_us;
}



//下位机发送业务包
void NUC_SendConfig(float Q[4])
{
    if (NUC.State != NUC_STATE_RUNNING)//只在这一状态下循环基础业务
        return;

    memset(NucTxBuf, 0, 88);

 
    NucTxBuf[0] = 0xA9;
    NucTxBuf[1] = 0xC8;
    NucTxBuf[2] = 0xB3;
    NucTxBuf[3] = 0xE5;

    /* 发送时间戳：MCU 视角，单位 us */
    uint64_t ts = DWT_GetTimeline_us();
    memcpy(&NucTxBuf[4], &ts, sizeof(uint64_t));

    NucTxBuf[12] = Vision.VisionMode;
    NucTxBuf[13] = Vision.DecisionMode;
		//测试时先默认A8红方
		NucTxBuf[14] = 0xA8;
    //NucTxBuf[14] = Referee.RobotStatus.RobotId < 100 ? 0xA8 : 0xB0;
	
    NucTxBuf[15] = 0x55;

    // 云台小 Yaw 四元数：q_w, q_x, q_y, q_z 
    if (Q != NULL)
    {
        memcpy(&NucTxBuf[16], Q, 16);
    }
    NucTxBuf[32] = 0x55;

    //云台大 Yaw 四元数
    {
        float big_yaw_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
        memcpy(&NucTxBuf[33], big_yaw_q, 16);
    }
    NucTxBuf[49] = 0x55;

    //底盘四元数
    {
        float chassis_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
        memcpy(&NucTxBuf[50], chassis_q, 16);
    }
    NucTxBuf[66] = 0x55;

    //弹丸弹速：协议要求 uint16(float(原始弹速)
    uint16_t bullet_speed = (uint16_t)(Referee.ShootData.InitialSpeed * 1000.0f);
    NucTxBuf[67] = bullet_speed & 0xFF;
    NucTxBuf[68] = (bullet_speed >> 8) & 0xFF;

    // 比赛状态、剩余时间：测试暂时填比赛中 剩余时间300秒
    NucTxBuf[69] = 0x04;
    NucTxBuf[70] = 0x2C;
	
    NucTxBuf[71] = 0x01;
    NucTxBuf[72] = 0x55;

    // 车辆 vx/vy/vz：暂时填 0
    {
        float vx = 0.0f;
        float vy = 0.0f;
        float vz = 0.0f;
        memcpy(&NucTxBuf[73], &vx, 4);
        memcpy(&NucTxBuf[77], &vy, 4);
        memcpy(&NucTxBuf[81], &vz, 4);
    }

    NucTxBuf[85] = 0xFF;

    /* CRC16：计算 0~85 字节，放在 86~87 */
    CRC16_Append(NucTxBuf, 86, CRC16_CCITT);

	HAL_UART_Transmit(&huart1, NucTxBuf, 88, 10);
	CDC_Transmit_FS(NucTxBuf, 88);
}

void NUC_RC_RxCallback(void)
{
    uint32_t NowTick = HAL_GetTick();

    NUC.RC.Mouse.X = 0;
    NUC.RC.Mouse.Y = 0;
    NUC.RC.Mouse.Z = 0;
    NUC.RC.Mouse.L = KEY_IS_PRESSED(&NucRxBuf[27], MOUSE_LEFT);
    NUC.RC.Mouse.R = KEY_IS_PRESSED(&NucRxBuf[27], MOUSE_RIGHT);

    NUC.RC.Key.Code.W = KEY_IS_PRESSED(&NucRxBuf[27], KEY_W);
    NUC.RC.Key.Code.A = KEY_IS_PRESSED(&NucRxBuf[27], KEY_A);
    NUC.RC.Key.Code.S = KEY_IS_PRESSED(&NucRxBuf[27], KEY_S);
    NUC.RC.Key.Code.D = KEY_IS_PRESSED(&NucRxBuf[27], KEY_D);
    NUC.RC.Key.Code.Ctrl = KEY_IS_PRESSED(&NucRxBuf[27], KEY_LCTRL);
    NUC.RC.Key.Code.Shift = KEY_IS_PRESSED(&NucRxBuf[27], KEY_LSHIFT);
    NUC.RC.Key.Code.Q = KEY_IS_PRESSED(&NucRxBuf[27], KEY_Q);
    NUC.RC.Key.Code.E = KEY_IS_PRESSED(&NucRxBuf[27], KEY_E);
    NUC.RC.Key.Code.R = KEY_IS_PRESSED(&NucRxBuf[27], KEY_R);
    NUC.RC.Key.Code.F = KEY_IS_PRESSED(&NucRxBuf[27], KEY_F);
    NUC.RC.Key.Code.G = KEY_IS_PRESSED(&NucRxBuf[27], KEY_G);
    NUC.RC.Key.Code.Z = KEY_IS_PRESSED(&NucRxBuf[27], KEY_Z);
    NUC.RC.Key.Code.X = KEY_IS_PRESSED(&NucRxBuf[27], KEY_X);
    NUC.RC.Key.Code.C = KEY_IS_PRESSED(&NucRxBuf[27], KEY_C);
    NUC.RC.Key.Code.V = KEY_IS_PRESSED(&NucRxBuf[27], KEY_V);
    NUC.RC.Key.Code.B = KEY_IS_PRESSED(&NucRxBuf[27], KEY_B);

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

void NUC_RxCallback(uint8_t *rxBuf, int rxSize)
{
	//如果是固定帧
	if(rxBuf[0] == 0xA9 && rxBuf[1] == 0xC8 
				&& rxBuf[2] == 0xB3 && rxBuf[3] == 0xE5
				&& rxBuf[17] == 0x55 && rxBuf[23] == 0x55 
        && rxBuf[26] == 0x55 && rxBuf[37] == 0x55
		    && rxBuf[51] == 0x55 
				&& CRC16_Verify(rxBuf, NUC_PACKAGE_LENGTH, CRC16_CCITT) == HAL_OK)
	{
			NUC.State = NUC_STATE_RUNNING;
			NUC_SendConfig(Inertial.Q);
			memcpy(NucRxBuf, rxBuf, NUC_PACKAGE_LENGTH);
			NUC_Fixed_frame_processing();
			//业务帧回包在惯导任务里面
	}
	
	//如果是同步帧
	else if(rxBuf[0] == 0xAA && rxBuf[1] == 0xAA 
				&& CRC16_Verify(rxBuf, NUC_SYNC_PACKAGE_LENGTH, CRC16_CCITT) == HAL_OK)
	{
			NUC.State = NUC_STATE_RUNNING;
		
			memcpy(NucSyncRxBuf, rxBuf, NUC_SYNC_PACKAGE_LENGTH);
			NUC_SyncPacketHandler();   //回包
	}
	
	//如果都不是，那么是帧错位
	else
	{
		NUC.ErrorCount++;
	}
	
	//重启DMA，准备接收下一帧
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rxBuf, sizeof(NucDmaBuf));
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
 
}

void NUC_Fixed_frame_processing()
{
    uint32_t NowTick = HAL_GetTick();
	
	  //新增时间戳检查 
    uint64_t tx_timestamp_us;
    memcpy(&tx_timestamp_us, &NucRxBuf[4], sizeof(uint64_t));
    uint64_t local_us =DWT_GetTimeline_us();
	
    int64_t diff_us = (int64_t)local_us - (int64_t)tx_timestamp_us;
	
    if (diff_us >= 50000)
    {
        NUC.State = NUC_STATE_UNSYNC;
        NUC.ErrorCount++;
        return;
    }
		
		NUC.LastRxTimeUs = local_us;

    NUC.Yaw_ControlMode = NucRxBuf[12];
    Uint8ArrayToFloat32(&NUC.YawDataRecieve, &NucRxBuf[13]);

    NUC.Pitch_ControlMode = NucRxBuf[18];
    Uint8ArrayToFloat32(&NUC.PitchDataRecieve, &NucRxBuf[19]);

    NUC.VisonResult = NucRxBuf[24];
    NUC.ShootingPremision = NucRxBuf[25] == 0xA8 ? 1 : 0;

    NUC_RC_RxCallback();

	//大概每 500ms 统计一次业务包接收频率
    NUC.UpdateCounter++;
    if (NowTick - NUC.LastFreqUpdateTick > 500)
    {
        NUC.Freq = NUC.UpdateCounter * 1000 / (NowTick - NUC.LastFreqUpdateTick);
        NUC.UpdateCounter = 0;
        NUC.LastFreqUpdateTick = NowTick;
    }

    NUC.LastOnlineTick = NowTick;

    if (VisionHandle)
        osSignalSet(VisionHandle, 0x0001);
}


void NUC_TimeoutCheck(void)
{
    if (NUC.State == NUC_STATE_UNSYNC)
        return;

    uint64_t now_us = DWT_GetTimeline_us();
    if (now_us - NUC.LastRxTimeUs >= 100000)
    {
        NUC.State = NUC_STATE_UNSYNC;
    }
}

