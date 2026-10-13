/**
 * @attention   采用UTF-8字符集编码
 * @brief       NUC数据收发
 * @authors     北工大PIP战队 刘诗厚
 * @version     v5.3
 * @date        2026-10-13
 * @details     已适配《PIP2027赛季上下位机通信协议 v5.3》
 *              - 删除时钟域同步机制，业务帧不再收发 uint64 发送时刻时间戳
 *              - 3.1 上位机 -> 下位机：67 -> 59 字节（删除原字节 4~11）
 *              - 3.2 下位机 -> 上位机：80 -> 72 字节（删除原字节 4~11）
 *              - 因删除 8 字节时间戳，收/发两侧所有 >= 4 的字节偏移整体 -8
 *              - 自瞄工作模式新增 0b0101 全自动工作模式
 *              - 时间戳同步帧（2.1）格式不变，收到后进入 SYNCING，收到业务帧才进入 RUNNING
 *              键位的提取上做了修改，上位机把所有的键位发送出来了,留出调试空间？而且上位机把鼠标的控制和云台自瞄耦合在一起了，所以下位机还需要做特化处理
 */

#include "header.h"

#define KEY_IS_PRESSED(buf, key) (((buf)[(key) >> 3] >> ((key) & 0x07)) & 0x01)

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

uint8_t NucRxBuf[NUC_RX_PACKAGE_LENGTH] = {0};
uint8_t NucSyncRxBuf[NUC_SYNC_PACKAGE_LENGTH] = {0};
uint8_t NucTxBuf[NUC_TX_PACKAGE_LENGTH] = {0};

//发送缓冲区定义在全局变量，若在函数内部定义则会出现函数退出栈变量生命周期结束，但DMA还没搬运的情况
uint8_t tx_buf[NUC_SYNC_PACKAGE_LENGTH];

//DMA接收临时缓冲区
uint8_t NucDmaBuf[NUC_DMA_BUF_SIZE] = {0};

//USB CDC 接收转发缓冲区
uint8_t NucUsbRxBuf[NUC_USB_BUF_SIZE] = {0};

NUC_t NUC;

float tempy = 1.5f; // 4.f;
float tempp = 1.2f;
extern osThreadId GimbalHandle;
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
	  DWT_Init(168);
	
    /* 状态机初始化 */
    NUC.State = NUC_STATE_UNSYNC;
    NUC.LastRxTimeUs = 0;
	
		NUC.NUC_RXNUM=0;
		NUC.NUC_TXNUM=0;

	 // 改用不定长 DMA 接收	
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, NucDmaBuf, sizeof(NucDmaBuf));
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

}


/// @brief 处理时间同步包 Package-A，并发送回包 Package-B
static void NUC_SyncPacketHandler(void)
{

    uint64_t mcu_clock_us =DWT_GetTimeline_us();

    memcpy(tx_buf, NucSyncRxBuf, NUC_SYNC_PACKAGE_LENGTH);
    memcpy(&tx_buf[10], &mcu_clock_us, sizeof(uint64_t));
  
    CRC16_Append(tx_buf, NUC_SYNC_PACKAGE_LENGTH-2, CRC16_CCITT);
	
		HAL_UART_Transmit_DMA(&huart1, tx_buf, NUC_SYNC_PACKAGE_LENGTH);

    NUC.LastRxTimeUs = mcu_clock_us;
}



/// @brief 下位机发送业务包（协议 3.2，共 72 字节）
void NUC_SendConfig(float Gimbal_Q[4],float Chassis_Q[4],float Robot_X_Velocity,float Robot_Y_Velocity)
{
    if (NUC.State != NUC_STATE_RUNNING)
        return;

    memset(NucTxBuf, 0, NUC_TX_PACKAGE_LENGTH);

    NucTxBuf[0] = 0xA9;
    NucTxBuf[1] = 0xC8;
    NucTxBuf[2] = 0xB3;
    NucTxBuf[3] = 0xE5;


    uint8_t color = (Referee.RobotStatus.RobotId >= 100) ? NUC_COLOR_BLUE : NUC_COLOR_RED;

    NucTxBuf[4] = (uint8_t)((((uint8_t)Vision.VisionMode) & NUC_VISION_MODE_MASK) << NUC_VISION_MODE_SHIFT)
                | (uint8_t)((((uint8_t)Vision.DecisionMode) & NUC_VISION_DECISION_MASK) << NUC_VISION_DECISION_SHIFT)
                | (uint8_t)(((color) & NUC_VISION_COLOR_MASK) << NUC_VISION_COLOR_SHIFT);

    NucTxBuf[5] = 0x55;

    if (Gimbal_Q != NULL)
    {
        memcpy(&NucTxBuf[6], Gimbal_Q, 16);
    }

    NucTxBuf[22] = 0x55;

    {
        float yaw_rate = Inertial.Gyro[AZ] * 57.295779513f;
        memcpy(&NucTxBuf[23], &yaw_rate, 4);
    }

    {
        float pitch_rate = Inertial.Gyro[AX] * 57.295779513f;
        memcpy(&NucTxBuf[27], &pitch_rate, 4);
    }

    NucTxBuf[31] = 0x55;

    {
        memcpy(&NucTxBuf[32], Chassis_Q, 16);
    }

    NucTxBuf[48] = 0x55;

    uint16_t bullet_speed = (uint16_t)(Referee.ShootData.InitialSpeed * 1000.0f);
    NucTxBuf[49] = bullet_speed & 0xFF;
    NucTxBuf[50] = (bullet_speed >> 8) & 0xFF;

    NucTxBuf[51] = Referee.GameStatus.GameProgress;

    NucTxBuf[52] = Referee.RobotStatus.CurrentHP & 0xFF;
    NucTxBuf[53] = (Referee.RobotStatus.CurrentHP >> 8) & 0xFF;

    NucTxBuf[54] = Referee.GameRobotHP.ally_base_HP & 0xFF;
    NucTxBuf[55] = (Referee.GameRobotHP.ally_base_HP >> 8) & 0xFF;
    NucTxBuf[56] = Referee.GameRobotHP.ally_outpost_HP & 0xFF;
    NucTxBuf[57] = (Referee.GameRobotHP.ally_outpost_HP >> 8) & 0xFF;

    NucTxBuf[58] = Referee.GameStatus.StageRemainTime & 0xFF;
    NucTxBuf[59] = (Referee.GameStatus.StageRemainTime >> 8) & 0xFF;

    NucTxBuf[60] = (uint8_t)(Referee.ProjectileAllowance.ProjectileAllowance17mm & 0xFF);

    NucTxBuf[61] = 0x55;

    {
        float vx = 0.0f;
        float vy = 0.0f;
        memcpy(&NucTxBuf[62], &vx, 4);
        memcpy(&NucTxBuf[66], &vy, 4);
    }

    CRC16_Append(NucTxBuf, NUC_TX_PACKAGE_LENGTH - 2, CRC16_CCITT);

	HAL_UART_Transmit(&huart1, NucTxBuf, NUC_TX_PACKAGE_LENGTH, 10);
	CDC_Transmit_FS(NucTxBuf, NUC_TX_PACKAGE_LENGTH);
}

/// @brief 解析键鼠输入块（协议 3.1 字节 32~41，与 v5.1 的位序一致）
void NUC_RC_RxCallback(void)
{
    uint8_t *KeyBuf = &NucRxBuf[NUC_RX_KEY_BYTE_OFFSET];

    NUC.RC.Mouse.X = 0;
    NUC.RC.Mouse.Y = 0;
    NUC.RC.Mouse.Z = 0;
    NUC.RC.Mouse.L = KEY_IS_PRESSED(KeyBuf, MOUSE_LEFT);
    NUC.RC.Mouse.R = KEY_IS_PRESSED(KeyBuf, MOUSE_RIGHT);

    NUC.RC.Key.Code.W = KEY_IS_PRESSED(KeyBuf, KEY_W);
    NUC.RC.Key.Code.A = KEY_IS_PRESSED(KeyBuf, KEY_A);
    NUC.RC.Key.Code.S = KEY_IS_PRESSED(KeyBuf, KEY_S);
    NUC.RC.Key.Code.D = KEY_IS_PRESSED(KeyBuf, KEY_D);
    NUC.RC.Key.Code.Ctrl = KEY_IS_PRESSED(KeyBuf, KEY_LCTRL);
    NUC.RC.Key.Code.Shift = KEY_IS_PRESSED(KeyBuf, KEY_LSHIFT);
    NUC.RC.Key.Code.Q = KEY_IS_PRESSED(KeyBuf, KEY_Q);
    NUC.RC.Key.Code.E = KEY_IS_PRESSED(KeyBuf, KEY_E);
    NUC.RC.Key.Code.R = KEY_IS_PRESSED(KeyBuf, KEY_R);
    NUC.RC.Key.Code.F = KEY_IS_PRESSED(KeyBuf, KEY_F);
    NUC.RC.Key.Code.G = KEY_IS_PRESSED(KeyBuf, KEY_G);
    NUC.RC.Key.Code.Z = KEY_IS_PRESSED(KeyBuf, KEY_Z);
    NUC.RC.Key.Code.X = KEY_IS_PRESSED(KeyBuf, KEY_X);
    NUC.RC.Key.Code.C = KEY_IS_PRESSED(KeyBuf, KEY_C);
    NUC.RC.Key.Code.V = KEY_IS_PRESSED(KeyBuf, KEY_V);
    NUC.RC.Key.Code.B = KEY_IS_PRESSED(KeyBuf, KEY_B);

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
	
	if (rxBuf == NULL || rxSize < (int)NUC_SYNC_PACKAGE_LENGTH)
	{
		NUC.ErrorCount++;
		return;
	}

	//如果是固定帧
	if(rxSize >= (int)NUC_RX_PACKAGE_LENGTH
				&& rxBuf[0] == 0xA9 && rxBuf[1] == 0xC8 
				&& rxBuf[2] == 0xB3 && rxBuf[3] == 0xE5
				&& rxBuf[NUC_RX_CHECK_POS_0] == NUC_RX_CHECK_VALUE
				&& rxBuf[NUC_RX_CHECK_POS_1] == NUC_RX_CHECK_VALUE
        && rxBuf[NUC_RX_CHECK_POS_2] == NUC_RX_CHECK_VALUE
		    && rxBuf[NUC_RX_CHECK_POS_3] == NUC_RX_CHECK_VALUE 
				&& CRC16_Verify(rxBuf, NUC_RX_PACKAGE_LENGTH, CRC16_CCITT) == HAL_OK)
	{
			NUC.State = NUC_STATE_RUNNING;
			memcpy(NucRxBuf, rxBuf, NUC_RX_PACKAGE_LENGTH);
			NUC_Fixed_frame_processing();
	}
	
	//如果是同步帧
	else if(rxSize >= (int)NUC_SYNC_PACKAGE_LENGTH
				&& rxBuf[0] == 0xAA && rxBuf[1] == 0xAA 
				&& CRC16_Verify(rxBuf, NUC_SYNC_PACKAGE_LENGTH, CRC16_CCITT) == HAL_OK)
	{
			if (NUC.State == NUC_STATE_UNSYNC)
				NUC.State = NUC_STATE_SYNCING;
		
			memcpy(NucSyncRxBuf, rxBuf, NUC_SYNC_PACKAGE_LENGTH);
			NUC_SyncPacketHandler();   //回包
	}
	
	//如果都不是，那么是帧错位
	else
	{
		NUC.ErrorCount++;
	}
	
	//重启DMA，准备接收下一帧
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, NucDmaBuf, sizeof(NucDmaBuf));
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
 
}

/// @brief 解析上位机业务帧
void NUC_Fixed_frame_processing()
{
    uint32_t NowTick = HAL_GetTick();
	
    NUC.LastRxTimeUs = DWT_GetTimeline_us();

    NUC.YawPosValid    = (NucRxBuf[4] & NUC_GIMBAL_YAW_POS)     ? 1 : 0;
    NUC.YawVelValid    = (NucRxBuf[4] & NUC_GIMBAL_YAW_VEL)     ? 1 : 0;
    NUC.YawAccValid    = (NucRxBuf[4] & NUC_GIMBAL_YAW_ACC)     ? 1 : 0;
    NUC.PitchPosValid  = (NucRxBuf[4] & NUC_GIMBAL_PITCH_POS)   ? 1 : 0;
    NUC.PitchVelValid  = (NucRxBuf[4] & NUC_GIMBAL_PITCH_VEL)   ? 1 : 0;
    NUC.PitchAccValid  = (NucRxBuf[4] & NUC_GIMBAL_PITCH_ACC)   ? 1 : 0;


    Uint8ArrayToFloat32(&NUC.YawAngle,      &NucRxBuf[5]);
    Uint8ArrayToFloat32(&NUC.YawVelocity,   &NucRxBuf[9]);
    Uint8ArrayToFloat32(&NUC.YawAccel,      &NucRxBuf[13]);
    Uint8ArrayToFloat32(&NUC.PitchAngle,    &NucRxBuf[17]);
    Uint8ArrayToFloat32(&NUC.PitchVelocity, &NucRxBuf[21]);
    Uint8ArrayToFloat32(&NUC.PitchAccel,    &NucRxBuf[25]);

    NUC.FireCtrlFlag = NucRxBuf[30];
    NUC.IsLocked     = (NucRxBuf[30] & NUC_FIRE_LOCKED) ? 1 : 0;
    NUC.IsFire       = (NucRxBuf[30] & NUC_FIRE_SHOOT)  ? 1 : 0;

    NUC.ChassisCtrlFlag    = NucRxBuf[43];
    NUC.ChassisXVelValid   = (NucRxBuf[43] & NUC_CHASSIS_X_VEL)   ? 1 : 0;
    NUC.ChassisYVelValid   = (NucRxBuf[43] & NUC_CHASSIS_Y_VEL)   ? 1 : 0;
    NUC.ChassisYawVelValid = (NucRxBuf[43] & NUC_CHASSIS_YAW_VEL) ? 1 : 0;

    Uint8ArrayToFloat32(&NUC.ChassisXVelocity,   &NucRxBuf[44]);
    Uint8ArrayToFloat32(&NUC.ChassisYVelocity,   &NucRxBuf[48]);
    Uint8ArrayToFloat32(&NUC.ChassisYawVelocity, &NucRxBuf[52]);

    /* ---------- 派生 v5.1 兼容视图（供现有控制链路使用） ----------
     * v5.2 起用“位置有效”位表达 v5.1 的 ANGLE_CONTROL，其余分量不下发到控制器。
     * 若需要把速度/加速度也接进控制链路，直接用上面的 YawVelocity / YawAccel 等字段。 */
//    NUC.Yaw_ControlMode   = NUC.YawPosValid   ? ANGLE_CONTROL : 0;
//    NUC.Pitch_ControlMode = NUC.PitchPosValid ? ANGLE_CONTROL : 0;
//    NUC.YawDataRecieve    = NUC.YawAngle;
//    NUC.PitchDataRecieve  = NUC.PitchAngle;
//    NUC.VisonResult       = NUC.IsLocked ? DETECTED_AND_LOCKED : NO_DETECTED;
//    NUC.ShootingPremision = NUC.IsFire;

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

    if (GimbalHandle)
        osSignalSet(GimbalHandle, 0x0001);
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
