/**
 * @attention   采用UTF-8字符集编码
 * @brief       NUC数据收发
 * @authors     北工大PIP战队 刘诗厚
 * @version     v5.2
 * @date        2026-10-11
 * @details     已适配《PIP2027赛季上下位机通信协议 v5.2》
 *              - 3.1 上位机 -> 下位机：72 -> 67 字节
 *              - 3.2 下位机 -> 上位机：88 -> 80 字节
 *              - 云台/火控/底盘控制改为位域标志位，位置/速度/加速度拆成独立字段
 *              - 键鼠输入块 27~36 -> 40~49
 *              - 上行帧删除“云台大 Yaw 四元数”“车辆 z 方向速度”与保留字节，
 *                新增云台 Yaw/Pitch 角速度、自身/基地/前哨站血量、剩余发弹量
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
uint8_t NucDmaBuf[68] = {0};

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


/**
 * @brief 处理时间同步包 Package-A，并发送回包 Package-B
 * @note  该帧格式在 v5.1 / v5.2 中完全一致，未做改动
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



/// @brief 下位机发送业务包（协议 3.2，共 80 字节）
/// @param Q 云台 IMU 四元数 q_w, q_x, q_y, q_z
void NUC_SendConfig(float Gimbal_Q[4],float Chassis_Q[4],float Robot_X_Velocity,float Robot_Y_Velocity)
{
    if (NUC.State != NUC_STATE_RUNNING)//只在这一状态下循环基础业务
        return;

    memset(NucTxBuf, 0, NUC_TX_PACKAGE_LENGTH);

    /* 0~3 帧头 + 帧内校验 */
    NucTxBuf[0] = 0xA9;
    NucTxBuf[1] = 0xC8;
    NucTxBuf[2] = 0xB3;
    NucTxBuf[3] = 0xE5;

    /* 4~11 发送时刻时间戳：MCU 视角，单位 us */
    uint64_t ts = DWT_GetTimeline_us();
    memcpy(&NucTxBuf[4], &ts, sizeof(uint64_t));

    /* 12 自瞄模式标志位
     *   bit0~bit3 自瞄工作模式 / bit4~bit5 打击决策优先级 / bit6~bit7 己方颜色
     *   颜色判定：RobotId 1~11 为红方，101~111 为蓝方（裁判系统离线时 RobotId=0 按红方处理）
     *   若要在台架上固定为红方，可改用下面注释掉的一行 */
    uint8_t color = (Referee.RobotStatus.RobotId >= 100) ? NUC_COLOR_BLUE : NUC_COLOR_RED;
    // uint8_t color = NUC_COLOR_RED;   // 测试时先默认红方
    NucTxBuf[12] = (uint8_t)((((uint8_t)Vision.VisionMode) & NUC_VISION_MODE_MASK) << NUC_VISION_MODE_SHIFT)
                 | (uint8_t)((((uint8_t)Vision.DecisionMode) & NUC_VISION_DECISION_MASK) << NUC_VISION_DECISION_SHIFT)
                 | (uint8_t)(((color) & NUC_VISION_COLOR_MASK) << NUC_VISION_COLOR_SHIFT);

    /* 13 帧内校验 */
    NucTxBuf[13] = 0x55;

    /* 14~29 云台 IMU 四元数 q_w, q_x, q_y, q_z */
    if (Gimbal_Q != NULL)
    {
        memcpy(&NucTxBuf[14], Gimbal_Q, 16);
    }

    /* 30 帧内校验 */
    NucTxBuf[30] = 0x55;

    /* 31~34 云台 Yaw 角速度：BMI088 读出为 rad/s，协议要求 deg/s */
    {
        float yaw_rate = Inertial.Gyro[AZ] * 57.295779513f;
        memcpy(&NucTxBuf[31], &yaw_rate, 4);
    }

    /* 35~38 云台 Pitch 角速度 */
    {
        float pitch_rate = Inertial.Gyro[AX] * 57.295779513f;
        memcpy(&NucTxBuf[35], &pitch_rate, 4);
    }

    /* 39 帧内校验 */
    NucTxBuf[39] = 0x55;

    /* 40~55 底盘 IMU 四元数 */
    {
        memcpy(&NucTxBuf[40], Chassis_Q, 16);
    }

    /* 56 帧内校验 */
    NucTxBuf[56] = 0x55;

    /* 57~58 弹丸弹速：协议要求 uint16(float(原始弹速) * 1e3) */
    uint16_t bullet_speed = (uint16_t)(Referee.ShootData.InitialSpeed * 1000.0f);
    NucTxBuf[57] = bullet_speed & 0xFF;
    NucTxBuf[58] = (bullet_speed >> 8) & 0xFF;

    /* 59 比赛状态：0x00~0x05，直接取裁判系统 GameProgress */
    NucTxBuf[59] = Referee.GameStatus.GameProgress;

    /* 60~61 当前机器人剩余血量 */
    NucTxBuf[60] = Referee.RobotStatus.CurrentHP & 0xFF;
    NucTxBuf[61] = (Referee.RobotStatus.CurrentHP >> 8) & 0xFF;

    /* 62~63 基地血量 / 64~65 前哨站血量，按己方颜色取 */
    uint16_t base_hp    = (color == NUC_COLOR_BLUE) ? Referee.GameRobotHP.BlueBaseHP    : Referee.GameRobotHP.RedBaseHP;
    uint16_t outpost_hp = (color == NUC_COLOR_BLUE) ? Referee.GameRobotHP.BlueOutpostHP : Referee.GameRobotHP.RedOutpostHP;
    NucTxBuf[62] = base_hp & 0xFF;
    NucTxBuf[63] = (base_hp >> 8) & 0xFF;
    NucTxBuf[64] = outpost_hp & 0xFF;
    NucTxBuf[65] = (outpost_hp >> 8) & 0xFF;

    /* 66~67 当前阶段剩余时间（秒） */
    NucTxBuf[66] = Referee.GameStatus.StageRemainTime & 0xFF;
    NucTxBuf[67] = (Referee.GameStatus.StageRemainTime >> 8) & 0xFF;

    /* 68 剩余发弹量 = uint8(原始数据) */
    NucTxBuf[68] = (uint8_t)(Referee.ProjectileAllowance.ProjectileAllowance17mm & 0xFF);

    /* 69 帧内校验 */
    NucTxBuf[69] = 0x55;

    /* 70~73 车辆 x 方向速度 / 74~77 车辆 y 方向速度 */
    {
        float vx = 0.0f;
        float vy = 0.0f;
        memcpy(&NucTxBuf[70], &vx, 4);
        memcpy(&NucTxBuf[74], &vy, 4);
    }

    /* 78~79 CRC16：计算 0~77 字节 */
    CRC16_Append(NucTxBuf, NUC_TX_PACKAGE_LENGTH - 2, CRC16_CCITT);

	HAL_UART_Transmit(&huart1, NucTxBuf, NUC_TX_PACKAGE_LENGTH, 10);
	CDC_Transmit_FS(NucTxBuf, NUC_TX_PACKAGE_LENGTH);
}

/// @brief 解析键鼠输入块（协议 3.1 字节 40~49，与 v5.1 的 27~36 位序一致）
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
	//如果是固定帧（协议 3.1，共 67 字节）
	if(rxBuf[0] == 0xA9 && rxBuf[1] == 0xC8 
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
	
	//如果是同步帧（协议 2.1，共 20 字节）
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

/// @brief 解析上位机业务帧（协议 3.1）
void NUC_Fixed_frame_processing()
{
    uint32_t NowTick = HAL_GetTick();
	
	  //新增时间戳检查 
    uint64_t tx_timestamp_us;
    memcpy(&tx_timestamp_us, &NucRxBuf[4], sizeof(uint64_t));
    uint64_t local_us =DWT_GetTimeline_us();
	
    int64_t diff_us = (int64_t)local_us - (int64_t)tx_timestamp_us;
	

		
		NUC.LastRxTimeUs = local_us;

    /* ---------- 字节 12 云台控制标志位 ---------- */
    NUC.YawPosValid    = (NucRxBuf[12] & NUC_GIMBAL_YAW_POS)     ? 1 : 0;
    NUC.YawVelValid    = (NucRxBuf[12] & NUC_GIMBAL_YAW_VEL)     ? 1 : 0;
    NUC.YawAccValid    = (NucRxBuf[12] & NUC_GIMBAL_YAW_ACC)     ? 1 : 0;
    NUC.PitchPosValid  = (NucRxBuf[12] & NUC_GIMBAL_PITCH_POS)   ? 1 : 0;
    NUC.PitchVelValid  = (NucRxBuf[12] & NUC_GIMBAL_PITCH_VEL)   ? 1 : 0;
    NUC.PitchAccValid  = (NucRxBuf[12] & NUC_GIMBAL_PITCH_ACC)   ? 1 : 0;


    /* ---------- 字节 13~36 云台控制量 ---------- */
    Uint8ArrayToFloat32(&NUC.YawAngle,      &NucRxBuf[13]);
    Uint8ArrayToFloat32(&NUC.YawVelocity,   &NucRxBuf[17]);
    Uint8ArrayToFloat32(&NUC.YawAccel,      &NucRxBuf[21]);
    Uint8ArrayToFloat32(&NUC.PitchAngle,    &NucRxBuf[25]);
    Uint8ArrayToFloat32(&NUC.PitchVelocity, &NucRxBuf[29]);
    Uint8ArrayToFloat32(&NUC.PitchAccel,    &NucRxBuf[33]);

    /* ---------- 字节 38 火控标志位 ---------- */
    NUC.IsLocked     = (NucRxBuf[38] & NUC_FIRE_LOCKED) ? 1 : 0;
    NUC.IsFire       = (NucRxBuf[38] & NUC_FIRE_SHOOT)  ? 1 : 0;

    /* ---------- 字节 51~63 底盘控制 ---------- */
    NUC.ChassisXVelValid   = (NucRxBuf[51] & NUC_CHASSIS_X_VEL)   ? 1 : 0;
    NUC.ChassisYVelValid   = (NucRxBuf[51] & NUC_CHASSIS_Y_VEL)   ? 1 : 0;
    NUC.ChassisYawVelValid = (NucRxBuf[51] & NUC_CHASSIS_YAW_VEL) ? 1 : 0;

    Uint8ArrayToFloat32(&NUC.ChassisXVelocity,   &NucRxBuf[52]);
    Uint8ArrayToFloat32(&NUC.ChassisYVelocity,   &NucRxBuf[56]);
    Uint8ArrayToFloat32(&NUC.ChassisYawVelocity, &NucRxBuf[60]);

    /* ---------- 派生 v5.1 兼容视图（供现有控制链路使用） ----------
     * v5.2 用“位置有效”位表达 v5.1 的 ANGLE_CONTROL，其余分量不下发到控制器。
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
