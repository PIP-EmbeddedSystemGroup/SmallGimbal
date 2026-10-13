#ifndef _NUC_H_
#define _NUC_H_

/**
 * @attention   采用UTF-8字符集编码
 * @brief       NUC(上位机)串口通信协议数据结构
 * @version     v5.2
 * @date        2026-10-11
 * @details     已适配《PIP2027赛季上下位机通信协议 v5.2》，与 v5.1 的差异：
 *              - 3.1 上位机 -> 下位机 业务帧：72 -> 67 字节
 *              - 3.2 下位机 -> 上位机 业务帧：88 -> 80 字节
 *              - 云台/火控/底盘控制由 0xB4 之类的枚举改为位域标志位
 *              - 云台位置/速度/加速度拆成独立字段，删除前馈与目标角速度/角加速度
 *              - 键鼠输入块由字节 27~36 移到 40~49
 *              - 下位机上行帧删除“云台大 Yaw 四元数”“车辆 z 方向速度”，
 *                新增云台 Yaw/Pitch 角速度与血量、剩余发弹量
 */

#include "stm32f4xx_hal.h"
#include "stdint.h"

extern DMA_HandleTypeDef hdma_usart1_rx;

/* ============================== 帧长度 ============================== */

#define NUC_RX_PACKAGE_LENGTH   67U     // 3.1 上位机 -> 下位机 业务帧
#define NUC_TX_PACKAGE_LENGTH   80U     // 3.2 下位机 -> 上位机 业务帧
#define NUC_SYNC_PACKAGE_LENGTH 20U     // 2.1 时间戳同步帧 Package-A / Package-B

/** 3.1 帧内校验字节位置（内容均固定为 0x55） */
#define NUC_RX_CHECK_POS_0      37U
#define NUC_RX_CHECK_POS_1      39U
#define NUC_RX_CHECK_POS_2      50U
#define NUC_RX_CHECK_POS_3      64U
#define NUC_RX_CHECK_VALUE      0x55U

/** 3.1 键鼠输入块起始字节（键盘 0~70 位 + 鼠标 72~78 位，共 10 字节） */
#define NUC_RX_KEY_BYTE_OFFSET  40U

/* ===================== 3.1 字节 12 云台控制标志位 ==================== */

#define NUC_GIMBAL_YAW_POS      (1U << 0)   // Yaw 位置有效
#define NUC_GIMBAL_YAW_VEL      (1U << 1)   // Yaw 速度有效
#define NUC_GIMBAL_YAW_ACC      (1U << 2)   // Yaw 加速度有效
#define NUC_GIMBAL_PITCH_POS    (1U << 4)   // Pitch 位置有效
#define NUC_GIMBAL_PITCH_VEL    (1U << 5)   // Pitch 速度有效
#define NUC_GIMBAL_PITCH_ACC    (1U << 6)   // Pitch 加速度有效

/* ====================== 3.1 字节 38 火控标志位 ====================== */

#define NUC_FIRE_LOCKED         (1U << 0)   // 是否锁定
#define NUC_FIRE_SHOOT          (1U << 1)   // 是否开火

/* ===================== 3.1 字节 51 底盘控制标志位 =================== */

#define NUC_CHASSIS_X_VEL       (1U << 0)   // x 速度使能
#define NUC_CHASSIS_Y_VEL       (1U << 1)   // y 速度使能
#define NUC_CHASSIS_YAW_VEL     (1U << 2)   // yaw 偏航角速度使能

/* ===================== 3.2 字节 12 自瞄模式标志位 =================== */

#define NUC_VISION_MODE_SHIFT       0U
#define NUC_VISION_MODE_MASK        0x0FU   // bit0~bit3 自瞄工作模式
#define NUC_VISION_DECISION_SHIFT   4U
#define NUC_VISION_DECISION_MASK    0x03U   // bit4~bit5 打击决策优先级
#define NUC_VISION_COLOR_SHIFT      6U
#define NUC_VISION_COLOR_MASK       0x03U   // bit6~bit7 己方颜色

#define NUC_COLOR_RED               0x00U   // 0b00 红方
#define NUC_COLOR_BLUE              0x01U   // 0b01 蓝方

/* =========================== v5.1 兼容定义 ========================== */
/* v5.1 用枚举 0xB4 / 0xF3 / 0xC5 描述云台的“角度/角速度/角加速度”控制模式，
 * v5.2 改成了“云台控制标志位”的位置/速度/加速度独立使能位。
 * 为不打乱既有控制链路，解析时把“位置有效”位派生回 ANGLE_CONTROL，
 * 这样 VisionTask 里 `NUC.Pitch_ControlMode == ANGLE_CONTROL` 之类的判断无需改动。
 * 速度/加速度/底盘等新字段已全部解析进 NUC_t，供后续自定义控制逻辑取用。 */
typedef enum {
    NUC_STATE_UNSYNC = 0,
    NUC_STATE_SYNCING,
    NUC_STATE_RUNNING
} NUC_State_t;

typedef struct
{
    /* ---------------- 3.1 字节 12 云台控制标志位 ---------------- */
    uint8_t GimbalCtrlFlag;     // 原始字节
    uint8_t YawPosValid;        // bit0
    uint8_t YawVelValid;        // bit1
    uint8_t YawAccValid;        // bit2
    uint8_t PitchPosValid;      // bit4
    uint8_t PitchVelValid;      // bit5
    uint8_t PitchAccValid;      // bit6

    /* ---------------- 3.1 字节 13~36 云台控制量 ---------------- */
    float YawAngle;             // 字节 13~16, deg,   ∈[-180, 180]
    float YawVelocity;          // 字节 17~20, deg/s
    float YawAccel;             // 字节 21~24, deg/s^2
    float PitchAngle;           // 字节 25~28, deg,   ∈[-180, 180]
    float PitchVelocity;        // 字节 29~32, deg/s
    float PitchAccel;           // 字节 33~36, deg/s^2

    /* ---------------- 3.1 字节 38 火控标志位 ---------------- */
    uint8_t FireCtrlFlag;       // 原始字节, 0x00~0x03
    uint8_t IsLocked;           // bit0
    uint8_t IsFire;             // bit1

    /* ---------------- 3.1 字节 51~63 底盘控制 ---------------- */
    uint8_t ChassisCtrlFlag;    // 原始字节
    uint8_t ChassisXVelValid;   // bit0
    uint8_t ChassisYVelValid;   // bit1
    uint8_t ChassisYawVelValid; // bit2
    float ChassisXVelocity;     // 字节 52~55, m/s
    float ChassisYVelocity;     // 字节 56~59, m/s
    float ChassisYawVelocity;   // 字节 60~63, deg/s

//    /* ---------- 由上述字段派生的 v5.1 兼容视图（只读） ---------- */
//    uint8_t Yaw_ControlMode;    // = YawPosValid   ? ANGLE_CONTROL : 0
//    uint8_t Pitch_ControlMode;  // = PitchPosValid ? ANGLE_CONTROL : 0
//    float YawDataRecieve;       // = YawAngle
//    float PitchDataRecieve;     // = PitchAngle
//    uint8_t VisonResult;        // = IsLocked ? DETECTED_AND_LOCKED : NO_DETECTED
//    uint8_t ShootingPremision;  // = IsFire


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
	volatile uint8_t NucRxNeedRestart;		// NUC 串口异常标志位
	
} NUC_t;

uint64_t NUC_GetClockUs(void);
void NUC_Init(void);
void NUC_SendConfig(float Gimbal_Q[4],float Chassis_Q[4],float Robot_X_Velocity,float Robot_Y_Velocity);
void NUC_RxCallback(uint8_t *rxBuf, int rxSize);
void NUC_TimeoutCheck(void);
	
extern uint8_t NucRxBuf[NUC_RX_PACKAGE_LENGTH];
extern  uint8_t NucDmaBuf[68];
extern NUC_t NUC;

#endif
