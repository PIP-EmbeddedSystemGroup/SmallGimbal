/**
 * @attention   采用UTF-8字符集编码
 * @brief       裁判系统读取
 * @authors     北工大PIP战队 李君豪 孙绍博 冯星侨
 * @version     v1.0
 * @date        2025-02-12
 * @details
 */

#include "header.h"

uint8_t RefereeRxBuf[192];
Referee_t Referee = { 0 };

void Referee_RxCallback_Parsing(uint8_t* rxBuf, int frameLen)
{
    // if (frameLen < 5)
    // {
    //     Referee.ErrorCount++;
    //     return;
    // }

    int dataLength = (rxBuf[1]) | (rxBuf[2] << 8);

    // if (frameLen < (dataLength + 9))
    // {
    //     Referee.ErrorCount++;
    //     return;
    // }

    if (rxBuf[0] != 0xA5 ||
        !CRC8_VerifyChecksum(rxBuf, 5) ||
        !CRC16_VerifyChecksum(rxBuf, 5 + 2 + dataLength + 2))
    {
        Referee.ErrorCount++;
        return;
    }
    Referee.PackageCount++;
    Referee.LastOnlineTick = HAL_GetTick();

    int CmdId = (rxBuf[5]) | (rxBuf[6] << 8);
    const uint8_t* Data = &rxBuf[7];

    switch (CmdId)
    {
    case 0x0001:
        memcpy(&Referee.GameStatus, Data, sizeof(Referee.GameStatus));
        break;
    case 0x0002:
        memcpy(&Referee.GameResult, Data, sizeof(Referee.GameResult));
        break;
    case 0x0003:
        memcpy(&Referee.GameRobotHP, Data, sizeof(Referee.GameRobotHP));
        break;
    case 0x0101:
        memcpy(&Referee.EventData, Data, sizeof(Referee.EventData));
        break;
    case 0x0104:
        memcpy(&Referee.RefereeWarning, Data, sizeof(Referee.RefereeWarning));
        break;
    case 0x0105:
        memcpy(&Referee.DartInfo, Data, sizeof(Referee.DartInfo));
        break;
    case 0x0201:
        memcpy(&Referee.RobotStatus, Data, sizeof(Referee.RobotStatus));
        break;
    case 0x0202:
        memcpy(&Referee.PowerHeatData, Data, sizeof(Referee.PowerHeatData));
        break;
    case 0x0203:
        memcpy(&Referee.RobotPos, Data, sizeof(Referee.RobotPos));
        break;
    case 0x0204:
        memcpy(&Referee.Buff, Data, sizeof(Referee.Buff));
        break;
    case 0x0206:
        memcpy(&Referee.HurtData, Data, sizeof(Referee.HurtData));
        switch (Referee.HurtData.ArmorId)
        {
        case 0:
            Referee.LastHurtTick.Front = HAL_GetTick();
            break;
        case 1:
            Referee.LastHurtTick.Left = HAL_GetTick();
            break;
        case 2:
            Referee.LastHurtTick.Back = HAL_GetTick();
            break;
        case 3:
            Referee.LastHurtTick.Right = HAL_GetTick();
            break;
        default:
            break;
        }
        break;
    case 0x0207:
        memcpy(&Referee.ShootData, Data, sizeof(Referee.ShootData));
        break;
    case 0x0208:
        memcpy(&Referee.ProjectileAllowance, Data, sizeof(Referee.ProjectileAllowance));  
        break;
    case 0x0209:
        memcpy(&Referee.RfidStatus, Data, sizeof(Referee.RfidStatus));
        break;
    case 0x020A:
        memcpy(&Referee.DartClientCmd, Data, sizeof(Referee.DartClientCmd));
        break;
    case 0x020B:
        memcpy(&Referee.GroundRobotPosition, Data, sizeof(Referee.GroundRobotPosition));
        break;
    case 0x020C:
        memcpy(&Referee.RadarMarkData, Data, sizeof(Referee.RadarMarkData));
        break;
    case 0x020D:
        memcpy(&Referee.SentryInfo, Data, sizeof(Referee.SentryInfo));
        break;
    case 0x020E:
        memcpy(&Referee.RadarInfo, Data, sizeof(Referee.RadarInfo));
        break;
    case 0x0301:
        memcpy(&Referee.RobotInteractionData, Data, sizeof(Referee.RobotInteractionData));
        break;
    case 0x0303:
        memcpy(&Referee.MapCommand, Data, sizeof(Referee.MapCommand));
        break;
    default:
        break;
    }
}

void Referee_Set(void)
{

    // 设置裁判系统数据
    Referee.LastHurtTick.Front = 0; // 前方伤害时间
    Referee.LastHurtTick.Left = 0;  // 左方伤害时间
    Referee.LastHurtTick.Back = 0;  // 后方伤害时间
    Referee.LastHurtTick.Right = 0; // 右方伤害时间

    Referee.GameStatus.GameType = 0;        // 游戏类型
    Referee.GameStatus.GameProgress = 0;    // 游戏进度
    Referee.GameStatus.StageRemainTime = 0; // 阶段剩余时间
    Referee.GameStatus.SyncTimeStamp = 0;   // 同步时间戳

    Referee.GameResult.Winner = 0; // 获胜者

    Referee.GameRobotHP.ally_1_robot_HP = 0;   // 1号机器人血量
    Referee.GameRobotHP.ally_2_robot_HP = 0;   // 2号机器人血量
    Referee.GameRobotHP.ally_3_robot_HP = 0;   // 3号机器人血量
    Referee.GameRobotHP.ally_4_robot_HP = 0;   // 4号机器人血量
    Referee.GameRobotHP.ally_7_robot_HP = 0;   // 7号机器人血量
    Referee.GameRobotHP.ally_base_HP = 0;       // 前哨站血量
    Referee.GameRobotHP.ally_outpost_HP = 0;   // 基地血量


    Referee.EventData.EventData = 0;

    Referee.RefereeWarning.Level = 0;            // 警告等级
    Referee.RefereeWarning.OffendingRobotId = 0; // 违规机器人ID
    Referee.RefereeWarning.Count = 0;            // 违规次数

    Referee.DartInfo.DartRemainingTime = 0; // 飞镖剩余时间
    Referee.DartInfo.DartInfo = 0;          // 飞镖信息

    Referee.RobotStatus.RobotId = 0;                      // 机器人ID
    Referee.RobotStatus.RobotLevel = 0;                   // 机器人等级
    Referee.RobotStatus.CurrentHP = 0;                    // 当前血量
    Referee.RobotStatus.MaximumHP = 0;                    // 最大血量
    Referee.RobotStatus.ShooterBarrelCoolingValue = 100;    // 射手炮管冷却值
    Referee.RobotStatus.ShooterBarrelHeatLimit = 150;       // 射手炮管热量限制
    Referee.RobotStatus.ChassisPowerLimit = 220;            // 底盘功率限制
    Referee.RobotStatus.PowerManagementGimbalOutput = 0;  // 功率管理云台输出
    Referee.RobotStatus.PowerManagementChassisOutput = 0; // 功率管理底盘输出
    Referee.RobotStatus.PowerManagementShooterOutput = 0; // 功率管理射手输出

    Referee.PowerHeatData.reserved = 0;         // 保留位
    Referee.PowerHeatData.reserved1 = 0;         // 保留位
    Referee.PowerHeatData.reserved2 = 0;           // 保留位
    Referee.PowerHeatData.BufferEnergy = 60;           // 缓冲能量
    Referee.PowerHeatData.Shooter17mm1BarrelHeat = 0; // 射手17mm1炮管热量
    Referee.PowerHeatData.Shooter42mmBarrelHeat = 0;  // 射手42mm炮管热量

    Referee.RobotPos.X = 0;     // X坐标
    Referee.RobotPos.Y = 0;     // Y坐标
    Referee.RobotPos.Angle = 0; // 角度

    Referee.Buff.RecoveryBuff = 0;      // 恢复增益
    Referee.Buff.CoolingBuff = 0;       // 冷却增益
    Referee.Buff.DefenceBuff = 0;       // 防御增益
    Referee.Buff.VulnerabilityBuff = 0; // 弱点增益
    Referee.Buff.AttackBuff = 0;        // 攻击增益

    Referee.HurtData.ArmorId = 0;           // 护甲ID
    Referee.HurtData.HPDeductionReason = 0; // 血量扣除原因

    Referee.ShootData.BulletType = 0;         // 子弹类型
    Referee.ShootData.ShooterNumber = 0;      // 射手编号
    Referee.ShootData.LaunchingFrequency = 0; // 发射频率
    Referee.ShootData.InitialSpeed = 0;       // 初始速度

    Referee.ProjectileAllowance.ProjectileAllowance17mm = 0; // 17mm弹丸许可
    Referee.ProjectileAllowance.ProjectileAllowance42mm = 0; // 42mm弹丸许可
    Referee.ProjectileAllowance.RemainingGoldCoin = 0;       // 剩余金币

    Referee.RfidStatus.RfidStatus = 0; // RFID状态

    Referee.DartClientCmd.DartLaunchOpeningStatus = 0; // 箭发射开启状态
    Referee.DartClientCmd.Reserved = 0;                // 保留字段
    Referee.DartClientCmd.TargetChangeTime = 0;        // 目标改变时间
    Referee.DartClientCmd.LatestLaunchCmdTime = 0;     // 最新发射命令时间

    Referee.GroundRobotPosition.HeroX = 0;      // 英雄X坐标
    Referee.GroundRobotPosition.HeroY = 0;      // 英雄Y坐标
    Referee.GroundRobotPosition.EngineerX = 0;  // 工程师X坐标
    Referee.GroundRobotPosition.EngineerY = 0;  // 工程师Y坐标
    Referee.GroundRobotPosition.Standard3X = 0; // 标准3X坐标
    Referee.GroundRobotPosition.Standard3Y = 0; // 标准3Y坐标
    Referee.GroundRobotPosition.Standard4X = 0; // 标准4X坐标
    Referee.GroundRobotPosition.Standard4Y = 0; // 标准4Y坐标
    Referee.GroundRobotPosition.reserved = 0; // 保留位
    Referee.GroundRobotPosition.reserved1 = 0; // 保留位

    Referee.RadarMarkData.MarkProgress = 0;

    Referee.SentryInfo.SentryInfo = 0;  // 哨兵信息
    Referee.SentryInfo.SentryInfo2 = 0; // 哨兵信息2

    Referee.RadarInfo.RadarInfo = 0; // 雷达信息

    Referee.RobotInteractionData.DataCmdId = 0;  // 数据命令ID
    Referee.RobotInteractionData.SenderId = 0;   // 发送者ID
    Referee.RobotInteractionData.ReceiverId = 0; // 接收者ID
    for (int i = 0; i < 112; i++)
        Referee.RobotInteractionData.UserData[i] = 0; // 用户数据（112字节）

    Referee.MapCommand.TargetPositionX = 0; // 目标位置X坐标
    Referee.MapCommand.TargetPositionY = 0; // 目标位置Y坐标
    Referee.MapCommand.CmdKeyboard = 0;     // 命令键盘
    Referee.MapCommand.TargetRobotId = 0;   // 目标机器人ID
    Referee.MapCommand.CmdSource = 0;       // 命令来源
}
