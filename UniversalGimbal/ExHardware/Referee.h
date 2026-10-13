#ifndef _REFEREE_H_
#define _REFEREE_H_

/**
 * @attention   采用UTF-8字符集编码
 */

#include "stdint.h"
#include "cmsis_os.h"

typedef struct
{
    int PackageCount;   //接收到的数据包数量
    int StickyCount;    //粘包数量
    int ErrorCount;     //错误数据计数
    uint32_t LastOnlineTick;

    struct
    {
        uint32_t Front;
        uint32_t Left;
        uint32_t Back;
        uint32_t Right;
    } LastHurtTick;

    __packed struct
    {
        uint8_t GameType : 4;
        uint8_t GameProgress : 4;
        uint16_t StageRemainTime;
        uint64_t SyncTimeStamp;
    } GameStatus;

    __packed struct
    {
        uint8_t Winner;
    } GameResult;

    __packed struct
    {
        uint16_t ally_1_robot_HP; 
        uint16_t ally_2_robot_HP; 
        uint16_t ally_3_robot_HP; 
        uint16_t ally_4_robot_HP; 
        uint16_t reserved; 
        uint16_t ally_7_robot_HP; 
        uint16_t ally_outpost_HP; 
        uint16_t ally_base_HP;
    } GameRobotHP;

    __packed struct
    {
        uint32_t EventData;
    } EventData;

    __packed struct
    {
        uint8_t Level;
        uint8_t OffendingRobotId;
        uint8_t Count;
    } RefereeWarning;

    __packed struct
    {
        uint8_t DartRemainingTime;
        uint16_t DartInfo;
    } DartInfo;

    __packed struct
    {
        uint8_t RobotId;
        uint8_t RobotLevel;
        uint16_t CurrentHP;
        uint16_t MaximumHP;
        uint16_t ShooterBarrelCoolingValue;
        uint16_t ShooterBarrelHeatLimit;
        uint16_t ChassisPowerLimit;
        uint8_t PowerManagementGimbalOutput : 1;
        uint8_t PowerManagementChassisOutput : 1;
        uint8_t PowerManagementShooterOutput : 1;
    } RobotStatus;

    __packed struct
    {
        uint16_t reserved;
        uint16_t reserved1;
        float reserved2;
        uint16_t BufferEnergy;
        uint16_t Shooter17mm1BarrelHeat;
        uint16_t Shooter42mmBarrelHeat;
    } PowerHeatData;

    __packed struct
    {
        float X;
        float Y;
        float Angle;
    } RobotPos;

    __packed struct
    {
        uint8_t RecoveryBuff;
        uint8_t CoolingBuff;
        uint8_t DefenceBuff;
        uint8_t VulnerabilityBuff;
        uint16_t AttackBuff;
        uint8_t RemainingEnergy;
    } Buff;

    __packed struct
    {
        uint8_t ArmorId : 4;
        uint8_t HPDeductionReason : 4;
    } HurtData;

    __packed struct
    {
        uint8_t BulletType;
        uint8_t ShooterNumber;
        uint8_t LaunchingFrequency;
        float InitialSpeed;
    } ShootData;

    __packed struct
    {
        uint16_t ProjectileAllowance17mm;
        uint16_t ProjectileAllowance42mm;
        uint16_t RemainingGoldCoin;
        uint16_t ProjectileAllowanceFortress;
    } ProjectileAllowance;

    __packed struct
    {
        uint32_t RfidStatus;
        uint8_t RfidStatus2;
    } RfidStatus;

    __packed struct
    {
        uint8_t DartLaunchOpeningStatus;
        uint8_t Reserved;
        uint16_t TargetChangeTime;
        uint16_t LatestLaunchCmdTime;
    } DartClientCmd;

    __packed struct
    {
        float HeroX;
        float HeroY;
        float EngineerX;
        float EngineerY;
        float Standard3X;
        float Standard3Y;
        float Standard4X;
        float Standard4Y;
        float  reserved;
        float  reserved1;
    } GroundRobotPosition;

    __packed struct
    {
        uint16_t MarkProgress;
    } RadarMarkData;

    __packed struct
    {
        uint32_t SentryInfo;
        uint16_t SentryInfo2;
    } SentryInfo;

    __packed struct
    {
        uint8_t RadarInfo;
    } RadarInfo;

    __packed struct
    {
        uint16_t DataCmdId;
        uint16_t SenderId;
        uint16_t ReceiverId;
        uint8_t UserData[112];
    } RobotInteractionData;

    __packed struct
    {
        float TargetPositionX;
        float TargetPositionY;
        uint8_t CmdKeyboard;
        uint8_t TargetRobotId;
        uint16_t CmdSource;
    } MapCommand;
} Referee_t;

void Referee_Init(void);
void Referee_RxCallback(uint8_t* rxBuf, int rxSize);

extern uint8_t RefereeRxBuf[192];
extern Referee_t Referee;

#endif

