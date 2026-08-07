/**
 * @attention   采用UTF-8字符集编码
 * @brief       射击火控任务
 * @authors     北工大PIP战队 樊郡捷
 * @version     v1.0
 * @date        2024-10-17
 * @details     实现了定频射击 卡弹检测
 */

#include "header.h"

#define SHOOTING_TASK_UPDATE_TICK 3
#define FEEDING_PERIOD_TICK (1000 * 1 / SHOOTING_FREQ)

Shooting_t Shooting;

void Shooting_FrictionSpeedPID_Calc(int frictionNum);
void Shooting_MotorOutput(void);
void Shooting_CheckStuck(void);
void Shooting_Disable(void);

int Shoot_flag = 0;
uint16_t ShooterBarrelHeat = 0;
uint16_t Heatdiff = 0;
uint32_t LastCoolTick = 0;
int ShooterBarrelHeat_Calc()
{
    // 如果射击标志位为真，则射击枪管热量增加10
    if (Shoot_flag)
    {
        ShooterBarrelHeat += 10;
    }

    // 如果射击枪管热量大于0且距离上次冷却时间超过100ms
    if (ShooterBarrelHeat > 0 && HAL_GetTick() - LastCoolTick > 100)
    {
        // 如果射击枪管热量减去冷却值后大于等于0，则射击枪管热量减去冷却值
        if (ShooterBarrelHeat -= Referee.RobotStatus.ShooterBarrelCoolingValue / 10 >= 0)
            ShooterBarrelHeat -= Referee.RobotStatus.ShooterBarrelCoolingValue / 10;
        // 否则射击枪管热量置为0
        else
            ShooterBarrelHeat = 0;
        // 更新上次冷却时间
        LastCoolTick = HAL_GetTick();
    }
    // 如果射击枪管热量小于70%的射击枪管热量限制减去功率热量数据中的射击17mm1枪管热量
    if (((Referee.RobotStatus.ShooterBarrelHeatLimit * 0.8f) - Referee.PowerHeatData.Shooter17mm1BarrelHeat) >= ((Referee.RobotStatus.ShooterBarrelHeatLimit * 0.8f) - ShooterBarrelHeat))
    {
        // 热量差为射击枪管热量限制减去射击枪管热量
        Heatdiff = Referee.RobotStatus.ShooterBarrelHeatLimit - ShooterBarrelHeat;
    }
    // 否则热量差为射击枪管热量限制减去功率热量数据中的射击17mm1枪管热量
    else
    {
        Heatdiff = Referee.RobotStatus.ShooterBarrelHeatLimit - Referee.PowerHeatData.Shooter17mm1BarrelHeat;
    }
    // 如果热量差大于40，则返回1
    if (Heatdiff > 40)
    {
        return 1;
    }
    // 否则返回0
    else
    {
        return 0;
    }
}

void ShootingTask(void const *argument)
{
    uint32_t PreviousWakeTime = osKernelSysTick();

    int ShootingRpmInit = 5950;
    float PrevShootingSpeed = 20;

    PID_InitStruct_t PID_InitParam = {0};
    PID_InitParam.DeltaTime = SHOOTING_TASK_UPDATE_TICK / 1000.f;

    M3508_Init(&Shooting.Friction[SHOOTING_FRI_L].Motor, 1);
    M3508_Init(&Shooting.Friction[SHOOTING_FRI_R].Motor, 2);
    M2006_Init(&Shooting.Feeding.Motor, 3);

    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    Shooting_BoxClose();

    // 摩擦轮采用分段参数 PID参数在计算函数中根据误差大小赋值
    PID_InitParam.kP = 9.8;
    PID_InitParam.kI = 5;
    PID_InitParam.kD = 0;
    PID_InitParam.MaxError = FLOAT_MAX_VAL;
    PID_InitParam.MaxOutput = 10000;
    PID_InitParam.I_Max = 200;
    PID_InitParam.DeadBand = 0;
    PID_InitParam.IntegralBand = 800;
    PID_Init(&Shooting.Friction[SHOOTING_FRI_L].SpeedPID, &PID_InitParam);
    PID_Init(&Shooting.Friction[SHOOTING_FRI_R].SpeedPID, &PID_InitParam);
    LPF_1stOrderIIR_Init(&Shooting.Friction[SHOOTING_FRI_L].SpeedFilter, SHOOTING_TASK_UPDATE_TICK / 1000.f, 10);
    LPF_1stOrderIIR_Init(&Shooting.Friction[SHOOTING_FRI_R].SpeedFilter, SHOOTING_TASK_UPDATE_TICK / 1000.f, 10);

    PID_InitParam.kP = 120;
    PID_InitParam.kI = 0;
    PID_InitParam.kD = 0;
    PID_InitParam.MaxError = FLOAT_MAX_VAL;
    PID_InitParam.MaxOutput = 6000;
    PID_InitParam.I_Max = 0;
    PID_InitParam.DeadBand = 0;
    PID_InitParam.IntegralBand = FLOAT_MAX_VAL;
    PID_Init(&Shooting.Feeding.PositionPID, &PID_InitParam);

    PID_InitParam.kP = 5;
    PID_InitParam.kI = 2;
    PID_InitParam.kD = 0;
    PID_InitParam.MaxError = FLOAT_MAX_VAL;
    PID_InitParam.MaxOutput = 10000;
    PID_InitParam.I_Max = 4000;
    PID_InitParam.DeadBand = 0;
    PID_InitParam.IntegralBand = FLOAT_MAX_VAL;
    PID_Init(&Shooting.Feeding.SpeedPID, &PID_InitParam);

    Shooting.Feeding.LastFeedingTick = 0;
    Shooting.Feeding.LastStuckTick = 0;

    Referee_Set();

    int PrevEnableState = RESET;
    while (1)
    {
        osDelayUntil(&PreviousWakeTime, SHOOTING_TASK_UPDATE_TICK);

        if (Controller.isEnable == RESET)
        {
            Shooting_Disable();
            PrevEnableState = RESET;
            continue;
        }

        if (PrevEnableState == RESET)
        {
            PrevEnableState = SET;
            Shooting_BoxClose();
        }
        uint32_t NowTick = HAL_GetTick();
        if ((!Controller.AimingOn || (Vision_ShootingControl()||Controller.isBossMode)) && // 如果开启了自瞄 则检查瞄准条件 这一项必须在第一条 因为内部记录了ShootingOn的状态
            ShooterBarrelHeat_Calc() &&
            Controller.ShootingOn &&
            Controller.AimingInvRotateEnable &&
            NowTick - Shooting.Feeding.LastFeedingTick > FEEDING_PERIOD_TICK && // 定频射击
            NowTick - Shooting.Feeding.LastStuckTick > 500 &&                   // 卡弹制动
            ((Heatdiff > 40) || Controller.isBossMode))
        {
            Shooting.Feeding.Motor.Position.UnwrapedAfterGear.Set += -(360.f / FEEDING_AMMO_PER_TURN); // 正转
            Shoot_flag = 1;
            Shooting.Feeding.Count++;
            Shooting.Feeding.LastFeedingTick = NowTick;
        }
        Shoot_flag = 0;
        if (Controller.FrictionOn == SET)
        {
            if (PrevShootingSpeed != Referee.ShootData.InitialSpeed)
            {
                if (Referee.ShootData.InitialSpeed > 23.0f)
                    ShootingRpmInit = ConstrainF(ShootingRpmInit - 100, 6050, 5000);
                if (Referee.ShootData.InitialSpeed < 20.5f)
                    ShootingRpmInit = ConstrainF(ShootingRpmInit + 50, 6050, 5000);
                PrevShootingSpeed = Referee.ShootData.InitialSpeed;
            }
            // ShootingRpmInit = 6000;
            Shooting.Friction[SHOOTING_FRI_L].Motor.Speed.Set = -ShootingRpmInit;
            Shooting.Friction[SHOOTING_FRI_R].Motor.Speed.Set = ShootingRpmInit;
        }
        else
        {
            Shooting.Friction[SHOOTING_FRI_L].Motor.Speed.Set = 0;
            Shooting.Friction[SHOOTING_FRI_R].Motor.Speed.Set = 0;
        }
        Shooting_FrictionSpeedPID_Calc(SHOOTING_FRI_L);
        Shooting_FrictionSpeedPID_Calc(SHOOTING_FRI_R);

        Shooting_CheckStuck();

        Motor_UpdateUnwrap(&Shooting.Feeding.Motor); // 更新拨弹盘的累计角度->解卷绕 注意采样定理
        Motor_UnwrapedAfterGearPositionPID_Calc(&Shooting.Feeding.Motor, &Shooting.Feeding.PositionPID, &Shooting.Feeding.SpeedPID);

        Shooting_MotorOutput();
    }
}

void Shooting_FrictionSpeedPID_Calc(int frictionNum)
{
    LPF_1stOrderIIR_Update(&Shooting.Friction[frictionNum].SpeedFilter, Shooting.Friction[frictionNum].Motor.Speed.Real);
    if (Shooting.Friction[frictionNum].Motor.Speed.Set == 0 && fabsf(Shooting.Friction[frictionNum].SpeedPID.Error[0]) < 100)
        Shooting.Friction[frictionNum].SpeedPID.I = 0;
    PID_Calc(&Shooting.Friction[frictionNum].SpeedPID, Shooting.Friction[frictionNum].SpeedFilter.Output, Shooting.Friction[frictionNum].Motor.Speed.Set);
    Shooting.Friction[frictionNum].Motor.Current.Set = Shooting.Friction[frictionNum].SpeedPID.Output;

    // if (Shooting.Friction[frictionNum].Motor.Speed.Set == 0 && fabsf(Shooting.Friction[frictionNum].SpeedPID.Error[0]) < 100)
    //     Shooting.Friction[frictionNum].SpeedPID.I = 0;
    // PID_Calc(&Shooting.Friction[frictionNum].SpeedPID, Shooting.Friction[frictionNum].Motor.Speed.Real, Shooting.Friction[frictionNum].Motor.Speed.Set);
    // Shooting.Friction[frictionNum].Motor.Current.Set = Shooting.Friction[frictionNum].SpeedPID.Output;
}

// 判断是否卡弹
void Shooting_CheckStuck(void)
{
    float Error = Shooting.Feeding.Motor.Position.UnwrapedAfterGear.Set - Shooting.Feeding.Motor.Position.UnwrapedAfterGear.Real; // 计算误差

    if (-Error > 2.5f * 360.f / FEEDING_AMMO_PER_TURN)
    {
        Shooting.Feeding.Motor.Position.UnwrapedAfterGear.Set -= -(4 * 360.f / FEEDING_AMMO_PER_TURN); // 反转
        Shooting.Feeding.Count -= 4;
        Shooting.Feeding.LastStuckTick = HAL_GetTick(); // 记录时间戳
    }
}

void Shooting_MotorOutput(void)
{
    DjiMotor_CanID_0x200_Output();
}

void Shooting_Disable(void)
{
    Shooting.Feeding.Motor.Current.Set = 0;
    Shooting.Friction[SHOOTING_FRI_L].Motor.Current.Set = 0;
    Shooting.Friction[SHOOTING_FRI_R].Motor.Current.Set = 0;
    Shooting_MotorOutput();

    htim8.Instance->CCR3 = 0;
}

void Shooting_BoxOpen(void)
{
    htim8.Instance->CCR3 = 130;
}

void Shooting_BoxClose(void)
{
    htim8.Instance->CCR3 = 230;
}
