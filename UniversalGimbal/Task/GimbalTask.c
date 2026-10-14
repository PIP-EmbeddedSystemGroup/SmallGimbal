/**
 * @attention   采用UTF-8字符集编码
 * @brief       云台任务 改进俯仰限幅
 * @authors     北工大PIP战队 樊郡捷
 * @version     v1.1
 * @date        2025-02-18
 * @details     云台控制器的更新任务
 */

#include "header.h"

#define GIMBAL_TASK_UPDATE_TICK 1

Gimbal_t Gimbal;
PID_t Gimbal_Yaw_Speed_Only;
PID_t Gimbal_Pitch_Speed_Only;

void Gimbal_YawPID_Calc(void);
void Gimbal_PitchPID_CalcWithConstrain(void);
void Gimbal_Disable(void);
float Gimbal_SpeedModeCalConstrain(float currentAngle, float targetSpeed,float topAngle, float bottomAngle, float circleResolution,float nearThreshold,float maxSpeed,float speedGain);

void GimbalTask(void const *argument)
{
    uint32_t PreviousWakeTime = osKernelSysTick();

    PID_InitStruct_t PID_InitParam = {0};
    PID_InitParam.DeltaTime = GIMBAL_TASK_UPDATE_TICK / 1000.f;

    GM6020_Init(&Gimbal.Yaw.Motor, 1);
    GM6020_Init(&Gimbal.Pitch.Motor, 3);

    PID_InitParam.kP = 0.5f;
    PID_InitParam.kI = 0;
    PID_InitParam.kD = 0;
    PID_InitParam.CircleResolution = 360;
    PID_InitParam.MaxError = 180;
    PID_InitParam.MaxOutput = 220;
    PID_InitParam.I_Max = 180;
    PID_InitParam.DeadBand = 0;
    PID_InitParam.IntegralBand = FLOAT_MAX_VAL;
    PID_Init(&Gimbal.Yaw.PositionPID, &PID_InitParam);

    PID_InitParam.kP = 3800;
    PID_InitParam.kI = 1000;
    PID_InitParam.kD = 0;
    PID_InitParam.MaxError = 36;
    PID_InitParam.MaxOutput = 15000;
    PID_InitParam.I_Max = 4000;
    PID_InitParam.DeadBand = 0;
    PID_InitParam.IntegralBand = FLOAT_MAX_VAL;
    PID_Init(&Gimbal.Yaw.SpeedPID, &PID_InitParam);

    PID_InitParam.DifferentialFreqDiv = 5;
    PID_InitParam.kP = 0.5f;
    PID_InitParam.kI = 0;
    PID_InitParam.kD = 0.f;
    PID_InitParam.CircleResolution = 360;
    PID_InitParam.MaxError = 90;
    PID_InitParam.MaxOutput = 72;
    PID_InitParam.I_Max = 0;
    PID_InitParam.DeadBand = 0;
    PID_InitParam.IntegralBand = 0;
    PID_Init(&Gimbal.Pitch.PositionPID, &PID_InitParam);

    PID_InitParam.kP = 1400;
    PID_InitParam.kI = 1000;					//pitch不够硬可以优先考虑加他
    PID_InitParam.kD = 0;
    PID_InitParam.MaxError = 72;
    PID_InitParam.MaxOutput = 15000;
    PID_InitParam.I_Max = 4000;
    PID_InitParam.DeadBand = 0;
    PID_InitParam.IntegralBand = 10;
    PID_Init(&Gimbal.Pitch.SpeedPID, &PID_InitParam);
		
		PID_InitParam.kP = 25;
    PID_InitParam.kI = 500;
    PID_InitParam.kD = 0;
    PID_InitParam.MaxError = 2000;
    PID_InitParam.MaxOutput = 15000;
    PID_InitParam.I_Max = 2000;
    PID_InitParam.DeadBand = 0;
    PID_InitParam.IntegralBand = FLOAT_MAX_VAL;
    PID_Init(&Gimbal_Pitch_Speed_Only, &PID_InitParam);
		
		PID_InitParam.kP = 40;
    PID_InitParam.kI = 400;
    PID_InitParam.kD = 0;
    PID_InitParam.MaxError = 2000;
    PID_InitParam.MaxOutput = 15000;
    PID_InitParam.I_Max = 2000;
    PID_InitParam.DeadBand = 0;
    PID_InitParam.IntegralBand = FLOAT_MAX_VAL;
    PID_Init(&Gimbal_Yaw_Speed_Only, &PID_InitParam);

    osDelay(1000);

    while (1)
    {
        osDelayUntil(&PreviousWakeTime, GIMBAL_TASK_UPDATE_TICK);

        if (!Controller.isEnable)
        {
            Gimbal_Disable();
            continue;
        }
				Gimbal_YawPID_Calc();
        Gimbal_PitchPID_CalcWithConstrain();
        DjiMotor_CanID_0x1FF_Output();
    }
}

/// @brief 计算Yaw轴PID
/// @param
void Gimbal_YawPID_Calc(void)
{
		if(NUC.YawPosValid && !NUC.YawVelValid && !NUC.YawAccValid)
		{
				PID_AngleCalc(&Gimbal.Yaw.PositionPID, Inertial.Yaw, NUC.YawAngle);
				PID_Calc(&Gimbal.Yaw.SpeedPID, Inertial.Gyro[AZ], Gimbal.Yaw.PositionPID.Output);
			  Gimbal.Yaw.Motor.Current.Set = Gimbal.Yaw.SpeedPID.Output;
		}
		else if(NUC.YawVelValid && !NUC.YawPosValid && !NUC.YawAccValid)
		{
			PID_Calc(&Gimbal_Yaw_Speed_Only, Inertial.Gyro[AZ] * 57.295779513f, NUC.YawVelocity);
			Gimbal.Yaw.Motor.Current.Set = Gimbal_Yaw_Speed_Only.Output;
		}

}

/// @brief 计算Pitch轴PID 根据Pitch轴电机编码器 计算超出限幅的角度大小 并检测云台目标角度是否会超出限幅
/// @param
void Gimbal_PitchPID_CalcWithConstrain(void)
{
    const int PitchTopEnc = PITCH_MAX_ENCODER;
    const int PitchBottomEnc = PITCH_MIN_ENCODER; // 云台上抬对应编码器数值增大
		const float SPEED_LIMIT_NEAR = 100.f;      // 接近限位时的编码器阈值
		const float SPEED_LIMIT_MAX = 100;    // 最大允许速度
		const float SPEED_LIMIT_GAIN = 100.f;     // 速度限制增益（速度/角度差）

		if(NUC.PitchPosValid && !NUC.PitchVelValid && !NUC.PitchAccValid)
		{
			 Controller.View.Pitch = NUC.PitchAngle;
		  // 下列定义永远对应上抬增大的结果
			// 与目标的差
			float ErrAngle = AngleDiffF(Controller.View.Pitch, Inertial.Pitch, Gimbal.Pitch.PositionPID.CircleResolution);
			// 与限幅的差
			int TopDeltaEnc, BottomDeltaEnc;
			if (AngleDiffF(PitchTopEnc, PitchBottomEnc, Gimbal.Pitch.Motor.EncoderResolution) > 0) // 云台上抬对应编码器数值增大
			{
					TopDeltaEnc = AngleDiffF(PitchTopEnc, Gimbal.Pitch.Motor.Position.Real, Gimbal.Pitch.Motor.EncoderResolution);
					BottomDeltaEnc = AngleDiffF(PitchBottomEnc, Gimbal.Pitch.Motor.Position.Real, Gimbal.Pitch.Motor.EncoderResolution);
			}
			else // 云台上抬对应编码器数值减小
			{
					TopDeltaEnc = -AngleDiffF(PitchTopEnc, Gimbal.Pitch.Motor.Position.Real, Gimbal.Pitch.Motor.EncoderResolution);
					BottomDeltaEnc = -AngleDiffF(PitchBottomEnc, Gimbal.Pitch.Motor.Position.Real, Gimbal.Pitch.Motor.EncoderResolution);
			}
			float TopDeltaAngle = TopDeltaEnc * 360.f / Gimbal.Pitch.Motor.EncoderResolution;
			float BottomDeltaAngle = BottomDeltaEnc * 360.f / Gimbal.Pitch.Motor.EncoderResolution;
			if (ErrAngle > TopDeltaAngle)
					Controller.View.Pitch = Controller.View.Pitch - ErrAngle + TopDeltaAngle;
			else if (ErrAngle < BottomDeltaAngle)
					Controller.View.Pitch = Controller.View.Pitch - ErrAngle + BottomDeltaAngle;

			PID_AngleCalc(&Gimbal.Pitch.PositionPID, Inertial.Pitch, Controller.View.Pitch);
			PID_Calc(&Gimbal.Pitch.SpeedPID, Inertial.Gyro[AX], Gimbal.Pitch.PositionPID.Output);
			Gimbal.Pitch.Motor.Current.Set = -Gimbal.Pitch.SpeedPID.Output;
		}
		else if(NUC.PitchVelValid && !NUC.PitchPosValid && !NUC.PitchAccValid)
		{
			float Gimbal_Pitch_Speed_Set = Gimbal_SpeedModeCalConstrain(Gimbal.Pitch.Motor.Position.Real,
																												NUC.PitchVelocity,
																												PitchTopEnc,
																												PitchBottomEnc,
																												Gimbal.Pitch.Motor.EncoderResolution,
																												SPEED_LIMIT_NEAR,
																												SPEED_LIMIT_MAX,
																												SPEED_LIMIT_GAIN);

			PID_Calc(&Gimbal_Pitch_Speed_Only, Inertial.Gyro[AX] * 57.295779513f, Gimbal_Pitch_Speed_Set);
			Gimbal.Pitch.Motor.Current.Set = -Gimbal_Pitch_Speed_Only.Output;
		}
   
}

void Gimbal_Disable(void)
{

    Gimbal.Pitch.Motor.Current.Set = 0;
    Gimbal.Yaw.Motor.Current.Set = 0;

    DjiMotor_CanID_0x1FF_Output();

    PID_Clear(&Gimbal.Pitch.PositionPID);
    PID_Clear(&Gimbal.Pitch.SpeedPID);
    PID_Clear(&Gimbal.Yaw.PositionPID);
    PID_Clear(&Gimbal.Yaw.SpeedPID);
}

/**
 * @brief 速度控制模式下的限速计算
 * @param currentAngle 当前编码值
 * @param targetSpeed 目标速度
 * @param topAngle 上限编码值
 * @param bottomAngle 下限编码值
 * @param circleResolution 一圈对应的编码值
 * @param nearThreshold 接近限位的编码值
 * @param maxSpeed 最大允许速度
 * @param speedGain 速度限制增益（速度/角度差）
 * @return 限制后的速度
 */
float Gimbal_SpeedModeCalConstrain(
    float currentAngle, 
    float targetSpeed,
    float topAngle, 
    float bottomAngle, 
    float circleResolution,
    float nearThreshold,
    float maxSpeed,
    float speedGain)
{
    float speedMax=maxSpeed;
    float speedMin=-maxSpeed;
    if(fabs(AngleDiffF(currentAngle, topAngle,circleResolution))<nearThreshold)
    {
        speedMax/=speedGain;
				if(fabs(AngleDiffF(currentAngle, topAngle,circleResolution))<50)
				{
					speedMax=0;
				}		
    }
    if(fabs(AngleDiffF(currentAngle, bottomAngle,circleResolution))<nearThreshold)
    {
        speedMin/=speedGain;
				if(fabs(AngleDiffF(currentAngle, bottomAngle,circleResolution))<50)
				{
					speedMin=0;
				}
    }				
    return ConstrainF(targetSpeed,speedMax,speedMin);
}
