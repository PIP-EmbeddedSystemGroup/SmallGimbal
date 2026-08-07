/*
 * smc.h
 * 自适应滑模控制（含 RLS 估计）头文件
 */
#ifndef SMC_H
#define SMC_H

#include <stdint.h>

/* ---------- SMC 参数（用户可配置） ----------
 * 说明（单位）：
 *  - lambda:    1/s        （滑模面系数，lambda*e 的单位为 rad/s）
 *  - eta:       rad/s^2    （当 u_rob 被视作角加速度时的增益；若将 eta 直接作为力矩增益，
 *                           则 ComputeOutput 中乘法需调整）
 *  - phi:       rad/s      （边界层厚度，s 的量纲为 rad/s）
 *  - mass_min/max: J 的界限（kg·m^2 or 等效单位）
 *  - mass_adapt_rate: 向后兼容的简单自适应率（RLS 启用时通常不用）
 *  - dt:        s         （控制/采样周期）
 *  - rls_forgetting: 遗忘因子 λ_f，靠近 1.0（例如 0.995~0.999）
 *  - rls_P0:    初始协方差对角（大的值表示初始不确定度高）
 *  - theta_min/max: 对估计参数 [J, b, G] 的投影边界，按索引顺序填写
 */
typedef struct {
    float lambda;            /* 1/s */
    float eta;               /* rad/s^2 */
    float phi;               /* rad/s */

    float mass_min;          /* J 下限 */
    float mass_max;          /* J 上限 */

    float mass_adapt_rate;   /* 兼容旧自适应律 */

    float dt;                /* 采样周期（s） */

    /* RLS（递归最小二乘）参数 */
    float rls_forgetting;    /* 遗忘因子（λ_f） */
    float rls_P0;            /* P 初始对角 */

    float theta_min[3];      /* [J_min, b_min, G_min] */
    float theta_max[3];      /* [J_max, b_max, G_max] */
} SMC_Params_t;

/* SMC 状态（运行时）
 * 说明：所有角度相关量均以弧度（rad）为单位，力矩以 N·m 为单位
 */
typedef struct {
    /* 传感器测得状态 */
    float position;          /* rad */
    float velocity;          /* rad/s */
    float prev_velocity;     /* rad/s，用于估算角加速度 */

    /* 目标信号（由上层控制器填写） */
    float target_pos;        /* rad */
    float target_vel;        /* rad/s */
    float prev_target_vel;   /* rad/s，用于估算目标加速度 */

    /* 估计与输出 */
    float mass_estimate;     /* J 估计（由 RLS 提供或保底） */
    float control_output;    /* 输出：力矩 N·m */
    float sliding_surface;   /* s，单位 rad/s */

    /* 外部提供的测量（若有，用于 RLS） */
    float measured_torque;   /* N·m */
    uint8_t torque_valid;    /* 1 表示 measured_torque 有效 */
    /* 可由外部测得的角加速度（rad/s^2），优先使用此值替代内部差分估计 */
    float measured_accel;    /* rad/s^2 */
    uint8_t accel_valid;     /* 1 表示 measured_accel 有效 */
} SMC_State_t;

/* RLS 估计状态（内部） */
typedef struct {
    float theta[3];      /* [J, b, G] */
    float P[3][3];       /* 协方差矩阵 */
    uint8_t initialized;
} SMC_RLS_t;

/* 主控制器对象 */
typedef struct {
    uint8_t axis;
    SMC_Params_t params;
    SMC_State_t state;
    SMC_RLS_t rls;
} SMC_Controller_t;

/* 接口 */
void SMC_Init(SMC_Controller_t *smc, uint8_t axis, const SMC_Params_t *params);
void SMC_UpdateState(SMC_Controller_t *smc, float pos, float vel);

/*
 * 计算控制输出（向后兼容）
 * - 如果之前调用过 SMC_SetMeasuredTorque 并标记有效，则 Compute 会先用 RLS 更新参数。
 * - 返回值为控制量（电流或力矩，取决于调用者如何解释）。
 */
float SMC_ComputeOutput(SMC_Controller_t *smc, float target_pos, float target_vel);

/* 更新参数（完整替换） */
void SMC_UpdateParams(SMC_Controller_t *smc, const SMC_Params_t *params);

/* 外部在读取到电机电流/扭矩时调用：喂入测量的扭矩（单位需与模型一致） */
void SMC_SetMeasuredTorque(SMC_Controller_t *smc, float torque);

/* 外部在读取到角加速度（rad/s^2）时可调用：用于替代内部差分估计（更稳健） */
void SMC_SetMeasuredAccel(SMC_Controller_t *smc, float ang_accel);

/* 直接获取 RLS 估计的参数（J,b,G） */
void SMC_GetEstimatedParams(const SMC_Controller_t *smc, float out_theta[3]);

/* 重置/初始化 RLS（可用于在大跳变后重启估计） */
void SMC_ResetRLS(SMC_Controller_t *smc, const float theta0[3]);

/* 简单工具：获取当前质量估计（J） */
float SMC_GetMassEstimate(const SMC_Controller_t *smc);

extern float Jm, u_eq_acc,u_rob_acc,b_es,B,G;


#endif /* SMC_H */
