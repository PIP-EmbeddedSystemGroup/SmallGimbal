/**
 * @attention   采用UTF-8字符集编码
 * @brief       自适应滑模控制算法实现
 * @authors     北工大PIP战队
 * @version     v1.0
 * @date        2025-10-28
 */

#include "smc.h"
#include "math.h"
#include <string.h>
#include "header.h"

/* 私有宏定义 */
#define SIGN(x) ((x) > 0 ? 1 : ((x) < 0 ? -1 : 0))
#define SAT(x, phi) ((x) > (phi) ? 1 : ((x) < -(phi) ? -1 : (x) / (phi)))

/* 数值保护 */
#define RLS_DENOM_EPS (1e-6f)
#define RLS_P_MAX (1e12f)
#define RLS_P_MIN (1e-9f)
/* 小的门限：当目标速度较小时使用 static 近似（位置控制场景） */
#define TARGET_VEL_EPS (1e-6f)

float Jm, u_eq_acc,u_rob_acc,b_es,B,G;


/* 辅助函数原型（文件内） */
static float clampf_fast(float v, float lo, float hi);
static void rls_update(SMC_Controller_t *smc, const float phi_vec[3], float y);
static void symmetrize_and_limit_P(float P[3][3]);

void SMC_Init(SMC_Controller_t *smc, uint8_t axis, const SMC_Params_t *params)
{
    smc->axis = axis;
    smc->params = *params;

    /* 初始化状态 */
    smc->state.position = 0;
    smc->state.velocity = 0;
    smc->state.target_pos = 0;
    smc->state.target_vel = 0;
    smc->state.mass_estimate = (params->mass_max + params->mass_min) / 2;
    smc->state.control_output = 0;
    smc->state.sliding_surface = 0;
    smc->state.prev_velocity = 0;
    smc->state.prev_target_vel = 0;
    smc->state.measured_torque = 0;
    smc->state.torque_valid = 0;
    smc->state.measured_accel = 0.0f;
    smc->state.accel_valid = 0;

    /* 初始化 RLS：初始 theta 取中间值（theta_min/theta_max）或小值 */
    float theta0[3];
    for (int i = 0; i < 3; ++i)
    {
        /* 若用户未设边界（theta_min >= theta_max），则使用 0/小正数初值 */
        if (smc->params.theta_min[i] < smc->params.theta_max[i])
            theta0[i] = 0.5f * (smc->params.theta_min[i] + smc->params.theta_max[i]);
        else
            theta0[i] = 0.0f;
    }
    SMC_ResetRLS(smc, theta0);
}

void SMC_UpdateState(SMC_Controller_t *smc, float pos, float vel)
{
    smc->state.position = pos;
    smc->state.velocity = vel;
}

float SMC_ComputeOutput(SMC_Controller_t *smc, float target_pos, float target_vel)
{
    /* 更新目标值 */
    smc->state.target_pos = target_pos;
    smc->state.target_vel = target_vel;

    /* 计算跟踪误差及其导数 */
    float pos_error = smc->state.target_pos - smc->state.position;
    float vel_error = smc->state.target_vel - smc->state.velocity;

    /* 计算滑模面 s = e_dot + lambda * e */
    smc->state.sliding_surface = vel_error + smc->params.lambda * pos_error;

    /* 等效项计算（角加速度级 u_eq_acc）
     * 若 target_vel 很小（位置控制场景），用近似 u_eq_acc = -lambda * vel
     * 否则用差分估计目标加速度： accel_des = (target_vel - prev_target_vel)/dt
     * u_eq_acc = accel_des + lambda*(target_vel - vel)
     */

    /* 位置控制：目标速度近似为 0 */
    u_eq_acc = -smc->params.lambda * smc->state.velocity;

    smc->state.prev_target_vel = smc->state.target_vel;

    /* 鲁棒项（角加速度量） */
    u_rob_acc = smc->params.eta * SAT(smc->state.sliding_surface, smc->params.phi);

    /* 估算角加速度（用于 RLS）——优先使用外部传入的测量角加速度，否则用速度差分估计 */
    float accel_meas;
    if (smc->state.accel_valid)
    {
        accel_meas = smc->state.measured_accel;
        /* 如果是一次性标志，上层可选择不清除以保持持续有效；这里清除以避免重复使用过时数据 */
        smc->state.accel_valid = 0;
    }
    else
    {
        accel_meas = (smc->state.velocity - smc->state.prev_velocity) / smc->params.dt;
    }
    smc->state.prev_velocity = smc->state.velocity;

    /* 如果上层喂入测得力矩，则用 RLS 更新参数 */
    if (smc->state.torque_valid)
    {
        /* 为了在 RLS 中忽略摩擦项 (b)，我们把速度项置 0，仅估计 J 和 G
         * phi = [accel, 0, sin(pos)] -> theta = [J, b_unused, G]
         * 注意：theta[1] 保持为 0，rls_update 将不会改变其值。
         */
        float phi_vec[3] = {accel_meas, 0.0f, sinf(smc->state.position)};
        rls_update(smc, phi_vec, smc->state.measured_torque);
        /* 清除标志，等待下次测量 */
        smc->state.torque_valid = 0;
    }

    /* 读取参数估计（若尚未估计则 theta 为 0 或初始值） */
    Jm = smc->state.mass_estimate;
    if (!(Jm > 0))
        Jm = smc->params.mass_min;
    /* 不使用摩擦估计，b=0 */
    float G_est = smc->rls.theta[2];
    G =G_est * sinf(smc->state.position);

    /* 最终力矩输出： tau = J*(u_eq_acc + u_rob_acc) + G*sin(pos) */
    float tau = Jm * (u_eq_acc + u_rob_acc) + G_est * sinf(smc->state.position);
    smc->state.control_output = tau;
    return smc->state.control_output;
}

void SMC_UpdateParams(SMC_Controller_t *smc, const SMC_Params_t *params)
{
    smc->params = *params;
}

float SMC_GetMassEstimate(const SMC_Controller_t *smc)
{
    return smc->state.mass_estimate;
}

/* ----------------- RLS / 辅助函数实现 ----------------- */
void SMC_SetMeasuredTorque(SMC_Controller_t *smc, float torque)
{
    smc->state.measured_torque = torque;
    smc->state.torque_valid = 1;
}

void SMC_SetMeasuredAccel(SMC_Controller_t *smc, float ang_accel)
{
    smc->state.measured_accel = ang_accel;
    smc->state.accel_valid = 1;
}

void SMC_GetEstimatedParams(const SMC_Controller_t *smc, float out_theta[3])
{
    for (int i = 0; i < 3; ++i)
        out_theta[i] = smc->rls.theta[i];
}

void SMC_ResetRLS(SMC_Controller_t *smc, const float theta0[3])
{
    /* 初始化 theta 和 P */
    for (int i = 0; i < 3; ++i)
    {
        smc->rls.theta[i] = theta0 ? theta0[i] : 0.0f;
        for (int j = 0; j < 3; ++j)
            smc->rls.P[i][j] = 0.0f;
        smc->rls.P[i][i] = smc->params.rls_P0 > 0 ? smc->params.rls_P0 : 1e3f;
    }
    /* 明确禁止估计摩擦项 theta[1]，将其固定为 0 并把对应 P 行/列清零以减少耦合 */
    smc->rls.theta[1] = 0.0f;
    for (int k = 0; k < 3; ++k)
    {
        smc->rls.P[1][k] = 0.0f;
        smc->rls.P[k][1] = 0.0f;
    }
    /* 给 theta[1] 的对角一个小正数以避免数值奇异 */
    smc->rls.P[1][1] = RLS_P_MIN;
    smc->rls.initialized = 1;
}

/* ---------- 辅助实现：简单工具与 RLS 更新 ---------- */
static float clampf_fast(float v, float lo, float hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

static void symmetrize_and_limit_P(float P[3][3])
{
    for (int i = 0; i < 3; ++i)
    {
        for (int j = i + 1; j < 3; ++j)
        {
            float s = 0.5f * (P[i][j] + P[j][i]);
            P[i][j] = P[j][i] = s;
        }
    }
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            if (P[i][j] > RLS_P_MAX)
                P[i][j] = RLS_P_MAX;
            if (P[i][j] < -RLS_P_MAX)
                P[i][j] = -RLS_P_MAX;
            if (i == j && fabsf(P[i][j]) < RLS_P_MIN)
                P[i][j] = (P[i][j] >= 0) ? RLS_P_MIN : -RLS_P_MIN;
        }
    }
}

static void rls_update(SMC_Controller_t *smc, const float phi_vec[3], float y)
{
    /* P * phi */
    float Pphi[3] = {0};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            Pphi[i] += smc->rls.P[i][j] * phi_vec[j];

    /* denom = lambda_f + phi^T * P * phi */
    float phiPphi = 0;
    for (int i = 0; i < 3; ++i)
        phiPphi += phi_vec[i] * Pphi[i];
    float denom = smc->params.rls_forgetting + phiPphi;
    if (denom < RLS_DENOM_EPS)
        denom = RLS_DENOM_EPS;

    /* K = Pphi / denom */
    float K[3];
    for (int i = 0; i < 3; ++i)
        K[i] = Pphi[i] / denom;

    /* e = y - phi^T theta */
    float phi_theta = 0;
    for (int i = 0; i < 3; ++i)
        phi_theta += phi_vec[i] * smc->rls.theta[i];
    float e = y - phi_theta;

    /* theta = theta + K * e */
    for (int i = 0; i < 3; ++i)
        smc->rls.theta[i] += K[i] * e;
    /* 强制固定摩擦项 theta[1] = 0（不估计摩擦）以避免对摩擦的在线调整 */
    smc->rls.theta[1] = 0.0f;

    /* P = (P - K * phi^T * P) / lambda_f */
    float newP[3][3];
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            /* phi^T * P column j = sum_k phi[k] * P[k][j] */
            float phT_P_j = phi_vec[0] * smc->rls.P[0][j] + phi_vec[1] * smc->rls.P[1][j] + phi_vec[2] * smc->rls.P[2][j];
            newP[i][j] = smc->rls.P[i][j] - K[i] * phT_P_j;
        }
    }
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            smc->rls.P[i][j] = newP[i][j] / smc->params.rls_forgetting;

    /* 对称化并限制 */
    symmetrize_and_limit_P(smc->rls.P);

    /* 为防止 P 在更新中对第1列/行产生耦合，显式清零第1行/列并给对角一个小值 */
    for (int k = 0; k < 3; ++k)
    {
        smc->rls.P[1][k] = 0.0f;
        smc->rls.P[k][1] = 0.0f;
    }
    smc->rls.P[1][1] = RLS_P_MIN;

    /* 参数投影（theta bounds） */
    for (int i = 0; i < 3; ++i)
    {
        if (smc->params.theta_min[i] < smc->params.theta_max[i])
        {
            smc->rls.theta[i] = clampf_fast(smc->rls.theta[i], smc->params.theta_min[i], smc->params.theta_max[i]);
        }
    }

    /* 更新质量估计字段并裁剪 */
    float J_est = smc->rls.theta[0];
    J_est = clampf_fast(J_est, smc->params.mass_min, smc->params.mass_max);
    smc->state.mass_estimate = J_est;
}
