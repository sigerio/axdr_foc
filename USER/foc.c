#include "foc.h"
#include "app_init.h"
#include "gloab.h"
#include "tim.h"
#include <math.h>

#include "moto_config.h"

foc_t foc;
pll_info_t pll_info;
volatile speed_td_t speed_td;
const vesc_observer_config_t vesc_observer_config = {
    .foc_motor_r = MOTOR_R,
    .foc_motor_ls = MOTOR_L,
    .foc_motor_flux_linkage = MOTOR_FLUX_LINKAGE,
};
vesc_observer_input_t vesc_observer_input;
vesc_observer_state_t vesc_observer_state;

void foc_pwm_start(void)
{
    /* CH4 已在运行，先将准备好的 CCR 预装载值同步生效，再开启桥臂。 */
	HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_UPDATE);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}


void foc_pwm_stop(void)
{
    /* 先关六路桥臂，再处理 CCR；保留 CH4 和 MOE，采样触发继续运行。 */
    CLEAR_BIT(htim1.Instance->CCER, TIM_CCER_CC1E | TIM_CCER_CC1NE |
                                  TIM_CCER_CC2E | TIM_CCER_CC2NE |
                                  TIM_CCER_CC3E | TIM_CCER_CC3NE);
	set_dtc_a(PWM_ARR());
	set_dtc_b(PWM_ARR());
	set_dtc_c(PWM_ARR());

    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}







void foc_pwm_run(void)
{
	set_dtc_a((uint16_t)(foc.dtc_a * PWM_ARR()));
	set_dtc_b((uint16_t)(foc.dtc_b * PWM_ARR()));
	set_dtc_c((uint16_t)(foc.dtc_c * PWM_ARR()));
}

void sin_cos_val(foc_t *foc)
{
    foc->sin_val = sinf(foc->theta); 
    foc->cos_val = cosf(foc->theta); 
}

void clarke_transform(foc_t *foc)
{
    foc->i_alpha = foc->i_a;
    foc->i_beta = (foc->i_b - foc->i_c) * ONE_BY_SQRT3;
}

void park_transform(foc_t *foc)
{
    foc->i_d = foc->i_alpha*foc->cos_val+foc->i_beta*foc->sin_val;
    foc->i_q = -foc->i_alpha*foc->sin_val+foc->i_beta*foc->cos_val;
}


void inverse_clarke(foc_t *foc)
{
    foc->v_a = foc->v_alpha;
    foc->v_b = foc->v_beta*SQRT3_BY_2-foc->v_alpha*0.5f;
    foc->v_c = -foc->v_beta*SQRT3_BY_2-foc->v_alpha*0.5f;
}


void inverse_park(foc_t *foc)
{
    foc->v_alpha = foc->v_d*foc->cos_val-foc->v_q*foc->sin_val;
    foc->v_beta = foc->v_d*foc->sin_val+foc->v_q*foc->cos_val;
}

static int svpwm_prepare(foc_t *foc, float *alpha, float *beta, float *bus)
{
    float scale;
    foc->dtc_a = foc->dtc_b = foc->dtc_c = 0.5f;
    foc->inv_vbus = 0.0f;
    if (!isfinite(foc->vbus) || foc->vbus <= 0.0f ||
        !isfinite(foc->v_alpha) || !isfinite(foc->v_beta))
        return 0;

    foc->inv_vbus = 1.0f / foc->vbus;
    if (!isfinite(foc->inv_vbus))
    {
        foc->inv_vbus = 0.0f;
        return 0;
    }
    scale = fmaxf(foc->vbus, fmaxf(fabsf(foc->v_alpha), fabsf(foc->v_beta)));
    *alpha = foc->v_alpha / scale;
    *beta = foc->v_beta / scale;
    *bus = foc->vbus / scale;
    return 1;
}

/* Only guards floating-point roundoff; vector limiting happens before this. */
static float duty_clamp(float duty)
{
    return fminf(1.0f, fmaxf(0.0f, duty));
}


void sector_judge(foc_t *foc)
{
    float alpha, beta, bus, x, y, z, t1, t2, t0, h, denominator;
    unsigned int code;
    if (!svpwm_prepare(foc, &alpha, &beta, &bus))
        return;

    /* x/y/z signs encode the sector; the code is NOT its geometric number. */
    x = beta;
    y = SQRT3_BY_2 * alpha - 0.5f * beta;
    z = -SQRT3_BY_2 * alpha - 0.5f * beta;
    code = (x >= 0.0f) | ((y >= 0.0f) << 1) | ((z >= 0.0f) << 2);

    /* Active vectors in increasing angle order, initially in scaled volts. */
    switch (code)
    {
        case 3: t1 =  y; t2 =  x; break; /* sector 1: V1, V2 */
        case 1: t1 = -z; t2 = -y; break; /* sector 2: V2, V3 */
        case 5: t1 =  x; t2 =  z; break; /* sector 3: V3, V4 */
        case 4: t1 = -y; t2 = -x; break; /* sector 4: V4, V5 */
        case 6: t1 =  z; t2 =  y; break; /* sector 5: V5, V6 */
        case 2: t1 = -x; t2 = -z; break; /* sector 6: V6, V1 */
        default: return;                /* origin: neutral duty */
    }
    t1 *= SQRT3;
    t2 *= SQRT3;
    /* T1/Ts, T2/Ts: if their sum exceeds 1, scale both to the hexagon edge. */
    denominator = fmaxf(bus, t1 + t2);
    t1 /= denominator;
    t2 /= denominator;
    t0 = fmaxf(0.0f, 1.0f - t1 - t2);
    h = 0.5f * t0; /* split the zero-vector time equally between V0 and V7 */

    switch (code)
    {
        case 3: foc->dtc_a = h+t1+t2; foc->dtc_b = h+t2;    foc->dtc_c = h;       break;
        case 1: foc->dtc_a = h+t1;    foc->dtc_b = h+t1+t2; foc->dtc_c = h;       break;
        case 5: foc->dtc_a = h;       foc->dtc_b = h+t1+t2; foc->dtc_c = h+t2;    break;
        case 4: foc->dtc_a = h;       foc->dtc_b = h+t1;    foc->dtc_c = h+t1+t2; break;
        case 6: foc->dtc_a = h+t2;    foc->dtc_b = h;       foc->dtc_c = h+t1+t2; break;
        case 2: foc->dtc_a = h+t1+t2; foc->dtc_b = h;       foc->dtc_c = h+t1;    break;
    }
    foc->dtc_a = duty_clamp(foc->dtc_a);
    foc->dtc_b = duty_clamp(foc->dtc_b);
    foc->dtc_c = duty_clamp(foc->dtc_c);
}



void zero_sequence(foc_t *foc)
{
    float alpha, beta, bus, va, vb, vc, vmax, vmin, offset, denominator;
    if (!svpwm_prepare(foc, &alpha, &beta, &bus))
        return;

    va = alpha;
    vb = -0.5f * alpha + SQRT3_BY_2 * beta;
    vc = -0.5f * alpha - SQRT3_BY_2 * beta;
    vmax = fmaxf(va, fmaxf(vb, vc));
    vmin = fminf(va, fminf(vb, vc));
    offset = -0.5f * (vmax + vmin);
    /* Phase span / bus equals (T1 + T2) / Ts: identical radial limiting. */
    denominator = fmaxf(bus, vmax - vmin);
    foc->dtc_a = duty_clamp(0.5f + (va + offset) / denominator);
    foc->dtc_b = duty_clamp(0.5f + (vb + offset) / denominator);
    foc->dtc_c = duty_clamp(0.5f + (vc + offset) / denominator);
}

static float clampf(float value, float min, float max)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

static float fast_atan2f(float y, float x)
{
    float abs_y = fabsf(y) + 1e-20f;
    float angle;

    if (x >= 0.0f)
    {
        float r = (x - abs_y) / (x + abs_y);
        float rsq = r * r;
        angle = ((0.1963f * rsq) - 0.9817f) * r + 0.7853981633974483f;
    }
    else
    {
        float r = (x + abs_y) / (abs_y - x);
        float rsq = r * r;
        angle = ((0.1963f * rsq) - 0.9817f) * r + 2.356194490192345f;
    }
    return y < 0.0f ? -angle : angle;
}

/* 清零的是观测器的积分状态，并不表示电机真实磁链为零；复位后仍需建立有效估计。 */
bool vesc_observer_reset(volatile vesc_observer_state_t *state,
                         const vesc_observer_config_t *config)
{
    if (!state || !config) return false;
    state->flux_alpha = 0.0f;
    state->flux_beta = 0.0f;
    state->lambda_est = config->foc_motor_flux_linkage;
    state->i_alpha_last = 0.0f;
    state->i_beta_last = 0.0f;
    return true;
}


/* 原理：从定子电压、电流估计永磁体磁链，再由磁链方向得到转子电角度。
 *
 * 以下 u、i、psi 均为静止 alpha/beta 坐标系中的二维矢量：
 *   u      = R*i + d(psi_s)/dt          定子电压 = 电阻压降 + 总磁链变化率
 *   psi_s  = L*i + psi_r               总磁链 = 电流产生的磁链 + 永磁体磁链
 * 两式合并（假设 L 恒定）：
 *   d(psi_r)/dt = u - R*i - L*di/dt
 * 即扣除电阻、电感压降后得到反电势 e = d(psi_r)/dt，再积分得到磁链估计。
 * flux_alpha/flux_beta 保存的是 psi_r 的估计分量，单位 Wb，不是电压或电流。
 *
 * 理想永磁体磁链满足：
 *   psi_r = psi_f * [cos(theta_e), sin(theta_e)]
 * 因而 theta_e = atan2(psi_r_beta, psi_r_alpha)，其中 theta_e 为转子电角度。
 *
 * 模型限制：单一 L 近似忽略凸极性；Ld != Lq 时会有模型误差。
 * 输入电压应对应采样区间内实际施加的平均电压，电流与 dt 应对应同一更新区间。
 * 纯积分会保留初值误差，并累积电压/电流偏置的影响，故加入幅值自适应、裁剪和低幅值增强。
 * 这些约束有助于维持磁链轨迹，但不能保证角度正确；零速/低速时反电势弱，尤其如此。
 */

 /*
 磁链观测器基本原理，根据三相电压和三相电流，经clark变换得到α β轴的电流电压信息，再根据定子电压方程构建等式，得到α β轴的磁链分量，
 由这俩磁链分量构成的磁链矢量就包含了此时的转子角度信息，随后使用atan2 即可得到相对于α轴的转子角度
 因为α β是静止坐标系，所以不会像dq那样携带交叉耦合项，所以 v = r*i + L*di/dt
 */
bool vesc_observer_update(const vesc_observer_config_t *config, const vesc_observer_input_t *input, volatile vesc_observer_state_t *state, volatile float *theta)
{
    if (!config || !input || !state)
        return false;

    /* 本次更新参数：R/L 为电机模型，dt 为实际周期，flux 为标称磁链。 */
    const float r = config->foc_motor_r;
    const float l = config->foc_motor_ls;
    const float dt = input->dt;
    const float flux = config->foc_motor_flux_linkage;

    /* 算法系数集中在此：这些经验系数用于调整幅值和约束漂移，并非电机方程推导出的常数。
     * lambda_gain = 0.05*gamma；上下界和低幅值阈值均相对于标称磁链 flux。
     * flux_boost 是每次调用的增强倍率，与观测器实际执行频率有关。
     */
    const float lambda_gain = 0.1f * (input->gamma_now * 0.5f);
    const float min_flux = flux * 0.3f;
    const float max_flux = flux * 2.5f;
    const float low_flux = flux * 0.5f;
    const float flux_boost = 1.1f;

    float flux_alpha, flux_beta, lambda;     /* 本次磁链分量及幅值估计，Wb */
    float norm_sq, flux_err;  /* 磁链长度平方及幅值平方误差，Wb^2 */

    /* 1. 对电压方程在上次采样到本次采样之间积分：
     * psi_r[k] - psi_r[k-1] = integral(u-R*i)dt - L*(i[k]-i[k-1])。
     * 用本次输入近似区间电压、电流，得到：
     * psi_r[k] = psi_r[k-1] + (u[k]-R*i[k])*dt - L*(i[k]-i[k-1])。
     * 电感项来自 integral(L*di/dt)dt = L*delta_i，因此不再乘 dt；三项单位均为 Wb。
     * alpha、beta 两轴使用同一公式，上一周期的磁链与电流从 state 读取。
     */
    flux_alpha = state->flux_alpha + ((input->v_alpha - r * input->i_alpha) * dt - l * (input->i_alpha - state->i_alpha_last));
    flux_beta = state->flux_beta + ((input->v_beta - r * input->i_beta) * dt - l * (input->i_beta - state->i_beta_last));

    /* 2. 根据当前磁链长度 rho 调整幅值估计 lambda：
     * rho^2 = flux_alpha^2 + flux_beta^2，flux_err = lambda^2 - rho^2。
     * 将本步 rho 暂视为固定值，可用代价函数 J = (lambda^2-rho^2)^2 / 4 解释：
     * dJ/dlambda = lambda*(lambda^2-rho^2) = lambda*flux_err。
     * 沿负梯度调整：d(lambda)/dt = -lambda_gain*lambda*flux_err，
     * 离散后就是下面的 lambda += -lambda_gain*lambda*flux_err*dt。
     * 正增益且 lambda > 0 时，rho > lambda 则增加 lambda，反之减小。
     * 这是幅值调整律，不是角度误差反馈，也不能视为整个观测器的收敛证明。
     */
    lambda = state->lambda_est;
    norm_sq = flux_alpha * flux_alpha + flux_beta * flux_beta;
    flux_err = lambda * lambda - norm_sq;
    lambda += lambda_gain * lambda * -flux_err * dt;

    /* 3. 约束积分漂移：若输入含固定电压误差 delta_u，纯积分误差会随 delta_u*t 增长。
     * 先将 lambda 限制在 [0.3*标称磁链, 2.5*标称磁链]，再限制每个分量的绝对值。
     * 分量裁剪对应 alpha/beta 平面中的方形边界，不是将矢量投影到半径 lambda 的圆上。
     * 因此两轴裁剪后可能改变方向，且矢量长度不必等于 lambda；它并非精确位置校正。
     */
    lambda = clampf(lambda, min_flux, max_flux);
    flux_alpha = clampf(flux_alpha, -lambda, lambda);
    flux_beta = clampf(flux_beta, -lambda, lambda);

    /* 4. 若 rho < 0.5*标称磁链，将两个分量同时乘 1.1，避免估计幅值持续过小。
     * 比较平方可避免开方；乘相同正数保持非零矢量的方向，只增加幅值。
     * 此步骤在裁剪之后执行，增强后并未再次限幅；它也是经验处理，不是电压方程的一项。
     * 全零矢量乘 1.1 仍为零，因此不能靠此步骤获取静止转子的初始方向。
     */
    norm_sq = flux_alpha * flux_alpha + flux_beta * flux_beta;
    if (norm_sq < low_flux * low_flux)
    {
        flux_alpha *= flux_boost;
        flux_beta *= flux_boost;
    }

    /* 5. 保存本次磁链和电流，下次更新继续积分并计算 L*delta_i。
     * 由 psi_r_alpha = psi_f*cos(theta_e)、psi_r_beta = psi_f*sin(theta_e)，
     * 用 atan2(beta, alpha) 同时判断角度和象限；这里使用近似实现，输出约为 [-pi, pi]。
     * 取的是磁链方向，不是反电势方向，所以不额外偏移 90 度。
     * phase 可为 NULL，此时仍更新状态；返回 true 仅表示完成计算，不代表估计角度可信。
     * 本函数不估计速度，后续可由 PLL 跟踪这个角度并估算电角速度。
     */
    state->flux_alpha = flux_alpha;
    state->flux_beta = flux_beta;
    state->lambda_est = lambda;
    state->i_alpha_last = input->i_alpha;
    state->i_beta_last = input->i_beta;
    if (theta)
    {
        *theta = fast_atan2f(flux_beta, flux_alpha);
        if(*theta < 0)
            *theta = *theta + M_2PI;
    } 
    
    return true;
}


void vesc_pll_init(float kp, float ki, pll_info_t* pll)
{
    pll->pll_kp = kp;
    pll->pll_ki = ki;
}


void vesc_pll_update(float phase, float dt, pll_info_t* pll )
{
    const float pi = 3.14159265359f;
    const float two_pi = 6.28318530718f;
    float error = phase - pll->pll_theta;

    /* 将相位误差回绕到 [-pi, pi]，避免跨越零点时误判方向。 */
    while (error > pi)
        error -= two_pi;
    while (error < -pi)
        error += two_pi;

    pll->omega += pll->pll_ki * error * dt; //角速度 rad/s
    pll->pll_theta += (pll->omega + pll->pll_kp * error) * dt;  //角度 rad

    /* 将输出角度保持在 [0, 2pi)，兼容正转和反转。 */
    while (pll->pll_theta >= two_pi)
        pll->pll_theta -= two_pi;
    while (pll->pll_theta < 0.0f)
        pll->pll_theta += two_pi;
}

void vesc_pll_reset(void)
{
    pll_info.omega = 0;
    pll_info.pll_theta = 0;
}

/* 临界阻尼 TD：speed' = accel，accel' = wn^2*(target-speed) - 2*wn*accel。
 * 将本次目标视为在 dt 内不变，使用该二阶方程的解析离散更新：
 * error = speed-target，c = accel+wn*error，decay = exp(-wn*dt)；
 * error_next = (error+c*dt)*decay，accel_next = (accel-wn*c*dt)*decay。
 * 两个结果均由旧状态计算，避免简单欧拉积分在大 wn*dt 下失稳。
 */
void speed_td_update(float target_rpm, float dt, float wn, volatile speed_td_t *td)
{
    float speed, accel, error, c, decay, next_speed, next_accel;

    if (!td || !isfinite(target_rpm) || !isfinite(dt) || dt <= 0.0f ||
        !isfinite(wn) || wn <= 0.0f)
        return;

    speed = td->speed_rpm;
    accel = td->accel_rpm_s;
    if (!isfinite(speed) || !isfinite(accel))
        return;

    error = speed - target_rpm;
    c = accel + wn * error;
    decay = expf(-wn * dt);
    next_speed = target_rpm + (error + c * dt) * decay;
    next_accel = (accel - wn * c * dt) * decay;
    if (!isfinite(next_speed) || !isfinite(next_accel))
        return;

    td->speed_rpm = next_speed;
    td->accel_rpm_s = next_accel;
}

/* 速度外环：调用周期为 1 ms，ki 已包含该周期；输出为 q 轴电流给定。
 * 电流反馈 foc.i_q 由 Park 更新，此处不能覆盖。切换初始化当拍不调用。
 */
void foc_speed_update(float speed_ref_rpm, float speed_feedback_rpm)
{
    iq_pi.ref_value = parallel_pid_ctrl(&spd_pi, speed_ref_rpm, speed_feedback_rpm);
}

/******双DQ切换*******/

/* 双 dq 瞬时交接：在 IF 电流 PI 已完成、本周期逆 Park 尚未执行时调用一次。
 * foc.theta、v_d/v_q 必须属于同一控制周期；i_alpha/i_beta 是该周期的反馈。
 * delta = 闭环角度 - IF角度，R(-delta) 只改变坐标表示，保持实际矢量不变。
 * 成功后本周期直接执行逆 Park/SVPWM，不再调用电流或速度 PI；下一周期正常更新。
 * 本函数不切换状态机、不写 PWM、不判断观测器是否可信，也不执行后续 id 退坡。
 */
bool foc_double_dq_switch(float rotor_theta, float speed_ref_rpm, float speed_feedback_rpm)
{
    float delta, sin_delta, cos_delta, rotor_sin, rotor_cos;
    float id_ref, iq_ref, id_feedback, iq_feedback, vd, vq;
    float id_error, iq_error, speed_error;
    float id_p, iq_p, speed_p, id_integral, iq_integral, speed_integral;

    /* 接口使用 PI；角度需已归一化，速度的输入单位统一为机械 rpm。 */
    if (!isfinite(rotor_theta) || rotor_theta < 0.0f || rotor_theta >= M_2PI ||
        !isfinite(speed_ref_rpm) || !isfinite(speed_feedback_rpm) ||
        id_pi.kd != 0.0f || iq_pi.kd != 0.0f || spd_pi.kd != 0.0f)
        return false;

    delta = rotor_theta - foc.theta;
    sin_delta = sinf(delta);
    cos_delta = cosf(delta);
    rotor_sin = sinf(rotor_theta);
    rotor_cos = cosf(rotor_theta);

    /* 1. 同一电流给定矢量转到新 dq 轴；不能在切换时直接把新 id 给定清零。 */
    id_ref = id_pi.ref_value * cos_delta + iq_pi.ref_value * sin_delta;
    iq_ref = -id_pi.ref_value * sin_delta + iq_pi.ref_value * cos_delta;

    /* 2. 用同一帧 alpha/beta 电流计算新坐标系的真实反馈，不把给定当作反馈。 */
    id_feedback = foc.i_alpha * rotor_cos + foc.i_beta * rotor_sin;
    iq_feedback = -foc.i_alpha * rotor_sin + foc.i_beta * rotor_cos;

    /* 3. 转换电压指令，使新旧逆 Park 得到相同的 alpha/beta 电压。 */
    vd = foc.v_d * cos_delta + foc.v_q * sin_delta;
    vq = -foc.v_d * sin_delta + foc.v_q * cos_delta;

    /* 4. 预置为本周期结束时的 PI 状态：I = 目标输出 - Kp*误差。
     * 电流 PI 输出单位 V，速度 PI 输出单位 A，接管新的 iq 给定。
     * 本周期不再执行 PI，故不额外扣除 ki*error；下一次调用正常积分。
     */
    id_error = id_ref - id_feedback;
    iq_error = iq_ref - iq_feedback;
    speed_error = speed_ref_rpm - speed_feedback_rpm;
    id_p = id_pi.kp * id_error;
    iq_p = iq_pi.kp * iq_error;
    speed_p = spd_pi.kp * speed_error;
    id_integral = vd - id_p;
    iq_integral = vq - iq_p;
    speed_integral = iq_ref - speed_p;

    /* 限幅会破坏交接连续性；不强行裁剪，失败时不修改任何控制状态。
     * 调用方需保证 PI 增益和限幅已正确配置，并在同一控制中断中独占调用。
     */
    if (!isfinite(id_ref) || !isfinite(iq_ref) ||
        !isfinite(id_feedback) || !isfinite(iq_feedback) ||
        !isfinite(vd) || !isfinite(vq) ||
        !isfinite(id_integral) || !isfinite(iq_integral) || !isfinite(speed_integral) ||
        vd < id_pi.out_min || vd > id_pi.out_max ||
        vq < iq_pi.out_min || vq > iq_pi.out_max ||
        iq_ref < spd_pi.out_min || iq_ref > spd_pi.out_max ||
        id_integral < id_pi.i_term_min || id_integral > id_pi.i_term_max ||
        iq_integral < iq_pi.i_term_min || iq_integral > iq_pi.i_term_max ||
        speed_integral < spd_pi.i_term_min || speed_integral > spd_pi.i_term_max)
        return false;

    /* 5. 全部检查通过后，提交三个 PI 的给定、反馈和内部状态。 */
    id_pi.ref_value = id_ref;
    id_pi.fback_value = id_feedback;
    id_pi.error = id_pi.pre_err = id_error;
    id_pi.p_term = id_p;
    id_pi.i_term = id_integral;
    id_pi.d_term = 0.0f;
    id_pi.out_value = vd;

    iq_pi.ref_value = iq_ref;
    iq_pi.fback_value = iq_feedback;
    iq_pi.error = iq_pi.pre_err = iq_error;
    iq_pi.p_term = iq_p;
    iq_pi.i_term = iq_integral;
    iq_pi.d_term = 0.0f;
    iq_pi.out_value = vq;

    spd_pi.ref_value = speed_ref_rpm;
    spd_pi.fback_value = speed_feedback_rpm;
    spd_pi.error = spd_pi.pre_err = speed_error;
    spd_pi.p_term = speed_p;
    spd_pi.i_term = speed_integral;
    spd_pi.d_term = 0.0f;
    spd_pi.out_value = iq_ref;

    /* 6. 切换 Park/逆 Park 使用的角度；alpha/beta 电压由调用方随后计算。 */
    foc.theta = rotor_theta;
    foc.sin_val = rotor_sin;
    foc.cos_val = rotor_cos;
    foc.i_d = id_feedback;
    foc.i_q = iq_feedback;
    foc.v_d = vd;
    foc.v_q = vq;
    return true;
}




