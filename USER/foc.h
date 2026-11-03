#ifndef _FOC_H_
#define _FOC_H_

#include "gloab.h"
#include <stdbool.h>

#define PWM_ARR() 4250  //__HAL_TIM_GET_AUTORELOAD(&htim1)  实际4249，简单的死区补偿
#define set_dtc_a(value)     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, value)
#define set_dtc_b(value)     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, value)
#define set_dtc_c(value)     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, value)

#define M_PI (3.14159265358f)         // 圆周率
#define M_2PI (6.28318530716f)        // 2倍圆周率
#define SQRT3 (1.73205080757f)        // 3的平方根
#define SQRT3_BY_2 (0.86602540378f)   // 3的平方根的一半
#define ONE_BY_SQRT3 (0.57735026919f) // 1除以3的平方根
#define TWO_BY_SQRT3 (1.15470053838f) // 2除以3的平方根

/* VESC MXLEMMING_LAMBDA_COMP：静止 alpha/beta 坐标系中的磁链观测器。 */
/* 固定配置：电机模型参数。 */
typedef struct
{
    float foc_motor_r;            /* 相电阻 Rs，ohm */
    float foc_motor_ls;      /* 模型使用的单一相电感 Ls，H */ 
    float foc_motor_flux_linkage; /* 标称永磁磁链，Wb */
} vesc_observer_config_t;

/* 本次输入：电压、电流、更新周期及当前观测增益。 */
typedef struct
{
    float v_alpha;   /* 采样区间对应的 alpha 轴电压，V */
    float v_beta;    /* 采样区间对应的 beta 轴电压，V */
    float i_alpha;   /* 本次 alpha 轴电流，A */
    float i_beta;    /* 本次 beta 轴电流，A */
    float dt;        /* 实际观测更新周期，s；20 kHz 对应 0.00005 s */
    float gamma_now; /* 磁链幅值自适应增益，非 PLL 增益 */
} vesc_observer_input_t;

/* 跨周期状态：磁链估计及上次电流。 */
typedef struct
{
    float flux_alpha;   /* 转子磁链 alpha 分量估计，Wb */
    float flux_beta;    /* 转子磁链 beta 分量估计，Wb */
    float lambda_est;   /* 自适应磁链幅值估计，Wb */
    float i_alpha_last; /* 上次 alpha 轴电流，A */
    float i_beta_last;  /* 上次 beta 轴电流，A */
} vesc_observer_state_t;

typedef struct 
{
    float pll_kp;
    float pll_ki;
    float omega; // rad/s
    float pll_theta; //rad
}pll_info_t;

extern pll_info_t pll_info;

/* 二阶线性跟踪微分器：平滑速度指令，阻尼比固定为 1。 */
typedef struct
{
    float speed_rpm;       /* 平滑后的机械转速给定，rpm */
    float accel_rpm_s;     /* 速度给定的变化率，rpm/s；不是实测加速度 */
} speed_td_t;

extern volatile speed_td_t speed_td;

/* dt: 秒；wn: 自然角频率，s^-1，越大跟踪越快。
 * 初次使用：speed_rpm 设为起始给定，accel_rpm_s 设为 0；后续保留状态。
 * 无效参数/状态或计算溢出时保持原状态；本接口不限制最大加速度。
 */
void speed_td_update(float target_rpm, float dt, float wn, volatile speed_td_t *td);

extern const vesc_observer_config_t vesc_observer_config;
extern vesc_observer_input_t vesc_observer_input;
extern vesc_observer_state_t vesc_observer_state;

bool vesc_observer_reset(volatile vesc_observer_state_t *state,
                         const vesc_observer_config_t *config);

/* phase 可为 NULL；非空时输出估计电角度，rad，约在 [-pi, pi]。
 * 调用方保证输入数值有效、dt > 0；返回 true 仅表示完成更新，不代表观测已收敛。
 */
bool vesc_observer_update(const vesc_observer_config_t *config,
                          const vesc_observer_input_t *input,
                          volatile vesc_observer_state_t *state,
                          volatile float *phase);

void vesc_pll_init(float kp, float ki, pll_info_t* pll);
void vesc_pll_update(float phase, float dt, pll_info_t* pll );
void vesc_pll_reset(void);



/* 双 dq 切换：复用全局 foc、id_pi/iq_pi/spd_pi，无新增控制状态。
 * rotor_theta: 闭环电角度 [0, 2pi)，rad；两个 speed 参数均为机械 rpm。
 * 在同一控制中断中，IF 电流 PI 完成后、逆 Park 前调用一次。
 * true: 已交接，本周期直接逆 Park/SVPWM，下一周期再运行 PI。
 * false: 参数无效、非 PI 或预置值超限，原状态不变，继续原 IF 输出。
 * 调用方负责观测有效性、切换状态机和闭环后续运行；详见 foc.c 的步骤说明。
 */
bool foc_double_dq_switch(float rotor_theta, float speed_ref_rpm, float speed_feedback_rpm);

/* 每 1 ms 调用：输入均为机械 rpm，使用 spd_pi 的增益/限幅更新 iq_pi.ref_value。
 * 保留双 dq 预置的积分状态和 id 给定，不修改电流反馈，不在交接当拍调用。
 */
void foc_speed_update(float speed_ref_rpm, float speed_feedback_rpm);

void foc_pwm_start(void);
void foc_pwm_stop(void);
void foc_pwm_run(void);

void sin_cos_val(foc_t *foc);
void clarke_transform(foc_t *foc);
void park_transform(foc_t *foc);
void inverse_clarke(foc_t *foc);
void inverse_park(foc_t *foc);

void zero_sequence(foc_t *foc);

void foc_in_maincyc(void);
void foc_in_interrupt(void);

#endif
