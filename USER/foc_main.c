#include "foc_main.h"
#include "foc.h"
#include "app_init.h"
#include "moto_config.h"
#include "main.h"
#include <math.h>

volatile foc_ctrl_t foc_ctrl;

#define FOC_TWO_PI (6.28318530718f)

static void foc_stop(void)
{
    foc_pwm_stop();
    foc_ctrl.start_cmd = 0;
    foc_ctrl.omega_speed = 0.0f;
    foc_ctrl.stage_time = 0;
    foc_ctrl.mecha = IDLE;
}

/* IDLE/PREPARE 由主循环调用，定位和强拖由 ADC 中断调用；均不得阻塞等待。 */
static void foc_idle(void)
{
    foc_pwm_stop();
    /* shortcut: 暂由调用方保证初始化和零偏校准完成，接入启动入口时补就绪判断。 */
    if (foc_ctrl.start_cmd)
        foc_ctrl.mecha = PREPARE;
}

static void foc_prepare(void)
{
    foc_pwm_stop();
    if (!foc_ctrl.start_cmd)
    {
        foc_ctrl.mecha = IDLE;
        return;
    }


    /* 清除上次运行的状态，保留 PI 增益、限幅和实时采样值。 */
    pid_clear(&id_pi);
    pid_clear(&iq_pi);
    pid_clear(&spd_pi);
    vesc_pll_reset();

    id_pi.ref_value = id_pi.fback_value = 0.0f;
    iq_pi.ref_value = iq_pi.fback_value = 0.0f;
    spd_pi.ref_value = spd_pi.fback_value = 0.0f;

    foc.theta = 0.0f;
    foc.sin_val = 0.0f;
    foc.cos_val = 1.0f;
    foc.i_alpha = foc.i_beta = 0.0f;
    foc.i_d = foc.i_q = 0.0f;
    
    foc.v_alpha = foc.v_beta = 0.0f;
    foc.v_a = foc.v_b = foc.v_c = 0.0f;
    foc.dtc_a = foc.dtc_b = foc.dtc_c = 0.5f;

    foc_ctrl.omega_speed = 0.0f;
    foc.v_d = fminf(FOC_ALIGN_VD, foc.vbus * 0.57735026919f);
    foc.v_q = 0.0f;
    foc_ctrl.stage_time = (uint16_t)HAL_GetTick();
    foc_pwm_start();
    vesc_observer_reset(&vesc_observer_state, &vesc_observer_config);

    vesc_observer_input.dt = FOC_DT;
    vesc_observer_input.gamma_now = FOC_OBSERVER_GAMMA;


    foc_ctrl.mecha = ALIGEN;
}

static void foc_align(void)
{
    foc.v_d = fminf(FOC_ALIGN_VD, foc.vbus * 0.57735026919f);
    foc.v_q = 0.0f;
    if ((uint16_t)((uint16_t)HAL_GetTick() - foc_ctrl.stage_time) >= FOC_ALIGN_TIME_MS)
    {
        

        foc_ctrl.f_ctrl = USE_IF;
        foc_ctrl.rpm = VF_TAR_RPM;
        if(foc_ctrl.f_ctrl == USE_VF)
        {
            /* shortcut: 定位电压补低速压降，负载变化较大时需标定 V/f 曲线或改用电流环。 */
            foc.v_d = fminf(FOC_ALIGN_VD + fabsf(foc_ctrl.omega_speed) * MOTOR_FLUX_LINKAGE, foc.vbus * 0.57735026919f);
            foc.v_q = 0.0f;
            
            foc_ctrl.mecha = OPEN_VF;
        } 
        else
        {
            iq_pi.kp = 0.16128937f;
            iq_pi.ki = 0.03533477f;
            id_pi.kp = 0.14042919f;
            id_pi.ki = 0.03533477f;

            iq_pi.ref_value = foc.i_q = 1.0f;
            id_pi.ref_value = foc.i_d = 0.0f;
            foc_ctrl.mecha = OPEN_IF;
        }
            
    }
        
}

static void open_theta(void)
{
    float target = foc_ctrl.rpm * (FOC_TWO_PI * MOTOR_POLE_PAIRS / 60.0f); //折算成电角速度
    float step = target / (VF_OVER_TIME*FOC_VF_UPDATE_HZ); //折算成电角速度增量

    /* omega_speed 为电角速度；按 ADC 周期积分并夹到目标。 */
    foc_ctrl.omega_speed += fminf(step, fmaxf(-step, target - foc_ctrl.omega_speed));
    foc.theta = fmodf(foc.theta + foc_ctrl.omega_speed / FOC_VF_UPDATE_HZ, FOC_TWO_PI);
    if (foc.theta < 0.0f)
        foc.theta += FOC_TWO_PI;
    if (foc.theta >= FOC_TWO_PI)
        foc.theta -= FOC_TWO_PI;
}

static void foc_open_vf(void)
{
    open_theta();
    foc.v_d = fminf(FOC_ALIGN_VD + fabsf(foc_ctrl.omega_speed) * MOTOR_FLUX_LINKAGE, foc.vbus * 0.57735026919f);
}

static void foc_open_if(void)
{
    /* TODO: 更新开环电角度，执行 dq 电流环并输出 PWM。 */
    open_theta();
    

}

static void foc_open2close(void)
{
    /* TODO: 确认角度反馈有效，平滑切换角度和控制指令后进入闭环。 */
}

static void foc_close(void)
{
    /* TODO: 使用反馈电角度执行电流环，计算并输出 PWM。 */
}

void foc_in_interrupt(void)
{
    if(!foc_ctrl.start_cmd)
    {
        foc_stop();
        return;
    }
    switch (foc_ctrl.mecha)
    {
    case IDLE:
    case PREPARE:
        return;
    
    case ALIGEN:
        
        break;
    
    case OPEN_VF:
        foc_open_vf();
        break;

    case OPEN_IF:
        foc_open_if();
        break;

    case OPEN2CLOSE:
        foc_open2close();
        return;

    case CLOSE:
        foc_close();
        break;

    case STOP:
    default:
        foc_stop();
        return;
    }
    

    if(foc_ctrl.mecha == OPEN_IF)
    {
        clarke_transform(&foc);

        vesc_observer_input.i_alpha = foc.i_alpha;
        vesc_observer_input.i_beta = foc.i_beta;
        vesc_observer_input.v_alpha = (2.0f*foc.dtc_a - foc.dtc_b - foc.dtc_c)/(3.0f) * foc.vbus;
        vesc_observer_input.v_beta = ONE_BY_SQRT3*(foc.dtc_b-foc.dtc_c)*foc.vbus;

        vesc_observer_update(&vesc_observer_config, &vesc_observer_input, &vesc_observer_state, &foc_sample.obs_theta);

        vesc_pll_update(foc_sample.obs_theta,FOC_DT,&pll_info);

        sin_cos_val(&foc);
        park_transform(&foc);
        foc.v_d = parallel_pid_ctrl(&id_pi, id_pi.ref_value, foc.i_d);
        foc.v_q = parallel_pid_ctrl(&iq_pi, iq_pi.ref_value, foc.i_q);
    }
    else if(foc_ctrl.mecha == CLOSE)
    {
        clarke_transform(&foc);

        
        vesc_observer_input.i_alpha = foc.i_alpha;
        vesc_observer_input.i_beta = foc.i_beta;
        vesc_observer_input.v_alpha = (2.0f*foc.dtc_a - foc.dtc_b - foc.dtc_c)/(3.0f) * foc.vbus;
        vesc_observer_input.v_beta = ONE_BY_SQRT3*(foc.dtc_b-foc.dtc_c)*foc.vbus;

        vesc_observer_update(&vesc_observer_config, &vesc_observer_input, &vesc_observer_state, &foc_sample.obs_theta);
        vesc_pll_update(foc_sample.obs_theta,FOC_DT,&pll_info);
        
        
        foc.theta = pll_info.pll_theta;
        sin_cos_val(&foc);

        park_transform(&foc);

        foc.v_d = parallel_pid_ctrl(&id_pi, id_pi.ref_value, foc.i_d);
        foc.v_q = parallel_pid_ctrl(&iq_pi, iq_pi.ref_value, foc.i_q);
    }
    else
    {
        sin_cos_val(&foc);
    }
    inverse_park(&foc);
    zero_sequence(&foc);
    foc_pwm_run();
}

void foc_in_maincyc(void)
{
    /* 主循环负责启动准备；运行阶段的计时、调速和停机由 ADC 中断处理。 */
    switch (foc_ctrl.mecha)
    {
    case IDLE:
        foc_idle();
        break;
    case PREPARE:
        foc_prepare();
        break;
    
    case ALIGEN:
        foc_align();
        break;
    case OPEN_VF:
        
        break;

    case OPEN_IF:
        /* TODO: 更新 IF 开环目标指令。 */
        break;

    case OPEN2CLOSE:
        /* TODO: 监视开闭环切换过程。 */
        break;

    case CLOSE:
        /* TODO: 按约定周期执行速度环，更新电流目标。 */
        spd_pi.ref_value = foc_ctrl.rpm;
        foc_sample.pll_speed = pll_info.omega * 60.0f / (M_2PI * MOTOR_POLE_PAIRS);
        iq_pi.ref_value = parallel_pid_ctrl(&spd_pi, spd_pi.ref_value, foc_sample.pll_speed);
        break;

    case STOP:
        /* TODO: 停止状态的非实时处理。 */
        break;
    
    default:
        break;
    }
}



