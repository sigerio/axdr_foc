/* Linux host test using actual FOC, PI and HAL timer functions.
 * Build like test_foc_pwm_stop.c, adding USER/foc_main.c and USER/pid.c.
 */
#define _GNU_SOURCE
#include <assert.h>
#include <math.h>
#include <sys/mman.h>
#include <unistd.h>
#include "tim.h"
#include "foc_main.h"
#include "foc.h"
#include "app_init.h"
#include "moto_config.h"

TIM_HandleTypeDef htim1;
pid_para_t id_pi, iq_pi, spd_pi;
static uint32_t tick_ms = 65000;

uint32_t HAL_GetTick(void)
{
    return tick_ms;
}

int main(void)
{
    const float two_pi = 6.28318530718f;
    const float to_electrical = two_pi * MOTOR_POLE_PAIRS / 60.0f;
    const uint32_t bridge_mask = TIM_CCER_CC1E | TIM_CCER_CC1NE |
                                 TIM_CCER_CC2E | TIM_CCER_CC2NE |
                                 TIM_CCER_CC3E | TIM_CCER_CC3NE;
    size_t page_size = (size_t)sysconf(_SC_PAGESIZE);
    uintptr_t base = (uintptr_t)TIM1 & ~(page_size - 1);
    void *mapped = mmap((void *)base, page_size, PROT_READ | PROT_WRITE,
                        MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
    pid_para_t *controllers[] = { &id_pi, &iq_pi, &spd_pi };
    unsigned int i;
    float held_angle;

    assert(mapped != MAP_FAILED);
    assert((uintptr_t)TIM1 - base + sizeof(*TIM1) <= page_size);
    htim1.Instance = TIM1;
    TIM1->ARR = 4249;
    TIM1->CCR4 = 4100;
    TIM_CHANNEL_STATE_SET(&htim1, TIM_CHANNEL_4, HAL_TIM_CHANNEL_STATE_READY);
    assert(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) == HAL_OK);
    assert(foc_ctrl.rpm == SPEED_TEST_START_RPM);

    /* A missing bus voltage prevents PWM startup. */
    foc_ctrl.start_cmd = 1;
    foc_in_maincyc();
    foc_in_maincyc();
    assert(foc_ctrl.mecha == IDLE && foc_ctrl.start_cmd == 0);
    assert((TIM1->CCER & bridge_mask) == 0);

    foc.vbus = 24.0f;
    foc.i_a = 0.1f;
    foc.i_b = 0.2f;
    foc.i_c = -0.3f;
    id_pi = iq_pi = spd_pi = (pid_para_t){
        .kp = 1.0f, .ki = 2.0f, .kd = 3.0f, .ctrl_period = 0.001f,
        .i_term_max = 4.0f, .i_term_min = -4.0f, .out_max = 5.0f, .out_min = -5.0f,
        .p_term = 1.0f, .i_term = 2.0f, .d_term = 3.0f,
        .ref_value = 4.0f, .fback_value = 5.0f, .error = 6.0f, .pre_err = 7.0f, .out_value = 8.0f
    };
    foc_ctrl.start_cmd = 1;
    foc_in_maincyc();
    assert(foc_ctrl.mecha == PREPARE);
    foc_ctrl.start_cmd = 0;
    foc_in_maincyc();
    assert(foc_ctrl.mecha == IDLE && (TIM1->CCER & bridge_mask) == 0);
    foc_ctrl.start_cmd = 1;
    foc_in_maincyc();
    foc_in_maincyc();
    assert(foc_ctrl.mecha == ALIGEN && foc_ctrl.stage_time == (uint16_t)tick_ms);
    assert(foc_ctrl.omega_speed == 0.0f && foc.theta == 0.0f);
    assert(foc.v_d == FOC_ALIGN_VD && foc.v_q == 0.0f);
    assert(foc.i_a == 0.1f && foc.i_b == 0.2f && foc.i_c == -0.3f);
    assert((TIM1->CCER & bridge_mask) == bridge_mask);
    assert((TIM1->EGR & TIM_EGR_UG) != 0);
    assert(TIM1->CCR1 == (uint16_t)(foc.dtc_a * PWM_ARR()));
    assert(TIM1->CCR2 == (uint16_t)(foc.dtc_b * PWM_ARR()));
    assert(TIM1->CCR3 == (uint16_t)(foc.dtc_c * PWM_ARR()));
    assert(TIM1->CCR4 == 4100);
    for (i = 0; i < 3; ++i)
    {
        pid_para_t *pi = controllers[i];
        assert(pi->kp == 1.0f && pi->ki == 2.0f && pi->kd == 3.0f);
        assert(pi->ctrl_period == 0.001f);
        assert(pi->i_term_max == 4.0f && pi->i_term_min == -4.0f);
        assert(pi->out_max == 5.0f && pi->out_min == -5.0f);
        assert(pi->p_term == 0.0f && pi->i_term == 0.0f && pi->d_term == 0.0f);
        assert(pi->ref_value == 0.0f && pi->fback_value == 0.0f);
        assert(pi->error == 0.0f && pi->pre_err == 0.0f && pi->out_value == 0.0f);
    }

    /* Main-loop load cannot shorten alignment; timestamp crosses 16-bit wrap. */
    for (i = 0; i < 2000; ++i)
        foc_in_maincyc();
    assert(foc_ctrl.mecha == ALIGEN);
    tick_ms += FOC_ALIGN_TIME_MS - 1;
    foc_in_interrupt();
    assert(foc_ctrl.mecha == ALIGEN && foc.theta == 0.0f);
    ++tick_ms;
    foc_in_interrupt();
    assert(foc_ctrl.mecha == OPEN_VF && foc_ctrl.omega_speed == 0.0f && foc.theta == 0.0f);

    foc_in_interrupt();
    assert(fabsf(foc_ctrl.omega_speed - SPEED_TEST_ACCEL * to_electrical / FOC_VF_UPDATE_HZ) < 1e-6f);
    assert(fabsf(foc.theta - foc_ctrl.omega_speed / FOC_VF_UPDATE_HZ) < 1e-9f);
    for (i = 1; i < (unsigned int)FOC_VF_UPDATE_HZ; ++i)
        foc_in_interrupt();
    assert(fabsf(foc_ctrl.omega_speed - SPEED_TEST_ACCEL * to_electrical) < 0.2f);
    assert(fabsf(foc.v_d - (FOC_ALIGN_VD + fabsf(foc_ctrl.omega_speed) * MOTOR_FLUX_LINKAGE)) < 1e-5f);

    for (i = 0; i < (unsigned int)(6 * FOC_VF_UPDATE_HZ); ++i)
    {
        foc_in_interrupt();
        assert(foc.theta >= 0.0f && foc.theta < two_pi);
        assert(foc_ctrl.omega_speed <= foc_ctrl.rpm * to_electrical);
    }
    assert(fabsf(foc_ctrl.omega_speed - SPEED_TEST_START_RPM * to_electrical) < 1e-4f);
    assert(foc_ctrl.mecha == OPEN_VF);

    foc_ctrl.rpm = 300.0f;
    for (i = 0; i < (unsigned int)(3.5f * FOC_VF_UPDATE_HZ); ++i)
        foc_in_interrupt();
    assert(fabsf(foc_ctrl.omega_speed - 300.0f * to_electrical) < 1e-4f);
    foc_ctrl.rpm = -600.0f;
    for (i = 0; i < (unsigned int)(10 * FOC_VF_UPDATE_HZ); ++i)
    {
        foc_in_interrupt();
        assert(foc.theta >= 0.0f && foc.theta < two_pi);
    }
    assert(fabsf(foc_ctrl.omega_speed + 600.0f * to_electrical) < 1e-4f);

    foc_ctrl.rpm = 0.0f;
    for (i = 0; i < (unsigned int)(7 * FOC_VF_UPDATE_HZ); ++i)
        foc_in_interrupt();
    assert(foc_ctrl.omega_speed == 0.0f);
    held_angle = foc.theta;
    foc_in_interrupt();
    assert(foc.theta == held_angle && foc.v_d == FOC_ALIGN_VD);

    foc_ctrl.start_cmd = 0;
    foc_in_interrupt();
    assert(foc_ctrl.mecha == IDLE && foc_ctrl.omega_speed == 0.0f);
    assert((TIM1->CCER & bridge_mask) == 0 && (TIM1->CCER & TIM_CCER_CC4E) != 0);
    assert((TIM1->BDTR & TIM_BDTR_MOE) != 0 && (TIM1->CR1 & TIM_CR1_CEN) != 0);

    /* Reverse restart includes a tiny negative angle step at theta=0. */
    foc_ctrl.rpm = -600.0f;
    foc_ctrl.start_cmd = 1;
    foc_in_maincyc();
    foc_in_maincyc();
    tick_ms += FOC_ALIGN_TIME_MS;
    foc_in_interrupt();
    foc_in_interrupt();
    assert(foc_ctrl.omega_speed < 0.0f && foc.theta >= 0.0f && foc.theta < two_pi);
    foc.vbus = 0.1f;
    foc_in_interrupt();
    assert(foc.v_d <= foc.vbus * 0.57735026919f);
    foc_ctrl.rpm = NAN;
    foc_in_interrupt();
    assert(foc_ctrl.mecha == IDLE && foc_ctrl.start_cmd == 0);
    assert((TIM1->CCER & bridge_mask) == 0);

    /* Stop also applies during alignment, before any VF angle integration. */
    foc.vbus = 24.0f;
    foc_ctrl.rpm = 600.0f;
    foc_ctrl.start_cmd = 1;
    foc_in_maincyc();
    foc_in_maincyc();
    assert(foc_ctrl.mecha == ALIGEN);
    foc_ctrl.start_cmd = 0;
    foc_in_interrupt();
    assert(foc_ctrl.mecha == IDLE && (TIM1->CCER & bridge_mask) == 0);
    assert(munmap(mapped, page_size) == 0);
    return 0;
}
