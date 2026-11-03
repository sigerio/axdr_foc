#include "app_init.h"
#include "tim.h"
#include "adc.h"
#include "gloab.h"
#include "pid.h"
#include "moto_config.h"
#include "foc.h"

volatile foc_sample_t foc_sample;
pid_para_t id_pi;
pid_para_t iq_pi;
pid_para_t spd_pi;

static void start_current_offset(void)
{
    int sum = 0;
    float ia = 0;
    float ib = 0;
    float ic = 0;
    uint32_t primask;
    foc_sample.sample_finish = 0;
    while (sum < 1000)
    {
        if (foc_sample.sample_finish)
        {
            /* 三相读数和标志一起消费，避免中途被下一帧采样打断。 */
            primask = __get_PRIMASK();
            __disable_irq();
            ia += foc_sample.ia;
            ib += foc_sample.ib;
            ic += foc_sample.ic;
            foc_sample.sample_finish = 0;
            __set_PRIMASK(primask);
            sum++;
        }
    }
    foc_sample.ia_offset = ia / (1000.0f);
    foc_sample.ib_offset = ib / (1000.0f);
    foc_sample.ic_offset = ic / (1000.0f);
}

static void harware_init(void)
{
    HAL_TIM_Base_Start(&htim8);
    HAL_TIM_Base_Start_IT(&htim2);

    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK ||
        HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK)
    {
        Error_Handler();
        return;
    }

    if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK || HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK)
    {
        Error_Handler();
        return;
    }


    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 4100);
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
        return;
    }
}

static void app_init(void)
{

    pid_para_init(&id_pi);
    pid_para_init(&iq_pi);
    pid_para_init(&spd_pi);

    pid_limit_init(&id_pi, 10.8f, -10.8f, 10.8f, -10.8f);
    pid_limit_init(&iq_pi, 10.8f, -10.8f, 10.8f, -10.8f);
    spd_pi.kp = FOC_SPEED_PI_KP;
    spd_pi.ki = FOC_SPEED_PI_KI;
    pid_limit_init(&spd_pi, 1.0f, -1.0f, 1.0f, -1.0f);

    vesc_pll_init(PLL_KP,PLL_KI, &pll_info);

    start_current_offset();
}

void user_init(void)
{
    harware_init();
    app_init();
}
