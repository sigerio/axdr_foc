#include "tim.h"
#include "adc.h"

#include "gloab.h"
#include "foc_main.h"



tim_tick_t tim_tick;
void interrupt_to_ticks(void)
{

    tim_tick.tick_ms++;
        

}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) //测试使用tim8(和tim1同频率)溢出中断来控制电角度的递增
{
    if (htim->Instance == TIM2)
    {
        interrupt_to_ticks();
    }
}




void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{

    if (hadc->Instance == ADC1)
    {
        TIM8->CNT = 0;
        foc_sample.start_time = TIM8->CNT; //注意如果开启了注入中断，角度累加会失效
        
        foc_sample.ia 	= ADC1->JDR3;
        foc_sample.ib 	= ADC1->JDR2;
        foc_sample.ic 	= ADC1->JDR1;
        foc_sample.sample_finish = 1;
        /* ADC2 shares the T1_CC4 trigger and converts one channel, so it
         * finishes before ADC1's three; its interrupt is not enabled. */
        foc.vbus = (float)ADC2->JDR1 * vratio;

        foc.i_a = ((float)foc_sample.ia_offset - foc_sample.ia)*iratio;
        foc.i_b = ((float)foc_sample.ib_offset - foc_sample.ib)*iratio;
        foc.i_c = ((float)foc_sample.ic_offset - foc_sample.ic)*iratio;

        
        foc_in_interrupt();

        foc_sample.stop_time = TIM8->CNT;
        
        foc_sample.run_time = foc_sample.stop_time - foc_sample.start_time;
        
    }

}

