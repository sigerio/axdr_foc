#ifndef _GLOAB_STR_H_
#define _GLOAB_STR_H_

#include <stdint.h>



#define PWM_PERIOD (1000.0f) //根据选择的定时器周期来进行
#define rse 0.001f // 采样电阻
#define iop 20u    // 电流增益
#define iratio ((3.3f / 4096.0f) / rse / iop)
#define vratio (21.0f * 3.3f / 4096.0f)

typedef struct
{
    uint8_t tick_ms;
}tim_tick_t;
extern tim_tick_t tim_tick;

typedef struct
{
    float ia;
    float ib;
    float ic;

    float ia_offset;
    float ib_offset;
    float ic_offset;

    uint16_t start_time;
    uint16_t stop_time;
    uint16_t run_time;

    float obs_theta;
    float pll_speed; /* PLL 估计机械转速，rpm；由电角速度换算 */

    uint8_t sample_finish;
}foc_sample_t;

extern volatile foc_sample_t foc_sample;




typedef struct
{
    float vbus;
    float inv_vbus; // 母线电压倒数，由 SVPWM 接口根据 vbus 更新

    float theta;   // 角度
    float sin_val; // 此角度对应的正弦值
    float cos_val; // 此角度对应的余弦值

    float i_a; // A 相电流
    float i_b; // B 相电流
    float i_c; // C 相电流

    float v_a; // A 相电压
    float v_b; // B 相电压
    float v_c; // C 相电压

    float i_d; // D 坐标系电流
    float i_q; // Q 坐标系电流

    float v_d; // D 坐标系电压
    float v_q; // Q 坐标系电压

    float i_alpha; // Alpha 坐标系电流
    float i_beta;  // Beta 坐标系电流

    float v_alpha; // Alpha 坐标系电压
    float v_beta;  // Beta 坐标系电压

    float dtc_a; // A 相上桥臂占空比 [0, 1]
    float dtc_b; // B 相上桥臂占空比 [0, 1]
    float dtc_c; // C 相上桥臂占空比 [0, 1]


}foc_t;
extern foc_t foc;






void user_init(void);

#endif
