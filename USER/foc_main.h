#ifndef _FOC_MAIN_H_
#define _FOC_MAIN_H_

#include <stdint.h>

typedef enum
{
    IDLE = 0,
    PREPARE,
    ALIGEN,
    OPEN_VF,
    OPEN_IF,
    OPEN2CLOSE,
    CLOSE,
    STOP,
}FOC_STATE_MECHA;

typedef enum
{
    USE_VF = 0,
    USE_IF,
}IVF;


typedef struct
{
    uint8_t start_cmd;
    FOC_STATE_MECHA mecha;
    uint16_t stage_time; /* ALIGEN: 毫秒时间戳低16位；CLOSE: 速度环ADC分频计数 */

    float omega_speed; /* 当前强拖电角速度指令，rad/s，不是实测转速 */
    float rpm;         /* 机械目标转速，rpm；正负号决定旋转方向 */
    
    IVF f_ctrl;
}foc_ctrl_t;
extern volatile foc_ctrl_t foc_ctrl;

/* ADC 中断入口：定位、VF 速度斜坡/角度积分及运行停机。 */
void foc_in_interrupt(void);
/* 主循环入口：处理 IDLE/PREPARE，并预留后续慢任务。 */
void foc_in_maincyc(void);

#endif
