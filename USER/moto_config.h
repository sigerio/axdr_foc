#ifndef _MOTO_CONFIG_H_
#define _MOTO_CONFIG_H_

#define MOTOR_POLE_PAIRS       (7.0f)
#define MOTOR_R                (0.11247408f) /* 相电阻，ohm */
#define MOTOR_LD               (0.00002235f) /* d 轴电感，H */
#define MOTOR_LQ               (0.00002567f) /* q 轴电感，H */
#define MOTOR_L                (0.00002401f) /* 平均电感，H；VESC 观测器使用 */
#define MOTOR_FLUX_LINKAGE     (0.00103001f) /* 永磁磁链，Wb */


#define FOC_OBSERVER_GAMMA         (1000000.0f)  //观测器增益
#define FOC_DT         (0.00005f)   //速度环dt
#define SPE_DT          (0.001f)

#define FOC_ALIGN_VD 0.4f /* V: open-loop alignment voltage */
#define FOC_ALIGN_TIME_MS 1000U /* 定位保持时间，须小于 65536 ms。 */
#define FOC_VF_UPDATE_HZ 20000.0f /* Actual ADC1 angle-update rate; verify on board. */
#define FOC_SPEED_PI_KP 0.005f /* A/rpm: initial speed-loop value */
#define FOC_SPEED_PI_KI 0.0005f /* A/(rpm * 1 ms): initial value */
#define FOC_SPEED_TD_WN 10.0f /* s^-1：临界阻尼 TD，从静止给定阶跃到 98% 约 0.58 s */

#define VF_TAR_RPM    400.0f /* 目标转速 */
#define VF_OVER_TIME    4.0f



#define PLL_KP  1000.0f
#define PLL_KI  250000.0f

#endif
