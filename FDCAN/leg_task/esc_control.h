#ifndef __ESC_CONTROL_H
#define __ESC_CONTROL_H

#include <stdint.h>

/* ======================== ESC 油门值 (TIM1 PWM Compare) ======================== */
#define ESC_PWM_MIN      100    /* 停机 1000us */
#define ESC_PWM_MAX      200    /* 校准用 2000us */
#define ESC_PWM_40PCT    130    /* 40%油门 1400us */

/* ======================== 外部变量 ======================== */
extern uint8_t motor_run_flag;   /* 0=停机, 1=运行 */

/* ======================== 外部接口 ======================== */

/**
 * @brief ESC 电调上电校准 (阻塞约 1.2s)
 *        先输出最大油门行程, 再降到最小, 让 ESC 记录行程范围
 * @note  必须在 MX_TIM1_Init() 之后调用
 */
void esc_init(void);

/**
 * @brief ESC 电调周期更新
 *        根据 motor_run_flag 控制 PWM 输出
 *        motor_run_flag=1 → 40%油门, =0 → 停机
 * @note  在主循环中调用, 建议 2~10ms 间隔
 */
void esc_update(void);

#endif /* __ESC_CONTROL_H */