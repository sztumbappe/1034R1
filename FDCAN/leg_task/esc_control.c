//
// esc_control.c
// ESC 电调控制模块 (TIM1 CH1 PWM)
// 负责上电油门行程校准 + 周期性启停控制
//

#include "esc_control.h"
#include "tim.h"
#include "cmsis_os2.h"

/* ======================== 电机启停标志 ======================== */
uint8_t motor_run_flag = 0;    /* 0=停机, 1=启动运行 */

/* ======================== 公共接口实现 ======================== */

/**
 * @brief ESC 电调上电校准 (阻塞约 1.2s)
 *        1. 启动 TIM1 PWM 通道
 *        2. 输出最大油门 (2000us) 让 ESC 记录上限
 *        3. 输出最小油门 (1000us) 让 ESC 记录下限
 *        4. 校准完成后 ESC 进入正常工作模式
 */
void esc_init(void)
{
    /* 启动 TIM1 PWM 通道 */
    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

    /* 油门行程校准: 先最大再最小, 只做一次 */
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, ESC_PWM_MAX);
    osDelay(200);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, ESC_PWM_MIN);
    osDelay(200);
}

/**
 * @brief ESC 电调周期更新
 *        根据 motor_run_flag 控制 PWM 输出占空比
 */
void esc_update(void)
{
    if (motor_run_flag == 1) {
        /* 启动状态: 固定 40% 油门上限 (1400us) */
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, ESC_PWM_40PCT);
    } else {
        /* 关闭状态: 最低油门停机 (1000us) */
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, ESC_PWM_MIN);
    }
}