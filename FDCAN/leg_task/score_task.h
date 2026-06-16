#ifndef __SCORE_TASK_H
#define __SCORE_TASK_H

#include "main.h"
#include <stdint.h>

/* ======================== 雷达→高度映射 ======================== */
typedef enum {
    LIFT_H100 = 1,
    LIFT_H200 = 2,
    LIFT_H400 = 3,
    LIFT_H600 = 4,
} lift_height_t;

/* ======================== 接口函数 ======================== */

/**
 * @brief 得分逻辑更新函数 (在 leg_task 主循环中调用)
 *        轮询 rc_has_new_cmd(), 根据 ATAKE/BTAKE 指令执行取料/存料流程
 * @note  非阻塞, 每次调用推进状态机一步
 */
void score_update(void);

#endif /* __SCORE_TASK_H */