#ifndef __LEG_TASK_H
#define __LEG_TASK_H

#include <stdbool.h>

/* ======================== 任务函数声明 ======================== */

/* 机械臂任务 (FreeRTOS任务入口) */
void leg_task(void *argument);

/* ======================== 内部功能函数 ======================== */

/* 机械臂初始化 (多阶段) */
void DoubleArmInit(void);

/* 任务状态机 (自动运行) */
void task(void);

#endif /* __LEG_TASK_H */
