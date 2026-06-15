#ifndef __LEG_TASK_H
#define __LEG_TASK_H

#include <stdbool.h>

/* ======================== 任务函数声明 ======================== */

/* 机械臂任务 */
void leg_task(void *argument);

/* ======================== 遥控器接口变量 ======================== */

extern volatile float target_red;    /* 1号臂抬升: 2=高200, 3=高400, 4=高600 */
extern volatile float target_blue;   /* 2号臂抬升: 2=高200, 3=高400, 4=高600 */
extern volatile float arm_init;      /* 启动信号: 设为1触发双臂动作 */

/* arm_close[0]=1号臂云台, arm_close[1]=2号臂云台 (0=展开到前面, 1=收到后面) */
/* 声明在 ROBSTRIDE.h: extern volatile float arm_close[2] */

#endif /* __LEG_TASK_H */
