#ifndef __LEG_TASK_H
#define __LEG_TASK_H

#include <stdbool.h>

/* ======================== 比赛模式宏 ======================== */
#define MATCH_MODE_NORMAL  0   /* 正常模式 */
#define MATCH_MODE_PRELIM  1   /* 预选赛2红方 */
#define MATCH_MODE_BLUE   -1   /* 预选赛2蓝区(左右互换) */
#define MATCH_MODE  MATCH_MODE_PRELIM   /* 发布时改为 1 或 -1 */

/* ======================== 任务函数声明 ======================== */

/* 机械臂任务 */
void leg_task(void *argument);

/* ======================== 遥控器接口变量 ======================== */

extern volatile float target_red;    /* 1号臂抬升: 1=高100, 2=高200, 3=高400, 4=高600 */
extern volatile float target_blue;   /* 2号臂抬升: 1=高100, 2=高200, 3=高400, 4=高600 */
extern volatile float arm_init;      /* 启动信号: 设为1触发双臂动作 */

/* arm_close[0]=1号臂云台, arm_close[1]=2号臂云台 (0=展开到前面, 1=收到后面) */
/* 声明在 ROBSTRIDE.h: extern volatile float arm_close[2] */

/* ======================== 预选赛灵足预备标志 ======================== */
#if MATCH_MODE != MATCH_MODE_NORMAL
extern uint8_t prelim_prep_flag[2];   /* [0]=左臂 [1]=右臂: 1=强制预备角度 */
extern float   prelim_prep_angle[2];  /* [0]=左臂 [1]=右臂: 预备目标角度(rad) */
#endif

#endif /* __LEG_TASK_H */
