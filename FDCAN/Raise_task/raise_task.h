//
// Created by zasxn on 2026/1/23.
//

#ifndef R2_3508_RAISE_TASK_H
#define R2_3508_RAISE_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "stdint.h"

/* 电机参数定义 */
#define CURRUNTMAX      5000
#define AIMDIC        4562067.0f    //3508电机转29圈
//#define AIMDIC          3146253.0f    //3508电机转20圈   
#define AimVar          1000

/* PID 结构体前置声明 */
struct pid;

/* 外部变量声明 */
extern int32_t total_ecd;
extern int32_t CycleNum;

/* 遥控器触发控制变量 */
extern uint8_t raise_init_trigger;    // 初始化触发（1=触发初始化+校准，0=无操作）
extern uint8_t raise_init_done;       // 初始化完成标志（0=未完成，1=已完成）
extern uint8_t raise_control_enable;  // 电机控制使能（1=使能位置控制，0=停止电机）

/* 全局PID实例 */
extern struct pid pid_var1;
extern struct pid pid_dic;
struct pid_param {
    float deadband;
    float p, i, d;
    float input_max_err;    // 最大输入误差
    float max_out;          // 输出限幅值
    float integral_limit;   // 积分项限幅值
};

struct pid
{
    struct pid_param param; // PID参数
    uint8_t enable;         // 使能标志（1启用，0禁用）
    float set;              // 设定值
    float get;              // 反馈值
    float err;              // 当前误差
    float last_err;         // 上一次误差
    float pout;             // 比例输出
    float iout;             // 积分输出
    float dout;             // 微分输出
    float out;              // 总输出
    // 函数指针（保留原逻辑）
    void (*f_param_init)(struct pid *pid,
                         float input_max_err,
                         float max_output,
                         float integral_limit,
                         float p,
                         float i,
                         float d);
    void (*f_pid_reset)(struct pid *pid, float p, float i, float d);
};

/* 任务相关声明 */
void Raise_task(void *argument);        // FreeRTOS任务函数
void PID_INIT(void);                    // PID初始化
void di3508_r2control_init(void);       // 电机零点校准
void di3508_r2control(void);            // 电机位置控制
void dj3508_r2_begin(void);             // 电机通讯检测

#endif //R2_3508_RAISE_TASK_H