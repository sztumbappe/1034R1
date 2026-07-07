#ifndef __3508_CONTROL_H
#define __3508_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "bsp_fdcan.h"

/* ======================== PID 结构体 ======================== */

struct arm_pid_param {
    float p, i, d;
    float input_max_err;    // 最大输入误差（死区）
    float max_out;          // 输出限幅值
    float integral_limit;   // 积分项限幅值
};

struct arm_pid {
    struct arm_pid_param param;
    uint8_t enable;         // 使能标志（1启用，0禁用）
    float set;              // 设定值（目标）
    float get;              // 反馈值（当前状态）
    float err;              // 当前误差
    float last_err;         // 上一次误差
    float pout;             // 比例输出
    float iout;             // 积分输出
    float dout;             // 微分输出
    float out;              // 总输出
    void (*f_param_init)(struct arm_pid *pid,
                         float input_max_err,
                         float max_output,
                         float integral_limit,
                         float p, float i, float d);
    void (*f_pid_reset)(struct arm_pid *pid, float p, float i, float d);
};

/* ======================== 电机运行状态 ======================== */

typedef struct {
    uint32_t last_timestamp;   // 上次调用时间戳 (ms)
    float last_cmd_spd;        // 上次指令速度
} MotorRun;

/* ======================== 电机电流限制 ======================== */
#define CURRUNTMAX_ARM  12000

/* ======================== 2006位置定义 (相对上电位置的偏移) ======================== */
/* 上电后手动转到目标位置，用Keil Watch读 total_ecd 填入下方值 */

/* 1号臂 2006 (control_3508_classic[0]) */
#define LIFT_RED_POS0       (894114)        /* 高100 */
#define LIFT_RED_POS1       (298515)        /* 高200 */
#define LIFT_RED_POS2       (-442819)       /* 高400 */
#define LIFT_RED_POS3       (-1035026)      /* 高600 */

/* 2号臂 2006 (control_3508_classic[1]) */
#define LIFT_BLUE_POS0      (-881435)        /* 高100*/
#if MATCH_MODE == MATCH_MODE_PRELIM3
#define LIFT_BLUE_POS1      (-110926)        /* 高200 崇武探幽预选赛3偏上(防打到存杆) */
#else
#define LIFT_BLUE_POS1      (-150300)        /* 高200 正常 */
#endif
#define LIFT_BLUE_POS2      (430772)       /* 高400 */
#define LIFT_BLUE_POS3      (1032170)      /* 高600 */

/* ======================== PID函数 ======================== */

void arm_pid_struct_init(struct arm_pid *pid, float input_max_err, float maxout,
                         float intergral_limit, float kp, float ki, float kd);
float arm_pid_calculate(struct arm_pid *pid, float get, float set);

/* ======================== 电机控制函数 ======================== */

/* PID初始化 */
void arm_PID_INIT(void);

/* 等待电机通讯建立 */
void di3508_r2control_Begin(void);

/* 获取电机总位置 */
float GetTotalPosition(motor_control *motor);

/* 设置电机速度 (速度环PID) */
void SetSpeed(motor_control *ptr, float speed);

/* 2006抬升初始化 (记录上电零点) */
void lift_init(void);

/* 2006设置目标 (idx: 0=1号臂, 1=2号臂, target: 编码器偏移) */
void lift_set_target(uint8_t idx, float target);

/* 2006抬升PID闭环 (主循环每周期调用, 持续保持位置) */
void lift_goto_target(void);

/* 调试变量 (Keil Watch中查看) */
extern int32_t lift_debug_offset[2];    /* 相对上电位置的编码器偏移 */

/* 2006调试偏移更新 (主循环调用) */
void lift_update_debug(void);

/* 检测2006是否到达当前目标位置 (复用Motor_SetPositionProfile的deadband) */
uint8_t lift_arrived(uint8_t idx);

/* ======================== 2006堵转保护 ======================== */

/* 堵转检测: 每周期在主循环调用, 比较PID输出和实际转速 */
void lift_stall_check(void);

/* 清除堵转标记 (目标实际改变时自动调用) */
void lift_stall_clear(uint8_t idx);

/* 查询是否堵转 (1=堵转已切断) */
uint8_t lift_is_stalled(uint8_t idx);

/* ======================== 位置梯形速度规划 ======================== */
float Motor_SetPositionProfile(motor_control *MOTOR, MotorRun *motor,
                               float target_pos, float max_spd,
                               float accel, float decel,
                               float deadband, float dead_spd);

/* ======================== 气缸控制 ======================== */

void cylinder_open(void);
void cylinder_close(void);

#endif /* __3508_CONTROL_H */