#ifndef __3508_CONTROL_H
#define __3508_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "bsp_fdcan.h"

/* ======================== PID 结构体 (arm前缀避免与raise_task冲突) ======================== */

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
#define CURRUNTMAX_ARM  10000


/* ======================== 机械臂位置数据 (arm_data) ======================== */

/* 粉色(Red)机械臂 - 抬升电机 (id=1) */
#define RedArmDownDic         (-1000)
#define RedArmMiddleDic       (-360.0f*2.5f*19.0f*0.5f)
#define RedArmPutDic          (-360.0f*2.5f*19.0f*0.7f)
#define RedArmTopDic          (-360.0f*2.5f*19.0f)

/* 蓝色(Blue)机械臂 - 抬升电机 (id=3) */
#define BlueArmDownDic         (0.0f)
#define BlueArmMiddleDic       (360.0f*2.5f*19.0f*0.5f)
#define BlueArmPutDic          (360.0f*2.5f*19.0f*0.7f)
#define BlueArmTopDic          (360.0f*2.5f*19.0f)

/* 粉色机械臂 - 云台电机 (id=0, 对应0x201) */
#define RedMiddleDic        (360.0f*19.0f*0.8f)
#define RedVerticalDic      (360.0f*19.0f*1.55f)
#define RedOutsideDic       (360.0f*19.0f*1.7f)
#define RedInsideDic        (360.0f*19.0f*0.7f)

/* 蓝色机械臂 - 云台电机 (id=2, 对应0x203) */
#define BlueMiddleDic      (-360.0f*19.0f*0.8f)
#define BlueVerticalDic    (-360.0f*19.0f*1.6f)
#define BlueOutsideDic     (-360.0f*19.0f*1.7f)
#define BlueInsideDic      (-360.0f*19.0f*0.7f)

/* ======================== PID函数 (arm前缀避免与raise_task冲突) ======================== */

void arm_pid_struct_init(struct arm_pid *pid, float input_max_err, float maxout,
                         float intergral_limit, float kp, float ki, float kd);
float arm_pid_calculate(struct arm_pid *pid, float get, float set);

/* ======================== 电机控制函数 ======================== */

/* PID初始化 */
void arm_PID_INIT(void);

/* 等待电机通讯建立 */
void di3508_r2control_Begin(void);

/* 电机零点校准 (arm前缀避免与raise_task冲突) */
void arm_di3508_r2control_init(void);

/* 抬升电机目标控制 (mode: 1~4 对应不同目标位置) */
void di3508_r2control_RunTarget(motor_control *ptr, MotorRun *motor,
                                uint8_t mode,
                                float target_dic1, float target_dic2,
                                float target_dic3, float target_dic4);

/* 云台电机目标控制 */
void di3508_r2control_gimbal(motor_control *ptr, MotorRun *motor,
                             uint8_t mode,
                             float target_dic1, float target_dic2,
                             float target_dic3, float target_dic4);

/* 云台电机限位查找 (阻塞) */
void di3508_r2control_gimbal_find(uint8_t mode);

/* 云台电机非阻塞限位查找 (单次迭代，适合与其它电机同时运行) */
void di3508_r2control_gimbal_find_nonblock(motor_control *MOTOR, float speed);

/* 获取电机总位置 */
float GetTotalPosition(motor_control *motor);

/* 设置电机速度 (速度环PID) */
void SetSpeed(motor_control *ptr, float speed);

/* 电机限位查找 */
void di3508_find_limitation(motor_control *MOTOR, float speed);

/* 位置梯形速度规划 */
float Motor_SetPositionProfile(motor_control *MOTOR, MotorRun *motor,
                               float target_pos, float max_spd,
                               float accel, float decel,
                               float deadband, float dead_spd);


/* ======================== 气缸控制 ======================== */

void cylinder_open(void);
void cylinder_close(void);

#endif /* __3508_CONTROL_H */