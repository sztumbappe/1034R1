//
// 3508_control.c
// 2006电机控制 (FDCAN3, RM协议)
//

#include "3508_control.h"
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include "fdcan.h"
#include "gpio.h"
#include "cmsis_os2.h"

/* ======================== 外部变量 ======================== */

extern motor_control control_3508_classic[5];

/* ======================== PID 实例 ======================== */

static struct arm_pid pid_var[4];

/* ======================== 电机运行状态 ======================== */

MotorRun motor_3508[4];

/* ======================== PID 核心实现 ======================== */

static void abs_limit(float *a, float ABS_MAX)
{
    if (*a > ABS_MAX)
    {
        *a = ABS_MAX;
    }
    if (*a < -ABS_MAX)
    {
        *a = -ABS_MAX;
    }
}

static void pid_param_init(
    struct arm_pid *pid,
    float input_max_err,
    float maxout,
    float integral_limit,
    float kp,
    float ki,
    float kd)
{
    pid->param.integral_limit = integral_limit;
    pid->param.max_out = maxout;
    pid->param.input_max_err = input_max_err;
    pid->param.p = kp;
    pid->param.i = ki;
    pid->param.d = kd;
}

static void pid_reset(struct arm_pid *pid, float kp, float ki, float kd)
{
    pid->param.p = kp;
    pid->param.i = ki;
    pid->param.d = kd;

    pid->pout = 0;
    pid->iout = 0;
    pid->dout = 0;
    pid->out = 0;
}

float arm_pid_calculate(struct arm_pid *pid, float get, float set)
{
    pid->get = get;
    pid->set = set;
    pid->err = set - get;

    // 死区判断
    if ((pid->param.input_max_err != 0) && (fabs(pid->err) < pid->param.input_max_err))
    {
        return 0;
    }

    pid->pout = pid->param.p * pid->err;
    pid->iout += pid->param.i * pid->err;
    pid->dout = pid->param.d * (pid->err - pid->last_err);

    abs_limit(&(pid->iout), pid->param.integral_limit);
    pid->out = pid->pout + pid->iout + pid->dout;
    abs_limit(&(pid->out), pid->param.max_out);
    pid->last_err = pid->err;

    if (pid->enable == 0)
    {
        pid->out = 0;
    }

    return pid->out;
}

void arm_pid_struct_init(
    struct arm_pid *pid,
    float input_max_err,
    float maxout,
    float integral_limit,
    float kp,
    float ki,
    float kd)
{
    pid->enable = 1;
    pid->f_param_init = pid_param_init;
    pid->f_pid_reset = pid_reset;

    pid->f_param_init(pid, input_max_err, maxout, integral_limit, kp, ki, kd);
    pid->f_pid_reset(pid, kp, ki, kd);
}

/* ======================== 位置梯形速度规划 ======================== */

float Motor_SetPositionProfile(motor_control *MOTOR, MotorRun *motor,
                               float target_pos, float max_spd,
                               float accel, float decel,
                               float deadband, float dead_spd)
{
    if (decel < 0) {
        decel = accel;
    }

    // 计算时间间隔
    uint32_t now = HAL_GetTick();
    if (motor->last_timestamp == 0) {
        motor->last_timestamp = now;
    }
    float dt = (now - motor->last_timestamp) / 1000.0f;
    motor->last_timestamp = now;

    if (dt <= 0.0f) {
        dt = 0.001f;
    }

    // 获取当前位置及误差
    float current_pos = GetTotalPosition(MOTOR);
    float pos_error = target_pos - current_pos;
    float dist = fabsf(pos_error);
    float dir = (pos_error > 0.0f) ? 1.0f : -1.0f;

    // 减速限制速度 (基于剩余距离)
    float v_dec_limit = sqrtf(decel * dist *2.0f);

    // 加速限制速度 (基于当前速度)
    float v_acc_limit = fabsf(motor->last_cmd_spd) + accel * dt;

    // 初步目标速度
    float target_vel = max_spd;
    if (target_vel > v_dec_limit) target_vel = v_dec_limit;
    if (target_vel > v_acc_limit) target_vel = v_acc_limit;

    // 方向处理
    if ((motor->last_cmd_spd * dir) < 0.0f) {
        target_vel = fabsf(motor->last_cmd_spd) - accel * dt;
        if (target_vel < 0.0f) {
            target_vel = 0.0f;
        }
        target_vel = (motor->last_cmd_spd > 0.0f) ? target_vel : -target_vel;
    } else {
        target_vel *= dir;
    }

    // 死区处理
    if (dist < deadband && fabsf(target_vel) < dead_spd) {
        target_vel = 0.0f;
    }

    motor->last_cmd_spd = target_vel;
    return target_vel;
}

/* ======================== 电机控制初始化 ======================== */

void arm_PID_INIT(void)
{
    // 2006速度环PID
    for (int i = 0; i < 4; i++) {
        arm_pid_struct_init(&pid_var[i],
            0,        // 死区
            15000,    // 最大输出（配合高速抬升）
            8000,     // 积分限幅
            15,       // KP
            0.5f,     // KI
            0);       // KD
    }
}

/* 等待2个2006电机通讯建立 = 等24V电源上电 (灵足和2006共用24V) */
/* 判据: 两个2006都收到过CAN反馈 (ecd非零) */
void di3508_r2control_Begin(void)
{
    while (1) {  /* 死等直到24V上电 (2006数据到达) */
        /* 持续发送零电流心跳, 保持通讯 */
        CAN_CMD_RM(&hfdcan3, CAN_CHASSIS_ALL_ID, 0, 0, 0, 0);
        osDelay(10);
        /* 判据: 两个2006都收到过CAN反馈 (ecd非零) */
        if (control_3508_classic[0].chassis_3508_motor.ecd != 0 &&
            control_3508_classic[1].chassis_3508_motor.ecd != 0)
        {
            break;
        }
    }
}

/* ======================== 基础控制函数 ======================== */

float GetTotalPosition(motor_control *motor)
{
    return 0.044f * (float)motor->total_ecd;
}

void SetSpeed(motor_control *ptr, float speed)
{
    float Pidcur = arm_pid_calculate(&pid_var[ptr->chassis_3508_motor.id - 1],
                                     ptr->chassis_3508_motor.speed_rpm, speed);
    ptr->target_speed = (int16_t)Pidcur;
}

/* ======================== 2006堵转保护 ======================== */
/* 判据: PID输出大(想动) + 实际转速接近0(动不了) 持续超时 → 限流 */
/* 不切断电机, 而是降低输出力矩; 障碍清除后自动恢复全出力 */

#define STALL_PID_THRESHOLD   3000.0f   /* PID输出超过此值认为在"努力驱动" */
#define STALL_SPEED_THRESHOLD  100.0f   /* 实际转速低于此值认为"没动" (rpm) */
#define STALL_TIMEOUT_TICKS      500    /* 持续500ms判定堵转 */
#define STALL_SAFE_CURRENT     5000     /* 堵转时限流为5000 (正常最大15000) */

static struct {
    uint32_t tick;      /* 开始检测的时刻 */
    uint8_t  active;    /* 1=已进入限流模式 */
} stall[2];

void lift_stall_clear(uint8_t idx)
{
    if (idx > 1) return;
    stall[idx].active = 0;
    stall[idx].tick = 0;
}

uint8_t lift_is_stalled(uint8_t idx)
{
    if (idx > 1) return 0;
    return stall[idx].active;
}

void lift_stall_check(void)
{
    for (int i = 0; i < 2; i++) {
        motor_control *m = &control_3508_classic[i];
        float pid_out = (float)abs(m->target_speed);               /* PID 指令输出 */
        float actual  = (float)abs(m->chassis_3508_motor.speed_rpm); /* 实际转速 */

        if (stall[i].active) {
            /* 限流模式: 检测是否已解除堵转 */
            if (actual > STALL_SPEED_THRESHOLD || pid_out < STALL_PID_THRESHOLD / 2) {
                stall[i].active = 0;  /* 自动恢复全出力 */
                stall[i].tick = 0;
            }
            continue;
        }

        /* 正常模式: 检测是否进入堵转 */
        if (pid_out > STALL_PID_THRESHOLD && actual < STALL_SPEED_THRESHOLD) {
            uint32_t now = osKernelGetTickCount();
            if (stall[i].tick == 0) {
                stall[i].tick = now;
            } else if (now - stall[i].tick >= STALL_TIMEOUT_TICKS) {
                stall[i].active = 1;  /* 进入限流模式 */
            }
        } else {
            stall[i].tick = 0;  /* 正常运行, 重置计时 */
        }
    }
}

/* ======================== 2006抬升控制 ======================== */

static float lift_origin[2] = {0.0f, 0.0f};      /* 上电时的位置 */
static float lift_target[2] = {0.0f, 0.0f};      /* 目标位置(缩放后) */

/* 调试变量: Keil Watch中查看相对上电位置的编码器偏移 */
int32_t lift_debug_offset[2] = {0, 0};

void lift_update_debug(void)
{
    lift_debug_offset[0] = control_3508_classic[0].total_ecd - (int32_t)(lift_origin[0] / 0.044f);
    lift_debug_offset[1] = control_3508_classic[1].total_ecd - (int32_t)(lift_origin[1] / 0.044f);
}

void lift_init(void)
{
    /* 记录上电位置作为零点 */
    lift_origin[0] = GetTotalPosition(&control_3508_classic[0]);
    lift_origin[1] = GetTotalPosition(&control_3508_classic[1]);
    /* 默认目标 = 当前位置(不动) */
    lift_target[0] = lift_origin[0];
    lift_target[1] = lift_origin[1];
}

void lift_set_target(uint8_t idx, float target)
{
    if (idx > 1) return;
    /* 目标实际改变时 → 自动解除堵转保护 */
    if (lift_target[idx] != lift_origin[idx] + 0.044f * target) {
        lift_stall_clear(idx);
    }
    /* target 是编码器偏移值, 换算成缩放位置 */
    lift_target[idx] = lift_origin[idx] + 0.044f * target;
}

void lift_goto_target(void)
{
    const float MAX_SPD = 15000.0f;   // 最大速度（加快抬升）
    const float ACCEL   = 25000.0f;   // 加速度

    for (int i = 0; i < 2; i++)
    {
        /* 堵转保护: 限流不切断, 保留驱动力 */
        if (lift_is_stalled(i)) {
            if      (control_3508_classic[i].target_speed >  STALL_SAFE_CURRENT)
                control_3508_classic[i].target_speed =  STALL_SAFE_CURRENT;
            else if (control_3508_classic[i].target_speed < -STALL_SAFE_CURRENT)
                control_3508_classic[i].target_speed = -STALL_SAFE_CURRENT;
        }

        float spd = Motor_SetPositionProfile(
            &control_3508_classic[i], &motor_3508[i],
            lift_target[i], MAX_SPD, ACCEL, ACCEL, 10.0f, 3000.0f);
        SetSpeed(&control_3508_classic[i], spd);
    }

    CAN_CMD_RM(&hfdcan3, CAN_CHASSIS_ALL_ID,
               control_3508_classic[0].target_speed,
               control_3508_classic[1].target_speed,
               0, 0);
}

/* ======================== 到位检测 ======================== */

uint8_t lift_arrived(uint8_t idx)
{
    if (idx > 1) return 0;
    float current = GetTotalPosition(&control_3508_classic[idx]);
    float dist = fabsf(lift_target[idx] - current);
    return (dist < 10.0f) ? 1 : 0;
}

/* ======================== 气缸控制 ======================== */

void cylinder_open(void)
{
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_SET);
}

void cylinder_close(void)
{
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_RESET);
}