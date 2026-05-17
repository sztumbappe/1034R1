//
// 3508_control.c
// 机械臂3508电机控制 - 从2026r1hand项目移植
// 适配FDCAN项目: 使用bsp_fdcan.h, FDCAN3总线
//

#include "3508_control.h"
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include "fdcan.h"
#include "gpio.h"
#include "cmsis_os2.h"

/* ======================== 外部变量 ======================== */

// 引入 bsp_fdcan.c 中定义的电机数据数组
extern motor_control control_3508_classic[5];

/* ======================== PID 实例 ======================== */

// 4个PID实例: [0]~[3] 对应4个电机(云台+抬升)
static struct arm_pid pid_var[4];

/* ======================== 电机运行状态 ======================== */

MotorRun motor_3508[4];

/* ======================== 静态变量 ======================== */

static bool limiation_flag = true;
static int32_t ZERO_ECD3 = 0;
static int32_t ZERO_ECD1 = 0;

/* 云台标志 */
bool yuntai_flag_take = false;
bool hand_raise = false;

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
    float v_dec_limit = sqrtf(decel * dist / 3.0f);

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
    // 云台电机 3508 (id=0, id=2): 减速比19, 惯量较大
    // 抬升电机 2006 (id=1, id=3): 减速比36, 响应更快但扭矩较小
    for (int i = 0; i < 4; i++) {
        if (i == 1 || i == 3) {
            // 抬升电机 2006: 降低输出限幅和KP (扭矩较小)
            arm_pid_struct_init(&pid_var[i],
                0,        // 死区
                6000,     // 最大输出 (2006扭矩小, 降低限幅)
                4000,     // 积分限幅
                6,        // KP (2006惯量小, 降低比例增益)
                0.2f,     // KI
                0);       // KD
        } else {
            // 云台电机 3508: 保持原参数
            arm_pid_struct_init(&pid_var[i],
                0,        // 死区
                8000,     // 最大输出
                6000,     // 积分限幅
                9,        // KP
                0.3f,     // KI
                0);       // KD
        }
    }
}

/* 等待4个电机通讯建立 (FDCAN3总线) */
void di3508_r2control_Begin(void)
{
    while (control_3508_classic[0].chassis_3508_motor.temperate == 0 ||
           control_3508_classic[1].chassis_3508_motor.temperate == 0 ||
           control_3508_classic[2].chassis_3508_motor.temperate == 0 ||
           control_3508_classic[3].chassis_3508_motor.temperate == 0)
    {
        CAN_CMD_RM(&hfdcan3, CAN_CHASSIS_ALL_ID, 0, 0, 0, 0);
        osDelay(10);
    }
}

/* 电机零点校准 */
void arm_di3508_r2control_init(void)
{
    while (control_3508_classic[3].ZERO_FLAG == false)
    {
        osDelay(1000);
    }
    if (control_3508_classic[1].ZERO_FLAG == true)
    {
        control_3508_classic[1].ZERO_ecp = GetTotalPosition(&control_3508_classic[1]);
    }
    if (control_3508_classic[3].ZERO_FLAG == true)
    {
        control_3508_classic[3].ZERO_ecp = GetTotalPosition(&control_3508_classic[3]);
    }
}

/* ======================== 抬升电机目标控制 (2006电机, 减速比36) ======================== */

void di3508_r2control_RunTarget(motor_control *ptr, MotorRun *motor,
                                uint8_t mode,
                                float target_dic1, float target_dic2,
                                float target_dic3, float target_dic4)
{
    float Pidcaculatspd = 0.0f;

    // 2006电机最大速度限幅 6000rpm (C610电调)
    const float RUN_MAX_SPD = 6000.0f;
    const float RUN_ACCEL   = 6000.0f;

    switch (mode)
    {
        case 1:
            Pidcaculatspd = Motor_SetPositionProfile(ptr, motor,
                target_dic1 + 0.044f * (float)(ptr->ZERO_ecp) + 1000.0f,
                RUN_MAX_SPD, RUN_ACCEL, RUN_ACCEL, 10, 25);
            if (Pidcaculatspd == 0)
            {
                ptr->flag_num++;
                if (ptr->flag_num == 25)
                {
                    ptr->flag_num = 0;
                    ptr->stage = STAGE_AT_P1;
                }
            }
            else
            {
                ptr->flag_num = 0;
            }
            SetSpeed(ptr, Pidcaculatspd);
            break;
        case 2:
            Pidcaculatspd = Motor_SetPositionProfile(ptr, motor,
                target_dic2 + 0.044f * (float)(ptr->ZERO_ecp),
                RUN_MAX_SPD, RUN_ACCEL, RUN_ACCEL, 10, 25);
            if (Pidcaculatspd == 0)
            {
                ptr->flag_num++;
                if (ptr->flag_num == 25)
                {
                    ptr->flag_num = 0;
                    ptr->stage = STAGE_AT_P2;
                }
            }
            else
            {
                ptr->flag_num = 0;
            }
            SetSpeed(ptr, Pidcaculatspd);
            break;
        case 3:
            Pidcaculatspd = Motor_SetPositionProfile(ptr, motor,
                target_dic3 + 0.044f * (float)(ptr->ZERO_ecp),
                RUN_MAX_SPD, RUN_ACCEL, RUN_ACCEL, 10, 25);
            if (Pidcaculatspd == 0)
            {
                ptr->flag_num++;
                if (ptr->flag_num == 25)
                {
                    ptr->flag_num = 0;
                    ptr->stage = STAGE_AT_P3;
                }
            }
            else
            {
                ptr->flag_num = 0;
            }
            SetSpeed(ptr, Pidcaculatspd);
            break;
        case 4:
            Pidcaculatspd = Motor_SetPositionProfile(ptr, motor,
                target_dic4 + 0.044f * (float)(ptr->ZERO_ecp),
                RUN_MAX_SPD, RUN_ACCEL, RUN_ACCEL, 10, 25);
            if (Pidcaculatspd == 0)
            {
                ptr->flag_num++;
                if (ptr->flag_num == 25)
                {
                    ptr->flag_num = 0;
                    ptr->stage = STAGE_AT_P4;
                }
            }
            else
            {
                ptr->flag_num = 0;
            }
            SetSpeed(ptr, Pidcaculatspd);
            break;
        default:
            break;
    }
}

/* ======================== 云台电机目标控制 ======================== */

void di3508_r2control_gimbal(motor_control *ptr, MotorRun *motor,
                             uint8_t mode,
                             float target_dic1, float target_dic2,
                             float target_dic3, float target_dic4)
{
    float Pidcaculatspd = 0.0f;

    switch (mode)
    {
        case 1:
            Pidcaculatspd = Motor_SetPositionProfile(ptr, motor,
                0.044f * (float)ptr->ZERO_ecp + target_dic1,
                8000, 8000, 8000, 10, 25);
            if (Pidcaculatspd == 0)
            {
                ptr->flag_num++;
                if (ptr->flag_num == 25)
                {
                    ptr->flag_num = 0;
                    ptr->stage = STAGE_AT_P1;
                }
            }
            else
            {
                ptr->flag_num = 0;
            }
            SetSpeed(ptr, Pidcaculatspd);
            break;
        case 2:
            Pidcaculatspd = Motor_SetPositionProfile(ptr, motor,
                0.044f * (float)ptr->ZERO_ecp + target_dic2,
                8000, 8000, 8000, 10, 25);
            if (Pidcaculatspd == 0)
            {
                ptr->flag_num++;
                if (ptr->flag_num == 25)
                {
                    ptr->flag_num = 0;
                    ptr->stage = STAGE_AT_P2;
                }
            }
            else
            {
                ptr->flag_num = 0;
            }
            SetSpeed(ptr, Pidcaculatspd);
            break;
        case 3:
            Pidcaculatspd = Motor_SetPositionProfile(ptr, motor,
                0.044f * (float)ptr->ZERO_ecp + target_dic3,
                8000, 8000, 8000, 10, 25);
            if (Pidcaculatspd == 0)
            {
                ptr->flag_num++;
                if (ptr->flag_num == 25)
                {
                    ptr->flag_num = 0;
                    ptr->stage = STAGE_AT_P3;
                }
            }
            else
            {
                ptr->flag_num = 0;
            }
            SetSpeed(ptr, Pidcaculatspd);
            break;
        case 4:
            Pidcaculatspd = Motor_SetPositionProfile(ptr, motor,
                0.044f * (float)ptr->ZERO_ecp + target_dic4,
                8000, 8000, 8000, 10, 25);
            if (Pidcaculatspd == 0)
            {
                ptr->flag_num++;
                if (ptr->flag_num == 25)
                {
                    ptr->flag_num = 0;
                    ptr->stage = STAGE_AT_P4;
                }
            }
            else
            {
                ptr->flag_num = 0;
            }
            SetSpeed(ptr, Pidcaculatspd);
            break;
        default:
            break;
    }
}

/* ======================== 云台限位查找 ======================== */

void di3508_r2control_gimbal_find(uint8_t mode)
{
    switch (mode)
    {
        case 1:
            di3508_find_limitation(&control_3508_classic[0], -1000.0f);
            break;
        case 2:
            di3508_find_limitation(&control_3508_classic[2], 1000.0f);
            break;
        default:
            break;
    }
}

void di3508_find_limitation(motor_control *MOTOR, float speed)
{
    float Pidcur = 0.0f;
    while (MOTOR->ZERO_FLAG == false)
    {
        Pidcur = arm_pid_calculate(&pid_var[MOTOR->chassis_3508_motor.id - 1],
                                   MOTOR->chassis_3508_motor.speed_rpm, speed);
        MOTOR->target_speed = (int16_t)Pidcur;
        if (fabsf(Pidcur) >= CURRUNTMAX_ARM)
        {
            MOTOR->ZERO_FLAG = true;
            MOTOR->ZERO_ecp = (float)MOTOR->total_ecd;
            CAN_CMD_RM(&hfdcan3, CAN_CHASSIS_ALL_ID, 0, 0, 0, 0);
            break;
        }
        CAN_CMD_RM(&hfdcan3, CAN_CHASSIS_ALL_ID,
                    control_3508_classic[0].target_speed,
                    control_3508_classic[1].target_speed,
                    control_3508_classic[2].target_speed,
                    control_3508_classic[3].target_speed);
        osDelay(10);
    }
}

/**
 * @brief 云台电机非阻塞限位查找 (单次迭代)
 * @note  每次调用执行一步PID计算，不阻塞。由外部循环反复调用。
 *        找到零点后设置 ZERO_FLAG=true 并停止电机。
 * @param MOTOR  电机控制结构体指针
 * @param speed  查找速度 (正/负决定方向)
 */
void di3508_r2control_gimbal_find_nonblock(motor_control *MOTOR, float speed)
{
    if (MOTOR->ZERO_FLAG == true)
    {
        return;  // 已找到零点，无需操作
    }

    float Pidcur = arm_pid_calculate(&pid_var[MOTOR->chassis_3508_motor.id - 1],
                                     MOTOR->chassis_3508_motor.speed_rpm, speed);
    MOTOR->target_speed = (int16_t)Pidcur;

    if (fabsf(Pidcur) >= CURRUNTMAX_ARM)
    {
        MOTOR->ZERO_FLAG = true;
        MOTOR->ZERO_ecp = (float)MOTOR->total_ecd;
        MOTOR->target_speed = 0;
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

/* ======================== 气缸控制 ======================== */

void cylinder_open(void)
{
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_SET);
}

void cylinder_close(void)
{
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_RESET);
}