#include "raise_task.h"
#include "bsp_fdcan.h"
#include "fdcan.h"
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include "cmsis_os2.h"
#include "gpio.h"

#include "gpio.h"
#include "cmsis_os2.h"


#define LIMIT_PIN   GPIO_PIN_0
#define LIMIT_PORT  GPIOG

#define SW_LPF_ALPHA    0.10f   // 数值越大越稳、响应越慢
float sw_filter_val = 0.0f;
uint8_t limit_state = 0;    // 0未触发 1限位触发

//限位开关读取+滤波处理
void Limit_Switch_GetState(void)
{
    // 读取原始电平
    uint8_t raw_level = HAL_GPIO_ReadPin(LIMIT_PORT, LIMIT_PIN);
    
    // 一阶低通滤波 消机械抖动
    sw_filter_val = SW_LPF_ALPHA * sw_filter_val + (1 - SW_LPF_ALPHA) * raw_level;

    // 阈值判定
    if(sw_filter_val > 0.5f)
    {
        limit_state = 1;
    }
    else
    {
        limit_state = 0;
    }
}



/* 全局变量定义 */
struct pid pid_var1;
struct pid pid_dic;
double Pidcur = 0;
double Pidvar = 0;
int32_t MOTOR_STECD = 0;
extern int32_t total_ecd;
bool speed_flag = true; // 电机限位检测标志
extern motor_measure_t chassis_3508_motor[8];
/************************ PID核心实现 ************************/
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
    struct pid *pid,
    float deadband,
    float maxout,
    float integral_limit,
    float kp,
    float ki,
    float kd)
{
    pid->param.integral_limit = integral_limit;
    pid->param.max_out = maxout;
    pid->param.p = kp;
    pid->param.i = ki;
    pid->param.d = kd;
}

static void pid_reset(struct pid *pid, float kp, float ki, float kd)
{
    pid->param.p = kp;
    pid->param.i = ki;
    pid->param.d = kd;
    pid->pout = 0;
    pid->iout = 0;
    pid->dout = 0;
    pid->out = 0;
}

float pid_calculate1(struct pid *pid, float get, float set)
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

    if (pid->enable == 0)
    {
        pid->out = 0;
    }

    pid->last_err = pid->err; // 补充：保存本次误差为下次微分用
    return pid->out;
}

void pid_struct_init(
    struct pid *pid,
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

/************************ 电机控制实现 ************************/
void PID_INIT(void)
{
    // 速度环PID初始化
    pid_struct_init(&pid_var1,
                    0,          // 死区
                    8000,       // 最大输出
                    6000,       // 积分限幅
                    9,          // KP
                    0.3f,       // KI
                    0);         // KD

    // 位置环PID初始化
    pid_struct_init(&pid_dic,
                    150,        // 死区
                    9000,       // 最大输出
                    9000,       // 积分限幅
                    0.2f,       // KP
                    0,          // KI
                    0.15f);     // KD
}

void dj3508_r2_begin(void)
{
    // 等待电机通讯建立（温度值非0表示通讯正常）
    while (chassis_3508_motor[2].temperate == 0)
    {
        CAN_CMD_RM(&hfdcan3, CAN_CHASSIS_ALL_ID, 0, 0, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(10)); // FreeRTOS延时（替代HAL_Delay）
    }
}

void di3508_r2control_init(void)
{
    // 电机零点校准：下降到机械限位，记录初始位置
    speed_flag = true;
    while (speed_flag == true)
    {
        Pidcur = pid_calculate1(&pid_var1, (float)chassis_3508_motor[2].speed_rpm, -500);
        if (fabs(Pidcur) >= CURRUNTMAX)
        {
            speed_flag = false;
            MOTOR_STECD = total_ecd;
            CAN_CMD_RM(&hfdcan3, CAN_CHASSIS_ALL_ID, 0, 0, 0, 0);
            break;
        }
        else
        {
            CAN_CMD_RM(&hfdcan3, CAN_CHASSIS_ALL_ID, 0, 0, (int)Pidcur, 0);
            vTaskDelay(pdMS_TO_TICKS(10)); // FreeRTOS延时
        }
    }
}

void di3508_r2control(void)
{
    // 电机位置闭环控制（向上转动）
    Pidvar = pid_calculate1(&pid_dic, (float)total_ecd, AIMDIC + (float)MOTOR_STECD);
    Pidcur = pid_calculate1(&pid_var1, (float)chassis_3508_motor[2].speed_rpm, Pidvar);
    CAN_CMD_RM(&hfdcan3, CAN_CHASSIS_ALL_ID, 0, 0, (int)Pidcur, 0);
}

/************************ FreeRTOS任务函数 ************************/
float raising=0;
float b;
void Raise_task(void *argument)
{
    UNUSED(argument);

//    // 初始化流程
//    PID_INIT();                          // PID参数初始化
//    dj3508_r2_begin();                   // 等待电机通讯建立
//   // di3508_r2control_init();             // 电机零点校准
//   	osDelay(4000);                       // 校准后延时

    // 任务主循环
    for (;;)
    {
		    // 读取限位
 Limit_Switch_GetState ();
    di3508_r2control();      // 电机位置控制
		 osDelay(10);
	  }
}