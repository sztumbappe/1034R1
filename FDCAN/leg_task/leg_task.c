//
// leg_task.c
// 双臂独立控制: 灵足05云台(MIT) + 2006抬升(RM)
//
// 上电后: 2006→高400, 灵足→0° 同时运动
// 收到KFS信号后: 灵足切到正常角度
//
// 遥控器接口变量:
//   target_red   — 1号臂抬升高度 (1=高100, 2=高200, 3=高400, 4=高600)
//   target_blue  — 2号臂抬升高度 (1=高100, 2=高200, 3=高400, 4=高600)
//   arm_close[0] — 1号臂云台 (0=展开到前面, 1=收到后面)
//   arm_close[1] — 2号臂云台 (0=展开到前面, 1=收到后面)
//

#include "leg_task.h"
#include "3508_control.h"
#include "bsp_fdcan.h"
#include "ROBSTRIDE.h"
#include "relay.h"
#include "score_task.h"
#include "esc_control.h"
#include "cmsis_os2.h"
#include "math.h"
#include "tim.h"

/* ======================== 遥控器接口变量 ======================== */
volatile float target_red  = 4;   /* 1号臂抬升: 1=高100, 2=高200, 3=高400, 4=高600 */
volatile float target_blue = 4;   /* 2号臂抬升: 1=高100, 2=高200, 3=高400, 4=高600 */
static float s;
volatile float arm_init;

/* ======================== 外部变量 ======================== */
extern uint8_t raise_control_enable;
extern float raise_target_pos;
extern Motor_Pos_RobStrite_Info Pos_Info[4];

/* ======================== 1. 硬件初始化 ======================== */
static void leg_hw_init(void)
{
    /* ---- 全局状态重置: 防止上电乱动 ---- */
    arm_init = 0;
    arm_close[0] = 0;
    arm_close[1] = 0;
    target_red = 4;
    target_blue = 4;
    motor_run_flag = 0;
    raise_control_enable = 0;
    raise_target_pos = 0;
    CAN_CMD_RM(&hfdcan3, CAN_CHASSIS_ALL_ID, 0, 0, 0, 0);

    robstride_goto_reset();      // 重置灵足静态变量(防Debug复位残留)
    esc_init();                  // ESC 电调上电校准 (阻塞约1.2s)
    arm_PID_INIT();              // 2006 PID初始化
    osDelay(300);                // 等待灵足05电机就绪
    robstride_init();            // 使能双臂灵足05电机
    di3508_r2control_Begin();    // 等待2006通讯建立
    osDelay(100);
    s++;
    lift_init();                 // 记录双2006上电零点
}

/* ======================== 2. 抬升高度选择 ======================== */
static float get_lift_pos(volatile float *target, float pos0, float pos1, float pos2, float pos3)
{
    if (*target == 1) return pos0;  /* 高100 */
    if (*target == 2) return pos1;  /* 高200 */
    if (*target == 3) return pos2;  /* 高400 */
    return pos3 ;                    /* 默认高600 */
}

/* ======================== 3. 灵足控制 (阶段感知) ======================== */
/**
 * arm_gimbal_update() — 单臂灵足控制
 *   arm_init != 1 → 灵足归0° (上电初始化阶段)
 *   arm_init == 1 → 按arm_close控制展开/收回 (比赛阶段)
 */
static void arm_gimbal_update(uint8_t idx)
{
    /* R1进攻/ R2预备模式: 强制特殊角度, 跳过正常逻辑 */
    extern uint8_t atready_mode;
    if (atready_mode) {
        if (idx == 0)
            robstride_goto_target(0.0f, 0);     /* 1号灵足 → 0° */
        else
            robstride_goto_target(-3.14f, 1);   /* 2号灵足 → -180° (R1进攻) */
        return;
    }
    extern uint8_t r2ready_mode;
    if (r2ready_mode) {
        if (idx == 0)
            robstride_goto_target(0.0f, 0);     /* 1号灵足 → 0° */
        else
            robstride_goto_target(-1.57f, 1);   /* 2号灵足 → -90° (R2预备) */
        return;
    }

    if (arm_init == 1) {
        /* KFS已收到: 正常运动控制 */
        float target_angle;
        if (idx == 0) {
            if (arm_close[0] == 1 && target_red > 2)
                target_angle = ROBSTRIDE_RETRACT_ANGLE;
            else
                target_angle = ROBSTRIDE_TARGET_ANGLE_1;
        } else {
            if (arm_close[1] == 1 && target_blue > 2)
                target_angle = ROBSTRIDE_RETRACT_ANGLE_BLUE;
            else
                target_angle = ROBSTRIDE_TARGET_ANGLE_2;
        }
        robstride_goto_target(target_angle, idx);
    } else {
        /* KFS未收到: 低增益平滑归零自检 */
		float init_angle = (idx == 0) ? -0.1f : 0.1f;
		robstride_goto_init(init_angle, idx);

    }
}

/* ======================== 4. 抬升控制 ======================== */
static float lift_tgt_red;
static float lift_tgt_blue;

static void lift_control_update(void)
{
    lift_tgt_red  = get_lift_pos(&target_red,  LIFT_RED_POS0,  LIFT_RED_POS1,  LIFT_RED_POS2,  LIFT_RED_POS3);
    lift_tgt_blue = get_lift_pos(&target_blue, LIFT_BLUE_POS0, LIFT_BLUE_POS1, LIFT_BLUE_POS2, LIFT_BLUE_POS3);
    lift_set_target(0, lift_tgt_red);
    lift_set_target(1, lift_tgt_blue);
    lift_goto_target();
}

/* ======================== 主任务 ======================== */
float s1, first;
float rc = 0;

void leg_task(void *argument)
{
    UNUSED(argument);
    osDelay(500);
    /* ---- 上电硬件初始化 ---- */
    leg_hw_init();

    /* ---- 初始化目标: 2006→高600, 灵足→0° 同时开始 ---- */
    lift_tgt_red  = (float)LIFT_RED_POS3;   /* 高600 */
    lift_tgt_blue = (float)LIFT_BLUE_POS3;
    lift_set_target(0, lift_tgt_red);
    lift_set_target(1, lift_tgt_blue);

    /* ======================== 主循环 ======================== */
    for (;;)
    {
        /* 气泵控制 */
        if (s1 > 0.5f) {
            motor_run_flag = 1;
            relay_pickup_kfs(s1);	
            s1 = -1;
        } else if (s1 == 0) {
            motor_run_flag = 0;    
		  relay_vacuum_on(1);  /* 真空阀1 通电 → 断真空 */
        }

        /* 1. 更新双臂抬升 (2006) */
        lift_control_update();

        /* 2. 更新灵足 (收到KFS前→0°, 收到后→正常角度) */
        arm_gimbal_update(0);
        arm_gimbal_update(1);

        /* 3. 处理RC指令 (得分状态机) */
        score_update();

        /* 4. ESC 电调控制 */
        esc_update();

        lift_update_debug();
        osDelay(2);
    }
}