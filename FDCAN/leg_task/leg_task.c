//
// leg_task.c
// 双臂独立控制: 灵足05云台(MIT) + 2006抬升(RM)
//
// 遥控器接口变量:
//   target_red   — 1号臂抬升高度 (2=高200, 3=高400, 4=高600)
//   target_blue  — 2号臂抬升高度 (2=高200, 3=高400, 4=高600)
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
    uint8_t rob_started_red  = 0;
    uint8_t rob_started_blue = 0;
/* ======================== 外部变量 ======================== */
extern uint8_t raise_control_enable;
extern float raise_target_pos;
extern Motor_Pos_RobStrite_Info Pos_Info[4];// 两个电机接收信息
/* ======================== 1. 硬件初始化 ======================== */
/**
 * leg_hw_init() — 上电硬件初始化 (不含等待信号)
 * PID初始化 → 灵足使能 → 2006通讯 → 记录零点
 */
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

	  esc_init();                  // ESC 电调上电校准 (阻塞约1.2s)
    arm_PID_INIT();              // 2006 PID初始化
	  osDelay(300);                // 等待灵足05电机就绪
    robstride_init();            // 使能双臂灵足05电机
	  di3508_r2control_Begin();    // 等待2006通讯建立
    osDelay(100);                // 等待灵足05电机就绪
    s++;
    lift_init();                 // 记录双2006上电零点
}

/* ======================== 2. 抬升高度选择 ======================== */
/**
 * get_lift_pos() — 根据 target 值返回对应高度的编码器偏移
 * target: 2=高200, 3=高400, 4=高600
 */
static float get_lift_pos(volatile float *target, float pos0, float pos1, float pos2, float pos3)
{
    if (*target == 1) return pos0;  /* 高100 */
    if (*target == 2) return pos1;  /* 高200 */
    if (*target == 3) return pos2;  /* 高400 */
    return pos3;                    /* 默认高600 */
}

/* ======================== 3. 检测2006到达1/5 ======================== */
/**
 * check_lift_reached() — 检测2006是否到达目标位置的1/5
 * @param idx: 电机索引 (0=1号臂, 1=2号臂)
 * @param lift_target: 当前抬升目标(编码器偏移)
 * @param threshold: 1/5阈值
 * @return 1=已到达, 0=未到达
 */
static uint8_t check_lift_reached(uint8_t idx, float lift_target, float threshold)
{
    float off = (float)lift_debug_offset[idx];
    if (lift_target < 0)
        return (off <= threshold) ? 1 : 0;
    else
        return (off >= threshold) ? 1 : 0;
}

/* ======================== 4. 单臂云台控制 ======================== */
/**
 * arm_gimbal_update() — 单臂: 检测2006到位 → 启动灵足 → 根据arm_close控制展开/收回
 * @param idx: 0=1号臂, 1=2号臂
 */
static void arm_gimbal_update(uint8_t idx, float lift_target, float threshold,
                               uint8_t *started)
{
    /* 检测2006到达1/5 → 启动灵足 */
    if (!(*started)) {
        if (check_lift_reached(idx, lift_target, threshold))
            *started = 1;
    }

    /* 无论是否启动，都发MIT指令保持灵足通信 */
    if (*started) {
        // 正常运动控制
        if (idx == 0) {
            if (arm_close[0] == 1 && target_red > 2)
                robstride_goto_target(ROBSTRIDE_RETRACT_ANGLE, 0);
            else
                robstride_goto_target(ROBSTRIDE_TARGET_ANGLE_1, 0);
        } else {
            if (arm_close[1] == 1 && target_blue > 2)
                robstride_goto_target(ROBSTRIDE_RETRACT_ANGLE_BLUE, 1);
            else
                robstride_goto_target(ROBSTRIDE_TARGET_ANGLE_2, 1);
        }
        } else {
        /* 2006还没到位：调用 keepalive 完成初始化+保持通信，原地不动 */
        robstride_keepalive(idx);
    }
}


/* ======================== 5. 抬升控制 ======================== */
/**
 * lift_control_update() — 更新双臂2006抬升目标并执行闭环
 */
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
float s1,first;
float rc=0;
void leg_task(void *argument)
{
    UNUSED(argument);
    osDelay(1000);
    /* ---- 上电硬件初始化 ---- */
    leg_hw_init();

    while (arm_init != 1) {
        lift_update_debug();
        score_update();      /* 等待期间处理RC指令 (KFS触发arm_init) */
        /* 发送保持力矩MIT指令，保持灵足通信不超时 */
        RobStrite_Motor_05_move_control(&Robstirde_Motor_05_hfdcan, 0,
            Pos_Info[1].Angle, 0, 100.0f, 4.5f, ROBSTRIDE_ID_ARM1);
        RobStrite_Motor_05_move_control(&Robstirde_Motor_05_hfdcan, 0,
            Pos_Info[2].Angle, 0, 100.0f, 4.5f, ROBSTRIDE_ID_ARM2);
        osDelay(50);
    }

    /* ---- 初始化抬升目标: 双臂高600 ---- */
    lift_tgt_red  = (float)LIFT_RED_POS3;
    lift_tgt_blue = (float)LIFT_BLUE_POS3;
    lift_set_target(0, lift_tgt_red);
    lift_set_target(1, lift_tgt_blue);

    /* ---- 灵足启动标志: 各臂2006到达1/5后才启动，防止撞到车内 ---- */
    float thresh_red  = lift_tgt_red  / 5.0f;
    float thresh_blue = lift_tgt_blue / 5.0f;
    uint8_t kfs_switched = 0;  /* 是否已从初始化高度切换到KFS高度 */

    /* ======================== 主循环 ======================== */
    for (;;)
    {
		if(s1>0.5){
			motor_run_flag = 1;    
			relay_pickup_kfs(s1);
			s1=-1;
			}
		else if(s1==0)
		{
		 motor_run_flag = 0;             /* 关闭气泵 */
		}
	// 		if(rc==1){
	// 			    relay_cylinder_extend(1);  /* 气缸伸出 */
    // osDelay(300);                   /* 等待吸附稳定 */
    // relay_cylinder_retract(1); /* 气缸缩回 */
	// 			rc=0;
	// 		}
        /* 1. 更新双臂抬升 */
        lift_control_update();

       /* 2. 更新1号臂云台 */
        arm_gimbal_update(0, lift_tgt_red, thresh_red, &rob_started_red);

        /* 3. 更新2号臂云台 */
        arm_gimbal_update(1, lift_tgt_blue, thresh_blue, &rob_started_blue);

        /* 4. 处理RC指令 (得分状态机) */
        score_update();

        /* 5. ESC 电调控制 */
        esc_update();

        /* 6. 初始化完成判断: 两臂灵足都启动后，切到KFS高度 */
        if (!kfs_switched && rob_started_red && rob_started_blue) {
            lift_tgt_red  = get_lift_pos(&target_red,  LIFT_RED_POS0,  LIFT_RED_POS1,  LIFT_RED_POS2,  LIFT_RED_POS3);
            lift_tgt_blue = get_lift_pos(&target_blue, LIFT_BLUE_POS0, LIFT_BLUE_POS1, LIFT_BLUE_POS2, LIFT_BLUE_POS3);
            lift_set_target(0, lift_tgt_red);
            lift_set_target(1, lift_tgt_blue);
            thresh_red  = lift_tgt_red  / 5.0f;
            thresh_blue = lift_tgt_blue / 5.0f;
            kfs_switched = 1;
        }

        lift_update_debug();
        osDelay(2);
    }
}