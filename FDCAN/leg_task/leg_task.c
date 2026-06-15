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
#include "cmsis_os2.h"
#include "math.h"

/* ======================== 遥控器接口变量 ======================== */
volatile float target_red  = 4;   /* 1号臂抬升: 2=高200, 3=高400, 4=高600 */
volatile float target_blue = 4;   /* 2号臂抬升: 2=高200, 3=高400, 4=高600 */

static float s;
volatile float arm_init;

/* ======================== 1. 硬件初始化 ======================== */
/**
 * leg_hw_init() — 上电硬件初始化 (不含等待信号)
 * PID初始化 → 灵足使能 → 2006通讯 → 记录零点
 */
static void leg_hw_init(void)
{
    arm_PID_INIT();              // 2006 PID初始化
    robstride_init();            // 使能双臂灵足05电机
    osDelay(100);                // 等待灵足05电机就绪
    s++;
    di3508_r2control_Begin();    // 等待2006通讯建立
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

    /* 灵足云台运动 */
    if (*started) {
        if (idx == 0) {
            /* 1号臂 (CAN ID=1) */
            if (arm_close[0] == 1)
                robstride_goto_target(ROBSTRIDE_RETRACT_ANGLE, 0);      /* 收到后面 */
            else
                robstride_goto_target(ROBSTRIDE_TARGET_ANGLE_1, 0);     /* 展开到前面 */
        } else {
            /* 2号臂 (CAN ID=2) */
            if (arm_close[1] == 1)
                robstride_goto_target(ROBSTRIDE_RETRACT_ANGLE_BLUE, 1); /* 收到后面 */
            else
                robstride_goto_target(ROBSTRIDE_TARGET_ANGLE_2, 1);     /* 展开到前面 */
        }
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
void leg_task(void *argument)
{
    UNUSED(argument);
    osDelay(500);

    /* ---- 上电硬件初始化 ---- */
    leg_hw_init();

    /* ---- 等待启动信号 ---- */
    while (arm_init != 1) {
        lift_update_debug();
        osDelay(10);
    }

    /* ---- 初始化抬升目标: 双臂高600 ---- */
    lift_tgt_red  = (float)LIFT_RED_POS3;
    lift_tgt_blue = (float)LIFT_BLUE_POS3;
    lift_set_target(0, lift_tgt_red);
    lift_set_target(1, lift_tgt_blue);

    /* ---- 灵足启动标志: 各臂2006到达1/5后才启动，防止撞到车内 ---- */
    uint8_t rob_started_red  = 0;
    uint8_t rob_started_blue = 0;
    float thresh_red  = lift_tgt_red  / 5.0f;
    float thresh_blue = lift_tgt_blue / 5.0f;

    /* ======================== 主循环 ======================== */
    for (;;)
    {
        /* 1. 更新双臂抬升 */
        lift_control_update();

        /* 2. 更新1号臂云台 */
        arm_gimbal_update(0, lift_tgt_red, thresh_red, &rob_started_red);

        /* 3. 更新2号臂云台 */
        arm_gimbal_update(1, lift_tgt_blue, thresh_blue, &rob_started_blue);

        /* 4. 处理RC指令 (得分状态机) */
        score_update();

        lift_update_debug();
        osDelay(2);
    }
}