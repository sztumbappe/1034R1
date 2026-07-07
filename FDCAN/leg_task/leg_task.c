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
#include "rc_protocol.h"
#include "stdint.h"

/* ======================== 遥控器接口变量 ======================== */
volatile float target_red  = 4;   /* 1号臂抬升: 1=高100, 2=高200, 3=高400, 4=高600 */
volatile float target_blue = 4;   /* 2号臂抬升: 1=高100, 2=高200, 3=高400, 4=高600 */
static float s;
volatile float arm_init;

/* ======================== 预选赛灵足预备标志 ======================== */
#if MATCH_MODE == MATCH_MODE_PRELIM || MATCH_MODE == MATCH_MODE_BLUE
uint8_t prelim_prep_flag[2] = {0, 0};   /* [0]=左臂 [1]=右臂: 1=强制预备角度 */
float   prelim_prep_angle[2] = {0, 0};  /* [0]=左臂 [1]=右臂: 预备目标角度(rad) */
#endif

/* ======================== 外部变量 ========·
================ */
extern uint8_t raise_control_enable;
extern float raise_target_pos;
extern Motor_Pos_RobStrite_Info Pos_Info[4];
extern volatile uint32_t can3_last_tick;  /* CAN3最后收到2006数据的时刻 */

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
    di3508_r2control_Begin();    // 等2006数据到达 = 等24V上电 (灵足和2006共用24V)
    robstride_init();            // 24V确认稳定后使能双臂灵足05电机
    osDelay(100);
    s++;
    lift_init();                 // 记录双2006上电零点

#if MATCH_MODE == MATCH_MODE_PRELIM || MATCH_MODE == MATCH_MODE_BLUE || MATCH_MODE == MATCH_MODE_DEBUG
    /* ---- 预选赛2/调试模式: 预存KFS高度 + 自动开泵 + 指定侧吸气 ---- */
    score_preset_kfs_height(4, 4);
    motor_run_flag = 1;
    esc_update();
#if MATCH_MODE == MATCH_MODE_PRELIM
    relay_vacuum_off(1);   /* 红方: 左臂断电→吸气, 操作员手动放KFS */
#elif MATCH_MODE == MATCH_MODE_BLUE
    relay_vacuum_off(2);   /* 蓝方: 右臂断电→吸气, 操作员手动放KFS */
#elif MATCH_MODE == MATCH_MODE_DEBUG
    relay_vacuum_off(1);   /* 调试模式: 双臂断电→吸气, 操作员手动放KFS */
    relay_vacuum_off(2);
#endif
    /* arm_init 保持 0, 等首次 ROUTLAY/LOUTLAY/RRECALL/LRECALL 时置 1 */
#endif
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
#if MATCH_MODE == MATCH_MODE_PRELIM || MATCH_MODE == MATCH_MODE_BLUE
    /* 预选赛2灵足预备: LRECALL步骤④强制对侧臂到预备角度 */
    if (prelim_prep_flag[idx]) {
        robstride_goto_target(prelim_prep_angle[idx], idx);
        return;
    }
#endif

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

/* ======================== 24V掉电恢复 ======================== */

/**
 * @brief 检测24V丢失并自动恢复 (非阻塞版本)
 * @return 0=正常, 1=24V丢失/恢复中(主循环应跳过电机控制)
 * @note  内部不含 osDelay，不阻塞主循环，状态机和通信始终运行
 */
static uint8_t power_supervise(void)
{
    static uint8_t  lost = 0;        /* 0=正常, 1=24V已丢失 */
    static uint8_t  recovering = 0;  /* 0=未恢复, 1=恢复中(等待500ms稳定) */
    static uint32_t recover_tick = 0;
    static uint32_t last_heartbeat = 0;

    if (!lost) {
        /* CAN3超300ms没收到数据 → 2006掉电 = 24V丢失 */
        if (osKernelGetTickCount() - can3_last_tick > 300) {
            lost = 1;
        }
        return 0;  /* 正常 */
    }

    /* 24V丢失模式: 检测CAN3数据是否恢复 */
    if (osKernelGetTickCount() - can3_last_tick <= 50) {
        if (!recovering) {
            /* 首次检测到CAN3恢复 → 立即重启灵足 (非阻塞) */
            recovering = 1;
            recover_tick = osKernelGetTickCount();
            robstride_restart();        /* 非阻塞: 只发使能指令 */
        }
        /* 等待500ms稳定后再归零2006 */
        if (osKernelGetTickCount() - recover_tick >= 500) {
            lift_init();
            lost = 0;
            recovering = 0;
            return 0;  /* 恢复完成 */
        }
        return 1;  /* 恢复中, 跳过电机控制 */
    } else {
        /* CAN3 数据又断了, 重置恢复状态 */
        recovering = 0;
    }

    /* 24V还没恢复: 每~100ms发一次心跳保持2006通讯 (非阻塞) */
    if (osKernelGetTickCount() - last_heartbeat >= 100) {
        CAN_CMD_RM(&hfdcan3, CAN_CHASSIS_ALL_ID, 0, 0, 0, 0);
        last_heartbeat = osKernelGetTickCount();
    }
    return 1;  /* 仍在丢失中 */
}

/* ======================== 主任务 ======================== */
float s1;
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
        uint8_t pwr_lost = power_supervise();  /* 非阻塞检测, 不跳过主循环 */

        /* ---- 电机控制 (仅24V正常时执行) ---- */
        if (!pwr_lost) {

            /* 气泵控制 */
            if (s1 > 0.5f) {
                relay_pickup_kfs(s1);
                s1 = -1;
            } else if (s1 == -3.0f) {
                motor_run_flag = 0;
                relay_vacuum_on(1);  /* 真空阀1 通电 → 断真空 */
            }

            /* 1. 更新双臂抬升 (2006) */
#if MATCH_MODE == MATCH_MODE_PRELIM3
            /* 崇武探幽预选赛3: 2号臂最低200, 禁止降到100 */
            if (target_blue < 2) target_blue = 2;
#endif
            lift_control_update();

            /* 1.5 2006堵转检测 */
            lift_stall_check();

            /* 3. 更新灵足 (收到KFS前→0°, 收到后→正常角度) */
            /* pattern巡查: 未就绪时原地不动, 就绪后切回正常控制 */
            if (Pos_Info[1].pattern != 2)
                robstride_keepalive(0);
            else
                arm_gimbal_update(0);
            if (Pos_Info[2].pattern != 2)
                robstride_keepalive(1);
            else
                arm_gimbal_update(1);

            /* 5. ESC 电调控制 */
            esc_update();

        }

        /* 2. 处理RC指令 (得分状态机) — 始终运行 */
        score_update();

        /* 4. RC串口通信轮询 (ACK发送 + 超时重发) — 始终运行 */
        rc_tx_poll();

        /* 6. 非阻塞 KFS 取料状态机推进 — 始终运行 */
        relay_pickup_poll();

        lift_update_debug();
        osDelay(2);
    }
}
