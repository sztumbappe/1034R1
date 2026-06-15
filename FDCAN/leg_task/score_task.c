//
// score_task.c
// 半自动指令状态机 (在 leg_task 主循环中调用)
// 轮询 rc_has_new_cmd(), 根据 ATAKE/BTAKE 指令执行取料/存料流程
//

#include "score_task.h"
#include "rc_protocol.h"
#include "relay.h"
#include "leg_task.h"
#include "3508_control.h"
#include "raise_task.h"
#include "cmsis_os2.h"

/* ======================== 外部变量 ======================== */
extern volatile float target_red;
extern volatile float target_blue;
extern volatile float arm_close[2];

/* ======================== 状态枚举 ======================== */
typedef enum {
    SCORE_IDLE,
    SCORE_PICKUP_STEP1, /* 取料: relay_pickup_kfs (含200ms阻塞) */
    SCORE_PICKUP_STEP2, /* 取料: 抬升到600, 等到位 */
    SCORE_STORE_STEP1,  /* 存料: 灵足收到后面 */
} score_state_t;

/* ======================== 内部变量 ======================== */
static score_state_t state[2] = {SCORE_IDLE, SCORE_IDLE};
static uint32_t enter_tick[2] = {0, 0};

/* ======================== 电磁阀状态跟踪 (用于SWITCH切换) ======================== */
static uint8_t vacuum_state[2] = {1, 1}; /* 上电后两个电磁阀都通电(1=开) */

/* ======================== 雷达数字→高度映射 ======================== */
static const lift_height_t radar_to_height[13] = {
    LIFT_H600,  /* 0: 无效, 默认600 */
    LIFT_H400,  /* 1: 400 */
    LIFT_H200,  /* 2: 200 */
    LIFT_H400,  /* 3: 400 */
    LIFT_H200,  /* 4: 200 */
    LIFT_H600,  /* 5: 无效, 默认600 */
    LIFT_H600,  /* 6: 600 */
    LIFT_H400,  /* 7: 400 */
    LIFT_H600,  /* 8: 无效, 默认600 */
    LIFT_H400,  /* 9: 400 */
    LIFT_H200,  /* 10: 200 */
    LIFT_H400,  /* 11: 400 */
    LIFT_H200,  /* 12: 200 */
};

/* ======================== 单臂状态机 ======================== */
static void arm_sm_update(uint8_t idx)
{
    if (state[idx] == SCORE_IDLE) {
        return;
    }

    uint32_t now = osKernelGetTickCount();

    switch (state[idx]) {

    /* ---------- 取料流程 ---------- */
    case SCORE_PICKUP_STEP1:
        /* relay_pickup_kfs: 关阀+气缸伸出200ms+缩回 (阻塞) */
        relay_pickup_kfs(idx + 1);
        state[idx] = SCORE_PICKUP_STEP2;
        enter_tick[idx] = now;
        break;

    case SCORE_PICKUP_STEP2:
        /* 抬升到600 */
        if (idx == 0) {
            target_red = 4;
        } else {
            target_blue = 4;
        }
        /* 等2秒让2006到达 */
        if (now - enter_tick[idx] >= 2000) {
            state[idx] = SCORE_IDLE;
        }
        break;

    /* ---------- 存料流程 ---------- */
    case SCORE_STORE_STEP1:
        /* 灵足收到后面 */
        arm_close[idx] = 1;
        state[idx] = SCORE_IDLE;
        break;

    default:
        state[idx] = SCORE_IDLE;
        break;
    }
}

/* ======================== 指令→高度转换 ======================== */
/* RKFS/LKFS 参数: 100=1, 200=2, 400=3, 600=4 */
static float height_to_target(uint8_t param)
{
    switch (param) {
        case 100: return 1.0f;
        case 200: return 2.0f;
        case 400: return 3.0f;
        case 600: return 4.0f;
        default:  return 4.0f; /* 默认600 */
    }
}

/* ======================== 指令派发 ======================== */
static void dispatch_cmd(const rc_cmd_t *cmd)
{
    switch (cmd->type) {

    /* ---- 半自动: 取料/存料 ---- */
    case RC_CMD_ATAKE:
        if (cmd->param1 == 0) {
            /* ATAKE00: 左臂取料 */
            state[0] = SCORE_PICKUP_STEP1;
            enter_tick[0] = osKernelGetTickCount();
        } else {
            /* ATAKE01: 左臂存料 */
            state[0] = SCORE_STORE_STEP1;
        }
        break;

    case RC_CMD_BTAKE:
        if (cmd->param1 == 0) {
            /* BTAKE00: 右臂取料 */
            state[1] = SCORE_PICKUP_STEP1;
            enter_tick[1] = osKernelGetTickCount();
        } else {
            /* BTAKE01: 右臂存料 */
            state[1] = SCORE_STORE_STEP1;
        }
        break;

    /* ---- 开场定位 ---- */
    case RC_CMD_KFS:
        if (cmd->param1 >= 1 && cmd->param1 <= 12) {
            target_red = (float)radar_to_height[cmd->param1];
        }
        if (cmd->param2 >= 1 && cmd->param2 <= 12) {
            target_blue = (float)radar_to_height[cmd->param2];
        }
        break;

    /* ---- 手动高度 ---- */
    case RC_CMD_RKFS:
        /* 右臂高度: RKFS100/200/400/600 */
        target_blue = height_to_target(cmd->param1);
        break;

    case RC_CMD_LKFS:
        /* 左臂高度: LKFS100/200/400/600 */
        target_red = height_to_target(cmd->param1);
        break;

    /* ---- 吸取 ---- */
    case RC_CMD_RABSORB:
        /* 右臂吸取并收回 */
        relay_pickup_kfs(2);
        break;

    case RC_CMD_LABSORB:
        /* 左臂吸取并收回 */
        relay_pickup_kfs(1);
        break;

    /* ---- 吸/放切换 ---- */
    case RC_CMD_RSWITCH:
        vacuum_state[1] = !vacuum_state[1];
        if (vacuum_state[1]) {
            relay_vacuum_on(2);
        } else {
            relay_vacuum_off(2);
        }
        break;

    case RC_CMD_LSWITCH:
        vacuum_state[0] = !vacuum_state[0];
        if (vacuum_state[0]) {
            relay_vacuum_on(1);
        } else {
            relay_vacuum_off(1);
        }
        break;

    /* ---- 升降控制 ---- */
    case RC_CMD_RRISING:
        /* 右臂抬升一级 */
        if (target_blue < 4) target_blue += 1;
        break;

    case RC_CMD_LRISING:
        /* 左臂抬升一级 */
        if (target_red < 4) target_red += 1;
        break;

    case RC_CMD_RGODOWN:
        /* 右臂下降一级 */
        if (target_blue > 1) target_blue -= 1;
        break;

    case RC_CMD_LGODOWN:
        /* 左臂下降一级 */
        if (target_red > 1) target_red -= 1;
        break;

    /* ---- 抬升切换 ---- */
    case RC_CMD_UPLIFT:
        if (cmd->param1 == 1) {
            /* UPLIFT1: 大抬升升到顶 */
            raise_target_pos = AIMDIC;
            raise_control_enable = 1;
        } else {
            /* UPLIFT0: 大抬升降到零点(限位开关) */
            raise_target_pos = 0;
            raise_control_enable = 1;
        }
        break;

    default:
        break;
    }
}

/* ======================== 公共接口 ======================== */

void score_update(void)
{
    /* 1. 检查新指令 */
    if (rc_has_new_cmd()) {
        rc_cmd_t *cmd = rc_get_cmd();
        dispatch_cmd(cmd);
        rc_clear_new_cmd();
    }

    /* 2. 执行左臂状态机 */
    arm_sm_update(0);

    /* 3. 执行右臂状态机 */
    arm_sm_update(1);
}