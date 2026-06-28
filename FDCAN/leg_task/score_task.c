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
    SCORE_PICKUP_DESCEND,  /* 取料: 降到KFS高度 */
    SCORE_PICKUP_STEP1,    /* 取料: relay_pickup_kfs (含200ms阻塞) */
    SCORE_STORE_RAISE,     /* 存料: 升到600 */
    SCORE_STORE_RETRACT,   /* 存料: 灵足收到后面 */
    SCORE_OUTLAY_WAIT_HEIGHT, /* 放料: 升到400 */
    SCORE_OUTLAY_EXTEND,      /* 放料: 灵足伸出+保持吸附 */
} score_state_t;

/* ======================== 内部变量 ======================== */
score_state_t state[2] = {SCORE_IDLE, SCORE_IDLE};
static uint32_t enter_tick[2] = {0, 0};

/* ======================== KFS目标高度存储 ======================== */
static float kfs_height_red = 4;   /* RCKFS存的左臂高度, 默认600 */
static float kfs_height_blue = 4;  /* RCKFS存的右臂高度, 默认600 */

/* ======================== 电磁阀状态跟踪 (用于SWITCH切换) ======================== */
static uint8_t vacuum_state[2] = {1, 1}; /* 上电后两个电磁阀都通电(1=开) */

/* ======================== R2预备/AtReady模式标志 ======================== */
uint8_t r2ready_mode = 0;  /* 1=R2预备模式, 1号→0°, 2号→-90° */
uint8_t atready_mode = 0;  /* 1=R1进攻姿态, 1号→0°, 2号→-180° */

/* ======================== 协议值(0~9)→高度映射 ======================== */
/* 协议值: 0→实际1, 1→2, 2→3, 3→4, 4→6, 5→7, 6→9, 7→10, 8→11, 9→12 */
static const lift_height_t protocol_to_height[10] = {
    LIFT_H400,  /* 0 → 实际1  → 高400 */
    LIFT_H200,  /* 1 → 实际2  → 高200 */
    LIFT_H400,  /* 2 → 实际3  → 高400 */
    LIFT_H200,  /* 3 → 实际4  → 高200 */
    LIFT_H600,  /* 4 → 实际6  → 高600 */
    LIFT_H400,  /* 5 → 实际7  → 高400 */
    LIFT_H400,  /* 6 → 实际9  → 高400 */
    LIFT_H200,  /* 7 → 实际10 → 高200 */
    LIFT_H400,  /* 8 → 实际11 → 高400 */
    LIFT_H200,  /* 9 → 实际12 → 高200 */
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
    case SCORE_PICKUP_DESCEND:
        /* 降到KFS高度 */
        if (idx == 0) {
            target_red = kfs_height_red;
        } else {
            target_blue = kfs_height_blue;
        }
        /* 到位检测 或 2秒超时 */
        if (lift_arrived(idx) || (now - enter_tick[idx] >= 2000)) {
            state[idx] = SCORE_PICKUP_STEP1;
        }
        break;

    case SCORE_PICKUP_STEP1:
        /* relay_pickup_kfs: 关阀+气缸伸出200ms+缩回 (阻塞) */
        relay_pickup_kfs(idx + 1);
        state[idx] = SCORE_IDLE;
        break;

    /* ---------- 存料流程 ---------- */
    case SCORE_STORE_RAISE:
        /* 先升到600 */
        if (idx == 0) {
            target_red = 4;
        } else {
            target_blue = 4;
        }
        /* 到位检测 或 2秒超时 */
        if (lift_arrived(idx) || (now - enter_tick[idx] >= 2000)) {
            state[idx] = SCORE_STORE_RETRACT;
        }
        break;

    case SCORE_STORE_RETRACT:
        /* 到600后灵足收到后面 */
        arm_close[idx] = 1;
        state[idx] = SCORE_IDLE;
        break;

    /* ---------- 放料流程 ---------- */
    case SCORE_OUTLAY_WAIT_HEIGHT:
        /* 强制升到400 */
        if (idx == 0) {
            target_red = 3;
        } else {
            target_blue = 3;
        }
        /* 到位检测 或 2秒超时 */
        if (lift_arrived(idx) || (now - enter_tick[idx] >= 2000)) {
            state[idx] = SCORE_OUTLAY_EXTEND;
        }
        break;

    case SCORE_OUTLAY_EXTEND:
        /* 灵足伸出到前面, 保持吸附(不动真空阀) */
        arm_close[idx] = 0;
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
    /* 收到任何非R2READY指令时自动退出R2预备模式 */
    if (cmd->type != RC_CMD_R2READY) {
        r2ready_mode = 0;
    }

    switch (cmd->type) {

    /* ---- 半自动: 取料/存料 ---- */        //先取右臂后取左
    case RC_CMD_ATAKE:
        if (cmd->param1 == 0) {
            /* ATAKE00: 右臂取料 → 先降到KFS高度 */
            state[1] = SCORE_PICKUP_DESCEND;
            enter_tick[1] = osKernelGetTickCount();
        } else {
            /* ATAKE01: 右臂存料 → 先升到600再收回 */
            state[1] = SCORE_STORE_RAISE;
            enter_tick[1] = osKernelGetTickCount();
        }
        break;

    case RC_CMD_BTAKE:
        if (cmd->param1 == 0) {
            /* BTAKE00: 左臂取料 → 先降到KFS高度 */
            state[0] = SCORE_PICKUP_DESCEND;
            enter_tick[0] = osKernelGetTickCount();
        } else {
            /* BTAKE01: 左臂存料 → 先升到600再收回 */
            state[0] = SCORE_STORE_RAISE;
            enter_tick[0] = osKernelGetTickCount();
        }
        break;

    /* ---- 开场定位 ---- */
    case RC_CMD_KFS:
        arm_init = 1;  /* 收到KFS指令时触发启动 */
        if (cmd->param1 <= 9) {
            kfs_height_red = (float)protocol_to_height[cmd->param2];
        }
        if (cmd->param2 <= 9) {
            kfs_height_blue = (float)protocol_to_height[cmd->param1];
        }
        break;

    /* ---- 手动高度 ---- */
    case RC_CMD_RKFS:
        /* 右臂高度: RKFS100/200/400/600 */
        target_blue = height_to_target(cmd->param1);
        kfs_height_blue = target_blue;
        arm_close[1] = 0;
        state[1] = SCORE_PICKUP_DESCEND;
        enter_tick[1] = osKernelGetTickCount();
        break;

    case RC_CMD_LKFS:
        /* 左臂高度: LKFS100/200/400/600 */
        target_red = height_to_target(cmd->param1);
        kfs_height_red = target_red;
        arm_close[0] = 0;
        state[0] = SCORE_PICKUP_DESCEND;
        enter_tick[0] = osKernelGetTickCount();
        break;

    case RC_CMD_RABSORB:
        /* 右臂吸取：走完整取料流程（吸+收回+抬升600+） */
        state[1] = SCORE_PICKUP_STEP1;
        enter_tick[1] = osKernelGetTickCount();
        break;

    case RC_CMD_LABSORB:
        /* 左臂吸取：走完整取料流程（吸+收回+抬升600+） */
        state[0] = SCORE_PICKUP_STEP1;
        enter_tick[0] = osKernelGetTickCount();
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

    /* ---- 左臂收回 (等价ATAKE01) ---- */
    case RC_CMD_LRECALL:
        state[0] = SCORE_STORE_RAISE;
        enter_tick[0] = osKernelGetTickCount();
        break;

    /* ---- 右臂收回 (等价BTAKE01) ---- */
    case RC_CMD_RRECALL:
        state[1] = SCORE_STORE_RAISE;
        enter_tick[1] = osKernelGetTickCount();
        break;

    /* ---- 左臂放料准备 (前提: 灵足已展开) ---- */
    case RC_CMD_LOUTLAY:
        if (arm_close[0] == 0) {
            state[0] = SCORE_OUTLAY_WAIT_HEIGHT;
            enter_tick[0] = osKernelGetTickCount();
        }
        break;

    /* ---- 右臂放料准备 (前提: 灵足已展开) ---- */
    case RC_CMD_ROUTLAY:
        if (arm_close[1] == 0) {
            state[1] = SCORE_OUTLAY_WAIT_HEIGHT;
            enter_tick[1] = osKernelGetTickCount();
        }
        break;

    /* ---- R2预备姿态 ---- */
    case RC_CMD_R2READY:
        r2ready_mode = 1;
        atready_mode = 0;
        break;

    /* ---- R1进攻姿态 ---- */
    case RC_CMD_ATREADY:
        atready_mode = 1;
        r2ready_mode = 0;
        break;

    /* ---- 触发1号臂放料准备: 收回+高400 ---- */
    case RC_CMD_TRIGGER:
        target_red = 3;    /* 高400 */
        arm_close[0] = 1;  /* 灵足收回 */
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