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
#include "ROBSTRIDE.h"
#include "cmsis_os2.h"
#include "esc_control.h"

/* ======================== 外部变量 ======================== */
extern volatile float target_red;
extern volatile float target_blue;
extern volatile float arm_close[2];
extern uint8_t motor_run_flag;
extern Motor_Pos_RobStrite_Info Pos_Info[4];

/* ======================== 状态枚举 ======================== */
typedef enum {
    SCORE_IDLE,
    SCORE_PICKUP_DESCEND,      /* 取料: 降到KFS高度 */
    SCORE_PICKUP_STEP1,        /* 取料: relay_pickup_kfs (含200ms阻塞) */
    SCORE_STORE_RAISE,         /* 存料: 升到600 */
    SCORE_STORE_RETRACT,       /* 存料: 灵足收到后面 */
    SCORE_OUTLAY_WAIT_HEIGHT,  /* 放料: 设target=4(高600) + 灵足展开 */
    SCORE_OUTLAY_WAIT_LEG,     /* 放料: 等灵足展开到位 */
    SCORE_OUTLAY_EXTEND,       /* 放料: 气缸伸出, 保持吸附 */
    /* ---- 新增 ---- */
    SCORE_ABSORB_WAIT_BOTH,    /* 取料: 降100 + 灵足展开, 等两路到位 */
    SCORE_ABSORB_DO,           /* 取料: 执行吸取 */
    SCORE_SWITCH_OPEN,         /* 放料: 开阀300ms后收气缸 */
    /* ---- 预选赛 ---- */
    PRELIM_LRECALL_CYL_OUT,    /* 左收①: 气缸伸出300ms */
    PRELIM_LRECALL_RELEASE,    /* 左收②: 开阀释放300ms */
    PRELIM_LRECALL_CYL_IN,     /* 左收③: 气缸缩回+100ms */
    PRELIM_LRECALL_RIGHT_PREP, /* 左收④: 对侧臂转±90°+关阀吸气+伸气缸 */
    PRELIM_ROUTLAY_CYL_IN,     /* 右伸: 先收气缸→到位后伸出 */
} score_state_t;

/* ======================== 内部变量 ======================== */
score_state_t state[2] = {SCORE_IDLE, SCORE_IDLE};
static uint32_t enter_tick[2] = {0, 0};
static uint8_t pump_was_idle[2] = {0, 0};  /* 进入等待状态时泵是否未转 */

/* ======================== KFS目标高度存储 ======================== */
float kfs_height_red = 4;   /* RCKFS存的左臂高度, 默认600 */
float kfs_height_blue = 4;  /* RCKFS存的右臂高度, 默认600 */

/* ======================== 预存KFS高度 (预选赛初始化调用) ======================== */
void score_preset_kfs_height(float red, float blue)
{
    kfs_height_red = red;
    kfs_height_blue = blue;
}

/* ======================== 灵足到位判断 (CAN位置反馈) ======================== */
#define LEG_ARRIVED_THRESHOLD 0.05f  /* ~2.9° 容差 */

static uint8_t leg_arrived(uint8_t idx)
{
    float target = (idx == 0) ? ROBSTRIDE_TARGET_ANGLE_1    /*  3.14f */
                               : ROBSTRIDE_TARGET_ANGLE_2;  /* -3.14f */
    float current = Pos_Info[idx + 1].Angle;
    return (fabsf(current - target) < LEG_ARRIVED_THRESHOLD);
}

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
        arm_close[idx] = 0;          //保证机械臂在前
        /* 首帧启动气泵 + 20ms guard，等 lift_control_update 刷新 lift_target */
        if (enter_tick[idx] == 0) {
            enter_tick[idx] = now;
            pump_was_idle[idx] = !motor_run_flag;
            motor_run_flag = 1;
            esc_update();
        }
        if (now - enter_tick[idx] < 20) {
            break;
        }
        /* 到位 + 泵已就绪 或 2秒超时 */
        if ((lift_arrived(idx) && leg_arrived(idx) && (!pump_was_idle[idx] || now - enter_tick[idx] >= 300))
            || (now - enter_tick[idx] >= 2000)) {
            state[idx] = SCORE_PICKUP_STEP1;
        }
        break;

    case SCORE_PICKUP_STEP1:
        /* relay_pickup_kfs: 关阀+气缸伸出200ms+缩回 (阻塞) */
        relay_pickup_kfs(idx + 1);
        state[idx] = SCORE_IDLE;
        break;

    /* ---------- 吸取流程 (RABSORB/LABSORB) ---------- */
    case SCORE_ABSORB_WAIT_BOTH:
        /* 降100 + 灵足展开 同步进行 */
        if (idx == 0) {
            target_red = 1;
        } else {
            target_blue = 1;
        }
        arm_close[idx] = 0;
        /* 首帧启动气泵 + 20ms guard，等 lift_control_update 刷新 lift_target */
        if (enter_tick[idx] == 0) {
            enter_tick[idx] = now;
            pump_was_idle[idx] = !motor_run_flag;
            motor_run_flag = 1;
            esc_update();
        }
        if (now - enter_tick[idx] < 20) {
            break;
        }
        /* 两路到位 + 泵已就绪 或 3秒超时 */
        if ((lift_arrived(idx) && leg_arrived(idx) && (!pump_was_idle[idx] || now - enter_tick[idx] >= 300))
            || (now - enter_tick[idx] >= 3000)) {
            state[idx] = SCORE_ABSORB_DO;
        }
        break;

    case SCORE_ABSORB_DO:
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
        /* 首帧+20ms跳过，等 lift_control_update 刷新 lift_target */
        if (enter_tick[idx] == 0) {
            enter_tick[idx] = now;
        }
        if (now - enter_tick[idx] < 20) {
            break;
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
        /* 设高600 + 灵足展开, 立即转入等灵足 */
        if (idx == 0) {
            target_red = 4;
        } else {
            target_blue = 4;
        }
        arm_close[idx] = 0;
        state[idx] = SCORE_OUTLAY_WAIT_LEG;
        enter_tick[idx] = 0;
        break;

    case SCORE_OUTLAY_WAIT_LEG:
        /* 等灵足展开到位 或 3秒超时 */
        /* 首帧+20ms跳过，等 lift_control_update 刷新 lift_target */
        if (enter_tick[idx] == 0) {
            enter_tick[idx] = now;
        }
        if (now - enter_tick[idx] < 20) {
            break;
        }
        if (leg_arrived(idx) || (now - enter_tick[idx] >= 3000)) {
            state[idx] = SCORE_OUTLAY_EXTEND;
        }
        break;

    case SCORE_OUTLAY_EXTEND:
        /* 灵足到位 → 气缸伸出, 保持不缩回 */
        relay_cylinder_extend(idx + 1);
        state[idx] = SCORE_IDLE;
        break;

    /* ---------- 放料切换 (SWITCH) ---------- */
    case SCORE_SWITCH_OPEN:
        /* 开电磁阀500ms后收气缸 */
        /* 首帧+20ms跳过，等 dispatch 本帧的 relay_vacuum_on 生效 */
        if (enter_tick[idx] == 0) {
            enter_tick[idx] = now;
        }
        if (now - enter_tick[idx] < 20) {
            break;
        }
        if (now - enter_tick[idx] >= 500) {
            relay_cylinder_retract(idx + 1);
            state[idx] = SCORE_IDLE;
        }
        break;

#if MATCH_MODE != MATCH_MODE_NORMAL
    /* ========== 预选赛左收状态机 (红方arm0 / 蓝方arm1) ========== */

    case PRELIM_LRECALL_CYL_OUT:
        /* ① 气缸伸出300ms */
        if (enter_tick[idx] == 0) {
            enter_tick[idx] = now;
            relay_cylinder_extend(idx + 1);
        }
        if (now - enter_tick[idx] >= 300) {
            state[idx] = PRELIM_LRECALL_RELEASE;
            enter_tick[idx] = 0;
        }
        break;

    case PRELIM_LRECALL_RELEASE:
        /* ② 开电磁阀释放500ms */
        if (enter_tick[idx] == 0) {
            enter_tick[idx] = now;
            relay_vacuum_on(idx + 1);
        }
        if (now - enter_tick[idx] >= 500) {
            state[idx] = PRELIM_LRECALL_CYL_IN;
            enter_tick[idx] = 0;
        }
        break;

    case PRELIM_LRECALL_CYL_IN:
        /* ③ 气缸缩回, 等100ms */
        if (enter_tick[idx] == 0) {
            enter_tick[idx] = now;
            relay_cylinder_retract(idx + 1);
        }
        if (now - enter_tick[idx] >= 100) {
            state[idx] = PRELIM_LRECALL_RIGHT_PREP;
            enter_tick[idx] = 0;
        }
        break;

    case PRELIM_LRECALL_RIGHT_PREP:
        /* ④ 对侧臂转±90° + 关阀吸气 + 气缸伸出 → IDLE */
#if MATCH_MODE == MATCH_MODE_PRELIM
        /* 红方: 右臂(arm1) → -90°(-1.57f) + 吸气 + 气缸伸出 */
        prelim_prep_flag[1] = 1;
        prelim_prep_angle[1] = -1.57f;
        relay_vacuum_off(2);
        relay_cylinder_extend(2);
#elif MATCH_MODE == MATCH_MODE_BLUE
        /* 蓝方: 左臂(arm0) → +90°(+1.57f) + 吸气 + 气缸伸出 */
        prelim_prep_flag[0] = 1;
        prelim_prep_angle[0] = 1.57f;
        relay_vacuum_off(1);
        relay_cylinder_extend(1);
#endif
        state[idx] = SCORE_IDLE;
        break;

    case PRELIM_ROUTLAY_CYL_IN:
        /* 先收气缸, 等200ms后衔接正常放料流程 */
        if (enter_tick[idx] == 0) {
            enter_tick[idx] = now;
            relay_cylinder_retract(idx + 1);
        }
        if (now - enter_tick[idx] >= 200) {
            state[idx] = SCORE_OUTLAY_WAIT_HEIGHT;
            enter_tick[idx] = 0;
        }
        break;
#endif /* MATCH_MODE != MATCH_MODE_NORMAL */

    default:
        state[idx] = SCORE_IDLE;
        break;
    }
}

/* ======================== 指令→高度转换 ======================== */
/* RKFS/LKFS 参数: 100=1, 200=2, 400=3, 600=4 */
static float height_to_target(uint16_t param)
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

/* 判断指令是否会触发灵足运动 (改变 arm_close 或触发相关状态机) */
static uint8_t cmd_triggers_gimbal(rc_cmd_type_t type)
{
    switch (type) {
    case RC_CMD_ATAKE:
    case RC_CMD_BTAKE:
    case RC_CMD_RKFS:
    case RC_CMD_LKFS:
    case RC_CMD_RABSORB:
    case RC_CMD_LABSORB:
    case RC_CMD_LRECALL:
    case RC_CMD_RRECALL:
    case RC_CMD_LOUTLAY:
    case RC_CMD_ROUTLAY:
    case RC_CMD_TRIGGER:
    case RC_CMD_KFS:
        return 1;
    default:
        return 0;
    }
}

static void dispatch_cmd(const rc_cmd_t *cmd)
{
    /* 触发灵足运动的指令 → 退出 R2预备/进攻模式, 灵足恢复正常控制 */
    if (cmd_triggers_gimbal(cmd->type)) {
        r2ready_mode = 0;
        atready_mode = 0;
    }

    switch (cmd->type) {

    // /* ---- 半自动: 取料/存料 ---- */        //先取右臂后取左
    // case RC_CMD_ATAKE:
    //     if (cmd->param1 == 0) {
    //         /* ATAKE00: 右臂取料 → 先降到KFS高度 */
    //         state[1] = SCORE_PICKUP_DESCEND;
    //         enter_tick[1] = 0;
    //     } else {
    //         /* ATAKE01: 右臂存料 → 先升到600再收回 */
    //         state[1] = SCORE_STORE_RAISE;
    //         enter_tick[1] = 0;
    //     }
    //     break;

    // case RC_CMD_BTAKE:
    //     if (cmd->param1 == 0) {
    //         /* BTAKE00: 左臂取料 → 先降到KFS高度 */
    //         state[0] = SCORE_PICKUP_DESCEND;
    //         enter_tick[0] = 0;
    //     } else {
    //         /* BTAKE01: 左臂存料 → 先升到600再收回 */
    //         state[0] = SCORE_STORE_RAISE;
    //         enter_tick[0] = 0;
    //     }
    //     break;

    // /* ---- 开场定位 ---- */
    // case RC_CMD_KFS:
    //     arm_init = 1;  /* 收到KFS指令时触发启动 */
    //     if (cmd->param1 <= 9) {
    //         kfs_height_red = (float)protocol_to_height[cmd->param2];
    //     }
    //     if (cmd->param2 <= 9) {
    //         kfs_height_blue = (float)protocol_to_height[cmd->param1];
    //     }
    //     break;
        /* ---- 半自动: 取料/存料 ---- */
    case RC_CMD_ATAKE:
        if (cmd->param1 == 0) {
            /* ATAKE00: 左臂取料 → 先降到KFS高度 */
            state[0] = SCORE_PICKUP_DESCEND;
            enter_tick[0] = 0;
        } else {
            /* ATAKE01: 左臂存料 → 先升到600再收回 */
            state[0] = SCORE_STORE_RAISE;
            enter_tick[0] = 0;
        }
        break;

    case RC_CMD_BTAKE:
        if (cmd->param1 == 0) {
            /* BTAKE00: 右臂取料 → 先降到KFS高度 */
            state[1] = SCORE_PICKUP_DESCEND;
            enter_tick[1] = 0;
        } else {
            /* BTAKE01: 右臂存料 → 先升到600再收回 */
            state[1] = SCORE_STORE_RAISE;
            enter_tick[1] = 0;
        }
        break;

    /* ---- 开场定位 ---- */
    case RC_CMD_KFS:
        arm_init = 1;  /* 收到KFS指令时触发启动 */
        if (cmd->param1 <= 9) {
            kfs_height_red = (float)protocol_to_height[cmd->param1];
        }
        if (cmd->param2 <= 9) {
            kfs_height_blue = (float)protocol_to_height[cmd->param2];
        }
        break;

    /* ---- 手动高度 ---- */
    case RC_CMD_RKFS:
        /* 右臂高度: RKFS100/200/400/600 */
        target_blue = height_to_target(cmd->param1);
        kfs_height_blue = target_blue;
        arm_close[1] = 0;
        state[1] = SCORE_PICKUP_DESCEND;
        enter_tick[1] = 0;
        break;

    case RC_CMD_LKFS:
        /* 左臂高度: LKFS100/200/400/600 */
        target_red = height_to_target(cmd->param1);
        kfs_height_red = target_red;
        arm_close[0] = 0;
        state[0] = SCORE_PICKUP_DESCEND;
        enter_tick[0] = 0;
        break;

    case RC_CMD_RABSORB:
        /* 右臂吸取：降100 + 灵足展开, 等两路到位后吸 */
        state[1] = SCORE_ABSORB_WAIT_BOTH;
        enter_tick[1] = 0;
        break;

    case RC_CMD_LABSORB:
        /* 左臂吸取：降100 + 灵足展开, 等两路到位后吸 */
        state[0] = SCORE_ABSORB_WAIT_BOTH;
        enter_tick[0] = 0;
        break;

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

    /* ---- 放料切换 ---- */
    case RC_CMD_RSWITCH:
        /* 右臂开电磁阀 → 300ms后收气缸 */
        relay_vacuum_on(2);
        state[1] = SCORE_SWITCH_OPEN;
        enter_tick[1] = 0;
        break;

    case RC_CMD_LSWITCH:
        /* 左臂开电磁阀 → 300ms后收气缸 */
        relay_vacuum_on(1);
        state[0] = SCORE_SWITCH_OPEN;
        enter_tick[0] = 0;
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

    case RC_CMD_LRECALL:
#if MATCH_MODE == MATCH_MODE_NORMAL
        state[0] = SCORE_STORE_RAISE;
        enter_tick[0] = 0;
#elif MATCH_MODE == MATCH_MODE_PRELIM
        /* 红方左收: 4步状态机 (气缸出→释放→气缸缩→右臂预备) */
        if (state[0] == SCORE_IDLE) {  /* 防重复触发 */
            arm_init = 1;
            state[0] = PRELIM_LRECALL_CYL_OUT;
            enter_tick[0] = 0;
        }
#elif MATCH_MODE == MATCH_MODE_BLUE
        /* 蓝方左收: 镜像红方左收, 运行在右臂(arm1) */
        if (state[1] == SCORE_IDLE) {  /* 防重复触发 */
            arm_init = 1;
            state[1] = PRELIM_LRECALL_CYL_OUT;
            enter_tick[1] = 0;
        }
#endif
        break;

    /* ---- 右臂收回 (等价BTAKE01) ---- */
    case RC_CMD_RRECALL:
#if MATCH_MODE == MATCH_MODE_NORMAL
        state[1] = SCORE_STORE_RAISE;
        enter_tick[1] = 0;
#elif MATCH_MODE == MATCH_MODE_PRELIM
        /* 红方右收: 双灵足前伸 + 左保KFS + 右降到底 */
        if (state[1] == SCORE_IDLE) {  /* 防重复触发 */
            arm_init = 1;
            arm_close[0] = 0;
            arm_close[1] = 0;
            target_red = kfs_height_red;
            target_blue = 1;
            state[1] = SCORE_IDLE;
        }
#elif MATCH_MODE == MATCH_MODE_BLUE
        /* 蓝方右收: 镜像红方右收 (右保KFS + 左降到底) */
        if (state[0] == SCORE_IDLE) {  /* 防重复触发 */
            arm_init = 1;
            arm_close[0] = 0;
            arm_close[1] = 0;
            target_blue = kfs_height_blue;
            target_red = 1;
            state[0] = SCORE_IDLE;
        }
#endif
        break;

    /* ---- 左臂放料准备 ---- */
    case RC_CMD_LOUTLAY:
#if MATCH_MODE != MATCH_MODE_NORMAL
        prelim_prep_flag[0] = 0;  /* 清除预备标志, 让 arm_close 接管 */
        prelim_prep_flag[1] = 0;
#endif
#if MATCH_MODE == MATCH_MODE_BLUE
        /* 蓝方左伸: 先收气缸 → 到位后伸出 */
        state[0] = PRELIM_ROUTLAY_CYL_IN;
#else
        state[0] = SCORE_OUTLAY_WAIT_HEIGHT;
#endif
        enter_tick[0] = 0;
        break;

    /* ---- 右臂放料准备 ---- */
    case RC_CMD_ROUTLAY:
#if MATCH_MODE != MATCH_MODE_NORMAL
        prelim_prep_flag[0] = 0;  /* 清除预备标志, 让 arm_close 接管 */
        prelim_prep_flag[1] = 0;
#endif
#if MATCH_MODE == MATCH_MODE_PRELIM
        /* 红方右伸: 先收气缸 → 到位后伸出 */
        state[1] = PRELIM_ROUTLAY_CYL_IN;
#else
        state[1] = SCORE_OUTLAY_WAIT_HEIGHT;
#endif
        enter_tick[1] = 0;
        break;

    /* ---- R2预备姿态 ---- */
    case RC_CMD_R2READY:
        r2ready_mode = 1;
        atready_mode = 0;
        target_red = 4;    /* 两臂升到600 */
        target_blue = 4;
        break;

    /* ---- R1进攻姿态 ---- */
    case RC_CMD_ATREADY:
        atready_mode = 1;
        r2ready_mode = 0;
        target_red = 4;    /* 两臂升到600 */
        target_blue = 4;
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