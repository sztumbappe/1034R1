#include "relay.h"
#include "cmsis_os2.h"
#include "esc_control.h"

/* ======================== 低层GPIO控制 ======================== */

static void vacuum_set(uint8_t arm_id, uint8_t on)
{
    if (arm_id == 1) {
        HAL_GPIO_WritePin(VACUUM1_PORT, VACUUM1_PIN,
                          on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    } else if (arm_id == 2) {
        HAL_GPIO_WritePin(VACUUM2_PORT, VACUUM2_PIN,
                          on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

static void cylinder_set(uint8_t arm_id, uint8_t on)
{
    if (arm_id == 1) {
        HAL_GPIO_WritePin(CYLINDER1_PORT, CYLINDER1_PIN,
                          on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    } else if (arm_id == 2) {
        HAL_GPIO_WritePin(CYLINDER2_PORT, CYLINDER2_PIN,
                          on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

/* ======================== 上电初始化 ======================== */

void relay_init(void)
{
    /* 上电: 两个电磁阀都通电 (HIGH), 两个气缸缩回 (LOW) */
    vacuum_set(1, 1);   /* 真空阀1 通电 → 断真空 */
    vacuum_set(2, 1);   /* 真空阀2 通电 → 断真空 */
    cylinder_set(1, 0); /* 气缸1 缩回 */
    cylinder_set(2, 0); /* 气缸2 缩回 */
}

/* ======================== 电磁阀控制 ======================== */

void relay_vacuum_off(uint8_t arm_id)
{
    /* 关闭电磁阀 (LOW) → 断电 → 产生真空吸力 → 吸KFS */
    vacuum_set(arm_id, 0);
}

void relay_vacuum_on(uint8_t arm_id)
{
    /* 开启电磁阀 (HIGH) → 通电 → 断真空 → 放KFS */
    vacuum_set(arm_id, 1);
}

/* ======================== 气缸控制 ======================== */

void relay_cylinder_extend(uint8_t arm_id)
{
    cylinder_set(arm_id, 1);  /* HIGH → 伸出 */
}

void relay_cylinder_retract(uint8_t arm_id)
{
    cylinder_set(arm_id, 0);  /* LOW → 缩回 */
}

/* ======================== KFS操作序列 ======================== */

/*
 * 吸KFS完整序列:
 *   1. 关闭电磁阀 (LOW, 断电 → 产生真空吸力)
 *   2. 气缸伸出 (HIGH, 推出吸住KFS)
 *   3. 延时200ms
 *   4. 气缸缩回 (LOW, 收回)
 * 注意: 调用方需负责先将机械臂抬升到目标高度, 吸完后抬升到600并收到后面
 */
void relay_pickup_kfs(uint8_t arm_id)
{
    /* 第一次调用时开启气泵，之后不再重复开启 */
    static uint8_t pump_started = 0;
    if (!pump_started) {
        motor_run_flag = 1;
        osDelay(200);               /* 等待气泵转起来 */
        pump_started = 1;
    }

    relay_vacuum_off(arm_id);       /* 关电磁阀 → 吸 */
    relay_cylinder_extend(arm_id);  /* 气缸伸出 */
    osDelay(300);                   /* 等待吸附稳定 */
    relay_cylinder_retract(arm_id); /* 气缸缩回 */
}

/*
 * 放KFS:
 *   开启电磁阀 (HIGH, 通电 → 破真空 → 释放KFS)
 */
void relay_release_kfs(uint8_t arm_id)
{
    relay_vacuum_on(arm_id);
}

/* ======================== 调试测试函数 ======================== */

void relay_test_all(void)
{
    /* 气缸1 (PE5) */
    relay_cylinder_extend(1);
    osDelay(200);
    relay_cylinder_retract(1);
    osDelay(100);

    /* 气缸2 (PE6) */
    relay_cylinder_extend(2);
    osDelay(200);
    relay_cylinder_retract(2);
    osDelay(100);

    /* 真空阀1 (PF3): 通电→断电→通电 */
    relay_vacuum_off(1);
    osDelay(100);
    relay_vacuum_on(1);
    osDelay(100);

    /* 真空阀2 (PF4): 通电→断电→通电 */
    relay_vacuum_off(2);
    osDelay(100);
    relay_vacuum_on(2);

    /* 最终状态: 两个电磁阀通电, 两个气缸缩回 (安全状态) */
}