#include "relay.h"

/* ======================== 低层GPIO控制函数 ======================== */

void relay_cylinder1_set(uint8_t on)
{
    HAL_GPIO_WritePin(CYLINDER1_PORT, CYLINDER1_PIN,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void relay_cylinder2_set(uint8_t on)
{
    HAL_GPIO_WritePin(CYLINDER2_PORT, CYLINDER2_PIN,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void relay_vacuum1_set(uint8_t on)
{
    HAL_GPIO_WritePin(VACUUM1_PORT, VACUUM1_PIN,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void relay_vacuum2_set(uint8_t on)
{
    HAL_GPIO_WritePin(VACUUM2_PORT, VACUUM2_PIN,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ======================== 高层KFS操作函数 ======================== */

/*
 * 拿第1个KFS:
 *   arm1: 伸出气缸1(PE5), 开启真空阀2(PF4) —— 用arm2的真空阀吸第一个KFS
 *   arm2: 伸出气缸2(PE6), 开启真空阀1(PF3) —— 用arm1的真空阀吸第一个KFS
 */
void kfs_pickup_first(uint8_t arm_id)
{
    if (arm_id == 1)
    {
        relay_cylinder1_set(1);   /* 气缸1伸出 */
        relay_vacuum2_set(1);     /* 真空阀2开启 */
    }
    else if (arm_id == 2)
    {
        relay_cylinder2_set(1);   /* 气缸2伸出 */
        relay_vacuum1_set(1);     /* 真空阀1开启 */
    }
}

/*
 * 拿第2个KFS:
 *   arm1: 伸出气缸1(PE5), 关闭真空阀1(PF3) —— 切断arm1真空阀(已吸附的KFS保持真空)
 *   arm2: 伸出气缸2(PE6), 关闭真空阀2(PF4) —— 切断arm2真空阀(已吸附的KFS保持真空)
 */
void kfs_pickup_second(uint8_t arm_id)
{
    if (arm_id == 1)
    {
        relay_cylinder1_set(1);   /* 气缸1伸出 */
        relay_vacuum1_set(0);     /* 真空阀1关闭 */
    }
    else if (arm_id == 2)
    {
        relay_cylinder2_set(1);   /* 气缸2伸出 */
        relay_vacuum2_set(0);     /* 真空阀2关闭 */
    }
}

/*
 * 放下KFS:
 *   arm1: 开启真空阀1(PF3) —— 释放吸附,放下KFS
 *   arm2: 开启真空阀2(PF4) —— 释放吸附,放下KFS
 */
void kfs_release(uint8_t arm_id)
{
    if (arm_id == 1)
    {
        relay_vacuum1_set(1);     /* 真空阀1开启 → 破真空,释放KFS */
    }
    else if (arm_id == 2)
    {
        relay_vacuum2_set(1);     /* 真空阀2开启 → 破真空,释放KFS */
    }
}

/* ======================== 调试测试函数 ======================== */

/*
 * relay_test_all - 上电时逐一测试4个继电器IO口
 * 顺序: PE5(气缸1) → PE6(气缸2) → PF3(真空阀1) → PF4(真空阀2)
 * 每个引脚 HIGH 500ms → LOW, 全部完成后回到安全状态
 */
void relay_test_all(void)
{
    /* 气缸1 (PE5) */
    relay_cylinder1_set(1);
    HAL_Delay(500);
    relay_cylinder1_set(0);
    HAL_Delay(200);

    /* 气缸2 (PE6) */
    relay_cylinder2_set(1);
    HAL_Delay(500);
    relay_cylinder2_set(0);
    HAL_Delay(200);

    /* 真空阀1 (PF3) */
    relay_vacuum1_set(1);
    HAL_Delay(500);
    relay_vacuum1_set(0);
    HAL_Delay(200);

    /* 真空阀2 (PF4) */
    relay_vacuum2_set(1);
    HAL_Delay(500);
    relay_vacuum2_set(0);
}
