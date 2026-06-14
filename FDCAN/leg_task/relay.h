#ifndef __RELAY_H
#define __RELAY_H

#include "main.h"
#include <stdint.h>

/* ======================== 继电器引脚定义 ======================== */
/* 机械臂1 (粉色臂, ID=1) */
#define CYLINDER1_PORT   GPIOE
#define CYLINDER1_PIN    GPIO_PIN_5    /* PE5 - 气缸1 */
#define VACUUM1_PORT     GPIOF
#define VACUUM1_PIN      GPIO_PIN_3    /* PF3 - 真空阀1 */

/* 机械臂2 (蓝色臂, ID=2) */
#define CYLINDER2_PORT   GPIOE
#define CYLINDER2_PIN    GPIO_PIN_6    /* PE6 - 气缸2 */
#define VACUUM2_PORT     GPIOF
#define VACUUM2_PIN      GPIO_PIN_4    /* PF4 - 真空阀2 */

/* ======================== 低层GPIO控制函数 ======================== */
/* 直接控制单个继电器 (1=通电/吸合, 0=断电/释放) */
void relay_cylinder1_set(uint8_t on);
void relay_cylinder2_set(uint8_t on);
void relay_vacuum1_set(uint8_t on);
void relay_vacuum2_set(uint8_t on);

/* ======================== 高层KFS操作函数 ======================== */

/**
 * @brief 拿取第1个KFS
 * @param arm_id  1=机械臂1(粉色), 2=机械臂2(蓝色)
 *        arm1: 伸出气缸1, 开启真空阀2
 *        arm2: 伸出气缸2, 开启真空阀1
 */
void kfs_pickup_first(uint8_t arm_id);

/**
 * @brief 拿取第2个KFS
 * @param arm_id  1=机械臂1(粉色), 2=机械臂2(蓝色)
 *        arm1: 伸出气缸1, 关闭真空阀1
 *        arm2: 伸出气缸2, 关闭真空阀2
 */
void kfs_pickup_second(uint8_t arm_id);

/**
 * @brief 放下KFS (开启对应真空阀释放吸附)
 * @param arm_id  1=机械臂1(粉色), 2=机械臂2(蓝色)
 *        arm1: 开启真空阀1
 *        arm2: 开启真空阀2
 */
void kfs_release(uint8_t arm_id);

/* ======================== 调试测试函数 ======================== */
/**
 * @brief 继电器IO口测试 (上电时调用, 逐一检测4个引脚)
 *        PE5(气缸1) → PE6(气缸2) → PF3(真空阀1) → PF4(真空阀2)
 *        每个引脚 HIGH 500ms → LOW, 最终全部回到 LOW
 * @note  测试完成后注释掉 main.c 中的调用即可
 */
void relay_test_all(void);

#endif /* __RELAY_H */
