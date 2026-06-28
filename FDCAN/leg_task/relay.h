#ifndef __RELAY_H
#define __RELAY_H

#include "main.h"
#include <stdint.h>

/* ======================== 继电器引脚定义 ======================== */
/* 机械臂1 (arm1, CAN ID=1) */
#define CYLINDER1_PORT   GPIOE
#define CYLINDER1_PIN    GPIO_PIN_5    /* PE5 - 气缸1 */
#define VACUUM1_PORT     GPIOF
#define VACUUM1_PIN      GPIO_PIN_3    /* PF3 - 真空阀1 */

/* 机械臂2 (arm2, CAN ID=2) */
#define CYLINDER2_PORT   GPIOE
#define CYLINDER2_PIN    GPIO_PIN_6    /* PE6 - 气缸2 */
#define VACUUM2_PORT     GPIOF
#define VACUUM2_PIN      GPIO_PIN_4    /* PF4 - 真空阀2 */

/* ======================== 上电初始化 ======================== */
/**
 * @brief 继电器初始化: 上电后两个电磁阀都通电 (HIGH)
 *        确保初始状态为"放"（断真空）
 */
void relay_init(void);

/* ======================== 电磁阀控制 ======================== */
/**
 * @brief 关闭电磁阀 (LOW) → 断电 → 产生真空吸力 → 吸KFS
 * @param arm_id  1=arm1(真空阀1/PF3), 2=arm2(真空阀2/PF4)
 */
void relay_vacuum_off(uint8_t arm_id);

/**
 * @brief 开启电磁阀 (HIGH) → 通电 → 断真空 → 放KFS
 * @param arm_id  1=arm1(真空阀1/PF3), 2=arm2(真空阀2/PF4)
 */
void relay_vacuum_on(uint8_t arm_id);

/* ======================== 气缸控制 ======================== */
/**
 * @brief 气缸伸出 (HIGH)
 * @param arm_id  1=arm1(气缸1/PE5), 2=arm2(气缸2/PE6)
 */
void relay_cylinder_extend(uint8_t arm_id);

/**
 * @brief 气缸缩回 (LOW)
 * @param arm_id  1=arm1(气缸1/PE5), 2=arm2(气缸2/PE6)
 */
void relay_cylinder_retract(uint8_t arm_id);

/* ======================== KFS操作序列 ======================== */
/**
 * @brief 吸KFS完整序列 (仅继电器动作)
 * @param arm_id  1=arm1, 2=arm2
 *        流程: 关闭电磁阀(吸) → 气缸伸出 → 延时200ms → 气缸缩回
 */
void relay_pickup_kfs(uint8_t arm_id);

/**
 * @brief 放KFS: 开启电磁阀 (通电破真空, 释放KFS)
 * @param arm_id  1=arm1, 2=arm2
 */
void relay_release_kfs(uint8_t arm_id);

/**
 * @brief 非阻塞轮询推进 (每2ms主循环调用)
 *        推进 relay_pickup_kfs 状态机, 替换 osDelay 避免阻塞 CAN 心跳
 */
void relay_pickup_poll(void);

/* ======================== 调试测试函数 ======================== */
/**
 * @brief 继电器IO口测试 (上电时调用)
 *        顺序: 气缸1 → 气缸2 → 真空阀1 → 真空阀2
 *        每个引脚 HIGH 500ms → LOW
 *        最终: 两个电磁阀通电 (安全状态)
 */
void relay_test_all(void);

#endif /* __RELAY_H */