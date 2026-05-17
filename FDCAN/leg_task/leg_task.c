//
// leg_task.c
// 机械臂任务 - 从2026r1hand项目移植
// 适配FDCAN项目: 使用FreeRTOS (cmsis_os2), FDCAN3总线
//

#include "leg_task.h"
#include "3508_control.h"
#include "bsp_fdcan.h"
#include "fdcan.h"
#include "gpio.h"
#include "cmsis_os2.h"
#include <stdbool.h>


int8_t init_flag = 1;
/* ======================== 外部变量 ======================== */

extern motor_control control_3508_classic[5];
extern MotorRun motor_3508[4];

/* ======================== 内部变量 ======================== */

static bool InitFlag = false;

/* ======================== 机械臂任务 ======================== */

void leg_task(void *argument)
{
    UNUSED(argument);

    // PID 参数初始化 (必须在任何控制计算之前)
    arm_PID_INIT();

    di3508_r2control_Begin();

    // 第3步: 主循环 — 执行双臂初始化状态机
    for (;;)
    {
        DoubleArmInit();
        osDelay(1);
    }
}

/* ======================== 任务状态机 ======================== */


void task(void)
{
    static int16_t dic_flag = 0;

    switch (dic_flag)
    {
    case 1:
        // 蓝色臂抬升到中间位置 (mode=2)
        di3508_r2control_RunTarget(&control_3508_classic[3], &motor_3508[3],
            2, BlueArmDownDic, BlueArmMiddleDic, BlueArmTopDic, BlueArmPutDic);
        if (control_3508_classic[1].stage == STAGE_AT_P2)
        {
            dic_flag = 2;
        }
        break;
    case 2: // 放KFS - 蓝色臂到放置位
        di3508_r2control_RunTarget(&control_3508_classic[3], &motor_3508[3],
            4, BlueArmDownDic, BlueArmMiddleDic, BlueArmTopDic, BlueArmPutDic);
        if (control_3508_classic[1].stage == STAGE_AT_P4)
        {
            dic_flag = 3;
        }
        break;
    case 3:
        // 红色臂到放置位
        di3508_r2control_RunTarget(&control_3508_classic[1], &motor_3508[1],
            4, RedArmDownDic, RedArmMiddleDic, RedArmTopDic, RedArmPutDic);
        if (control_3508_classic[1].stage == STAGE_AT_P4)
        {
            dic_flag = 4;
        }
        break;
    default:
        break;
    }

    // 发送CAN控制指令 (使用FDCAN3)
    CAN_CMD_RM(&hfdcan3, CAN_CHASSIS_ALL_ID,
               control_3508_classic[0].target_speed,
               control_3508_classic[1].target_speed,
               control_3508_classic[2].target_speed,
               control_3508_classic[3].target_speed);
}

/* ======================== 双臂初始化 ======================== */

/**
 * @brief 双机械臂初始化 (多阶段)
 *
 *  电机编号规定:
 *    粉色臂云台电机 id=0 (0x201)
 *    粉色臂抬升电机 id=1 (0x202)
 *    蓝色臂云台电机 id=2 (0x203)
 *    蓝色臂抬升电机 id=3 (0x204)
 *
 *  初始化流程:
 *    阶段1: 云台找零点 + 抬升向上 (同时进行)
 *    阶段2: 云台回到中间位置 + 抬升继续 (同时进行)
 *    阶段3: 全部回到最低位 + 云台保持中间
 *    阶段4: 初始化完成
 */
void DoubleArmInit(void)
{


    switch (init_flag)
    {
        case 1:
            // 云台找零点 + 双臂抬升向上 (同时进行)
            // 云台: 如果还没找到零点就继续找，已找到则跳过
            if (control_3508_classic[0].ZERO_FLAG == false)
            {
                di3508_r2control_gimbal_find_nonblock(&control_3508_classic[0], -1000.0f);
            }
            if (control_3508_classic[2].ZERO_FLAG == false)
            {
                di3508_r2control_gimbal_find_nonblock(&control_3508_classic[2], 1000.0f);
            }
            // 抬升: 粉色臂上升到中间, 蓝色臂保持最低
            di3508_r2control_RunTarget(&control_3508_classic[1], &motor_3508[1],
                2, RedArmDownDic, RedArmMiddleDic, RedArmTopDic, RedArmPutDic);
            di3508_r2control_RunTarget(&control_3508_classic[3], &motor_3508[3],
                1, BlueArmDownDic, BlueArmMiddleDic, BlueArmTopDic, BlueArmPutDic);
            // 条件: 双云台都找到零点 + 抬升到位
            if (control_3508_classic[0].ZERO_FLAG == true &&
                control_3508_classic[2].ZERO_FLAG == true &&
                control_3508_classic[1].stage == STAGE_AT_P2 &&
                control_3508_classic[3].stage == STAGE_AT_P1)
            {
                init_flag = 2;
            }
            break;
        case 2:
            // 云台回到中间位置 + 抬升保持位置
            di3508_r2control_RunTarget(&control_3508_classic[1], &motor_3508[1],
                2, RedArmDownDic, RedArmMiddleDic, RedArmTopDic, RedArmPutDic);
            di3508_r2control_RunTarget(&control_3508_classic[3], &motor_3508[3],
                1, BlueArmDownDic, BlueArmMiddleDic, BlueArmTopDic, BlueArmPutDic);
            di3508_r2control_gimbal(&control_3508_classic[0], &motor_3508[0],
                1, RedMiddleDic, RedVerticalDic, RedOutsideDic, RedInsideDic);
            di3508_r2control_gimbal(&control_3508_classic[2], &motor_3508[2],
                1, BlueMiddleDic, BlueVerticalDic, BlueOutsideDic, BlueInsideDic);
            if (control_3508_classic[1].stage == STAGE_AT_P2 &&
                control_3508_classic[3].stage == STAGE_AT_P1 &&
                control_3508_classic[0].stage == STAGE_AT_P1 &&
                control_3508_classic[2].stage == STAGE_AT_P1)
            {
                init_flag = 3;
            }
            break;
        case 3:
            // 全部回到最低位, 云台保持中间
            di3508_r2control_RunTarget(&control_3508_classic[1], &motor_3508[1],
                1, RedArmDownDic, RedArmMiddleDic, RedArmTopDic, RedArmPutDic);
            di3508_r2control_RunTarget(&control_3508_classic[3], &motor_3508[3],
                1, BlueArmDownDic, BlueArmMiddleDic, BlueArmTopDic, BlueArmPutDic);
            di3508_r2control_gimbal(&control_3508_classic[0], &motor_3508[0],
                1, RedMiddleDic, RedVerticalDic, RedOutsideDic, RedInsideDic);
            di3508_r2control_gimbal(&control_3508_classic[2], &motor_3508[2],
                1, BlueMiddleDic, BlueVerticalDic, BlueOutsideDic, BlueInsideDic);
            if (control_3508_classic[1].stage == STAGE_AT_P1 &&
                control_3508_classic[3].stage == STAGE_AT_P1 &&
                control_3508_classic[0].stage == STAGE_AT_P1 &&
                control_3508_classic[2].stage == STAGE_AT_P1)
            {
                init_flag = 4;
            }
            break;
        case 4:
            init_flag = 5;
            InitFlag = true;
            break;
        default:
            break;
    }

    // 发送CAN控制指令 (使用FDCAN3)
    CAN_CMD_RM(&hfdcan3, CAN_CHASSIS_ALL_ID,
               control_3508_classic[0].target_speed,
               control_3508_classic[1].target_speed,
               control_3508_classic[2].target_speed,
               control_3508_classic[3].target_speed);
}

