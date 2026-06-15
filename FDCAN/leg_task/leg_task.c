//
// leg_task.c
// 双臂灵足05云台(MIT) + 双2006抬升(RM)
// 上电: 双2006抬升到高600 → 1/5处启动对应灵足
// target=2/3/4: 切换双2006高度
// 粉色臂(motor_idx=0): 目标3.14f, 收回-0.3f
// 蓝色臂(motor_idx=1): 目标-3.14f, 收回0.3f
//

#include "leg_task.h"
#include "3508_control.h"
#include "bsp_fdcan.h"
#include "ROBSTRIDE.h"
#include "relay.h"
#include "cmsis_os2.h"
#include "math.h"

float s;
volatile float target;
volatile float arm_init;
void leg_task(void *argument)
{
    UNUSED(argument);
    osDelay(500);      
    arm_PID_INIT();             // 2006 PID初始化
    robstride_init();           // 使能双臂灵足05电机
    osDelay(100);               // 等待灵足05电机完全就绪
    s++;
    di3508_r2control_Begin();   // 等待2006通讯 (FDCAN3) - 暂时注释
    lift_init();                 // 记录双2006上电零点

    /* 等待 arm_init==1 后才启动 */
    while(arm_init != 1){
        lift_update_debug();
        osDelay(10);
    }

    /* arm_init==1 触发: 双2006抬升到高600 */
    float current_lift_target_red  = (float)LIFT_RED_POS3;   /* 粉色臂高600 */
    float current_lift_target_blue = (float)LIFT_BLUE_POS3;  /* 蓝色臂高600 */
    lift_set_target(0, current_lift_target_red);
    lift_set_target(1, current_lift_target_blue);

    /* 灵足启动标志: 各臂2006到达1/5目标后才启动对应灵足 */
    uint8_t robstride_started_red  = 0;
    uint8_t robstride_started_blue = 0;
    float lift_threshold_red  = current_lift_target_red  / 5.0f;
    float lift_threshold_blue = current_lift_target_blue / 5.0f;

    for (;;)
    {
        /* target=2/3/4: 切换双2006目标高度 */
        if(target==2){
            current_lift_target_red  = (float)LIFT_RED_POS1;   /* 粉色臂高200 */
            current_lift_target_blue = (float)LIFT_BLUE_POS1;  /* 蓝色臂高200 */
        } else if(target==3){
            current_lift_target_red  = (float)LIFT_RED_POS2;   /* 粉色臂高400 */
            current_lift_target_blue = (float)LIFT_BLUE_POS2;  /* 蓝色臂高400 */
        } else if(target==4){
            current_lift_target_red  = (float)LIFT_RED_POS3;   /* 粉色臂高600 */
            current_lift_target_blue = (float)LIFT_BLUE_POS3;  /* 蓝色臂高600 */
        }
        lift_set_target(0, current_lift_target_red);
        lift_set_target(1, current_lift_target_blue);
        lift_goto_target();

        /* 检测粉色臂2006是否到达1/5目标 → 启动粉色灵足 */
        if(!robstride_started_red){
            float offset_red = (float)lift_debug_offset[0];
            if(current_lift_target_red < 0){
                if(offset_red <= lift_threshold_red)
                    robstride_started_red = 1;
            } else {
                if(offset_red >= lift_threshold_red)
                    robstride_started_red = 1;
            }
        }

        /* 检测蓝色臂2006是否到达1/5目标 → 启动蓝色灵足 */
        if(!robstride_started_blue){
            float offset_blue = (float)lift_debug_offset[1];
            if(current_lift_target_blue < 0){
                if(offset_blue <= lift_threshold_blue)
                    robstride_started_blue = 1;
            } else {
                if(offset_blue >= lift_threshold_blue)
                    robstride_started_blue = 1;
            }
        }

        /* 粉色臂灵足05(motor_idx=0): 到达1/5后开始运动 */
        if(robstride_started_red){
            if(arm_close == 1)
                robstride_goto_target(ROBSTRIDE_RETRACT_ANGLE, 0);      /* 粉色臂收回到-0.3f */
            else
                robstride_goto_target(ROBSTRIDE_TARGET_ANGLE_1, 0);     /* 粉色臂到3.14f */
        }

        /* 蓝色臂灵足05(motor_idx=1): 到达1/5后开始运动 */
        if(robstride_started_blue){
            if(arm_close == 1)
                robstride_goto_target(ROBSTRIDE_RETRACT_ANGLE_BLUE, 1); /* 蓝色臂收到0.3f */
            else
                robstride_goto_target(ROBSTRIDE_TARGET_ANGLE_2, 1);     /* 蓝色臂到-3.14f */
        }

        lift_update_debug();
        osDelay(2);
    }
}