//
// leg_task.c
// 灵足05云台(MIT) + 2006抬升(RM)
// 上电: 2006抬升到高600 → 1/5处启动灵足
// target=2/3/4: 切换2006高度
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
    robstride_init();          
    osDelay(100);               // 等待灵足05电机完全就绪
    s++;
    // di3508_r2control_Begin();   // 等待2006通讯 (FDCAN3) - 暂时注释
    lift_init();                 // 记录2006上电零点

    /* 等待 arm_init==1 后才启动 */
    while(arm_init != 1){
        lift_update_debug();
        osDelay(10);
    }

    /* arm_init==1 触发: 2006抬升到高600 */
    float current_lift_target = (float)LIFT_BLUE_POS3;
    lift_set_target(1, current_lift_target);

    /* 灵足启动标志: 2006到达1/5目标后才启动 */
    uint8_t robstride_started = 0;
    float lift_threshold = current_lift_target / 5.0f;

    for (;;)
    {
        /* target=2/3/4: 切换2006目标高度 */
        if(target==2){
            current_lift_target = (float)LIFT_BLUE_POS1;  /* 高200 */
        } else if(target==3){
            current_lift_target = (float)LIFT_BLUE_POS2;  /* 高400 */
        } else if(target==4){
            current_lift_target = (float)LIFT_BLUE_POS3;  /* 高600 */
        }
        lift_set_target(1, current_lift_target);
        lift_goto_target();

        /* 检测2006是否到达1/5目标 → 启动灵足 */
        if(!robstride_started){
            float offset = (float)lift_debug_offset[1];
            if(current_lift_target < 0){
                if(offset <= lift_threshold)
                    robstride_started = 1;
            } else {
                if(offset >= lift_threshold)
                    robstride_started = 1;
            }
        }

        /* 灵足05: 到达1/5后开始运动 */
        if(robstride_started){
            robstride_goto_target();
        }

        lift_update_debug();
        osDelay(2);
    }
}