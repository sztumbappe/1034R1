//
// leg_task.c
// 灵足05云台(MIT) + 2006抬升(RM) - 上电转目标位置
//

#include "leg_task.h"
#include "3508_control.h"
#include "bsp_fdcan.h"
#include "ROBSTRIDE.h"
#include "cmsis_os2.h"

void leg_task(void *argument)
{
    UNUSED(argument);

    arm_PID_INIT();             // 2006 PID初始化
    robstride_init();           // 灵足05使能 (FDCAN1)
    di3508_r2control_Begin();   // 等待2006通讯 (FDCAN3)
    lift_init();                // 记录2006上电零点

    /* 2006初始不运动 (POS值为0时目标=上电位置, 电机不动) */
    /* 测量完位置值后, 取消下方注释启用初始目标 */
    // lift_set_target(0, LIFT_RED_POS1);
    // lift_set_target(1, LIFT_BLUE_POS1);

    for (;;)
    {
        robstride_goto_target();    // 灵足05平滑转目标角度
        lift_update_debug();        // 更新2006调试偏移 (Watch看 lift_debug_offset)
        // lift_goto_target();      // 2006: 取消注释以启用PID闭环控制
        osDelay(1);
    }
}