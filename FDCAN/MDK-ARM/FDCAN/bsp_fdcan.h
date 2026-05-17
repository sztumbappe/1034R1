#ifndef _BSP_FDCAN_H
#define _BSP_FDCAN_H

#include "main.h"
#include "stdint.h"
#include <stdbool.h>

typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_GIMBAL_ALL_ID = 0x1FF,

    CAN_M1_ID = 0x201,
    CAN_M2_ID = 0x202,
    CAN_M3_ID = 0x203,
    CAN_M4_ID = 0x204,
    CAN_M5_ID = 0x205,
    CAN_M6_ID = 0x206,
    CAN_M7_ID = 0x207,
    CAN_M8_ID = 0x208
} CAN_MX_ID;

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
    int16_t id;
} motor_measure_t;

typedef enum {
    STAGE_INIT,          // 空闲
    STAGE_AT_P1,         // 已到达位置1
    STAGE_AT_P2,
    STAGE_AT_P3,
    STAGE_AT_P4,
} MotorStage;

typedef struct
{
    MotorStage stage;
    motor_measure_t chassis_3508_motor;
    int32_t CycleNum;
    int32_t total_ecd;
    bool ZERO_FLAG;
    int32_t count_num;
    float ZERO_ecp;
    int16_t target_speed;
    int8_t dic_flag;
    int8_t flag_num;
} motor_control;

void fdcan_config(void);
void CAN_CMD_RM(FDCAN_HandleTypeDef *hfdcan, uint32_t STDID, int16_t M1, int16_t M2, int16_t M3, int16_t M4);

// 电机控制数组 (与参考项目一致)
extern motor_control control_3508_classic[5];

#endif