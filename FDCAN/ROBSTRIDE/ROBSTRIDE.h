#ifndef __ROBSTRIDE_H
#define __ROBSTRIDE_H

#include "main.h"
#include "fdcan.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
 
#define Robstirde_Motor_05_hfdcan  hfdcan1

/* 双臂灵足05 CAN ID */
#define ROBSTRIDE_ID_ARM1   1       /* 1号臂云台 */
#define ROBSTRIDE_ID_ARM2   2       /* 2号臂云台 */

/* 灵足05目标角度 (rad) */
#define ROBSTRIDE_TARGET_ANGLE_1       (3.14f)   /* 1号臂展开到前面 */
#define ROBSTRIDE_TARGET_ANGLE_2       (-3.14f)  /* 2号臂展开到前面 */
#define ROBSTRIDE_RETRACT_ANGLE        (-0.3f)   /* 1号臂收到后面 */
#define ROBSTRIDE_RETRACT_ANGLE_BLUE   (0.3f)    /* 2号臂收到后面 */

/* 灵足05活动范围 */
#define ROBSTRIDE_ANGLE_MIN   (-0.5f)
#define ROBSTRIDE_ANGLE_MAX   ( 3.14f)
 
 #define Set_mode 		 'j'				//设置控制模式
 #define Set_parameter 'p'				//设置参数
 //各种控制模式
 #define move_control_mode  0	//运控模式
 #define Pos_control_mode   1	//位置模式
 #define Speed_control_mode 2 //速度模式
 #define Elect_control_mode 3 //电流模式
 #define Set_Zero_mode      4 //零点模式
 //通信地址
#define Communication_Type_Get_ID 0x00     					//获取设备的ID和64位MCU唯一标识符`
#define Communication_Type_MotionControl 0x01 			//运控模式用来向主机发送控制指令
#define Communication_Type_MotorRequest 0x02				//用来向主机反馈电机运行状态
#define Communication_Type_MotorEnable 0x03					//电机使能运行
#define Communication_Type_MotorStop 0x04						//电机停止运行
#define Communication_Type_SetPosZero 0x06					//设置电机机械零位
#define Communication_Type_Can_ID 0x07							//更改当前电机CAN_ID
#define Communication_Type_Control_Mode 0x12				//设置电机模式
#define Communication_Type_GetSingleParameter 0x11	//读取单个参数
#define Communication_Type_SetSingleParameter 0x12	//设定单个参数
#define Communication_Type_ErrorFeedback 0x15				//故障反馈帧

static const uint16_t Index_List[] = {0X7005, 0X7006, 0X700A, 0X700B, 0X7010, 0X7011, 0X7014, 0X7016, 0X7017, 0X7018, 0x7019, 0x701A, 0x701B, 0x701C, 0x701D};

typedef void (*data_read)(const uint16_t *); 

typedef struct
{
		uint16_t index;
	float data;
}data_read_write_one;


typedef struct
{
			data_read_write_one run_mode;				//0:运控模式 1:位置模式 2:速度模式 3:电流模式 4:零点模式 uint8  1byte
		data_read_write_one iq_ref;					//电流模式Iq指令   				float 	4byte 	-23~23A
		data_read_write_one spd_ref;				//转速模式转速指令 				float 	4byte 	-30~30rad/s 
		data_read_write_one imit_torque;		//转矩限制 								float 	4byte 	0~12Nm  
		data_read_write_one cur_kp;					//电流的 Kp 							float 	4byte 	默认值 0.125  
		data_read_write_one cur_ki;					//电流的 Ki 							float 	4byte 	默认值 0.0158  
		data_read_write_one cur_filt_gain;	//电流滤波系数filt_gain 	float 	4byte 	0~1.0，默认值0.1  
		data_read_write_one loc_ref;				//位置模式角度指令				float 	4byte 	rad  
		data_read_write_one limit_spd;			//位置模式速度设置				float 	4byte 	0~30rad/s  
		data_read_write_one limit_cur;			//速度位置模式电流设置 		float 	4byte 	0~23A
		//以下只可读
		data_read_write_one mechPos;				//负载端计圈机械角度			float 	4byte 	rad
		data_read_write_one iqf;						//iq 滤波值 							float 	4byte 	-23~23A
		data_read_write_one	mechVel;				//负载端转速							float 	4byte 	-30~30rad/s 	
		data_read_write_one	VBUS;						//母线电压								float 	4byte 	V	
		data_read_write_one	rotation;				//圈数 										int16 	2byte   圈数
    data_read data_read_one;
 	
}data_read_write;

typedef struct
{
	float Angle;
	float Speed;
	float Torque;
	float Temp;
	int pattern; //电机模式（0复位1标定2运行）
	float offset_angle;      // 上电时的角度（作为零位参考）
	float real_Angle;
  uint8_t is_initialized; 
	float continuous_angle; // 连续角度
  float last_wrapped_angle; // 上一帧的缠绕角度
  int32_t revolutions;      // 圈数计数器
}Motor_Pos_RobStrite_Info;

typedef struct
{
	int set_motor_mode;
	float set_current;
	float set_speed;
	float set_Torque;
	float set_angle;
	float set_limit_cur;
	float set_Kp;
	float set_Ki;
	float set_Kd;
}Motor_Set;

void fdcan_init(void);
void robstride_init(void);
void get_motor_05_true_angle(void);
void Robstirde_Motor_01_rx_callback(FDCAN_HandleTypeDef *hfdcan);
void Robstirde_Motor_02_rx_callback(FDCAN_HandleTypeDef *hfdcan);//电机信息获取
void Robstirde_Motor_03_rx_callback(FDCAN_HandleTypeDef *hfdcan);//电机信息获取
void Robstirde_Motor_05_rx_callback(FDCAN_HandleTypeDef *hfdcan);
void RobStrite_Get_CAN_ID(FDCAN_HandleTypeDef *hfdcan,uint8_t can_id);
void Set_RobStrite_Motor_parameter(FDCAN_HandleTypeDef *hfdcan,uint16_t Index, float Value, char Value_mode,uint8_t can_id);
void Get_RobStrite_Motor_parameter(FDCAN_HandleTypeDef *hfdcan,uint16_t Index,uint8_t can_id);
void RobStrite_Motor_Analysis(uint8_t *DataFrame,uint32_t ID_ExtId);
void RobStrite_Motor_05_move_control(FDCAN_HandleTypeDef *hfdcan, float Torque, float Angle, float Speed, float Kp, float Kd, uint8_t can_id);
void RobStrite_Motor_Pos_control(FDCAN_HandleTypeDef *hfdcan,float Speed, float Angle,uint8_t can_id);
void RobStrite_Motor_Speed_control(FDCAN_HandleTypeDef *hfdcan,float Speed, float limit_cur,uint8_t can_id);
void RobStrite_Motor_current_control(FDCAN_HandleTypeDef *hfdcan,float current,uint8_t can_id);
void RobStrite_Motor_Set_Zero_control(FDCAN_HandleTypeDef *hfdcan,uint8_t can_id);
void Enable_Motor(FDCAN_HandleTypeDef *hfdcan,uint8_t can_id);
void Disenable_Motor(FDCAN_HandleTypeDef *hfdcan, uint8_t clear_error,uint8_t can_id);
void Set_CAN_ID(FDCAN_HandleTypeDef *hfdcan,uint8_t Set_CAN_ID,uint8_t last_can_id);
void Set_ZeroPos(FDCAN_HandleTypeDef *hfdcan,uint8_t can_id);
void data_read_1(const uint16_t *index_list);
void Robstirde_Motor_05_process_frame(FDCAN_RxHeaderTypeDef *RX_Header, uint8_t *RX_BUFFER);
void robstride_init_offset_filtered(uint8_t motor_id);
float robstride_get_relative_angle(uint8_t motor_id);
void RobStride_Enable_AutoReport(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, uint8_t master_id);
extern volatile float arm_close[2];   /* [0]=1号臂 [1]=2号臂: 0=展开到前面, 1=收到后面 */
void robstride_goto_target(float tgt, uint8_t motor_idx);
void robstride_goto_init(float tgt, uint8_t motor_idx);
void robstride_goto_reset(void);
void robstride_keepalive(uint8_t motor_idx);
/* 重力补偿系数 (可在运行时调整) */
extern float K_GRAVITY[2];  /* [0]=粉色臂 [1]=蓝色臂, 单位 Nm */
#endif
