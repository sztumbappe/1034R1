#include "ROBSTRIDE.H"
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#include "main.h"
#include "fdcan.h"
#include "gpio.h"
#include "string.h"
#include "math.h"
#include "bsp_fdcan.h"
#include "stm32h7xx_hal_fdcan.h" 
#include "gpio.h"
#include "relay.h"
#include <stdbool.h>
#include "cmsis_os2.h"
extern uint8_t rx_data[8];//数据缓存区
// 02起始误差圈数
extern float motor_cir[2];

// 01
#define P_MIN_01 -12.5f
#define P_MAX_01 12.5f
#define V_MIN_01 -44.0f 
#define V_MAX_01 44.0f
#define KP_MIN_01 0.0f
#define KP_MAX_01 500.0f
#define KD_MIN_01 0.0f
#define KD_MAX_01 5.0f
#define T_MIN_01 -17.0f
#define T_MAX_01 17.0f


//05
#define P_MIN_05 -12.57f
#define P_MAX_05 12.57f
#define V_MIN_05 -50.0f
#define V_MAX_05 50.0f
#define KP_MIN_05 0.0f
#define KP_MAX_05 500.0f
#define KD_MIN_05 0.0f
#define KD_MAX_05 5.0f
#define T_MIN_05 -5.5f
#define T_MAX_05 5.5f


//uint32_t Mailbox; // 定义邮箱变量
uint8_t RX_BUFFER[8];//接收数据邮箱
uint8_t error_code;
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;


Motor_Pos_RobStrite_Info Pos_Info[4];// 两个电机接收信息
data_read_write drw;
uint8_t CAN_get_ID;
uint8_t Master_CAN_ID;
Motor_Set Motor_Set_All;
float output;
uint16_t angle_deal_flag=0;

//uint16_t angle_deal_flag;
float motor_cir[2];

// 将目标角映射到最接近参考角的等价 2π 区间
static inline float wrap_to_nearest_2pi(float ref, float target)
{
    while (target - ref >  M_PI) target -= 2.0f * M_PI;
    while (target - ref < -M_PI) target += 2.0f * M_PI;
    return target;
}

typedef struct {
    float last_angle;
    float max_angle_step;        // 单步最大角度变化，如 0.3 rad
    float max_torque_rate;       // 单步最大力矩变化，如 2.0 N·m
    bool initialized;
} SafeGuard_t;



/*******************************************************************************
* @功能     		: uint16_t型转float型浮点数
* @参数1        : 需要转换的值
* @参数2        : x的最小值
* @参数3        : x的最大值
* @参数4        : 需要转换的进制数
* @返回值 			: 十进制的float型浮点数
* @概述  				: None
*******************************************************************************/
float uint16_to_float(uint16_t x,float x_min,float x_max,int bits){
    uint32_t span = (1 << bits) - 1;
    float offset = x_max - x_min;
    return offset * x / span + x_min;
}
/*******************************************************************************
* @功能     		: float浮点数转int型
* @参数1        : 需要转换的值
* @参数2        : x的最小值
* @参数3        : x的最大值
* @参数4        : 需要转换的进制数
* @返回值 			: 十进制的int型整数
* @概述  				: None
*******************************************************************************/
int float_to_uint1(float x,float x_min,float x_max,int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	if(x > x_max) x = x_max;
	else if(x < x_min) x = x_min;
	return (int) ((x - offset)*((float)((1<<bits)-1))/span);
}
/*******************************************************************************
* @功能     		: uint8_t数组转float浮点数
* @参数        	: 需要转换的数组
* @返回值 			: 十进制的float型浮点数
* @概述  				: None
*******************************************************************************/
float Byte_to_float(uint8_t* bytedata)  
{  
	uint32_t data = bytedata[7]<<24|bytedata[6]<<16|bytedata[5]<<8|bytedata[4];
	float data_float = *(float*)(&data);
  return data_float;  
}  


// 将 target 映射到最接近 ref 的 2π 等价角（避免大角度跨圈）
static inline float wrap_to_nearest(float ref, float target){
        while (target - ref >  M_PI) target -= 2.0f * M_PI;
        while (target - ref < -M_PI) target += 2.0f * M_PI;
        return target;
}

// 功能使用函数
/*******************************************************************************
* @功能     		: 灵足初始化函数
* @参数        	: None
* @返回值 			: None
* @概述  				: None
*******************************************************************************/
void robstride_init()
{
	Master_CAN_ID = 0x11;	
	Motor_Set_All.set_motor_mode = move_control_mode;
	drw.data_read_one = data_read_1;
	drw.data_read_one(Index_List);
	osDelay(50);
	/* 第1步: 清除电机错误 (clear_error=1) */
	Disenable_Motor(&Robstirde_Motor_05_hfdcan, 1, ROBSTRIDE_ID_ARM1);
	osDelay(100);
	Disenable_Motor(&Robstirde_Motor_05_hfdcan, 1, ROBSTRIDE_ID_ARM2);
	osDelay(100);

	/* 第2步: 使能双臂灵足05电机 (运控模式/MIT模式) */
	Enable_Motor(&Robstirde_Motor_05_hfdcan, ROBSTRIDE_ID_ARM1);
	osDelay(50);
	Enable_Motor(&Robstirde_Motor_05_hfdcan, ROBSTRIDE_ID_ARM2);
	osDelay(100);

	/* 第3步: 等待电机完成归零 (pattern == 2), 带超时保护 */
	uint32_t timeout = 0;
	while ((Pos_Info[1].pattern != 2 || Pos_Info[2].pattern != 2) && timeout < 1000)
	{
		/* 持续发送运控指令 (空力矩), 保持通信活跃 */
		RobStrite_Motor_05_move_control(&Robstirde_Motor_05_hfdcan, 0,
			Pos_Info[1].Angle, 0, 0, 0, ROBSTRIDE_ID_ARM1);
		RobStrite_Motor_05_move_control(&Robstirde_Motor_05_hfdcan, 0,
			Pos_Info[2].Angle, 0, 0, 0, ROBSTRIDE_ID_ARM2);
		osDelay(20);
		timeout++;
	}

	for (int i = 0; i < 4; i++) {
		Pos_Info[i].is_initialized = 0;
	}
}

/*******************************************************************************
* @功能     		: 灵足回调函数
* @参数        	: FDCAN句柄
* @返回值 			: None
* @概述  				: None
*******************************************************************************/
void Robstirde_Motor_05_process_frame(FDCAN_RxHeaderTypeDef *RX_Header, uint8_t *RX_BUFFER)
{
    if (RX_Header->IdType == FDCAN_EXTENDED_ID) {
        if ((int)((RX_Header->Identifier & 0x3F000000) >> 24) == 2) {
            CAN_get_ID = (uint8_t)((RX_Header->Identifier & 0xFF00) >> 8);
            
            switch(CAN_get_ID) {
                case 0:
                case 1:
                case 2:
                case 3:
                {
                    // 直接使用原始角度，不做归一化
                    Pos_Info[CAN_get_ID].Angle = uint16_to_float(RX_BUFFER[0]<<8 | RX_BUFFER[1], P_MIN_05, P_MAX_05, 16);
                    Pos_Info[CAN_get_ID].Speed  = uint16_to_float(RX_BUFFER[2]<<8 | RX_BUFFER[3], V_MIN_05, V_MAX_05, 16);
                    Pos_Info[CAN_get_ID].Torque = uint16_to_float(RX_BUFFER[4]<<8 | RX_BUFFER[5], T_MIN_05, T_MAX_05, 16);
                    Pos_Info[CAN_get_ID].Temp   = (RX_BUFFER[6]<<8 | RX_BUFFER[7]) * 0.1f;
                    
                    error_code = (uint8_t)((RX_Header->Identifier & 0x3F0000) >> 16);
                    Pos_Info[CAN_get_ID].pattern = (uint8_t)((RX_Header->Identifier & 0xC00000) >> 22);
                }
                break; 
            }
        }
        else if ((int)((RX_Header->Identifier & 0x3F000000) >> 24) == 17) {
            for (int index_num = 0; index_num <= 13; index_num++) {
                if ((RX_BUFFER[1]<<8 | RX_BUFFER[0]) == Index_List[index_num]) {
                    switch(index_num) {
                        case 0:  drw.run_mode.data = (uint8_t)(RX_BUFFER[4]); break;
                        case 1:  drw.iq_ref.data = Byte_to_float(RX_BUFFER); break;
                        case 2:  drw.spd_ref.data = Byte_to_float(RX_BUFFER); break;
                        case 3:  drw.imit_torque.data = Byte_to_float(RX_BUFFER); break;
                        case 4:  drw.cur_kp.data = Byte_to_float(RX_BUFFER); break;
                        case 5:  drw.cur_ki.data = Byte_to_float(RX_BUFFER); break;
                        case 6:  drw.cur_filt_gain.data = Byte_to_float(RX_BUFFER); break;
                        case 7:  drw.loc_ref.data = Byte_to_float(RX_BUFFER); break;
                        case 8:  drw.limit_spd.data = Byte_to_float(RX_BUFFER); break;
                        case 9:  drw.limit_cur.data = Byte_to_float(RX_BUFFER); break;
                        case 10: drw.mechPos.data = Byte_to_float(RX_BUFFER); break;
                        case 11: drw.iqf.data = Byte_to_float(RX_BUFFER); break;
                        case 12: drw.mechVel.data = Byte_to_float(RX_BUFFER); break;
                        case 13: drw.VBUS.data = Byte_to_float(RX_BUFFER); break;
                        default: break;
                    }
                }
            }
        }
        else if ((uint8_t)((RX_Header->Identifier & 0xFF)) == 0xFE) {
            CAN_get_ID = (uint8_t)((RX_Header->Identifier & 0xFF00) >> 8);
        }
    }
}

/*******************************************************************************
* @功能     		: RobStrite电机获取设备ID和MCU（通信类型0）
* @参数         : None
* @返回值 				: void
* @概述  				: None
*******************************************************************************/
void RobStrite_Get_CAN_ID(FDCAN_HandleTypeDef *hfdcan,uint8_t can_id)
{
	uint8_t txdata[8] = {0};						   	
	FDCAN_TxHeaderTypeDef TxMessage; 	

	TxMessage.IdType = FDCAN_EXTENDED_ID; // 扩展 ID 类型（
	TxMessage.TxFrameType = FDCAN_DATA_FRAME; // 数据帧
	TxMessage.DataLength = 8; // 数据长度（替代 DLC = 8）
	TxMessage.Identifier = (uint32_t)(Communication_Type_Get_ID << 24 | Master_CAN_ID << 8 | can_id); // 扩展 ID 
	TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxMessage.BitRateSwitch = FDCAN_BRS_OFF; 
	TxMessage.FDFormat = FDCAN_CLASSIC_CAN; 
	TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // 不存储发送事件
	TxMessage.MessageMarker = 0; // 消息标记
	
  HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxMessage, txdata);
}
/*******************************************************************************
* @功能     		: RobStrite电机05运控模式  （通信类型1）
* @参数1        : 力矩
* @参数2        : 目标角度
* @参数3        : 目标角速度
* @参数4        : Kp
* @参数5        : Kd
* @返回值 			: void
* @概述  				: None
*******************************************************************************/
// RobStrite 电机05 运控模式（原始角度，不裁剪）
void RobStrite_Motor_05_move_control(FDCAN_HandleTypeDef *hfdcan, float Torque, float Angle, float Speed, float Kp, float Kd, uint8_t can_id)
{
    uint8_t txdata[8] = {0};
    FDCAN_TxHeaderTypeDef TxMessage;

    // 直接使用原始角度，不做裁剪
    txdata[0] = float_to_uint1(Angle, P_MIN_05, P_MAX_05, 16) >> 8;
    txdata[1] = float_to_uint1(Angle, P_MIN_05, P_MAX_05, 16);
    txdata[2] = float_to_uint1(Speed, V_MIN_05, V_MAX_05, 16) >> 8;
    txdata[3] = float_to_uint1(Speed, V_MIN_05, V_MAX_05, 16);
    txdata[4] = float_to_uint1(Kp, KP_MIN_05, KP_MAX_05, 16) >> 8;
    txdata[5] = float_to_uint1(Kp, KP_MIN_05, KP_MAX_05, 16);
    txdata[6] = float_to_uint1(Kd, KD_MIN_05, KD_MAX_05, 16) >> 8;
    txdata[7] = float_to_uint1(Kd, KD_MIN_05, KD_MAX_05, 16);

    TxMessage.IdType = FDCAN_EXTENDED_ID;
    TxMessage.TxFrameType = FDCAN_DATA_FRAME;
    TxMessage.DataLength = 8;
    TxMessage.Identifier = (uint32_t)(Communication_Type_MotionControl << 24 |
                                     float_to_uint1(Torque, T_MIN_05, T_MAX_05, 16) << 8 |
                                     can_id);
    TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxMessage.BitRateSwitch = FDCAN_BRS_OFF;
    TxMessage.FDFormat = FDCAN_CLASSIC_CAN;
    TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxMessage.MessageMarker = 0;

    HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxMessage, txdata);
}


/*******************************************************************************
* @功能     		: RobStrite电机位置模式 
* @参数1        : 目标角速度(-30rad/s~30rad/s)
* @参数2        : 目标角度(-4π~4π)
* @返回值 			: void
* @概述  				: None
*******************************************************************************/
void RobStrite_Motor_Pos_control(FDCAN_HandleTypeDef *hfdcan,float Speed, float Angle,uint8_t can_id)
{
	Motor_Set_All.set_speed = Speed;
	Motor_Set_All.set_angle = Angle;
	if (drw.run_mode.data != 1 && Pos_Info[can_id].pattern == 2)
	{
		Set_RobStrite_Motor_parameter(hfdcan,0X7005, Pos_control_mode, Set_mode,can_id);		//设置电机模式
		Get_RobStrite_Motor_parameter(hfdcan,0x7005,can_id);
		Motor_Set_All.set_motor_mode = Pos_control_mode;
	}
	Set_RobStrite_Motor_parameter(hfdcan,0X7017, Motor_Set_All.set_speed, Set_parameter,can_id);
	Set_RobStrite_Motor_parameter(hfdcan,0X7016, Motor_Set_All.set_angle, Set_parameter,can_id);
}
/*******************************************************************************
* @功能     		: RobStrite电机速度模式 
* @参数1        : 目标角速度(-30rad/s~30rad/s)
* @参数2        : 目标电流限制(0~23A)
* @返回值 			: void
* @概述  				: None
*******************************************************************************/
uint8_t count_set_motor_mode_Speed = 0;
void RobStrite_Motor_Speed_control(FDCAN_HandleTypeDef *hfdcan,float Speed, float limit_cur,uint8_t can_id)
{
	Motor_Set_All.set_speed = Speed;
	Motor_Set_All.set_limit_cur = limit_cur;
	if (Motor_Set_All.set_motor_mode != 2 && Pos_Info[can_id].pattern == 2)
	{
		Set_RobStrite_Motor_parameter(hfdcan,0X7005, Speed_control_mode, Set_mode,can_id);		//设置电机模式
		Get_RobStrite_Motor_parameter(hfdcan,0x7005,can_id);
		Motor_Set_All.set_motor_mode = Speed_control_mode;
	}
	Set_RobStrite_Motor_parameter(hfdcan,0X7005, Speed_control_mode, Set_mode,can_id);		//设置电机模式
	count_set_motor_mode_Speed++;
	Set_RobStrite_Motor_parameter(hfdcan,0X7018, Motor_Set_All.set_limit_cur, Set_parameter,can_id);	
	Set_RobStrite_Motor_parameter(hfdcan,0X700A, Motor_Set_All.set_speed, Set_parameter,can_id);
}
/*******************************************************************************
* @功能     		: RobStrite电机电流模式
* @参数         : 目标电流(-23~23A)
* @返回值 			: void
* @概述  				: None
*******************************************************************************/
uint8_t count_set_motor_mode = 0;
void RobStrite_Motor_current_control(FDCAN_HandleTypeDef *hfdcan,float current,uint8_t can_id)
{
	Motor_Set_All.set_current = current;
	output = Motor_Set_All.set_current;
	if (Pos_Info[can_id].pattern == 2 && Motor_Set_All.set_motor_mode != 3)
	{
		Set_RobStrite_Motor_parameter(hfdcan,0X7005, Elect_control_mode, Set_mode,can_id);		//设置电机模式
		Get_RobStrite_Motor_parameter(hfdcan,0x7005,can_id);
		Motor_Set_All.set_motor_mode = Elect_control_mode;
	}
	if (count_set_motor_mode % 50 == 0)
		Set_RobStrite_Motor_parameter(hfdcan,0X7005, Elect_control_mode, Set_mode,can_id);		//设置电机模式
	count_set_motor_mode++;
	Set_RobStrite_Motor_parameter(hfdcan,0X7006, Motor_Set_All.set_current, Set_parameter,can_id);
}
/*******************************************************************************
* @功能     		: RobStrite电机零点模式
* @参数         : None
* @返回值 			: void
* @概述  				: None
*******************************************************************************/
void RobStrite_Motor_Set_Zero_control(FDCAN_HandleTypeDef *hfdcan,uint8_t can_id)
{
	Set_RobStrite_Motor_parameter(hfdcan,0X7005, Set_Zero_mode, Set_mode,can_id);					//设置电机模式
}
/*******************************************************************************
* @功能     		: RobStrite电机使能 （通信类型3）
* @参数         : None
* @返回值 			: void
* @概述  				: None
*******************************************************************************/
void Enable_Motor(FDCAN_HandleTypeDef *hfdcan,uint8_t can_id)
{
	uint8_t txdata[8] = {0};				//发送数据
	FDCAN_TxHeaderTypeDef TxMessage; 	//发送邮箱
	
	// 正确配置 FDCAN Tx 头
	TxMessage.IdType = FDCAN_EXTENDED_ID;
	TxMessage.TxFrameType = FDCAN_DATA_FRAME;
	TxMessage.DataLength = 8;
	// 扩展 ID：Communication_Type_MotorEnable（24-31位） + Master_CAN_ID（8-15位） + can_id（0-7位）
	TxMessage.Identifier = (uint32_t)(Communication_Type_MotorEnable << 24 | Master_CAN_ID << 8 | can_id);
	TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxMessage.BitRateSwitch = FDCAN_BRS_OFF;
	TxMessage.FDFormat = FDCAN_CLASSIC_CAN;
	TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxMessage.MessageMarker = 0;

	// 发送函数
	HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxMessage, txdata);
}
/*******************************************************************************
* @功能     		: RobStrite电机失能 （通信类型4）
* @参数         : 是否清除错误位（0不清除 1清除）
* @返回值 			: void
* @概述  				: None
*******************************************************************************/
void Disenable_Motor(FDCAN_HandleTypeDef *hfdcan, uint8_t clear_error, uint8_t can_id)
{
    uint8_t txdata[8] = {0};					   	
    FDCAN_TxHeaderTypeDef TxMessage; 	
    
    txdata[0] = clear_error;
    
    TxMessage.IdType = FDCAN_EXTENDED_ID; // 扩展ID类型
    TxMessage.TxFrameType = FDCAN_DATA_FRAME; // 数据帧
    TxMessage.DataLength = 8; // 数据长度8字节
    TxMessage.Identifier = (uint32_t)(Communication_Type_MotorStop << 24 | Master_CAN_ID << 8 | can_id); // 原ExtId内容移至此
    TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxMessage.BitRateSwitch = FDCAN_BRS_OFF;
    TxMessage.FDFormat = FDCAN_CLASSIC_CAN;
    TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxMessage.MessageMarker = 0;
    

    HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxMessage, txdata);
}
/*******************************************************************************
* @功能     		: RobStrite电机写入参数 （通信类型18）
* @参数1        : 参数地址
* @参数2        : 参数数值
* @参数3        : 选择是传入控制模式 还是其他参数 （Set_mode设置控制模式 Set_parameter设置参数）
* @返回值 				: void
* @概述  				: None
*******************************************************************************/
void Set_RobStrite_Motor_parameter(FDCAN_HandleTypeDef *hfdcan,uint16_t Index, float Value, char Value_mode,uint8_t can_id)
{
	uint8_t txdata[8] = {0};						   	
	FDCAN_TxHeaderTypeDef TxMessage; 	

	TxMessage.IdType = FDCAN_EXTENDED_ID;
	TxMessage.TxFrameType = FDCAN_DATA_FRAME;
	TxMessage.DataLength = 8;
	// 扩展 ID：Communication_Type_SetSingleParameter（24-31位） + Master_CAN_ID（8-15位） + can_id（0-7位）
	TxMessage.Identifier = (uint32_t)(Communication_Type_SetSingleParameter << 24 | Master_CAN_ID << 8 | can_id);
	TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxMessage.BitRateSwitch = FDCAN_BRS_OFF;
	TxMessage.FDFormat = FDCAN_CLASSIC_CAN;
	TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxMessage.MessageMarker = 0;

	// 原数据填充逻辑不变
	txdata[0] = Index;
	txdata[1] = Index>>8;
	txdata[2] = 0x00;
	txdata[3] = 0x00;	
	if (Value_mode == 'p') // Set_parameter
	{
		memcpy(&txdata[4],&Value,4);
	}
	else if (Value_mode == 'j') // Set_mode
	{
		Motor_Set_All.set_motor_mode = (int)(Value);
		txdata[4] = (uint8_t)Value;
		txdata[5] = 0x00;	
		txdata[6] = 0x00;	
		txdata[7] = 0x00;	
	}

	// 正确发送函数
	HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxMessage, txdata);
}
/*******************************************************************************
* @功能     		: RobStrite电机单个参数读取 （通信类型17）
* @参数         : 参数地址
* @返回值 			: void
* @概述  				: None
*******************************************************************************/
void Get_RobStrite_Motor_parameter(FDCAN_HandleTypeDef *hfdcan, uint16_t Index, uint8_t can_id)
{
    uint8_t txdata[8] = {0};
    FDCAN_TxHeaderTypeDef TxMessage; // 发送邮箱
    txdata[0] = Index;
    txdata[1] = Index >> 8;
    
    // 配置 FDCAN 发送头
    TxMessage.IdType = FDCAN_EXTENDED_ID;          // 声明为扩展 ID
    TxMessage.TxFrameType = FDCAN_DATA_FRAME;      // 数据帧类型
    TxMessage.DataLength = 8;                      // 数据长度 8 字节
    // 存储扩展 ID
    TxMessage.Identifier = (uint32_t)(Communication_Type_GetSingleParameter << 24 | Master_CAN_ID << 8 | can_id);

    HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxMessage, txdata);
}
/*******************************************************************************
* @功能     		: RobStrite电机设置CAN_ID （通信类型7）
* @参数         : 修改后（预设）CANID
* @返回值 			: void
* @概述  				: None
*******************************************************************************/
void Set_CAN_ID(FDCAN_HandleTypeDef *hfdcan, uint8_t Set_CAN_ID, uint8_t last_can_id)
{
    Disenable_Motor(hfdcan, 0, last_can_id);
    uint8_t txdata[8] = {0};						   	
    FDCAN_TxHeaderTypeDef TxMessage; 	
    
    // xtId → Identifier，补充完整结构体成员
    TxMessage.IdType = FDCAN_EXTENDED_ID;
    TxMessage.TxFrameType = FDCAN_DATA_FRAME;
    TxMessage.DataLength = 8;
    TxMessage.Identifier = (uint32_t)(Communication_Type_Can_ID << 24 | Set_CAN_ID << 16 | Master_CAN_ID << 8 | last_can_id); // 原ExtId内容移至此
    TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxMessage.BitRateSwitch = FDCAN_BRS_OFF;
    TxMessage.FDFormat = FDCAN_CLASSIC_CAN;
    TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxMessage.MessageMarker = 0;
    
    // 发送函数替换
    HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxMessage, txdata);
}
/*******************************************************************************
* @功能     		: RobStrite电机设置机械零点 （通信类型6）
* @参数         : None
* @返回值 			: void
* @概述  				: 会把当前电机位置设为机械零位， 会先失能电机, 再使能电机
*******************************************************************************/
void Set_ZeroPos(FDCAN_HandleTypeDef *hfdcan, uint8_t can_id)
{
    Disenable_Motor(hfdcan, 0, can_id);						
    uint8_t txdata[8] = {0};						   	
    FDCAN_TxHeaderTypeDef TxMessage; 	
    txdata[0] = 1;
    
    // 修正1：ExtId → Identifier，补充完整结构体成员
    TxMessage.IdType = FDCAN_EXTENDED_ID;
    TxMessage.TxFrameType = FDCAN_DATA_FRAME;
    TxMessage.DataLength = 8;
    TxMessage.Identifier = (uint32_t)(Communication_Type_SetPosZero << 24 | Master_CAN_ID << 8 | can_id); // 原ExtId内容移至此
    TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxMessage.BitRateSwitch = FDCAN_BRS_OFF;
    TxMessage.FDFormat = FDCAN_CLASSIC_CAN;
    TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxMessage.MessageMarker = 0;
    
    // 修正2：发送函数替换
    HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxMessage, txdata);
    Enable_Motor(hfdcan, can_id);
}

/*******************************************************************************
* @功能     		: RobStrite电机数据的参数地址初始化
* @参数         : 数据的参数地址数组
* @返回值 			: void
* @概述  				: 会在创建电机类时自动调用
*******************************************************************************/
void data_read_1(const uint16_t *index_list)
{
	drw.run_mode.index = index_list[0];
	drw.iq_ref.index = index_list[1];
	drw.spd_ref.index = index_list[2];
	drw.imit_torque.index = index_list[3];
	drw.cur_kp.index = index_list[4];
	drw.cur_ki.index = index_list[5];
	drw.cur_filt_gain.index = index_list[6];
	drw.loc_ref.index = index_list[7];
	drw.limit_spd.index = index_list[8];
	drw.limit_cur.index = index_list[9];
	drw.mechPos.index = index_list[10];
	drw.iqf.index = index_list[11];
	drw.mechVel.index = index_list[12];
	drw.VBUS.index = index_list[13];	
	drw.rotation.index = index_list[14];
}

//灵足电机自动上报
/*******************************************************************************
* @功能     		: 开启RobStride电机自动上报功能（按通信类型24协议实现）
* @参数1        : FDCAN句柄（hfdcan1/hfdcan2）
* @参数2        : 目标电机CAN ID（0/1/2/3）
* @返回值 			: void
* @概述  				: 通信类型24协议：0x18，数据区Byte6为01开启，00关闭，默认上报周期10ms
*******************************************************************************/
void RobStride_Enable_AutoReport(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, uint8_t master_id)
{
    uint8_t tx_data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x01, 0x00}; // F_CMD=01开启
    FDCAN_TxHeaderTypeDef tx_header;

    // 配置29位扩展ID
    tx_header.IdType = FDCAN_EXTENDED_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = 8;
    tx_header.Identifier = (uint32_t)(0x18 << 24) | ((uint32_t)master_id << 8) | motor_id; // 0X18=通信类型24
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;

    // 发送开启指令
    HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, tx_data);
    HAL_Delay(10); // 确保指令生效

    // 可选：验证开启结果（读参数0X7026，确认上报周期非0）
    Get_RobStrite_Motor_parameter(hfdcan, 0x7026, motor_id);
}

/*******************************************************************************
* @功能      : 双臂灵足05平滑转到目标角度 (MIT运控模式)
* @说明      : 原始角度，不限幅
* @参数      : None
* @返回值    : void
*******************************************************************************/
static inline float robstride_normalize_angle_diff(float diff)
{
    while (diff >  M_PI) diff -= 2.0f * M_PI;
    while (diff < -M_PI) diff += 2.0f * M_PI;
    return diff;
}

static inline float robstride_clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}


float virtual_angle[2] = {0.0f, 0.0f};

/**
 * robstride_keepalive() — 仅初始化+保持通信，不做运动
 * 2006未到位时调用，完成 initialized 初始化，保持灵足不超时
 */
void robstride_keepalive(uint8_t motor_idx)
{
    if (motor_idx > 1) return;
    uint8_t can_id = motor_idx + 1;

    /* 首次调用：记录当前角度作为初始值 */
    static uint8_t ka_init[2] = {0, 0};
    if (!ka_init[motor_idx]) {
        virtual_angle[motor_idx] = Pos_Info[can_id].Angle;
        ka_init[motor_idx] = 1;
    }

    /* 用当前角度发 MIT 指令，保持通信+原地不动 */
    RobStrite_Motor_05_move_control(&Robstirde_Motor_05_hfdcan, 0,
        Pos_Info[can_id].Angle, 0, 100.0f, 4.5f, can_id);
}

volatile float arm_close[2] = {0, 0};  /* [0]=粉色臂 [1]=蓝色臂: 0=展开, 1=收回 */
/* 重力补偿参数 (每臂独立, 初始值待实测标定) */
float K_GRAVITY[2] = {1.5f, 2.0f};  /* [0]=粉色臂 [1]=蓝色臂, 单位 Nm */
///*******************************************************************************
//* @功能      : 双臂灵足05平滑转到目标角度 (MIT运控模式)
//* @参数1     : tgt 目标角度 (rad)
//* @参数2     : motor_idx 电机索引 (0=粉色臂CAN ID=1, 1=蓝色臂CAN ID=2)
//* @返回值   : void
//*******************************************************************************/
void robstride_goto_target(float tgt, uint8_t motor_idx)
{
    /* 控制参数 */
    const float MAX_WRIST_SPEED = 3.5f;
    const float WRIST_ANGLE_EPS = 0.004f;
    const float STABLE_ALPHA    = 0.10f;
    const float DT              = 0.002f;
    const float Kp              = 100.0f;
    const float Kd              = 4.5f;

    static float stable_target[2] = {0.0f, 0.0f};
    static uint8_t initialized[2] = {0, 0};

    /* 索引保护 */
    if (motor_idx > 1) return;
    uint8_t can_id = motor_idx + 1;  /* motor_idx=0→CAN ID=1(粉色臂), motor_idx=1→CAN ID=2(蓝色臂) */

    /* 首次初始化：直接用目标角度（先初始化，再检查数据） */
    if (!initialized[motor_idx])
    {
        stable_target[motor_idx] = tgt;
        virtual_angle[motor_idx] = Pos_Info[can_id].Angle;
        initialized[motor_idx] = 1;
    }

    /* CAN 数据有效性检查：初始化后再检查 */
    if (Pos_Info[can_id].Angle == 0.0f &&
        Pos_Info[can_id].Speed == 0.0f &&
        Pos_Info[can_id].Torque == 0.0f)
    {
        return;
    }

    /* 目标平滑更新 */
    float error_to_target = tgt - stable_target[motor_idx];
    if (fabsf(error_to_target) < 0.01f)
        stable_target[motor_idx] = mid_target[motor_idx];
    else
        stable_target[motor_idx] += STABLE_ALPHA * error_to_target;

    /* 限速插值 + 减速区 */
    float delta_total = stable_target[motor_idx] - virtual_angle[motor_idx];
    float abs_rem = fabsf(delta_total);
    float max_step = MAX_WRIST_SPEED * DT;

    /* 读取真空阀状态判断负载 */
    uint8_t has_load;
    if (motor_idx == 0)
        has_load = (HAL_GPIO_ReadPin(VACUUM1_PORT, VACUUM1_PIN) == GPIO_PIN_RESET);  // PF3 LOW=吸住
    else
        has_load = (HAL_GPIO_ReadPin(VACUUM2_PORT, VACUUM2_PIN) == GPIO_PIN_RESET);  // PF4 LOW=吸住

    float DECEL_ZONE = has_load ? 1.5f : 0.5f;

    /* 减速区: 剩余距离进入减速区时减速 */
    if (abs_rem < DECEL_ZONE && abs_rem > WRIST_ANGLE_EPS) {
        float ratio = abs_rem / DECEL_ZONE;
        float scale = 0.1f + 0.90f * sqrtf(ratio);  /* 开根号减速 */
        max_step *= scale;
    }

    if (abs_rem > WRIST_ANGLE_EPS)
    {
        float step = fminf(max_step, abs_rem);
        virtual_angle[motor_idx] += (delta_total > 0) ? step : -step;
    }
    else
    {
        virtual_angle[motor_idx] = stable_target[motor_idx];
    }

    /* 速度前馈 */
    float dummy_speed = Pos_Info[can_id].Speed;

    float torque_ff = K_GRAVITY[motor_idx];
    /* 发送MIT运控指令 (带重力补偿) */
    RobStrite_Motor_05_move_control(&Robstirde_Motor_05_hfdcan,
                                    torque_ff,
                                    virtual_angle[motor_idx],
                                    dummy_speed,
                                    Kp, Kd,
                                    can_id);
}

//float h;
//float virtual_angle[2] = {0.0f, 0.0f};
//volatile float arm_close[2] = {0, 0};  /* [0]=粉色臂 [1]=蓝色臂: 0=展开, 1=收回 */

///* 重力补偿参数 (每臂独立, 初始值待实测标定) */
//float K_GRAVITY[2] = {1.5f, 1.5f};  /* [0]=粉色臂 [1]=蓝色臂, 单位 Nm */

///* 全局变量: S曲线状态 (非static，复位后自动归零) */
//float rob_stable_target[2] = {0.0f, 0.0f};
//float rob_start_pos[2] = {0.0f, 0.0f};
//float rob_prev_target[2] = {0.0f, 0.0f};
//uint8_t rob_initialized[2] = {0, 0};

///*******************************************************************************
//* @功能      : 双臂灵足05平滑转到目标角度 (MIT运控模式)
//* @参数1     : tgt 目标角度 (rad)
//* @参数2     : motor_idx 电机索引 (0=粉色臂CAN ID=1, 1=蓝色臂CAN ID=2)
//* @返回值   : void
//*******************************************************************************/
//void robstride_goto_target(float tgt, uint8_t motor_idx)
//{
//    /* 控制参数 */
//    const float MAX_WRIST_SPEED = 3.0f;
//    const float WRIST_ANGLE_EPS = 0.004f;
//    const float STABLE_ALPHA    = 0.10f;
//    const float DT              = 0.002f;
//    const float Kp              = 100.0f;
//    const float Kd              = 4.5f;
//    const float ACCEL_DIST      = 0.4f;

//    /* 索引保护 */
//    if (motor_idx > 1) return;
//    uint8_t can_id = motor_idx + 1;

//    /* CAN 数据有效性检查 */
//    if (Pos_Info[can_id].Angle == 0.0f &&
//        Pos_Info[can_id].Speed == 0.0f &&
//        Pos_Info[can_id].Torque == 0.0f)
//    {
//        return;
//    }

//    /* 首次初始化：从当前实际角度开始 (全局变量，复位后自动归零) */
//    if (!rob_initialized[motor_idx])
//    {
//        rob_stable_target[motor_idx] = Pos_Info[can_id].Angle;
//        rob_start_pos[motor_idx]     = Pos_Info[can_id].Angle;
//        rob_prev_target[motor_idx]   = tgt;
//        rob_initialized[motor_idx]   = 1;
//    }

//    /* 目标平滑更新 */
//    float error_to_target = tgt - rob_stable_target[motor_idx];
//    if (fabsf(error_to_target) < 0.01f)
//        rob_stable_target[motor_idx] = tgt;
//    else
//        rob_stable_target[motor_idx] += STABLE_ALPHA * error_to_target;

//    /* 检测新运动起点 */
//    if (fabsf(rob_stable_target[motor_idx] - rob_prev_target[motor_idx]) > 0.01f)
//        rob_start_pos[motor_idx] = Pos_Info[can_id].Angle;
//    rob_prev_target[motor_idx] = rob_stable_target[motor_idx];

//    /* ---- S曲线梯形加减速 (用Pos_Info实时角度) ---- */
//    float cur = Pos_Info[can_id].Angle;
//    float delta_total = rob_stable_target[motor_idx] - cur;
//    float abs_remaining = fabsf(delta_total);
//    float max_step = MAX_WRIST_SPEED * DT;
//    float virtual_pos;

//    if (abs_remaining > WRIST_ANGLE_EPS)
//    {
//        float total_dist = fabsf(rob_stable_target[motor_idx] - rob_start_pos[motor_idx]);
//        float traveled   = fabsf(cur - rob_start_pos[motor_idx]);

//        float accel_region = fminf(ACCEL_DIST, total_dist * 0.4f);
//        float decel_region = fminf(ACCEL_DIST, total_dist * 0.3f);
//        float scale = 1.0f;

//        if (traveled < accel_region && accel_region > 0.0f)
//            scale = 0.3f + 0.7f * (traveled / accel_region);
//        else if (abs_remaining < decel_region && decel_region > 0.0f)
//            scale = 0.3f + 0.7f * (abs_remaining / decel_region);

//        float step = fminf(max_step * scale, abs_remaining);
//        virtual_pos = cur + ((delta_total > 0) ? step : -step);
//    }
//    else
//    {
//        virtual_pos = rob_stable_target[motor_idx];
//    }

//    /* 速度前馈 */
//    float speed_ff = robstride_clampf(delta_total / DT, -MAX_WRIST_SPEED, MAX_WRIST_SPEED);

//    /* 重力补偿前馈力矩 */
//    float torque_ff = K_GRAVITY[motor_idx] * cosf(virtual_pos);

//    /* 发送MIT运控指令 */
//    RobStrite_Motor_05_move_control(&Robstirde_Motor_05_hfdcan,
//                                    torque_ff,
//                                    virtual_pos,
//                                    speed_ff,
//                                    Kp, Kd,
//                                    can_id);
//    h++;
//}


