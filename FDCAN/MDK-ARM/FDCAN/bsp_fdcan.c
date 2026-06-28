#include "bsp_fdcan.h"
#include "fdcan.h"
#include "ROBSTRIDE.h"
#include "cmsis_os2.h"
// 电机数据接收 (与参考项目一致)
motor_control control_3508_classic[5];

// 兼容变量: 供 raise_task.c 使用
motor_measure_t chassis_3508_motor[8];
int32_t total_ecd = 0;
int CycleNum = 0;

// 初始化列表
static motor_control control_3508_classic_init[5] = {
	[0].stage = STAGE_INIT,
	[0].ZERO_FLAG = false,
	[0].count_num = 0,
	[0].CycleNum = 0,
	[0].total_ecd = 0,
	[0].ZERO_ecp = 0,
	[0].target_speed = 0,
	[0].dic_flag = 0,

	[1].stage = STAGE_INIT,
	[1].ZERO_FLAG = false,
	[1].count_num = 0,
	[1].CycleNum = 0,
	[1].total_ecd = 0,
	[1].ZERO_ecp = 0,
	[1].target_speed = 0,
	[1].dic_flag = 0,

	[2].stage = STAGE_INIT,
	[2].ZERO_FLAG = false,
	[2].count_num = 0,
	[2].CycleNum = 0,
	[2].total_ecd = 0,
	[2].ZERO_ecp = 0,
	[2].target_speed = 0,
	[2].dic_flag = 0,

	[3].stage = STAGE_INIT,
	[3].ZERO_FLAG = false,
	[3].count_num = 0,
	[3].CycleNum = 0,
	[3].total_ecd = 0,
	[3].ZERO_ecp = 0,
	[3].target_speed = 0,
	[3].dic_flag = 0,

	[4].stage = STAGE_INIT,
	[4].ZERO_FLAG = false,
	[4].count_num = 0,
	[4].CycleNum = 0,
	[4].total_ecd = 0,
	[4].ZERO_ecp = 0,
	[4].target_speed = 0,
	[4].dic_flag = 0,
};

// fdcan配置
void fdcan_config(void)
{
	FDCAN_FilterTypeDef FDCAN1_FilterConfig;
	FDCAN1_FilterConfig.IdType = FDCAN_EXTENDED_ID;
	FDCAN1_FilterConfig.FilterIndex = 0;
	FDCAN1_FilterConfig.FilterType = FDCAN_FILTER_MASK;
	FDCAN1_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	FDCAN1_FilterConfig.FilterID1 = 0x00000000;
	FDCAN1_FilterConfig.FilterID2 = 0x00000000;

	HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN1_FilterConfig);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_Start(&hfdcan1);

	FDCAN_FilterTypeDef FDCAN2_FilterConfig;
	FDCAN2_FilterConfig.IdType = FDCAN_STANDARD_ID;
	FDCAN2_FilterConfig.FilterIndex = 0;
	FDCAN2_FilterConfig.FilterType = FDCAN_FILTER_MASK;
	FDCAN2_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	FDCAN2_FilterConfig.FilterID1 = 0x00000000;
	FDCAN2_FilterConfig.FilterID2 = 0x00000000;

	HAL_FDCAN_ConfigFilter(&hfdcan2, &FDCAN2_FilterConfig);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_Start(&hfdcan2);

	FDCAN_FilterTypeDef FDCAN3_FilterConfig;
	FDCAN3_FilterConfig.IdType = FDCAN_STANDARD_ID;
	FDCAN3_FilterConfig.FilterIndex = 0;
	FDCAN3_FilterConfig.FilterType = FDCAN_FILTER_MASK;
	FDCAN3_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	FDCAN3_FilterConfig.FilterID1 = 0x00000000;
	FDCAN3_FilterConfig.FilterID2 = 0x00000000;

	HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN3_FilterConfig);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_Start(&hfdcan3);
}

// 数据接收函数 (与参考项目Get_motor_measure一致)
static void Get_motor_measure(motor_control *ptr, uint8_t *data)
{
	motor_measure_t *m = &ptr->chassis_3508_motor;
	m->last_ecd = m->ecd;
	m->ecd = data[0] << 8 | data[1];
	m->speed_rpm = data[2] << 8 | data[3];
	m->given_current = data[4] << 8 | data[5];
	m->temperate = data[6];

	// 零点处理 (与参考项目完全一致)
	if (m->ecd - m->last_ecd >= 4096)
	{
		ptr->CycleNum--;
	}
	if (m->ecd - m->last_ecd < -4096)
	{
		ptr->CycleNum++;
	}
	ptr->total_ecd = ptr->CycleNum * 8192 + m->ecd;

	// 零点检测 (仅id=2和id=4的电机，与参考项目一致)
	switch (m->id)
	{
	case 2:
	case 4:
		if (ptr->ZERO_FLAG == false)
		{
			if (ptr->count_num >= 100)
			{
				ptr->ZERO_FLAG = true;
			}
			else if (m->speed_rpm)
			{
				ptr->count_num = 0;
			}
			else
			{
				ptr->count_num++;
			}
		}
		break;
	default:
		break;
	}
}

// 这是大疆格式
FDCAN_TxHeaderTypeDef CANx_tx_message;
uint8_t Canx_send_data[8];
void CAN_CMD_RM(FDCAN_HandleTypeDef *hfdcan, uint32_t STDID, int16_t M1, int16_t M2, int16_t M3, int16_t M4)
{
	CANx_tx_message.Identifier = STDID;
	CANx_tx_message.IdType = FDCAN_STANDARD_ID;
	CANx_tx_message.TxFrameType = FDCAN_DATA_FRAME;
	CANx_tx_message.DataLength = 0x08;
	CANx_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	CANx_tx_message.BitRateSwitch = FDCAN_BRS_OFF;
	CANx_tx_message.FDFormat = FDCAN_CLASSIC_CAN;
	CANx_tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	CANx_tx_message.MessageMarker = 0;

	Canx_send_data[0] = M1 >> 8;
	Canx_send_data[1] = M1;
	Canx_send_data[2] = M2 >> 8;
	Canx_send_data[3] = M2;
	Canx_send_data[4] = M3 >> 8;
	Canx_send_data[5] = M3;
	Canx_send_data[6] = M4 >> 8;
	Canx_send_data[7] = M4;
	HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &CANx_tx_message, Canx_send_data);
}

// 同步兼容变量 (供 raise_task.c 使用)
static void SyncCompatVars(int idx)
{
	motor_measure_t *m = &control_3508_classic[idx].chassis_3508_motor;
	chassis_3508_motor[idx].ecd = m->ecd;
	chassis_3508_motor[idx].last_ecd = m->last_ecd;
	chassis_3508_motor[idx].speed_rpm = m->speed_rpm;
	chassis_3508_motor[idx].given_current = m->given_current;
	chassis_3508_motor[idx].temperate = m->temperate;
	total_ecd = control_3508_classic[2].total_ecd;
	CycleNum = control_3508_classic[2].CycleNum;
}

volatile uint32_t can3_last_tick = 0;  /* CAN3最后一次收到2006数据的时刻 */

// CAN3接收回调 (与参考项目Fdcan1_rx_callback一致，但使用FDCAN3)
static void Fdcan3_rx_callback(void)
{
  can3_last_tick = osKernelGetTickCount();  /* 记录2006数据更新时间戳 */
	FDCAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];
	HAL_FDCAN_GetRxMessage(&hfdcan3, FDCAN_RX_FIFO0, &rx_header, rx_data);
	switch (rx_header.Identifier)
	{
	case 0x201:
		control_3508_classic[0].chassis_3508_motor.id = 1;
		Get_motor_measure(&control_3508_classic[0], rx_data);
		SyncCompatVars(0);
		break;
	case 0x202:
		control_3508_classic[1].chassis_3508_motor.id = 2;
		Get_motor_measure(&control_3508_classic[1], rx_data);
		SyncCompatVars(1);
		break;
	case 0x203:
		control_3508_classic[2].chassis_3508_motor.id = 3;
		Get_motor_measure(&control_3508_classic[2], rx_data);
		SyncCompatVars(2);
		break;
	case 0x204:
		control_3508_classic[3].chassis_3508_motor.id = 4;
		Get_motor_measure(&control_3508_classic[3], rx_data);
		SyncCompatVars(3);
		break;
	case 0x205:
		control_3508_classic[4].chassis_3508_motor.id = 5;
		Get_motor_measure(&control_3508_classic[4], rx_data);
		SyncCompatVars(4);
		break;
	default:
		break;
	}
}

float a;
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if (hfdcan == &hfdcan1)
	{
		// FDCAN1 回调 - 循环读取FIFO中所有消息
		FDCAN_RxHeaderTypeDef rx_header;
		uint8_t rx_data_buf[8];
		while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0)
		{
			if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx_header, rx_data_buf) != HAL_OK)
				break;
			if (rx_header.IdType == FDCAN_EXTENDED_ID)
			{
				Robstirde_Motor_05_process_frame(&rx_header, rx_data_buf);
				a++;
			}
		}
	}
	if (hfdcan == &hfdcan3)
	{
		Fdcan3_rx_callback();
	}
}