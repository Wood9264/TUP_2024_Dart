#include "BSP_can.h"
#include "struct_typedef.h"
#include "main.h"
#include "cmsis_os.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

static CAN_TxHeaderTypeDef CAN1_200_tx_message;
static uint8_t CAN1_200_send_data[8];
static CAN_TxHeaderTypeDef CAN1_1FF_tx_message;
static uint8_t CAN1_1FF_send_data[8];
static CAN_TxHeaderTypeDef CAN2_200_tx_message;
static uint8_t CAN2_200_send_data[8];
static CAN_TxHeaderTypeDef CAN2_1FF_tx_message;
static uint8_t CAN2_1FF_send_data[8];
static CAN_TxHeaderTypeDef Reset_tx_message;
static uint8_t Reset_tx_send_data[8];
static CAN_TxHeaderTypeDef chassis_tx_message;
static uint8_t chassis_can_send_data[8];

void can_filter_init(void)
{

	CAN_FilterTypeDef can_filter_st;
	can_filter_st.FilterActivation = ENABLE;
	can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
	can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
	can_filter_st.FilterIdHigh = 0x0000;
	can_filter_st.FilterIdLow = 0x0000;
	can_filter_st.FilterMaskIdHigh = 0x0000;
	can_filter_st.FilterMaskIdLow = 0x0000;
	can_filter_st.FilterBank = 0;
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	can_filter_st.SlaveStartFilterBank = 14;
	can_filter_st.FilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief          hal库CAN回调函数,接收电机数据
 * @param[in]      hcan:CAN句柄指针
 * @retval         none
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	detect_hook(CAN_TOE);
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data1[8];
	uint8_t rx_data2[8];

	if (hcan->Instance == CAN1)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data1);

		switch (rx_header.StdId)
		{
		case CAN1_3508_BL_ID:
		{
			get_motor_measure(BL, rx_data1);
			break;
		}
		case CAN1_3508_BR_ID:
		{
			get_motor_measure(BR, rx_data1);
			break;
		}
		case CAN1_3508_FR_ID:
		{
			get_motor_measure(FR, rx_data1);
			break;
		}
		case CAN1_3508_FL_ID:
		{
			get_motor_measure(SLIPPER_MOTOR, rx_data1);
			break;
		}
		}
	}

	if (hcan->Instance == CAN2)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data2);

		switch (rx_header.StdId)
		{
		case CAN2_SLIPPER_MOTOR_ID:
		{
			get_motor_measure(SLIPPER_MOTOR, rx_data2);
			break;
		}
		case CAN2_YAW_MOTOR_ID:
		{
			get_motor_measure(YAW, rx_data2);
			break;
		}
		case CAN2_TRIGGER_MOTOR_ID:
		{
			get_motor_measure(TRIGGER, rx_data2);
			break;
		}
		case CAN2_PITCH_Motor_ID:
		{
			get_motor_measure(PITCH, rx_data2);
			break;
		}
		}
	}
}

//控制CAN1ID:1-4
/**
 * @brief          发送电机控制电流（控制CAN1 ID:1-4）
 * @param[in]      can1_motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      can1_motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      can1_motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      can1_motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
 * @retval         none
 */
void CAN1_200_cmd_motor(int16_t can1_motor1, int16_t can1_motor2, int16_t can1_motor3, int16_t can1_motor4)
{
	uint32_t send_mail_box;
	CAN1_200_tx_message.StdId = 0x200;
	CAN1_200_tx_message.IDE = CAN_ID_STD;
	CAN1_200_tx_message.RTR = CAN_RTR_DATA;
	CAN1_200_tx_message.DLC = 0x08;
	CAN1_200_send_data[0] = (can1_motor1 >> 8);
	CAN1_200_send_data[1] = can1_motor1;
	CAN1_200_send_data[2] = (can1_motor2 >> 8);
	CAN1_200_send_data[3] = can1_motor2;
	CAN1_200_send_data[4] = (can1_motor3 >> 8);
	CAN1_200_send_data[5] = can1_motor3;
	CAN1_200_send_data[6] = (can1_motor4 >> 8);
	CAN1_200_send_data[7] = can1_motor4;
	HAL_CAN_AddTxMessage(&CHASSIS_CAN, &CAN1_200_tx_message, CAN1_200_send_data, &send_mail_box);
}

//控制CAN1ID:5-8
/**
 * @apply          can1_motor5:YAW  can1_motor6:PITCH
 * @brief          发送电机控制电流（控制CAN1 ID:5-8）
 * @param[in]      can1_motor5: (0x205) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      can1_motor6: (0x206) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      can1_motor7: (0x207) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      can1_motor8: (0x208) 3508电机控制电流, 范围 [-16384,16384]
 * @retval         none
 */
void CAN1_1FF_cmd_motor(int16_t can1_motor5, int16_t can1_motor6, int16_t can1_motor7, int16_t can1_motor8)
{
	uint32_t send_mail_box;
	CAN1_1FF_tx_message.StdId = 0x1FF;
	CAN1_1FF_tx_message.IDE = CAN_ID_STD;
	CAN1_1FF_tx_message.RTR = CAN_RTR_DATA;
	CAN1_1FF_tx_message.DLC = 0x08;
	CAN1_1FF_send_data[0] = (can1_motor5 >> 8);
	CAN1_1FF_send_data[1] = can1_motor5;
	CAN1_1FF_send_data[2] = (can1_motor6 >> 8);
	CAN1_1FF_send_data[3] = can1_motor6;
	CAN1_1FF_send_data[4] = (can1_motor7 >> 8);
	CAN1_1FF_send_data[5] = can1_motor7;
	CAN1_1FF_send_data[6] = (can1_motor8 >> 8);
	CAN1_1FF_send_data[7] = can1_motor8;
	HAL_CAN_AddTxMessage(&CHASSIS_CAN, &CAN1_1FF_tx_message, CAN1_1FF_send_data, &send_mail_box);
}

//控制CAN2ID:1-4
/**
 * @apply          can2_motor1:FireL  can2_motor2:FireR
 * @brief          发送电机控制电流（控制CAN2 ID:1-4）
 * @param[in]      can2_motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      can2_motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      can2_motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      can2_motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
 * @retval         none
 */
void CAN2_200_cmd_motor(int16_t can2_motor1, int16_t can2_motor2, int16_t can2_motor3, int16_t can2_motor4)
{
	uint32_t send_mail_box;
	CAN2_200_tx_message.StdId = 0x200;
	CAN2_200_tx_message.IDE = CAN_ID_STD;
	CAN2_200_tx_message.RTR = CAN_RTR_DATA;
	CAN2_200_tx_message.DLC = 0x08;
	CAN2_200_send_data[0] = (can2_motor1 >> 8);
	CAN2_200_send_data[1] = can2_motor1;
	CAN2_200_send_data[2] = (can2_motor2 >> 8);
	CAN2_200_send_data[3] = can2_motor2;
	CAN2_200_send_data[4] = (can2_motor3 >> 8);
	CAN2_200_send_data[5] = can2_motor3;
	CAN2_200_send_data[6] = (can2_motor4 >> 8);
	CAN2_200_send_data[7] = can2_motor4;
	HAL_CAN_AddTxMessage(&GIMBAL_CAN, &CAN2_200_tx_message, CAN2_200_send_data, &send_mail_box);
}

// c
/**
 * @apply          暂未使用
 * @brief          发送电机控制电流（控制CAN2 ID:5-8）
 * @param[in]      can2_motor5: (0x205) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      can2_motor6: (0x206) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      can2_motor7: (0x207) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      can2_motor8: (0x208) 3508电机控制电流, 范围 [-16384,16384]
 * @retval         none
 */
void CAN2_1FF_cmd_motor(int16_t can2_motor5, int16_t can2_motor6, int16_t can2_motor7, int16_t can2_motor8)
{
	uint32_t send_mail_box;
	CAN2_1FF_tx_message.StdId = 0x1FF;
	CAN2_1FF_tx_message.IDE = CAN_ID_STD;
	CAN2_1FF_tx_message.RTR = CAN_RTR_DATA;
	CAN2_1FF_tx_message.DLC = 0x08;
	CAN2_1FF_send_data[0] = (can2_motor5 >> 8);
	CAN2_1FF_send_data[1] = can2_motor5;
	CAN2_1FF_send_data[2] = (can2_motor6 >> 8);
	CAN2_1FF_send_data[3] = can2_motor6;
	CAN2_1FF_send_data[4] = (can2_motor7 >> 8);
	CAN2_1FF_send_data[5] = can2_motor7;
	CAN2_1FF_send_data[6] = (can2_motor8 >> 8);
	CAN2_1FF_send_data[7] = can2_motor8;
	HAL_CAN_AddTxMessage(&GIMBAL_CAN, &CAN2_1FF_tx_message, CAN2_1FF_send_data, &send_mail_box);
}

//返回对应电机地址
const motor_t *get_motor_measure_class(uint16_t type)
{
	return &motor[type];
}

/**
 * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
 * @param[in]      none
 * @retval         none
 */
void CAN_cmd_chassis_reset_ID(void)
{
	uint32_t send_mail_box;
	Reset_tx_message.StdId = 0x700;
	Reset_tx_message.IDE = CAN_ID_STD;
	Reset_tx_message.RTR = CAN_RTR_DATA;
	Reset_tx_message.DLC = 0x08;
	Reset_tx_send_data[0] = 0;
	Reset_tx_send_data[1] = 0;
	Reset_tx_send_data[2] = 0;
	Reset_tx_send_data[3] = 0;
	Reset_tx_send_data[4] = 0;
	Reset_tx_send_data[5] = 0;
	Reset_tx_send_data[6] = 0;
	Reset_tx_send_data[7] = 0;

	HAL_CAN_AddTxMessage(&CHASSIS_CAN, &Reset_tx_message, Reset_tx_send_data, &send_mail_box);
}
