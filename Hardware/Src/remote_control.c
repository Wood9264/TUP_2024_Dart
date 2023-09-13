/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       remote_control.c/h
  * @brief      遥控器处理，遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note       该任务是通过串口中断启动，不是freeRTOS任务
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-01-2019     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "remote_control.h"
#include "usart.h"
#include "detect_task.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

// remote control data
//遥控器控制变量
RC_ctrl_t rc_ctrl;

// receive data, 18 bytes one frame, but set 36 bytes
//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
void remote_control_init(void)
{
	RC_Init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
const RC_ctrl_t *get_remote_control_point(void)
{
	return &rc_ctrl;
}

//串口中断
void USART3_IRQHandler(void)
{
	if (huart3.Instance->SR & UART_FLAG_RXNE) //接收到数据
	{
		__HAL_UART_CLEAR_PEFLAG(&huart3);
	}
	else if (USART3->SR & UART_FLAG_IDLE)
	{
		static uint16_t this_time_rx_len = 0;

		__HAL_UART_CLEAR_PEFLAG(&huart3);

		if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
		{
			/* Current memory buffer used is Memory 0 */

			// disable DMA
			//失效DMA
			__HAL_DMA_DISABLE(&hdma_usart3_rx);

			// get receive data length, length = set_data_length - remain_length
			//获取接收数据长度,长度 = 设定长度 - 剩余长度
			this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

			// reset set_data_lenght
			//重新设定数据长度
			hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

			// set memory buffer 1
			//设定缓冲区1
			hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;

			// enable DMA
			//使能DMA
			__HAL_DMA_ENABLE(&hdma_usart3_rx);

			if (this_time_rx_len == RC_FRAME_LENGTH)
			{
				sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
				detect_hook(DBUS_TOE);
			}
		}
		else
		{
			/* Current memory buffer used is Memory 1 */
			// disable DMA
			//失效DMA
			__HAL_DMA_DISABLE(&hdma_usart3_rx);

			// get receive data length, length = set_data_length - remain_length
			//获取接收数据长度,长度 = 设定长度 - 剩余长度
			this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

			// reset set_data_lenght
			//重新设定数据长度
			hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

			// set memory buffer 0
			//设定缓冲区0
			DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

			// enable DMA
			//使能DMA
			__HAL_DMA_ENABLE(&hdma_usart3_rx);

			if (this_time_rx_len == RC_FRAME_LENGTH)
			{
				//处理遥控器数据
				sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
				detect_hook(DBUS_TOE);
			}
		}
	}
}

/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
	if (sbus_buf == NULL || rc_ctrl == NULL)
	{
		return;
	}

	rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;		//!< Channel 0
	rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
	rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |			//!< Channel 2
						 (sbus_buf[4] << 10)) &
						0x07ff;
	rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
	rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);						//!< Switch left
	rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;					//!< Switch right
	rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);					//!< Mouse X axis
	rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);					//!< Mouse Y axis
	rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);					//!< Mouse Z axis
	rc_ctrl->mouse.press_l = sbus_buf[12];									//!< Mouse Left Is Press ?
	rc_ctrl->mouse.press_r = sbus_buf[13];									//!< Mouse Right Is Press ?
	rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);					//!< KeyBoard value
	rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);					// NULL

	rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
	rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
	rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
	rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
	rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}

/**
 * @brief       判断按键是否被按下或摇杆是否在指定位置
 * @param[in]   channel：要判断的通道
 * @retval      指定通道被激活返回1，否则返回0
 * @attention   本函数只由RC_time_count函数调用
 */
static bool_t is_channel_activate(CHANNEL_int channel)
{
	switch (channel)
	{
	case MOUSE_L:
		return IF_MOUSE_PRESSED_LEFT;
	case MOUSE_R:
		return IF_MOUSE_PRESSED_RIGH;
	case RIGHT_ROCKER_RIGHT_TOP:
		return IF_RIGHT_ROCKER_RIGHT_TOP;
	case RIGHT_ROCKER_LEFT_TOP:
		return IF_RIGHT_ROCKER_LEFT_TOP;
	case RIGHT_ROCKER_LEFT_BOTTOM:
		return IF_RIGHT_ROCKER_LEFT_BOTTOM;
	case RIGHT_ROCKER_RIGHT_BOTTOM:
		return IF_RIGHT_ROCKER_RIGHT_BOTTOM;
	case LEFT_ROCKER_RIGHT_TOP:
		return IF_LEFT_ROCKER_RIGHT_TOP;
	case LEFT_ROCKER_LEFT_TOP:
		return IF_LEFT_ROCKER_LEFT_TOP;
	case LEFT_ROCKER_LEFT_BOTTOM:
		return IF_LEFT_ROCKER_LEFT_BOTTOM;
	case LEFT_ROCKER_RIGHT_BOTTOM:
		return IF_LEFT_ROCKER_RIGHT_BOTTOM;
	default:
		return (rc_ctrl.key.v & ((uint16_t)1 << channel) != 0);
	}
}

/**
 * @brief       判断按键保持按下或摇杆保持在指定位置的时间是否超过阈值
 * @param[in]   channel：要检测的通道
 * @param[in]   time：保持的时间阈值
 * @retval      如果通道连续保持了指定的时间，返回1；否则返回0
 */
bool_t RC_held_continuous_return(CHANNEL_int channel, uint16_t time)
{
	static uint16_t active_time[25];

	if (is_channel_activate(channel))
	{
		if (active_time[channel] < time)
		{
			active_time[channel]++;
			return 0;
		}
		else if (active_time[channel] >= time)
		{
			return 1;
		}
	}
	else
	{
		active_time[channel] = 0;
		return 0;
	}
}

/**
 * @brief       判断按键保持按下或摇杆保持在指定位置的时间是否超过阈值，并在满足条件时只返回一次1
 * @param[in]   channel：要检测的通道
 * @param[in]   time：保持的时间阈值
 * @retval      如果通道连续保持了指定的时间，只返回一次1；否则返回0
 */
bool_t RC_held_single_return(CHANNEL_int channel, uint16_t time)
{
	static bool_t has_returned_one[25];

	if (RC_held_continuous_return(channel, time))
	{
		if (has_returned_one[channel])
		{
			return 0;
		}
		else
		{
			has_returned_one[channel] = 1;
			return 1;
		}
	}
	else
	{
		has_returned_one[channel] = 0;
		return 0;
	}
}

/**
 * @brief       判断双组合键保持按下或左右摇杆同时保持在指定位置的时间是否超过阈值，并在满足条件时只返回一次1
 * @param[in]   channel：要检测的通道
 * @param[in]   time：保持的时间阈值
 * @retval      如果通道连续保持了指定的时间，只返回一次1；否则返回0
 */
bool_t RC_double_held_single_return(CHANNEL_int channel_1, CHANNEL_int channel_2, uint16_t time)
{
	static bool_t has_returned_one[25];

	if (RC_held_continuous_return(channel_1, time) && RC_held_continuous_return(channel_2, time))
	{
		if (has_returned_one[channel_1] || has_returned_one[channel_2])
		{
			return 0;
		}
		else
		{
			has_returned_one[channel_1] = 1;
			has_returned_one[channel_2] = 1;
			return 1;
		}
	}
	else
	{
		has_returned_one[channel_1] = 0;
		has_returned_one[channel_2] = 0;
		return 0;
	}
}
