#include "pm01.h"
#include "judge.h"
volatile pm01_od_t pm01_od;

volatile uint16_t  pm01_access_id;    /* 正在访问的标识符     */
volatile uint16_t  pm01_response_flg; /* 控制器响应成功标志位 */

uint16_t g_cmd_set   = 2;
uint16_t g_power_set = 12000;
uint16_t g_vout_set  = 2400;
uint16_t g_iout_set  = 500;

/**
  * @brief          控制命令发送
  * @param[in]      new_cmd   0x00: 停机
															0x01: 运行，不打开输出负载开关（只给超级电容充电）
															0x02: 运行，打开输出负载开关（正常运行使用该指令）
	                  save_flg: 0x00: 不保存至EEPROM  0x01: 保存至EEPROM
  * @retval         none
  */
void pm01_cmd_send( uint16_t new_cmd, uint8_t save_flg )
{
	uint32_t send_mail_box;

	CAN_TxHeaderTypeDef  power_tx_message;
	uint8_t              power_can_send_data[8];

	power_tx_message.StdId = 0x600;
	power_tx_message.IDE   = CAN_ID_STD;
	power_tx_message.RTR   = CAN_RTR_DATA;
	power_tx_message.DLC   = 0x04;
	power_can_send_data[0] = (uint8_t)(new_cmd >> 8   );
	power_can_send_data[1] = (uint8_t)(new_cmd &  0xFF);
	power_can_send_data[2] = 0x00;
	power_can_send_data[3] = (save_flg == 0x01);			

	HAL_CAN_AddTxMessage(&hcan2, &power_tx_message, power_can_send_data, &send_mail_box);
}

/**
  * @brief          设置功率
  * @param[in]      new_power：新的功率值
                    save_flg: 0x00: 不保存至EEPROM  0x01: 保存至EEPROM
  * @retval         none
  */
void pm01_power_set( uint16_t new_power, uint8_t save_flg )
{
	uint32_t send_mail_box;

	CAN_TxHeaderTypeDef  power_tx_message;
	uint8_t              power_can_send_data[8];

	power_tx_message.StdId = 0x601;
	power_tx_message.IDE   = CAN_ID_STD;
	power_tx_message.RTR   = CAN_RTR_DATA;
	power_tx_message.DLC   = 0x04;
	power_can_send_data[0] = (uint8_t)(new_power >> 8   );
	power_can_send_data[1] = (uint8_t)(new_power &  0xFF);
	power_can_send_data[2] = 0x00;
	power_can_send_data[3] = (save_flg == 0x01);			

	HAL_CAN_AddTxMessage(&hcan2, &power_tx_message, power_can_send_data, &send_mail_box);
	
}
/**
  * @brief          设置输出电压
  * @param[in]      new_volt：新的电压值
                    save_flg: 0x00: 不保存至EEPROM  0x01: 保存至EEPROM
  * @retval         none
  */
void pm01_voltage_set( uint16_t new_voltage, uint8_t save_flg )
{
	uint32_t send_mail_box;

	CAN_TxHeaderTypeDef  power_tx_message;
	uint8_t              power_can_send_data[8];

	power_tx_message.StdId = 0x602;
	power_tx_message.IDE   = CAN_ID_STD;
	power_tx_message.RTR   = CAN_RTR_DATA;
	power_tx_message.DLC   = 0x04;
	power_can_send_data[0] = (uint8_t)(new_voltage >> 8   );
	power_can_send_data[1] = (uint8_t)(new_voltage &  0xFF);
	power_can_send_data[2] = 0x00;
	power_can_send_data[3] = (save_flg == 0x01);			


	HAL_CAN_AddTxMessage(&hcan2, &power_tx_message, power_can_send_data, &send_mail_box);
	
}
/**
  * @brief          设置输出电流
  * @param[in]      new_current：新的电流值
                    save_flg: 0x00: 不保存至EEPROM  0x01: 保存至EEPROM
  * @retval         none
  */
void pm01_current_set( uint16_t new_current, uint8_t save_flg )
{
	uint32_t send_mail_box;

	CAN_TxHeaderTypeDef  power_tx_message;
	uint8_t              power_can_send_data[8];

	power_tx_message.StdId = 0x603;
	power_tx_message.IDE   = CAN_ID_STD;
	power_tx_message.RTR   = CAN_RTR_DATA;
	power_tx_message.DLC   = 0x04;
	power_can_send_data[0] = (uint8_t)(new_current >> 8   );
	power_can_send_data[1] = (uint8_t)(new_current &  0xFF);
	power_can_send_data[2] = 0x00;
	power_can_send_data[3] = (save_flg == 0x01);			

	HAL_CAN_AddTxMessage(&hcan2, &power_tx_message, power_can_send_data, &send_mail_box);
	
}
/**
  * @brief          轮询访问PM01参数，请以100HZ频率调用
  * @param[in]      none
  * @retval         none
  */
void pm01_access_poll(void)
{
	
	static uint8_t   m_state   = 0x00;   /* 状态    */
	static uint16_t  m_can_id  = 0x600;  /* 标识符  */
	static uint32_t  m_timeout = 0;      /* 超时    */
	
	static uint16_t  i = 0;
//	uint16_t chassis_limit_power;
//	chassis_limit_power = JUDGE_usGetChassisPowerLimit();
//	g_power_set = chassis_limit_power*1000;
//	g_iout_set = g_power_set/24*1000;
	CAN_TxHeaderTypeDef  power_tx_message;

	uint8_t              power_can_send_data[8];

	uint32_t             send_mail_box;
	
	switch( m_state )
	{
		case 0x00:  /* 发送数据 */
		
		  m_timeout = 0;   /* 清除超时定时器 */		

		  m_state = 0x01;  /* 切换到等待状态 */

		  switch(i)
			{
					case 0:
					case 1:
					case 2:
					case 3:
					case 4:
					case 5:
					case 6:
					case 7:
							pm01_access_id = m_can_id; /* 记录本次访问的标识符 */

							power_tx_message.StdId = m_can_id;       /* 发送相应的标识符 */
							power_tx_message.IDE   = CAN_ID_STD;     /* 标准帧           */
							power_tx_message.RTR   = CAN_RTR_REMOTE; /* 远程帧           */
							power_tx_message.DLC   = 0x00;

							HAL_CAN_AddTxMessage(&hcan2, &power_tx_message, power_can_send_data, &send_mail_box);
					  break;
					case 8:
					
							pm01_access_id = m_can_id; /* 记录本次访问的标识符 */					
					
						  pm01_cmd_send( g_cmd_set, 0x00 );
					  break;
					case 9:
					
							pm01_access_id = m_can_id; /* 记录本次访问的标识符 */					
					
						  pm01_power_set( g_power_set, 0x00 );
					  break;
					case 10:
					
							pm01_access_id = m_can_id; /* 记录本次访问的标识符 */					
					
						  pm01_voltage_set( g_vout_set, 0x00 );
					  break;
					case 11:
					
							pm01_access_id = m_can_id; /* 记录本次访问的标识符 */					
					
						  pm01_current_set( g_iout_set, 0x00 );
					  break;						
			}
		
			break;
		case 0x01: /* 等待响应 */
		
	
      if( pm01_response_flg == 0x01 )
			{
				
				pm01_response_flg = 0x00; /* 复位应答标志位 */
				
				 /* 访问成功，继续访问下一个标识符 */
				
				i = (i+1) % 12;
				

				switch( i )
				{
					case 0: m_can_id = 0x600; break;
					case 1: m_can_id = 0x601; break;
					case 2: m_can_id = 0x602; break;
					case 3: m_can_id = 0x603; break;
					case 4: m_can_id = 0x610; break;
					case 5: m_can_id = 0x611; break;
					case 6: m_can_id = 0x612; break;
					case 7: m_can_id = 0x613; break;
				  case 8: m_can_id = 0x600; break; /* 写 */
					case 9: m_can_id = 0x601; break; /* 写 */
					case 10: m_can_id = 0x602; break; /* 写 */
					case 11: m_can_id = 0x603; break; /* 写 */
				}
				
				m_state = 0x00;  /* 启动下一轮访问 */							
				
			}
			else
			{
				m_timeout++;
			}

		  /* 超时1s，重新访问  */
      if( m_timeout > 100 )
			{
				m_state = 0x00;	
			}				
		
			break;
		default:
			m_state = 0x00;
	
	}
}
/**
  * @brief          应答处理，在CAN接收中断中调用
  * @param[in]      none
  * @retval         none
  */
void pm01_response_handle(CAN_RxHeaderTypeDef  *can_rx_header, uint8_t *can_rx_data )
{

	uint16_t m_tmp;
	
	pm01_response_flg = ( pm01_access_id == can_rx_header->StdId );
	
	if( can_rx_header->RTR == CAN_RTR_REMOTE )return;
	
	switch( can_rx_header->StdId )
	{
		case 0x600:
		
      m_tmp = (uint16_t)can_rx_data[0] << 8 | can_rx_data[1];	
		
		  pm01_od.ccr = m_tmp;
		
			break;
		case 0x601:
			
		  m_tmp = (uint16_t)can_rx_data[0] << 8 | can_rx_data[1];	

			pm01_od.p_set = m_tmp;
		
			break;
		case 0x602:
			
		  m_tmp = (uint16_t)can_rx_data[0] << 8 | can_rx_data[1];	

			pm01_od.v_set = m_tmp;
					
			break;
		case 0x603:
			
		  m_tmp = (uint16_t)can_rx_data[0] << 8 | can_rx_data[1];	

			pm01_od.i_set = m_tmp;
					
			break;
		case 0x610:
			
		  m_tmp = (uint16_t)can_rx_data[0] << 8 | can_rx_data[1];	

			pm01_od.sta_code.all = m_tmp;
		
		  m_tmp = (uint16_t)can_rx_data[2] << 8 | can_rx_data[3];	

			pm01_od.err_code = m_tmp;
		
			break;
		case 0x611:
			
		  m_tmp = (uint16_t)can_rx_data[0] << 8 | can_rx_data[1];	

			pm01_od.p_in = m_tmp;
		
		  m_tmp = (uint16_t)can_rx_data[2] << 8 | can_rx_data[3];	

			pm01_od.v_in = m_tmp;	
		
		  m_tmp = (uint16_t)can_rx_data[4] << 8 | can_rx_data[5];	

			pm01_od.i_in = m_tmp;
		
			break;
		case 0x612:
			
		  m_tmp = (uint16_t)can_rx_data[0] << 8 | can_rx_data[1];	

			pm01_od.p_out = m_tmp;
		
		  m_tmp = (uint16_t)can_rx_data[2] << 8 | can_rx_data[3];	

			pm01_od.v_out = m_tmp;	
		
		  m_tmp = (uint16_t)can_rx_data[4] << 8 | can_rx_data[5];	

			pm01_od.i_out = m_tmp;
					
			break;
		case 0x613:
			
		  m_tmp = (uint16_t)can_rx_data[0] << 8 | can_rx_data[1];	

			pm01_od.temp = m_tmp;
		
		  m_tmp = (uint16_t)can_rx_data[2] << 8 | can_rx_data[3];	

			pm01_od.total_time = m_tmp;	
		
		  m_tmp = (uint16_t)can_rx_data[4] << 8 | can_rx_data[5];	

			pm01_od.run_time = m_tmp;
					
			break;
	}


}


/*
 *  [] END OF FILE 
 */
