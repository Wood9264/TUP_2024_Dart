#include "vision.h"
#include "cmsis_os.h"
#include "usbd_cdc_if.h"
#include "cstring"
#include "judge.h"


vision_info_t vision_info;


vision_info_t* vision_info_point(void)
{
    return &vision_info;
}


extern float Firc_L_firc3508_motor_speed,Firc_R_firc3508_motor_speed;
extern float Firc_L_tempreate,Firc_R_tempreate,gloab_bullet_speed;

extern float per_fir_delay;
bool vision_update_flag;

void Vision_Read_Data(uint8_t *ReadFromUsart)
{
	//判断帧头数据是否为0xA5
	if(ReadFromUsart[0] == 0xA5)
	{
		//帧头CRC8校验
		if(Verify_CRC8_Check_Sum( ReadFromUsart, VISION_LEN_HEADER ) == true)
		{
			//帧尾CRC16校验
			if(Verify_CRC16_Check_Sum( ReadFromUsart, VISION_LEN_PACKED ) == true)
			{
				//接收数据拷贝
				memcpy(&vision_info.RxPacketFir, ReadFromUsart, VISION_LEN_PACKED);	
				 //帧计算
			  vision_info.Fir_State.rx_time_now  = xTaskGetTickCountFromISR();
				vision_info.Fir_State.rx_time_fps  = vision_info.Fir_State.rx_time_now - vision_info.Fir_State.rx_time_prev;
				vision_info.Fir_State.rx_time_prev = vision_info.Fir_State.rx_time_now;
				vision_info.Vision_empty_time=20;
				vision_info.Fir_State.rx_data_update = true;//标记视觉数据更新了
			}
		}
	}
	else if(ReadFromUsart[0] == 0xB5)
	{
	  //帧头CRC8校验
		if(Verify_CRC8_Check_Sum( ReadFromUsart, VISION_LEN_HEADER ) == 1)
		{
			//帧尾CRC16校验
			if(Verify_CRC16_Check_Sum( ReadFromUsart, VISION_LEN_PACKED ) == 1)
			{
				//接收数据拷贝
				memcpy( &vision_info.RxPacketSed, ReadFromUsart, VISION_LEN_PACKED);	
				 //帧计算
			  vision_info.Sed_State.rx_time_now  = xTaskGetTickCountFromISR();
				vision_info.Sed_State.rx_time_fps  = vision_info.Sed_State.rx_time_now - vision_info.Sed_State.rx_time_prev;
				vision_info.Sed_State.rx_time_prev = vision_info.Sed_State.rx_time_now;
				vision_info.Vision_empty_time=20;
				vision_info.Sed_State.rx_data_update = true;//标记视觉数据更新了
				
			}
		}
	
	
	}
  else if(ReadFromUsart[0] == 0xC5)
	{
	
	  //帧头CRC8校验
		if(Verify_CRC8_Check_Sum( ReadFromUsart, VISION_LEN_HEADER ) == 1)
		{
			//帧尾CRC16校验
			if(Verify_CRC16_Check_Sum( ReadFromUsart, VISION_LEN_PACKED ) == 1)
			{
				//接收数据拷贝
				memcpy( &vision_info.RxPacketThi, ReadFromUsart, VISION_LEN_PACKED);	
        //帧计算
			  vision_info.Thi_State.rx_time_now  = xTaskGetTickCountFromISR();
				vision_info.Thi_State.rx_time_fps  = vision_info.Thi_State.rx_time_now - vision_info.Thi_State.rx_time_prev;
				vision_info.Thi_State.rx_time_prev = vision_info.Thi_State.rx_time_now;
				vision_info.Vision_empty_time=20;
				vision_info.Thi_State.rx_data_update = true;//标记视觉数据更新了
			}
		}
	}
	memset(ReadFromUsart, 0, VISION_LEN_PACKED);
}


void vision_info_t::Vision_Send_Fir_Data( uint8_t CmdID )
{
	uint8_t vision_sendfir_pack[64] = {0};//大于22就行

	
	TxHandFir.SOF = VISION_SOF;
	TxHandFir.CmdID = CmdID;//对视觉来说最重要的数据
	
	//写入帧头
	memcpy( vision_sendfir_pack, &TxHandFir, VISION_LEN_HEADER );
	
	//帧头CRC8校验协议
	Append_CRC8_Check_Sum( vision_sendfir_pack, VISION_LEN_HEADER );
	
	//中间数据不用管,视觉用不到,用到了也是后面自瞄自动开火,用到角度补偿数据
	TxPacketFir.INS_quat_send[0] = INS_quat[0];
	TxPacketFir.INS_quat_send[1] = INS_quat[1];
	TxPacketFir.INS_quat_send[2] = INS_quat[2];
	TxPacketFir.INS_quat_send[3] = INS_quat[3];
	TxPacketFir.INS_gyro_send[0] = INS_gyro[0];
	TxPacketFir.INS_gyro_send[1] = INS_gyro[1];
	TxPacketFir.INS_gyro_send[2] = INS_gyro[2];
	TxPacketFir.INS_accel_send[0] = INS_accel[0];
	TxPacketFir.INS_accel_send[1] = INS_accel[1];
	TxPacketFir.INS_accel_send[2] = INS_accel[2];
	TxPacketFir.revolver_speed =JUDGE_usGetSpeedHeat();//射速
	TxPacketFir.firc_error=per_fir_delay;//打弹延迟
  //TxPacketFir.Color=BLUE;
  //TxPacketFir.Color=RED;



	memcpy( vision_sendfir_pack + 3, &TxPacketFir, VISION_LEN_DATA);
	
	//帧尾CRC16校验协议
	Append_CRC16_Check_Sum( vision_sendfir_pack, VISION_LEN_PACKED );
	
	//将打包好的数据发送
	
	CDC_Transmit_FS(vision_sendfir_pack, VISION_LEN_PACKED);
	
	memset(vision_sendfir_pack, 0, VISION_LEN_PACKED);
	
		if(Vision_empty_time <= 0)
		{
		  RxPacketSed.target_linear[0] = 0;
			RxPacketSed.target_linear[1] = 0;
			RxPacketSed.target_angular[2] = 0;
		}
		else 
		{
		  Vision_empty_time--;
		}
}



void vision_info_t::Vision_Send_Sed_Data(uint8_t CmdID)
{
   uint8_t vision_send_sentry_1_pack[64] = {0};//大于22就行
	 
   TxHandSed.SOF = VISION_SENTRY1_SOF;
	 TxHandSed.CmdID = CmdID;//对视觉来说最重要的数据
	 //写入帧头
	 memcpy( vision_send_sentry_1_pack, &TxHandSed, VISION_LEN_HEADER );
   //帧头CRC8校验协议
	 Append_CRC8_Check_Sum( vision_send_sentry_1_pack, VISION_LEN_HEADER );
	 //中间数据为发送给哨兵 的全场位置坐标信息  14*4 56位
	 for(int i=0;i<=13;i++)
	 {
	    TxPacketSed.robots_position[i]=1;
	 }
	 
	 memcpy( vision_send_sentry_1_pack + 3, &TxPacketSed, VISION_LEN_DATA);
	 
	 //帧尾CRC16校验协议
	 Append_CRC16_Check_Sum( vision_send_sentry_1_pack, VISION_LEN_PACKED );
	
	 //将打包好的数据发送
	 CDC_Transmit_FS(vision_send_sentry_1_pack, VISION_LEN_PACKED);
	 memset(vision_send_sentry_1_pack, 0, VISION_LEN_PACKED);
 }

 
void vision_info_t:: Vision_Send_Thi_Data(uint8_t CmdID) 
 {
	 uint8_t vision_send_sentry_pack[64] = {0};//大于22就行
	 TxHandThi.SOF = VISION_SENTRY2_SOF;
	 TxHandThi.CmdID = CmdID;//对视觉来说最重要的数据
	 //写入帧头
	 memcpy( vision_send_sentry_pack, &TxHandThi, VISION_LEN_HEADER );
   //帧头CRC8校验协议
	 Append_CRC8_Check_Sum( vision_send_sentry_pack, VISION_LEN_HEADER );
	 //中间数据为发送给哨兵的全场位置坐标信息  6*4 24位
	 for(int a=0;a<=5;a++)
	 {
	    TxPacketThi.robots_position[a]=2;
	 }
	 //发送给哨兵的全场机器人血量信息   10*2 20位
	 
	 for(int j=0;j<=9;j++)
	 {
		 TxPacketThi.robots_hp[j]= 3;
	 }

   //全场剩余时间 1*2 2位
	 TxPacketThi.stage_remain_time= 4;
	 
	 memcpy( vision_send_sentry_pack + 3, &TxPacketThi, VISION_LEN_DATA);
	 
	 //帧尾CRC16校验协议
	 Append_CRC16_Check_Sum( vision_send_sentry_pack, VISION_LEN_PACKED );
	
	 //将打包好的数据发送
	 CDC_Transmit_FS(vision_send_sentry_pack, VISION_LEN_PACKED);
	 memset(vision_send_sentry_pack, 0, VISION_LEN_PACKED);
}


/**
  * @brief  获取yaw误差角度
  * @param  误差指针
  * @retval void
  */
float vision_info_t::Vision_Error_Angle_Yaw(float *error)
{
	*error = -(1.0f*RxPacketFir.yaw_angle +0.f) *PI / 180.f ;
	return *error;
}

/**
  * @brief  获取pitch误差角度
  * @param  误差指针
  */
float vision_info_t::Vision_Error_Angle_Pitch(float *error)
{	
	*error = -(1.0f*RxPacketFir.pitch_angle + 0.f) *PI / 180.f ;
  return *error;
}

/**
  * @brief  获取距离
  * @param  void
  * @retval void
  * @attention  
  */

void vision_info_t::Vision_Get_Distance(float *distance)
{
	*distance =RxPacketFir.distance;

}

void vision_info_t::Vision_Get_coordinate_system(float *x_axis,float *y_axis,float *z_axis)
{
	*x_axis = RxPacketFir.x_info;
	*y_axis = RxPacketFir.y_info;
	*z_axis = RxPacketFir.z_info;
}

void vision_info_t::Vision_Get_predict_coordinate(float *predict_x,float *predict_y,float *predict_z)
{
	*predict_x = RxPacketFir.predict_x_info;
	*predict_y = RxPacketFir.predict_y_info;
	*predict_z = RxPacketFir.predict_z_info;
}
