#include "communicate.h"
#include "cmsis_os.h"
#include "cmath"

Vision_process_t Vision_process;

Vision_process_t *Vision_process_point(void)
{
	return &Vision_process;
}
/**
  * @brief          视觉交流任务
  * @param[in]      null
  */
void communi_task(void const *pvParameters)
{
	
	while(1)
	{
    
	  Vision_process.Transmit_info();
    Vision_process.Get_info();
		Vision_process.Pridict_info();
		Vision_process.Control();
	  osDelay(2);
	}
	
}

/**
  * @brief          构造函数初始化
  * @param[in]      null
  */
Vision_process_t::Vision_process_t()
{
	speed_queue.queueLength = 60;
	accel_queue.queueLength = 60;
	dis_queue.queueLength = 60;
}


void Vision_process_t::Transmit_info()
{
  	vision_info_point()->Vision_Send_Fir_Data(1);
//		osDelay(2);
//		vision_info_point()->Vision_Send_Sed_Data(5);
//		osDelay(2);
//		vision_info_point()->Vision_Send_Thi_Data(5);	
}
/**
  * @brief          读取视觉数据
  * @param[in]      null
  */
void Vision_process_t::Get_info()
{
	Yaw_error=vision_info_point()->Vision_Error_Angle_Yaw(&(gimbal_point()->gimbal_kalman.Auto_Error_Yaw[NOW]));
  Pitch_error=vision_info_point()->Vision_Error_Angle_Pitch(&(gimbal_point()->gimbal_kalman.Auto_Error_Pitch[NOW]));
	vision_info_point()->Vision_Get_Distance(&(gimbal_point()->gimbal_kalman.Auto_Distance));

	data_kal.YawGet_KF = gimbal_point()->gimbal_kalman.Yaw_Error_Vis_Kalman.KalmanFilter(gimbal_point()->gimbal_kalman.Auto_Error_Yaw[NOW]); 	/*对视觉角度数据做卡尔曼滤波*/
  data_kal.PitchGet_KF = gimbal_point()->gimbal_kalman.Pitch_Error_Vis_Kalman.KalmanFilter(gimbal_point()->gimbal_kalman.Auto_Error_Pitch[NOW]);
	data_kal.DistanceGet_KF =gimbal_point()->gimbal_kalman.Vision_Distance_Kalman.KalmanFilter(gimbal_point()->gimbal_kalman.Auto_Distance);

	YawTarget_now = lastupdate_cloud_yaw + data_kal.YawGet_KF;
  PitchTarget_now = lastupdate_cloud_pitch + data_kal.PitchGet_KF;
	
  lastupdate_cloud_yaw = update_cloud_yaw;
  lastupdate_cloud_pitch = update_cloud_pitch;     
  update_cloud_yaw = gimbal_point()->Yaw_motor.absolute_angle;
  update_cloud_pitch =  gimbal_point()->Pitch_motor.absolute_angle;
	
}



/**
  * @brief          视觉预测
  * @param[in]      null
  */
void Vision_process_t::Pridict_info()
{
	static float acc_use = 1.0f;
  static float predic_use = 0.6f;
  float dir_factor;
 
	//对视觉发送的add值+电机lastangle  进行数据推演
	
	//速度推演
	speed_get = Get_Diff(3,&speed_queue,YawTarget_now);//20
  speed_get = 20 * (speed_get/vision_info_point()->Fir_State.rx_time_fps); //每毫秒
  speed_get = gimbal_point()->gimbal_kalman.Gimbal_Yaw_Gyro_Kalman.KalmanFilter(speed_get);
// speed_get = DeathZoom(speed_get,0,1);
  speed_get = fp32_constrain(speed_get, -0.030, 0.030);
  //加速度推演
  accel_get = Get_Diff(5,&accel_queue,speed_get);	 /*新版获取加速度10*/
  accel_get = 10 * (accel_get/vision_info_point()->Fir_State.rx_time_fps);//每毫秒
  accel_get = gimbal_point()->gimbal_kalman.Gimbal_Yaw_Accle_Kalman.KalmanFilter(accel_get);
// accel_get = DeathZoom(accel_get,0,0.1);		/*死区处理 - 滤除0点附近的噪声*/
  accel_get = fp32_constrain(Vision_process.accel_get, -0.023, 0.023);
  //速度推演
  distend_get =  Get_Diff(5,&dis_queue,data_kal.DistanceGet_KF);
	
	 if( (speed_get * accel_get)>=0 )
  {
    dir_factor= 0.5f;//1  2
  }
  else
  {
    dir_factor= 1.0f;//1.5  4
  }

  feedforwaurd_angle = acc_use * accel_get;  	//计算前馈角

  predict_angle = predic_use * (1.1f * speed_get * data_kal.DistanceGet_KF
												      + 1.5f * dir_factor * feedforwaurd_angle * data_kal.DistanceGet_KF);
  predict_angle = fp32_constrain(predict_angle, -0.092, 0.092);
	
}


void Vision_process_t::Control()
{
  //不给预测
 data_kal.YawTarget_KF=gimbal_point()->gimbal_kalman.Yaw_Set_Gim_Kalman.KalmanFilter(YawTarget_now);
 data_kal.PitchTarget_KF=gimbal_point()->gimbal_kalman.Pitch_Set_Gim_Kalman.KalmanFilter(PitchTarget_now);
 data_kal.YawTarget_KF=gimbal_point()->gimbal_kalman.Yaw_Set_Gim_Kalman.KalmanFilter(data_kal.YawTarget_KF);

//  //给预测
//	data_kal.YawTarget_KF=YawTarget_now+Vision_process.predict_angle;
//  data_kal.YawTarget_KF=gimbal_point()->gimbal_kalman.Yaw_Set_Gim_Kalman.KalmanFilter(data_kal.YawTarget_KF);	
//  data_kal.PitchTarget_KF= gimbal_point()->gimbal_kalman.Pitch_Set_Gim_Kalman.KalmanFilter(PitchTarget_now);

	if(isnan(data_kal.YawTarget_KF) || isinf(data_kal.YawTarget_KF))
 {
		data_kal.YawTarget_KF = gimbal_point()->gimbal_kalman.Auto_Error_Yaw[NOW];

 }
 
	
}


