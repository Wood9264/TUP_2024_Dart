#include "communicate.h"
#include "cmsis_os.h"
#include "cmath"

Vision_process_t Vision_process;
uint16_t vision_mode_ui=1;


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
		Vision_process.Ui_follow_calc();
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
	vision_mode=1;
	//相机焦距参数
	fx = 883.3170;
	fy = 882.6257;
	//相机光心坐标
	cx = 960;
	cy = 530;
	f=sqrt(fx*fx+fy*fy);
}
/**
  * @brief          自瞄UI反馈计算
  * @param[in]      null
  */
void Vision_process_t::Ui_follow_calc()
{
	x_coordinate = x_value*fx/z_value + cx;
	y_coordinate = -y_value*fy/z_value + cy;
	pre_x_coordinate = predict_x_value*fx/predict_z_value + cx;
	pre_y_coordinate = -predict_y_value*fy/predict_z_value + cy;
	follow_radius = 0.05f*f/gimbal_point()->gimbal_kalman.Auto_Distance;
	if(isnan(x_coordinate)) x_coordinate=0;
	if(isinf(follow_radius)) follow_radius = 0;
}
/**
  * @brief          自瞄模式切换
  * @param[in]      null
  */
void Vision_process_t::Transmit_info()
{
    if(IF_KEY_PRESSED_F)  //自瞄
{
			vision_mode = 1;
			vision_mode_ui=1;
}
		if(IF_KEY_PRESSED_C)  //小符
{
			vision_mode = 3;
			vision_mode_ui=3;
}
		if(IF_KEY_PRESSED_V)  //大符
{
			vision_mode =4;
			vision_mode_ui=4;
}
		if(IF_KEY_PRESSED_R)  //前哨站
{
		  vision_mode = 5;
			vision_mode_ui=5;
}
		if(IF_KEY_PRESSED_X)  //关闭预测
{
		  vision_mode = 0;
			vision_mode_ui=0;
}


		vision_mode_ui=vision_mode;
  	vision_info_point()->Vision_Send_Fir_Data(vision_mode);


}
/**
  * @brief          读取视觉数据
  * @param[in]      null
  */
void Vision_process_t::Get_info()
{
  //获取原始数据
	Yaw_error=vision_info_point()->Vision_Error_Angle_Yaw(&(gimbal_point()->gimbal_kalman.Auto_Error_Yaw[NOW]));
  Pitch_error=vision_info_point()->Vision_Error_Angle_Pitch(&(gimbal_point()->gimbal_kalman.Auto_Error_Pitch[NOW]));
	vision_info_point()->Vision_Get_Distance(&(gimbal_point()->gimbal_kalman.Auto_Distance));

	vision_info_point()->Vision_Get_coordinate_system(&x_value,&y_value,&z_value);
	vision_info_point()->Vision_Get_predict_coordinate(&predict_x_value,&predict_y_value,&predict_z_value);

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
  accel_get = 10 * (accel_get/vision_info_point()->Fir_State.rx_time_fps);//每毫秒//fps为0时解析数据直接为nan
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
 //data_kal.PitchTarget_KF=gimbal_point()->gimbal_kalman.Pitch_Set_Gim_Kalman.KalmanFilter(data_kal.PitchTarget_KF);


//  //给预测
//	data_kal.YawTarget_KF=YawTarget_now+Vision_process.predict_angle;
//  data_kal.YawTarget_KF=gimbal_point()->gimbal_kalman.Yaw_Set_Gim_Kalman.KalmanFilter(data_kal.YawTarget_KF);	
//  data_kal.PitchTarget_KF= gimbal_point()->gimbal_kalman.Pitch_Set_Gim_Kalman.KalmanFilter(PitchTarget_now);

	if(isnan(data_kal.YawTarget_KF) || isinf(data_kal.YawTarget_KF))
 {
		data_kal.YawTarget_KF = gimbal_point()->gimbal_kalman.Auto_Error_Yaw[NOW];

 }
 
	
}


