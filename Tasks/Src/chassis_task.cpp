// #include "chassis_task.h"
// #include "struct_typedef.h"
// #include "user_lib.h"
// #include "arm_math.h"
// #include "task.h"
// #include "i2c.h"
// extern "C"{
// #include "pm01.h"
// #include "ina226.h"
// }
// chassis_t chassis_move;
// /***************************************/
// extern uint16_t g_power_set;
// uint8_t armor_hart_ID;
// fp32 chassis_rpm_1;
// fp32 chassis_rpm_2;
// fp32 chassis_rpm_3;
// fp32 chassis_rpm_4;

// /***************************************/
// /**
//   * @brief          底盘类指针
//   * @param[in]      null
//   * @retval         底盘对象
//   */

// chassis_t* chassispoint(void)
// {
// 	return &chassis_move;
// }

// /**
//   * @brief          底盘任务
//   * @param[in]      none
//   * @retval         none
//   */
// void chassis_task(void const * pvParameters)
// {
// 	 //空闲一段时间
//   vTaskDelay(CHASSIS_TASK_INIT_TIME);
// 	uint32_t currentTime;
// 	while(1)
// 	{
// 		currentTime = xTaskGetTickCount();//当前系统时间
//     chassis_move.Get_info();
// 	  chassis_move.Control();
//     CAN_CMD_CAP(chassis_move.chassis_max_power*100);//超级电容输出功率
//  g_power_set=chassis_move.chassis_max_power*100;
// 		CAN1_200_cmd_motor(chassis_move.chassis_motor[0].give_current,chassis_move.chassis_motor[1].give_current,
// 		                      chassis_move.chassis_motor[2].give_current,chassis_move.chassis_motor[3].give_current);
// 	  vTaskDelayUntil(&currentTime, 2);
// 	}
// }



// /**
//   * @brief          构造函数初始化
//   * @param[in]      none
//   * @retval         none
//   */
// chassis_t::chassis_t()
// {
//   //初始化底盘电机pid 旋转环pid
// 	fp32 chassis_yaw_pid[3] = {CHASSIS_MOTOR_YAW_PID_KP,CHASSIS_MOTOR_YAW_PID_KI,CHASSIS_MOTOR_YAW_PID_KD}; 
// 	fp32 chassis_yaw_spin_pid[3]={-10,0,-5};

// 	chassis_angle_pid.init(PID_POSITION,chassis_yaw_pid, MOTOR_YAW_PID_MAX_OUT, MOTOR_YAW_PID_MAX_IOUT);
// 	chassis_angle_spin_pid.init(PID_POSITION,chassis_yaw_spin_pid,MOTOR_YAW_PID_MAX_OUT,MOTOR_YAW_PID_MAX_IOUT);

// 	fp32 motor_current_pid[3] = {1.5,0.0,0.0f};//{1.5f,0.0,0}

// 	chassis_cap_masure = get_cap_measure_point();
// 	fp32 motor_speed_pid[3] ={CHASSIS_MOTOR_SPEED_PID_KP, CHASSIS_MOTOR_SPEED_PID_KI, CHASSIS_MOTOR_SPEED_PID_KD};
// 	for(int i=0;i<=3;i++)
//   {
// 		chassis_motor[i].chassis_motor_measure=get_motor_measure_class(i);
// 	  chassis_motor_speed_pid[i].init(PID_POSITION,motor_speed_pid,M3508_MOTOR_SPEED_PID_MAX_OUT, M3508_MOTOR_SPEED_PID_MAX_IOUT);
// 		//chassis_motor_speed_pid[i].init(PID_POSITION,motor_speed_pid,16000, 2000);
// 		chassis_current_pid[i].init(PID_POSITION,motor_current_pid,16300, 1000);

// }	
// //	chassis_motor_speed_pid[0].init(PID_POSITION,motor_speed_pid,30000, 1000);
// //	chassis_motor_speed_pid[1].init(PID_POSITION,motor_speed_pid,30000, 1000);
// //	chassis_motor_speed_pid[2].init(PID_POSITION,motor_speed_pid,16000, 1000);
// //	chassis_motor_speed_pid[3].init(PID_POSITION,motor_speed_pid,30000, 1000);
// //	
// //	chassis_current_pid[0].init(PID_POSITION,motor_current_pid,30000, 1000);
// //	chassis_current_pid[1].init(PID_POSITION,motor_current_pid,30000, 1000);
// //	chassis_current_pid[2].init(PID_POSITION,motor_current_pid,16300, 1000);
// //	chassis_current_pid[3].init(PID_POSITION,motor_current_pid,30000, 1000);


// 	//设置vxvy最大最小速度
// 	vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
// 	vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;
// 	vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
// 	vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

	
	
// }

// /**
//   * @brief          底盘数据读取
//   * @param[in]      none
//   * @retval         none
//   */
// void chassis_t::Get_info()
// {
// 	  //获取当前模式
//     chassis_mode=syspoint()->sys_mode;
// 	 //获取云台电机数据指针
// 	  chassis_yaw_motor = gimbal_point()->Yaw_motor;
// 		ina226_data.BusV = INA226_getBusV(&hi2c2,INA226_ADDRESS)/1000;
// 		ina226_data.Current = INA226_getCurrent(&hi2c2,INA226_ADDRESS)/1000;
// 		ina226_data.Power = INA226_getPower(&hi2c2,INA226_ADDRESS)/1000;

// 	  //获取电机速度，将电机的反馈的速度换算成底盘的速度
// 	   for ( int i = 0; i <= 3; i++)
//     {
// 			chassis_motor[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN *chassis_motor[i].chassis_motor_measure->speed_rpm;	   
// 		}
//     //获取底盘前进速度 x， 平移速度y，旋转速度wz，坐标系为右手系
//     vx = (-chassis_motor[0].speed +chassis_motor[1].speed 
// 		                           + chassis_motor[2].speed - chassis_motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
//     vy = (-chassis_motor[0].speed - chassis_motor[1].speed 
// 		                           + chassis_motor[2].speed + chassis_motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
//     wz = (-chassis_motor[0].speed - chassis_motor[1].speed 
// 		                           -chassis_motor[2].speed - chassis_motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;
// 		C_RESET();

// 		armor_hart_ID=Judge_armor_id();


// }



// /**
//   * @brief          底盘控制量设置与计算
//   * @param[in]      none
//   * @retval         none
//   */
// void chassis_t::Control()
// {
// 		if(syspoint()->sys_mode== INIT||syspoint()->sys_mode== ZERO_FORCE||syspoint()->sys_mode==DBUS_MISSING_CONTROL)
// 		{
// 			chassis_zero_control_set();
// 		}else if(syspoint()->sys_mode== ABSOLUTE_ANGLE||syspoint()->sys_mode == AUTO)
// 		{
// 			chassis_follow_gimbal_control_set();
// 		}else if(syspoint()->sys_mode== SPIN||syspoint()->sys_mode==SPIN_AUTO)
// 		{
// 			chassis_spin_control_set();
// 		}else if(syspoint()->sys_mode== NO_FOLLOW_YAW)
// 		{
// 		  chassis_no_follow_gimbal_control_set();
// 		}
// 		chassis_control_loop();
	
// }
// /**
//   * @brief          底盘无力模式控制量设置
//   * @param[in]      none
//   * @retval         none
//   */
// void chassis_t::chassis_zero_control_set()
// {
//     vx_set = 0.0f;
//     vy_set = 0.0f;
//     wz_set = 0.0f;
//     syspoint()->chassis_vx_set_temp=0;
// 	  syspoint()->chassis_vy_set_temp=0;
// }

// /**
//   * @brief          底盘跟随云台模式控制量设置
//   * @param[in]      none
//   * @retval         none
//   */

// void chassis_t::chassis_follow_gimbal_control_set()
// {
//     syspoint()->Chassis_value_calc();
//     vx_follow_set=syspoint()->chassis_vx_set_temp;
// 	  vy_follow_set=syspoint()->chassis_vy_set_temp;
//     fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
// 		//旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
// 		sin_yaw = arm_sin_f32(-chassis_yaw_motor.relative_angle);
// 		cos_yaw = arm_cos_f32(-chassis_yaw_motor.relative_angle);
// 		vx_set = (cos_yaw * vx_follow_set + sin_yaw * vy_follow_set)*scale_k;
// 		vy_set = (-sin_yaw * vx_follow_set + cos_yaw * vy_follow_set)*scale_k;
// 		//设置控制相对云台角度
// 	  angle_set=0;
// 		chassis_relative_angle_set = rad_format(angle_set);
//     chassis_angle_pid.out=chassis_angle_pid.calc(chassis_yaw_motor.relative_angle,chassis_relative_angle_set);
// 		wz_set=chassis_angle_pid.out*scale_k;

// 		//速度限幅
// 		vx_set = fp32_constrain(vx_set, vx_min_speed, vx_max_speed);
// 		vy_set = fp32_constrain(vy_set, vy_min_speed, vy_max_speed);
    
// }

// /**
//   * @brief          底盘不跟随云台控制量设置
//   * @param[in]      none
//   * @retval         none
//   */
// void chassis_t::chassis_no_follow_gimbal_control_set()
// {
//     syspoint()->Chassis_value_calc();
//     vx_set=(syspoint()->chassis_vx_set_temp)*scale_k;
// 	  vy_set=(syspoint()->chassis_vx_set_temp)*scale_k;
// 	  angle_set= -CHASSIS_WZ_RC_SEN * syspoint()->system_rc_ctrl->rc.ch[CHASSIS_WZ_CHANNEL];

// 	  wz_set=angle_set*scale_k;
// 		//速度限幅
//     vx_set = fp32_constrain(vx_set, vx_min_speed, vx_max_speed);
// 		vy_set = fp32_constrain(vy_set, vy_min_speed, vy_max_speed);

// }

// /**
//   * @brief          底盘小陀螺模式控制量设置 
//   * @param[in]      none
//   * @retval         none
//   */
// void chassis_t::chassis_spin_control_set()
// {
    
// 	  fp32 p=0.7;
//     vx_follow_set *=p;
// 		vy_follow_set *=p;
//     syspoint()->Chassis_value_calc();
//     vx_follow_set=syspoint()->chassis_vx_set_temp;
// 	  vy_follow_set=syspoint()->chassis_vy_set_temp;
//     angle_set = 1.5f * (8 - fabs(vx_follow_set)- fabs(vy_follow_set));  //变速模式
	
//   	fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
// 		//旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
// 		sin_yaw = arm_sin_f32(-chassis_yaw_motor.relative_angle);
// 		cos_yaw = arm_cos_f32(-chassis_yaw_motor.relative_angle); 

// 		vx_set = (cos_yaw * vx_follow_set + sin_yaw  * vy_follow_set)*scale_k;
// 		vy_set = (-sin_yaw * vx_follow_set + cos_yaw * vy_follow_set)*scale_k;
	
// 		wz_set = angle_set*scale_k;
// 	   //速度限幅
// 		vx_set = fp32_constrain(vx_set, vx_min_speed, vx_max_speed);
// 		vy_set = fp32_constrain(vy_set, vy_min_speed, vy_max_speed);
// }


// /**
//   * @brief          底盘控制量计算
//   * @param[in]      none
//   * @retval         none
//   */
// void chassis_t::chassis_control_loop()
// {
// 		fp32 max_vector = 0.0f, vector_rate = 0.0f;
// 		fp32 temp = 0.0f;
// 		if(chassis_mode== INIT||chassis_mode== ZERO_FORCE)
// 		{
// 			 for(int i=0;i<=3;i++)
// 			{
// 			  chassis_motor_speed_pid[i].out=0;
// 			  chassis_motor[i].give_current=0;
// 			}
// 			 return;
// 		}
// 		chassis_vector_to_mecanum_wheel_speed(vx_set,vy_set,wz_set,wheel_speed);
// 		//计算轮子控制最大速度，并限制其最大速度i
// 		for (int i = 0; i <= 3; i++)
// 		{
// 				chassis_motor[i].speed_set = wheel_speed[i];
// 				temp = fabs(chassis_motor[i].speed_set);
// 				if (max_vector < temp)
// 				{
// 						max_vector = temp;
// 				}
// 		}

// 		if (max_vector > MAX_WHEEL_SPEED)
// 		{
// 				vector_rate = MAX_WHEEL_SPEED / max_vector;
// 				for (int i = 0; i <= 3; i++)
// 				{
// 						chassis_motor[i].speed_set *= vector_rate;
// 				}
// 		}
	
// 	  for (int i = 0; i <=3; i++)
//     {
//         chassis_motor_speed_pid[i].out=chassis_motor_speed_pid[i].calc( chassis_motor[i].speed,chassis_motor[i].speed_set);
//     }
//    //chassis_motor_speed_pid[2].out=chassis_motor_speed_pid[2].calc( chassis_motor[2].speed,chassis_motor[2].speed_set*0.78);		
    
//      chassis_power_limit();
// 		//赋值电流值
//     for (int i = 0; i <= 3; i++)
//     {
//          chassis_motor[i].give_current = (int16_t)( chassis_motor_speed_pid[i].out);
//     }

// }

// /**
//   * @brief          麦轮数据解算
//   * @param[in]      none
//   * @retval         none
//   */
// void chassis_t::chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])	
// {   
	
// 		wheel_speed[0] = (-vx_set - vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set);
// 		wheel_speed[1] = (vx_set - vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set);
// 		wheel_speed[2] = (vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set);
// 		wheel_speed[3] = (-vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set);
	
	
// }
// /**
//   * @brief          底盘功率限制
//   * @param[in]      none
//   * @retval         none
// 	*
// 				超级电容分为超级电容管理板、电容组、电容裁判三个主要部分，
// 		管理板可以控制电管给的功率即电容组的输入功率、裁判系统显示的底盘功率，
// 		保证底盘不会超功率。以下主要任务是合理分配放电策略，尽可能地达到充放平衡
// 	*
//   */

// int IF_SUPER_OK;
// void chassis_t::chassis_power_limit()
// { 

// 	fp32 temp_real_power,temp_error_power;
// 	fp32 temp_k;
// 	fp32 cap_v_out;
// 	fp32 cap_i_out;

// //获取裁判系统信息
//   chassis_power          		=    JUDGE_usGetChassisPowerLimit();   //获取功率限制
//   chassis_power_buffer   		=    JUDGE_fGetRemainEnergy();         //获取缓冲能量
// 	real_power_chassis     		=    JUDGE_fGetChassisPower();				 //获取底盘功率
//   chassis_max_power_limit		=    JUDGE_usGetChassisPowerLimit();		

// //分配充电功率上限
// 	if((IF_KEY_PRESSED_SHIFT||rc_ctrl.rc.ch[4]>=630)&&IF_SUPER_OK)
// 	{
// 			chassis_power=chassis_max_power_limit;
// 	}

// 	else if(chassis_power==0)
// 	{
// 			chassis_power=50;
// 	}
// 	else
// 	{
// 			chassis_power = JUDGE_usGetChassisPowerLimit();
// 	}

// 	chassis_max_power =chassis_power;

// //读取超电管理模块
// 	cap_v_out = chassis_cap_masure->Cap_voltage;
// 	cap_i_out = chassis_cap_masure->Cap_current;
// 	cap_power_fdb=cap_i_out*cap_v_out;
// 	fp32 a = cap_v_out;
// 	fp32 b = 28;
// 	temp_real_power=cap_v_out*chassis_cap_masure->Cap_current;
//   temp_error_power=temp_real_power-real_power_chassis;

// //低电压保护
// if(chassis_cap_masure->Cap_voltage>=19)										
// {
// 			IF_SUPER_OK=1;
// }
// else{
// 			IF_SUPER_OK=0;
// 			}


// //开启超级电容
//   if((IF_KEY_PRESSED_SHIFT||rc_ctrl.rc.ch[4]>=630))
// 	{
// 			//scale_k 用来控制输出电流
// 			scale_k = (a/b)*((120+chassis_max_power)/120.0f);
// 			temp_k=scale_k;
// 			if(syspoint()->sys_mode==SPIN||syspoint()->sys_mode==SPIN_AUTO)//小陀螺功率限制，全向轮只有在小陀螺时才是真正的全功率输出
// 			{
// 				  scale_k=temp_k*1.1f;
// 			}
// 	}
// 	else
// 	{
// 			scale_k = (a/b)*(a/b)*(a/b)*((120+chassis_max_power)/120.0f);
// 			shift_flag=0;
//   }

// //临时功率限制  如果没有超级电容就打开
// 			//scale_k=1;
// }


// /**
//   * @brief          一键C板重启
//   * @param[in]      none
//   * @retval         none
//   */
// int rc_keyB_time;

// static void C_RESET(void)
// {
// if ( IF_KEY_PRESSED_B)//
// 		{				
// 				if(rc_keyB_time<500)//模式转换时间
// 				 {
// 					 rc_keyB_time++;
					 
// 					 if(rc_keyB_time==500)
// 					 {
// 								HAL_NVIC_SystemReset();
// 								rc_keyB_time=0;
// 					 }
// 					}
// 		}		
// else
// 		{
// 				rc_keyB_time=0;
// 				return;
// 		}
// }

