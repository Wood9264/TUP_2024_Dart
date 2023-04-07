#include "gimbal_task.h"
#include "cmsis_os.h"
#include "task.h"

gimbal_control_t gimbal;		
	uint32_t currentTime;
/**
  * @brief          取云台指针
  * @param[in]      null
  * @retval         云台对象
  */
gimbal_control_t *gimbal_point(void)
{
	return &gimbal;
}

/**
  * @brief          云台主任务
  * @param[in]      null
  */		
void gimbal_task(void const *pvParameters)
{
	vTaskDelay(GIMBAL_TASK_INIT_TIME);


	while(1)
	{
		currentTime = xTaskGetTickCount();//当前系统时间
		gimbal.Judge_init_state();
		gimbal.Get_info();
		gimbal.Control();

		CAN2_1FF_cmd_motor(0,gimbal.Pitch_motor.given_current,0,0);
		CAN1_1FF_cmd_motor(0,gimbal.Yaw_motor.given_current,0,0);
		osDelay(2);

	}
}	

/**
  * @brief          云台类构造函数进行初始化
  * @param[in]      null
  */
gimbal_control_t::gimbal_control_t()
{
	add_pit = add_yaw = 0.0f;
	//电机数据指针获取
	Yaw_motor.get_gimbal_motor = get_motor_measure_class(YAW);
	Pitch_motor.get_gimbal_motor = get_motor_measure_class(PITCH);
	//陀螺仪数据指针获取
	gimbal_INT_angle_point = get_INS_angle_point();
	gimbal_INT_gyro_point = get_gyro_data_point();
	//LADRC_FDW初始化
	Pitch_motor.LADRC_FDW.init(30, 0.0030, 85,30000,80,0);
  //Yaw_motor.LADRC_FDW.init(25, 0.003, 85, 30000,80,1);
	//Yaw_motor.LADRC_FDW.init(30, 0.004, 150, 30000,90,1);
  Yaw_motor.LADRC_FDW.init(25, 0.005, 85, 30000,70,0);
	
//自瞄LADRC_FDW初始化
	Pitch_motor.AUTO_LADRC_FDW.init(15, 0.0023, 80,30000,80,0.2);
	Yaw_motor.AUTO_LADRC_FDW.init(17, 0.003, 100, 30000,90,0.4);
	
//		LADRC_FDW_init(&gimbal_mode_change->gimbal_pitch_motor.Gimbal_ladrc_fdw,25, 0.010, 120,30000,30,0);   //120
////		    LADRC_FDW_init(&gimbal_mode_change->gimbal_yaw_motor.Gimbal_ladrc_fdw,25, 0.004, 90, 30000,30,0);//30, 0.0050, 180, 30000,90,1);  //120
//         LADRC_FDW_init(&gimbal_mode_change->gimbal_yaw_motor.Gimbal_ladrc_fdw,30, 0.004, 150, 30000,90,0);//30, 0.0050, 180, 30000,90,1);  //120

	//PITCH轴最大最小相对角度设置
	Pitch_motor.max_relative_angle =0.4290f;
	Pitch_motor.min_relative_angle = -0.74f;
	//YAW轴最大最小相对角度设置
	Yaw_motor.max_relative_angle =  Motor_Ecd_to_Rad*2048.0f;
	Yaw_motor.min_relative_angle = Motor_Ecd_to_Rad*-2048.0f;
	//电机中值
	Yaw_motor.offset_ecd =2000;
	Pitch_motor.offset_ecd = 4213;
	//Yaw轴角度设置
	Yaw_motor.relative_angle_set = 0;
	//Pitch轴角度设置
	Pitch_motor.absolute_angle_set = 0;
	Pitch_motor.relative_angle_set = 0;
	//卡尔曼滤波器初始化

	/*PID角度误差卡尔曼,一阶*/
	gimbal_kalman.Yaw_Error_Vis_Kalman.KalmanCreate( 1,  0);
	gimbal_kalman.Pitch_Error_Vis_Kalman.KalmanCreate( 1,  0);
	gimbal_kalman.Yaw_Set_Gim_Kalman.KalmanCreate( 1,  400);
	gimbal_kalman.Pitch_Set_Gim_Kalman.KalmanCreate( 1,  100);
	gimbal_kalman.Vision_Distance_Kalman.KalmanCreate( 1,  10);
	gimbal_kalman.Gimbal_Yaw_Accle_Kalman.KalmanCreate( 1,  10);
	gimbal_kalman.Gimbal_Yaw_Gyro_Kalman.KalmanCreate( 1,  100);
}



/**
  * @brief          数据更新
  * @param[in]      null
  */
void gimbal_control_t::Get_info()
{
	
	Pitch_motor.absolute_angle = -*(gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET);
	Roll_motor.absolute_angle= *(gimbal_INT_angle_point + INS_ROLL_ADDRESS_OFFSET);
	Yaw_motor.absolute_angle = *(gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);
	
  Pitch_motor.relative_angle =  motor_ecd_to_angle_change(Pitch_motor.get_gimbal_motor->ecd,
	                                                                    Pitch_motor.offset_ecd);	
	Yaw_motor.relative_angle = motor_ecd_to_angle_change(Yaw_motor.get_gimbal_motor->ecd,
                                                                  Yaw_motor.offset_ecd);
	
	Pitch_motor.motor_gyro = -*(gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);
	
//	Yaw_motor.motor_gyro =arm_cos_f32(Pitch_motor.absolute_angle) * (*(gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET))
//													+arm_sin_f32(Pitch_motor.absolute_angle) * (*(gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET));
	Yaw_motor.motor_gyro = arm_cos_f32(Pitch_motor.relative_angle) * (*(gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET))
													-arm_sin_f32(Roll_motor.absolute_angle) * (*(gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET));

}


/**
  * @brief          计算ecd与offset_ecd之间的相对角度
  * @param[in]      ecd: 电机当前编码
  * @param[in]      offset_ecd: 电机中值编码
  * @retval         相对角度，单位rad
  */
fp32 gimbal_control_t::motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}


/**
  * @brief          分任务控制
  * @param[in]      null
  */
void gimbal_control_t::Control()
{
	if(syspoint()->sys_mode == ZERO_FORCE )
	{
		Zero_force_control();
	}
	else if(syspoint()->sys_mode == DBUS_MISSING_CONTROL)
	{
		Dbus_missing_control();
	}
	else if(syspoint()->sys_mode == ABSOLUTE_ANGLE||syspoint()->sys_mode== NO_FOLLOW_YAW)
	{
		Absolute_angle_control();
	}
	else if(syspoint()->sys_mode == SPIN)
	{
		Spin_control();
	}
	else if(syspoint()->sys_mode == AUTO)
	{
		Auto_control();
	}
	else if(syspoint()->init_state == 1)
	{
		Init_control();
	}
}


/**
  * @brief          无力控制
  * @param[in]      null
  */
void gimbal_control_t::Zero_force_control()
{

	Yaw_motor.current_set = 0;
	Pitch_motor.current_set = 0;
	Yaw_motor.given_current = (int16_t)(Yaw_motor.current_set);
	Pitch_motor.given_current = (int16_t)(Pitch_motor.current_set);
}

/**
  * @brief          失联模式控制
  * @param[in]      null
  */
void gimbal_control_t::Dbus_missing_control()
{
	Yaw_motor.current_set = 0;
	Pitch_motor.current_set = 0;
	Yaw_motor.given_current = (int16_t)(Yaw_motor.current_set);
	Pitch_motor.given_current = (int16_t)(Pitch_motor.current_set);
}

/**
  * @brief          初始化模式控制
  * @param[in]      null
  */
void gimbal_control_t::Init_control()
{

	//Init_angle_set();

//	Yaw_motor.Relative_anlge_limit(add_yaw);
//	Pitch_motor.Relative_anlge_limit(add_pit);

  Yaw_motor.relative_angle_set=INIT_YAW_SET;
  Pitch_motor.relative_angle_set=INIT_Relative_PITCH_SET;

	Yaw_motor.Relative_adrc_control();
	Pitch_motor.Relative_adrc_control();
}

/**
  * @brief          绝对角度控制（底盘跟随云台）
  * @param[in]      null
  */
void gimbal_control_t::Absolute_angle_control()
{
  syspoint()->Gimbal_value_calc();
	add_yaw= syspoint()->rc_add_yaw;
	add_pit= syspoint()->rc_add_pit;
	Keyboard_angle_set();
	
	Yaw_motor.Absolute_angle_limit(add_yaw);
	Pitch_motor.Absolute_angle_limit(add_pit);

	Yaw_motor.Absolute_adrc_control();
	Pitch_motor.Absolute_adrc_control();
}


/**
  * @brief          小陀螺模式控制
  * @param[in]      null
  */
void gimbal_control_t::Spin_control()
{
  syspoint()->Gimbal_value_calc();
	add_yaw= syspoint()->rc_add_yaw;
	add_pit= syspoint()->rc_add_pit;
	
	Yaw_motor.Absolute_angle_nolimit(add_yaw);
	Pitch_motor.Absolute_angle_limit(add_pit);
	
	Yaw_motor.Absolute_adrc_control();
	Pitch_motor.Absolute_adrc_control();
}


/**
  * @brief          自瞄模式控制
  * @param[in]      null
  */
void gimbal_control_t::Auto_control()
{
	Gimbal_auto_angle_get();
	Yaw_motor.Absolute_auto_adrc_control();
	Pitch_motor.Absolute_auto_adrc_control();
}




/**
  * @brief          初始化模式角度控制
  * @param[in]      null
  */
void gimbal_control_t::Init_angle_set()
{
	if(fabs(INIT_PITCH_SET - Pitch_motor.relative_angle) > GIMBAL_INIT_ANGLE_ERROR)
	{
		add_pit = (INIT_PITCH_SET - Pitch_motor.relative_angle) * GIMBAL_INIT_PITCH_SPEED;
    add_yaw = 0.0f;
	}
	else 	if(fabs(INIT_YAW_SET - Yaw_motor.relative_angle) > GIMBAL_INIT_ANGLE_ERROR)
	{
		add_pit =(INIT_PITCH_SET - Pitch_motor.relative_angle) * GIMBAL_INIT_PITCH_SPEED;
    add_yaw = (INIT_YAW_SET - Yaw_motor.relative_angle) * GIMBAL_INIT_YAW_SPEED;
	}
}

/**
  * @brief          自瞄角度获取
  * @param[in]      null
  */
void gimbal_control_t::Gimbal_auto_angle_get()
{

//获取视觉数据
	  if(vision_info_point()->RxPacketFir.is_findtarget==1)
		{
			Yaw_motor.absolute_angle_set =  Vision_process_point()->data_kal.YawTarget_KF;
			Pitch_motor.absolute_angle_set =  Vision_process_point()->data_kal.PitchTarget_KF;
		}
//else{
//			Yaw_motor.absolute_angle_set = Yaw_motor.absolute_angle;
//			Pitch_motor.absolute_angle_set = Pitch_motor.absolute_angle;
//		}

}

/**
  * @brief          不同按键对应角度控制
  * @param[in]      null
  */
void gimbal_control_t::Keyboard_angle_set()
{
	 static uint16_t last_turn_keyboard = 0;
   static uint8_t gimbal_turn_flag = 0;
   static fp32 gimbal_end_angle = 0.0f;

   if ((syspoint()->system_rc_ctrl->key.v & TURN_KEYBOARD) && !(last_turn_keyboard & TURN_KEYBOARD))
   {
     if (gimbal_turn_flag == 0)
     {
       gimbal_turn_flag = 1;
       //保存掉头的目标值
       gimbal_end_angle = rad_format(Yaw_motor.absolute_angle + PI);
     }
    }
    last_turn_keyboard = syspoint()->system_rc_ctrl->key.v ;

    if (gimbal_turn_flag)
    {
      //不断控制到掉头的目标值，正转，反装是随机
      if (rad_format(gimbal_end_angle - Yaw_motor.absolute_angle) > 0.0f)
      {
        add_yaw += TURN_SPEED;
      }
      else
      {
        add_yaw -= TURN_SPEED;
      }
    }
    //到达pi （180°）后停止
    if (gimbal_turn_flag && fabs(rad_format(gimbal_end_angle - Yaw_motor.absolute_angle)) < 0.01f)
    {
      gimbal_turn_flag = 0;
    }
}

/**
  * @brief          判断初始化是否结束
  * @param[in]      null
  */
void gimbal_control_t::Judge_init_state()
{
	if(syspoint()->init_state == 1)
	{
		static uint16_t init_time = 0;
    static uint16_t init_stop_time = 0;
    init_time++;
        
    if ((fabs(Yaw_motor.relative_angle - INIT_YAW_SET) < GIMBAL_INIT_ANGLE_ERROR &&
         fabs(Pitch_motor.absolute_angle - INIT_PITCH_SET) < GIMBAL_INIT_ANGLE_ERROR))
    {        
      if (init_stop_time < GIMBAL_INIT_STOP_TIME)
      { 
        init_stop_time++;
      }
    }
    else
    {     
      if (init_time < GIMBAL_INIT_TIME)
      {
        init_time++;
      }
    }

    //超过初始化最大时间，或者已经稳定到中值一段时间，退出初始化状态开关打下档，或者掉线
    if (init_time < GIMBAL_INIT_TIME && init_stop_time < GIMBAL_INIT_STOP_TIME &&
        !switch_is_down(syspoint()->system_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]) )//&& !toe_is_error(DBUS_TOE)
    {
      return;
    }
    else
    {
      init_stop_time = 0;
      init_time = 0;
			syspoint()->init_state =0;
    }
	}
	
}

/**
  * @brief          绝对角度限制
  * @param[in]      fp32 add加值
  */
void gimbal_motor_t::Absolute_angle_limit(fp32 add)
{
	static fp32 bias_angle;
  static fp32 angle_set;
	//当前控制角度，误差角度（循环限幅之后的）
	bias_angle = rad_format(absolute_angle_set - absolute_angle);
	//relative angle + angle error + add_angle > max_relative angle
  //云台相对角度+ 误差角度 + 新增角度 如果大于 最大机械角度
  if (relative_angle + bias_angle + add > max_relative_angle)
  {
    //如果是往最大机械角度控制方向
    if (add > 0.0f)
    {
      //calculate max add_angle
      //计算出一个最大的添加角度，
      add = max_relative_angle - relative_angle - bias_angle;
    }
  }
  else if (relative_angle + bias_angle + add < min_relative_angle)
  {
    if (add < 0.0f)
    {
      add = min_relative_angle - relative_angle - bias_angle;
    }
  }
  angle_set = absolute_angle_set;
  absolute_angle_set = rad_format(angle_set + add);
}

/**
  * @brief          无绝对角度限制
  * @param[in]      fp32 add加值
  */
void gimbal_motor_t::Absolute_angle_nolimit(fp32 add)
{
	static fp32 angle_set;
	angle_set = absolute_angle_set;
  absolute_angle_set = rad_format(angle_set + add);
}

/**
  * @brief          相对角度限制
  * @param[in]      fp32 add加值
  */
void gimbal_motor_t::Relative_anlge_limit(fp32 add)
{
	relative_angle_set += add;
	//是否超过最大 最小值
	if (relative_angle_set > max_relative_angle)
  {
    relative_angle_set = max_relative_angle;
  }
  else if (relative_angle_set < min_relative_angle)
  {
    relative_angle_set = min_relative_angle;
  }

}

/**
  * @brief          控制器绝对角度控制
  * @param[in]      null
  */
void gimbal_motor_t::Absolute_adrc_control()
{
	current_set = LADRC_FDW.FDW_calc(absolute_angle ,absolute_angle_set ,motor_gyro);
	given_current = (int16_t)(current_set);
}

void gimbal_motor_t::Absolute_auto_adrc_control()
{
	current_set = AUTO_LADRC_FDW.FDW_calc(absolute_angle ,absolute_angle_set ,motor_gyro);
	given_current = (int16_t)(current_set);
}

/**
  * @brief          控制器相对角度控制
  * @param[in]      null
  */
void gimbal_motor_t::Relative_adrc_control()
{
	current_set = LADRC_FDW.FDW_calc(relative_angle , relative_angle_set , motor_gyro);
	given_current = (int16_t)current_set;
}
