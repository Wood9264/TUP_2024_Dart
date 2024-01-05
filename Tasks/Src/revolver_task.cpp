#include "system_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Revolver_task.h"
#include "judge_task.h"
#include "BSP_buzzer.h"
#include "monitor_task.h"
#include "load_task.h"

revolver_task_t  revolver; 

revolver_task_t* revolver_point(void)
{
	return &revolver;
}

fp32 SPEED[4];
fp32 TEMP[4];

/**
  * @brief          发射机构任务
  * @param[in]      null
  */
void revolver_task(void const *pvParameters)
{
	//延时
	vTaskDelay(REVOLVER_TASK_INIT_TIME);
	uint32_t currentTime;
	while(1)
	{
		//获取当前系统时间
		currentTime = xTaskGetTickCount();
		//数据更新
		revolver.data_update();
		//分任务控制
		revolver.control();
		//发送电流
		CAN1_200_cmd_motor(revolver.fric_motor[0].give_current, revolver.fric_motor[1].give_current, revolver.fric_motor[2].give_current, revolver.fric_motor[3].give_current);
		CAN2_200_cmd_motor(revolver.yaw_motor.give_current, 0, 0, 0);
		vTaskDelayUntil(&currentTime, 1);

		for(int i = 0; i < 4; i++)
		{
			SPEED[i] = revolver.fric_motor[i].motor_measure->speed_rpm;
			TEMP[i] = revolver.fric_motor[i].motor_measure->temperate;
		}
  }
}

/**
  * @brief          发射机构任务初始化
  * @param[in]      null
  */
revolver_task_t::revolver_task_t()
{
	//初始化PID
	// fp32 slipper_speed_pid[3] = {SLIPPER_SPEED_PID_KP, SLIPPER_SPEED_PID_KI, SLIPPER_SPEED_PID_KD};
	// fp32 slipper_position_pid[3] = {SLIPPER_POSITION_PID_KP, SLIPPER_POSITION_PID_KI, SLIPPER_POSITION_PID_KD};
	fp32 yaw_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
	fp32 yaw_position_pid[3] = {YAW_POSITION_PID_KP, YAW_POSITION_PID_KI, YAW_POSITION_PID_KD};
	fp32 Fric_speed_pid[3] = {FRIC_SPEED_PID_KP, FRIC_SPEED_PID_KI, FRIC_SPEED_PID_KD};

	// slipper_motor.speed_pid.init(PID_POSITION, slipper_speed_pid,SLIPPER_SPEED_PID_MAX_OUT, SLIPPER_SPEED_PID_MAX_IOUT);
	// slipper_motor.position_pid.init(PID_POSITION, slipper_position_pid,SLIPPER_POSITION_PID_MAX_OUT, SLIPPER_POSITION_PID_MAX_IOUT);
	yaw_motor.speed_pid.init(PID_POSITION, yaw_speed_pid,YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);
	yaw_motor.position_pid.init(PID_POSITION, yaw_position_pid,YAW_POSITION_PID_MAX_OUT, YAW_POSITION_PID_MAX_IOUT);

	for(int i = 0; i < 4; i++)
	{
		fric_motor[i].speed_pid.init(PID_POSITION, Fric_speed_pid,FRIC_SPEED_PID_MAX_OUT, FRIC_SPEED_PID_MAX_IOUT);
	}
	
	fric_speed_offset = 0;
	yaw_motor.has_calibrated = 0;

	yaw_motor.offset_num[0] = 5.3;
	yaw_motor.offset_num[1] = 10.5;
	yaw_motor.offset_num[2] = -5.5;
	yaw_motor.offset_num[3] = -10.3;
	yaw_motor.offset_num[4] = -10.3;
	// slipper_motor.calibrate_begin = 0;
	// slipper_motor.has_calibrated = 0;
	// slipper_motor.if_shoot_begin = 0;

	//电机指针
	// slipper_motor.motor_measure = get_motor_measure_class(LOADER_MOTOR);
	fric_motor[0].motor_measure = get_motor_measure_class(FL);
	fric_motor[1].motor_measure = get_motor_measure_class(FR);
	fric_motor[2].motor_measure = get_motor_measure_class(BL);
	fric_motor[3].motor_measure = get_motor_measure_class(BR);
	yaw_motor.motor_measure = get_motor_measure_class(YAW_MOTOR);

	revolver_rc_ctrl = get_remote_control_point();
}

/**
  * @brief          发射机构数据更新
  * @param[in]      null
  */
void revolver_task_t::data_update()
{
	static fp32 speed_fliter_1 = 0.0f;
	static fp32 speed_fliter_2 = 0.0f;
	static fp32 speed_fliter_3 = 0.0f;

	//滑块电机速度滤波
	static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};
	//二阶低通滤波
	speed_fliter_1 = speed_fliter_2;
	speed_fliter_2 = speed_fliter_3;
	speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (slipper_motor.motor_measure->speed_rpm * Motor_RMP_TO_SPEED) * fliter_num[2];
	slipper_motor.motor_speed = speed_fliter_3;
	//累计编码值
	slipper_motor.accumulate_ecd = slipper_motor.motor_measure->num * 8192 + slipper_motor.motor_measure->ecd;
	yaw_motor.accumulate_ecd = yaw_motor.motor_measure->num * 8192 + yaw_motor.motor_measure->ecd;

	slipper_motor.bottom_tick = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11);

	if (slipper_motor.bottom_tick)
	{
		buzzer_warn(0, 0, 3, 10000);
	}

	static int16_t recali_time = 0;
	//除了校准的时候，其它时候滑块不会碰到底部触点开关；如果碰到说明校准出错了，要重新校准
	if(slipper_motor.has_calibrated ==1 && slipper_motor.bottom_tick == 1)
	{
		recali_time++;
		if(recali_time > 20)
		{
			slipper_motor.has_calibrated = 0;
		}
	}
	else
	{
		recali_time = 0;
	}
}

/**
 * @brief	分任务控制
*/
void revolver_task_t::control()
{
	if(syspoint()->sys_mode == ZERO_FORCE)
	{
		ZERO_FORCE_control();
		// if(IF_RC_SW1_MID)
		// {
		// 	//摩擦轮转速补偿
		// 	fric_speed_offset_control();
		// }
		// //遥控器读取摩擦轮转速
		// fric_speed_buzzer();
	}
	else if(syspoint()->sys_mode == CALIBRATE)
	{
		CALIBRATE_control();
	}
	else if(syspoint()->sys_mode == SHOOT)
	{
		SHOOT_control();
	}
}

/**
 * @brief	无力模式
*/
void revolver_task_t::ZERO_FORCE_control()
{
	for(int i = 0; i < 4; i++)
	{
		fric_motor[i].speed_set = RAMP_float(0, fric_motor[i].speed_set, FRIC_RAMP_BUFF);
		fric_motor[i].current_calculate();
	}
	yaw_motor.give_current = 0;
}

/**
 * @brief	遥控器调摩擦轮的转速
*/
// void revolver_task_t::fric_speed_offset_control()
// {
// 	static bool_t offset_flag = 0;

// 	//遥控器右摇杆↗调整千位
// 	if (IF_RIGHT_ROCKER_RIGHT_TOP)
// 	{
// 		//左摇杆右拨一次+1000， 左拨一次-1000
// 		if (revolver_rc_ctrl->rc.ch[2] > 500)
// 		{
// 			if (offset_flag == 0)
// 			{
// 				fric_speed_offset += 1000;
// 				offset_flag = 1;
// 				buzzer_warn(0, 0, 1, 10000);
// 			}
// 		}
// 		else if (revolver_rc_ctrl->rc.ch[2] < -500)
// 		{
// 			if (offset_flag == 0)
// 			{
// 				fric_speed_offset -= 1000;
// 				offset_flag = 1;
// 				buzzer_warn(0, 0, 1, 10000);
// 			}
// 		}
// 		else
// 		{
// 			offset_flag = 0;
// 		}
// 	}
// 	//遥控器右摇杆↖调整百位
// 	else if (IF_RIGHT_ROCKER_LEFT_TOP)
// 	{
// 		//左摇杆右拨一次+100， 左拨一次-100
// 		if (revolver_rc_ctrl->rc.ch[2] > 500)
// 		{
// 			if (offset_flag == 0)
// 			{
// 				fric_speed_offset += 100;
// 				offset_flag = 1;
// 				buzzer_warn(0, 0, 2, 10000);
// 			}
// 		}
// 		else if (revolver_rc_ctrl->rc.ch[2] < -500)
// 		{
// 			if (offset_flag == 0)
// 			{
// 				fric_speed_offset -= 100;
// 				offset_flag = 1;
// 				buzzer_warn(0, 0, 2, 10000);
// 			}
// 		}
// 		else
// 		{
// 			offset_flag = 0;
// 		}
// 	}
// 	//遥控器右摇杆↙调整十位
// 	else if (IF_RIGHT_ROCKER_LEFT_BOTTOM)
// 	{
// 		//左摇杆右拨一次+10， 左拨一次-10
// 		if (revolver_rc_ctrl->rc.ch[2] > 500)
// 		{
// 			if (offset_flag == 0)
// 			{
// 				fric_speed_offset += 10;
// 				offset_flag = 1;
// 				buzzer_warn(0, 0, 3, 10000);
// 			}
// 		}
// 		else if (revolver_rc_ctrl->rc.ch[2] < -500)
// 		{
// 			if (offset_flag == 0)
// 			{
// 				fric_speed_offset -= 10;
// 				offset_flag = 1;
// 				buzzer_warn(0, 0, 3, 10000);
// 			}
// 		}
// 		else
// 		{
// 			offset_flag = 0;
// 		}
// 	}
// }

/**
 * @brief	调整模式，可以手动调整滑块的位置
*/
void revolver_task_t::CALIBRATE_control()
{
	for (int i = 0; i < 4; i++)
	{
		fric_motor[i].speed_set = RAMP_float(0, fric_motor[i].speed_set, FRIC_RAMP_BUFF);
		fric_motor[i].current_calculate();
	}

	if(IF_RC_SW1_DOWN)
	{
		yaw_motor.ecd_set += revolver_rc_ctrl->rc.ch[2] * RC_TO_YAW_ECD_SET;
	}
	else if(IF_RC_SW1_MID)
	{
		yaw_motor.calibrate();
	}
	yaw_motor.speed_set = yaw_motor.position_pid.calc(yaw_motor.accumulate_ecd, yaw_motor.ecd_set);
	yaw_motor.give_current = yaw_motor.speed_pid.calc(yaw_motor.motor_measure->speed_rpm, yaw_motor.speed_set);
}

/**
 * @brief	校准YAW轴电机
*/
void yaw_motor_t::calibrate()
{
	//遥控器↖↗校准yaw轴
	if(RC_double_held_single_return(LEFT_ROCKER_LEFT_TOP,RIGHT_ROCKER_RIGHT_TOP, 400))
	{
		calibrated_point = accumulate_ecd;
		has_calibrated = 1;
		buzzer_warn(0, 0, 3, 10000);
	}
}

/**
 * @brief	手动调整滑块位置
*/
// void slipper_motor_t::SLIPPER_control()
// {
// 	speed_set = (revolver.revolver_rc_ctrl->rc.ch[3]) * RC_TO_SLIPPER_SEPPD_SET;
// 	//底部限位缓冲
// 	if(motor_speed < 0)
// 	{
// 		// position_limit_buffer(slipper_position_ecd[0]);
// 	}
// 	//顶部限位缓冲
// 	else if(motor_speed > 0)
// 	{
// 		// position_limit_buffer(slipper_position_ecd[MAX_DART_NUM]);
// 	}

// 	//软件限位
// 	if(accumulate_ecd < slipper_position_ecd[0] && speed_set < 0)
// 	{
// 		speed_set = 0;
// 	}
// 	else if(accumulate_ecd > slipper_position_ecd[MAX_DART_NUM] && speed_set> 0)
// 	{
// 		speed_set = 0;
// 	}
// }

// void slipper_motor_t::position_limit_buffer(fp32 limit_point)
// {
// 	if(abs(limit_point - accumulate_ecd) < POSITION_LIMIT_BUFFER_DISTANCE)
// 	{
// 		speed_set = speed_set * (abs(limit_point - accumulate_ecd) / POSITION_LIMIT_BUFFER_DISTANCE);
// 	}
// }

/**
 * @brief	校准模式，设定最底部为滑块的零点
 */
// void slipper_motor_t::CALIBRATE_control()
// {
// 	//遥控器↙↘开始自动校准
// 	if (RC_double_held_single_return(LEFT_ROCKER_LEFT_BOTTOM, RIGHT_ROCKER_RIGHT_BOTTOM, 400))
// 	{
// 		calibrate_begin = 1;
// 	}

// 	//遥控器↘↙手动校准，适用于触点开关失效的情况
// 	if (RC_double_held_single_return(LEFT_ROCKER_RIGHT_BOTTOM, RIGHT_ROCKER_LEFT_BOTTOM, 400))
// 	{
// 		manual_calibrate();
// 	}

// 	if (calibrate_begin == 1)
// 	{
// 		auto_calibrate();
// 	}
// 	else 
// 	{
// 		speed_set = 0;
// 		return;
// 	}
// }

/**
 * @brief	自动校准
 */
// void slipper_motor_t::auto_calibrate()
// {
// 	static bool_t found_zero_point = 0; //是否找到零点

// 	//触点开关被压下，找到零点
// 	if (bottom_tick)
// 	{
// 		found_zero_point = 1;
// 	}

// 	//滑块下移
// 	if (found_zero_point == 0)
// 	{
// 		speed_set = CALIBRATE_DOWN_SPEED;
// 	}
// 	//压下触点开关后上移
// 	else if (found_zero_point == 1 && bottom_tick == 1)
// 	{
// 		speed_set = CALIBRATE_UP_SPEED;
// 	}
// 	//滑块离开触点开关，校准完毕
// 	else if (found_zero_point == 1 && bottom_tick == 0)
// 	{
// 		speed_set = 0;
// 		has_calibrated = 1;
// 		found_zero_point = 0;
// 		calibrate_begin = 0;

// 		//设置飞镖发射时滑块每次停留的位置
// 		for (int i = 0; i <= MAX_DART_NUM; i++)
// 		{
// 			slipper_position_ecd[i] = accumulate_ecd + i * ONE_DART_ECD + CALIBRATE_OFFSET;
// 		}
// 	}
// }

/**
 * @brief	手动校准，适用于触点开关失效的情况
 */
// void slipper_motor_t::manual_calibrate()
// {
// 	speed_set = 0;
// 	has_calibrated = 1;
// 	calibrate_begin = 0;

// 	//设置飞镖发射时滑块每次停留的位置
// 	for (int i = 0; i <= MAX_DART_NUM; i++)
// 	{
// 		slipper_position_ecd[i] = accumulate_ecd + i * ONE_DART_ECD + CALIBRATE_OFFSET;
// 	}
// 	buzzer_warn(0, 0, 3, 10000);
// }

/**
 * @brief	发射模式
*/
void revolver_task_t::SHOOT_control()
{
	if (IF_RC_SW1_DOWN)
	{
		//YAW电机发射初始化
		yaw_motor.init();

		//遥控器↖↗，开启摩擦轮
		if (RC_double_held_single_return(LEFT_ROCKER_LEFT_TOP, RIGHT_ROCKER_RIGHT_TOP, 400))
		{
			is_fric_wheel_on = 1;
		}

		if (is_fric_wheel_on == 0)
		{
			for (int i = 0; i < 4; i++)
			{
				fric_motor[i].speed_set = 0;
			}
		}
		else if (is_fric_wheel_on == 1)
		{
			fric_motor[0].speed_set = RAMP_float(-(BASE_SPEED + fric_speed_offset), fric_motor[0].speed_set, FRIC_RAMP_BUFF);
			fric_motor[1].speed_set = RAMP_float((BASE_SPEED + fric_speed_offset), fric_motor[1].speed_set, FRIC_RAMP_BUFF);
			fric_motor[2].speed_set = RAMP_float(-(BASE_SPEED + fric_speed_offset), fric_motor[2].speed_set, FRIC_RAMP_BUFF);
			fric_motor[3].speed_set = RAMP_float((BASE_SPEED + fric_speed_offset), fric_motor[3].speed_set, FRIC_RAMP_BUFF);
		}
	}
	else if (IF_RC_SW1_MID)
	{
		//YAW电机发射
		yaw_motor.shooting();

		//摩擦轮转速设定
		if (is_fric_wheel_on == 0)
		{
			for (int i = 0; i < 4; i++)
			{
				fric_motor[i].speed_set = 0;
			}
		}
		else if (is_fric_wheel_on == 1)
		{
			fric_motor[0].speed_set = -(BASE_SPEED + fric_speed_offset + 1000);
			fric_motor[1].speed_set = (BASE_SPEED + fric_speed_offset + 1000);
			fric_motor[2].speed_set = -(BASE_SPEED + fric_speed_offset);
			fric_motor[3].speed_set = (BASE_SPEED + fric_speed_offset);
		}
	}

	for (int i = 0; i < 4; i++)
	{
		fric_motor[i].current_calculate();
	}

	yaw_motor.speed_set = yaw_motor.position_pid.calc(yaw_motor.accumulate_ecd, yaw_motor.ecd_set);
	yaw_motor.give_current = yaw_motor.speed_pid.calc(yaw_motor.motor_measure->speed_rpm, yaw_motor.speed_set);
}

/**
 * @brief	YAW电机发射初始化
*/
void yaw_motor_t::init()
{
	//转盘初始化时YAW轴电机一起初始化
    if (rotary_motor_point()->has_shoot_init_started)
    {
        ecd_set = calibrated_point + offset_num[syspoint()->active_dart_index] * 8192;
        has_shoot_init_started = 1;
        has_shoot_init_finished = 0;
    }

	// speed_set = position_pid.calc(accumulate_ecd, ecd_set);
	// give_current = speed_pid.calc(motor_measure->speed_rpm, speed_set);

	//YAW轴电机初始化结束
    if (fabs(ecd_set - accumulate_ecd) < YAW_GIMBAL_MOTOR_TOE && has_shoot_init_started == 1)
    {
        has_shoot_init_started = 0;
        has_shoot_init_finished = 1;
    }
}

/**
 * @brief	发射
*/
void yaw_motor_t::shooting()
{
	//设定最终角度
	final_ecd_set = calibrated_point + offset_num[syspoint()->active_dart_index] * 8192;

	//装填电机上移完毕且转盘电机不锁定时，设定值变为final
    if (loader_motor_point()->has_shoot_up_finished && !rotary_motor_point()->should_lock)
    {
        ecd_set = final_ecd_set;
    }

	// speed_set = position_pid.calc(accumulate_ecd, ecd_set);
	// give_current = speed_pid.calc(motor_measure->speed_rpm, speed_set);

	//目标值和实际值之差小于一定值，可认为转到位
    if (fabs(final_ecd_set - motor_measure->ecd) < YAW_ECD_TOLERANCE)
    {
        has_move_to_next_finished = 1;
    }
}

/**
 * @brief	准备发射，在此模式下才能开启摩擦轮
*/
// void revolver_task_t::READY_control()
// {
// 	static fp32 angle_out = 0;

// 	//遥控器↗↖，开启摩擦轮
// 	if(RC_double_held_single_return(LEFT_ROCKER_RIGHT_TOP, RIGHT_ROCKER_LEFT_TOP, 400))
// 	{
// 		is_fric_wheel_on = 1;
// 	}

// 	if(is_fric_wheel_on == 0)
// 	{
// 		for(int i = 0; i < 4; i++)
// 		{
// 			fric_motor[i].speed_set = 0;
// 		}
// 	}
// 	else if (is_fric_wheel_on == 1)
// 	{
// 		fric_motor[0].speed_set = RAMP_float( (BASE_SPEED + fric_speed_offset), fric_motor[0].speed_set, FRIC_RAMP_BUFF);
// 		fric_motor[1].speed_set = RAMP_float(-(BASE_SPEED + fric_speed_offset), fric_motor[1].speed_set, FRIC_RAMP_BUFF);
// 		fric_motor[2].speed_set = RAMP_float( (BASE_SPEED + fric_speed_offset), fric_motor[2].speed_set, FRIC_RAMP_BUFF);
// 		fric_motor[3].speed_set = RAMP_float(-(BASE_SPEED + fric_speed_offset), fric_motor[3].speed_set, FRIC_RAMP_BUFF);
// 	}

// 	//滑块位置锁定
// 	angle_out = slipper_motor.position_pid.calc(slipper_motor.accumulate_ecd, slipper_motor.ecd_set);
// 	slipper_motor.give_current = slipper_motor.speed_pid.calc(slipper_motor.motor_speed, angle_out);
// }

// /**
//  * @brief	开始发射
// */
// void revolver_task_t::SHOOTING_control()
// {
// 	//摩擦轮转速设定
// 	if(is_fric_wheel_on == 0)
// 	{
// 		for(int i = 0; i < 4; i++)
// 		{
// 			fric_motor[i].speed_set = 0;
// 		}
// 	}
// 	else if (is_fric_wheel_on == 1)
// 	{
// 		fric_motor[0].speed_set =  (BASE_SPEED + fric_speed_offset + 1000);
// 		fric_motor[1].speed_set = -(BASE_SPEED + fric_speed_offset + 1000);
// 		fric_motor[2].speed_set =  (BASE_SPEED + fric_speed_offset);
// 		fric_motor[3].speed_set = -(BASE_SPEED + fric_speed_offset);
// 	}

// 	slipper_motor.SHOOTING_slipper_control();
	
// }

/**
 * @brief	发射时滑块的控制
*/
// void slipper_motor_t::SHOOTING_slipper_control()
// {
// 	static fp32 angle_out = 0;

// 	//如果未校准，滑块锁定
// 	if (has_calibrated == 0)
// 	{
// 		angle_out = position_pid.calc(accumulate_ecd, ecd_set);
// 		give_current = speed_pid.calc(motor_speed, angle_out);
// 		return;
// 	}

// 	//遥控器左摇杆↖每打一次，发射数量+1
// 	if (RC_held_single_return(LEFT_ROCKER_LEFT_TOP, 1))
// 	{
// 		if_shoot_begin = 1;
// 		bullet_num_set++;
// 	}

// 	//开始发射前，滑块位置锁死在当前位置
// 	if (if_shoot_begin == 0)
// 	{
// 		angle_out = position_pid.calc(accumulate_ecd, ecd_set);
// 		give_current = speed_pid.calc(motor_speed, angle_out);
// 		return;
// 	}

// 	//未打完所有飞镖
// 	if (bullet_num_set <= MAX_DART_NUM)
// 	{
// 		ecd_set = slipper_position_ecd[bullet_num_set];
// 		angle_out = position_pid.DLcalc(accumulate_ecd, ecd_set, SLIPPER_SHOOTING_SPEED);
// 		give_current = speed_pid.calc(motor_speed, angle_out);
// 	}
// 	//如果打完所有飞镖，自动回到零点
// 	else
// 	{
// 		ecd_set = slipper_position_ecd[0];
// 		angle_out = position_pid.DLcalc(accumulate_ecd, ecd_set, SLIPPER_BACK_SPEED);
// 		give_current = speed_pid.calc(motor_speed, angle_out);
// 	}
// }

/**
 * @brief	计算滑块距离哪个停留点最近，就取哪个记作已经发射的飞镖数
*/
void slipper_motor_t::bullet_num_cal()
{
	int i = 0, j = 0;
	fp32 ecd_offset[MAX_DART_NUM + 1]; //当前滑块位置的编码值与每个停留点编码值的差

	for(i = 0; i <= MAX_DART_NUM; i++)
	{
		ecd_offset[i] = abs(accumulate_ecd - slipper_position_ecd[i]);
	}

	for(i = 0; i <= MAX_DART_NUM; i++)
	{
		if(ecd_offset[i] < ecd_offset[j])
		{
			j = i;
		}
	}
	bullet_num = j;
}

/**
 * @brief	摩擦轮电流计算
*/
void fric_motor_t::current_calculate()
{
	give_current = speed_pid.calc(motor_measure->speed_rpm, speed_set);
}

/**
 * @brief	遥控器读取摩擦轮转速
*/
// void revolver_task_t::fric_speed_buzzer()
// {
// 	static uint8_t show_num = 0;

// 	//遥控器右摇杆↘进行读取
// 	if (IF_RIGHT_ROCKER_RIGHT_BOTTOM)
// 	{
// 		//左摇杆↗读取千位
// 		if (IF_LEFT_ROCKER_RIGHT_TOP)
// 		{
// 			show_num = (BASE_SPEED + fric_speed_offset) / 1000;
// 			buzzer_warn(show_num, 100, 1, 10000);

// 		}
// 		//左摇杆↖读取百位
// 		else if (IF_LEFT_ROCKER_LEFT_TOP)
// 		{
// 			show_num = ((int)(BASE_SPEED + fric_speed_offset) / 100) % 10;
// 			buzzer_warn(show_num, 100, 1, 10000);
// 		}
// 		//左摇杆↙读取十位
// 		else if (IF_LEFT_ROCKER_LEFT_BOTTOM)
// 		{
// 			show_num = ((int)(BASE_SPEED + fric_speed_offset) / 10) % 10;
// 			buzzer_warn(show_num, 100, 1, 10000);
// 		}
// 		//左摇杆↘读取个位
// 		else if (IF_LEFT_ROCKER_RIGHT_BOTTOM)
// 		{
// 			show_num = ((int)(BASE_SPEED + fric_speed_offset)) % 10;
// 			buzzer_warn(show_num, 100, 1, 10000);
// 		}
// 	}
// }
