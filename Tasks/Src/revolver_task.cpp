#include "system_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Revolver_task.h"
#include "judge_task.h"
#include "ui_task.h"

revolver_task_t  revolver; 

revolver_task_t* revolver_point(void)
{
	return &revolver;
}

/**
  * @brief          发射机构任务
  * @param[in]      null
  */
void revolver_task(void const *pvParameters)
{
	//延时
	vTaskDelay(REVOLVER_TASK_INIT_TIME);
	rc_ctrl.rc.s[0] = 3;
	while(1)
	{
		//数据更新
		revolver.revolver_feedback_update();
		//分任务控制
		revolver.control();
		//发送电流
		CAN1_200_cmd_motor(revolver.slipper_motor.give_current, revolver.fric_motor[1].give_current, revolver.fric_motor[2].give_current, revolver.fric_motor[3].give_current);
		// CAN1_200_cmd_motor(revolver.fric_motor[0].give_current, revolver.fric_motor[1].give_current, revolver.fric_motor[2].give_current, revolver.fric_motor[3].give_current);
		CAN2_200_cmd_motor(revolver.slipper_motor.give_current, 0, 0, 0);
		vTaskDelay(2);
  }
}

/**
  * @brief          发射机构任务初始化
  * @param[in]      null
  */
revolver_task_t::revolver_task_t()
{
	//初始化PID
	fp32 slipper_speed_pid[3] = {SLIPPER_SPEED_PID_KP, SLIPPER_SPEED_PID_KI, SLIPPER_SPEED_PID_KD};
	fp32 slipper_position_pid[3] = {SLIPPER_POSITION_PID_KP, SLIPPER_POSITION_PID_KI, SLIPPER_POSITION_PID_KD};
	fp32 Fric_speed_pid[3] = {FRIC_SPEED_PID_KP, FRIC_SPEED_PID_KI, FRIC_SPEED_PID_KD};

	slipper_motor.speed_pid.init(PID_POSITION, slipper_speed_pid,SLIPPER_SPEED_PID_MAX_OUT, SLIPPER_SPEED_PID_MAX_IOUT);
	slipper_motor.position_pid.init(PID_POSITION, slipper_position_pid,SLIPPER_POSITION_PID_MAX_OUT, SLIPPER_POSITION_PID_MAX_IOUT);

	for(int i = 0; i < 4; i++)
	{
		fric_motor[i].speed_pid.init(PID_POSITION, Fric_speed_pid,FRIC_SPEED_PID_MAX_OUT, FRIC_SPEED_PID_MAX_IOUT);
	}
	
	slipper_motor.has_calibrated = 0;
	slipper_motor.if_shoot_begin = 0;
	slipper_motor.should_lock = 0;

	//电机指针
	slipper_motor.motor_measure = get_motor_measure_class(SLIPPER_MOTOR);
	fric_motor[0].motor_measure = get_motor_measure_class(FL);
	fric_motor[1].motor_measure = get_motor_measure_class(FR);
	fric_motor[2].motor_measure = get_motor_measure_class(BL);
	fric_motor[3].motor_measure = get_motor_measure_class(BR);

	revolver_rc_ctrl = get_remote_control_point();
}

/**
  * @brief          发射机构数据更新
  * @param[in]      null
  */
void revolver_task_t::revolver_feedback_update()
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

	slipper_motor.accumulate_ecd = slipper_motor.motor_measure->num * 8192 + slipper_motor.motor_measure->ecd;

	slipper_motor.bottom_tick = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11);

	static int16_t recali_time = 0;
	//除了校准的时候，其它时候滑块不会碰到底部触点开关；碰到说明校准出错了，要重新校准
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

void revolver_task_t::control()
{
	if(syspoint()->sys_mode == ZERO_FORCE)
	{
		ZERO_FORCE_control();
		fric_speed_offset_control();
	}
	else if(syspoint()->sys_mode == ADJUST)
	{
		ADJUST_control();
	}
	else if(syspoint()->sys_mode == SHOOT)
	{
		SHOOT_control();
	}
}

void revolver_task_t::ZERO_FORCE_control()
{
	for(int i = 0; i < 4; i++)
	{
		fric_motor[i].speed_set = 0;
		fric_motor[i].current_calculate();
	}
	slipper_motor.ecd_set = slipper_motor.accumulate_ecd;//改到sys里
	slipper_motor.give_current = 0;
}

void revolver_task_t::fric_speed_offset_control()
{
	static bool_t offset_flag = 0;

	if (IF_RIGHT_ROCKER_RIGHT_TOP)
	{
		if (revolver_rc_ctrl->rc.ch[2] > 500)
		{
			if (offset_flag == 0)
			{
				fric_speed_offset += 100;
				offset_flag = 1;
			}
		}
		else if (revolver_rc_ctrl->rc.ch[2] < -500)
		{
			if (offset_flag == 0)
			{
				fric_speed_offset -= 100;
				offset_flag = 1;
			}
		}
		else
		{
			offset_flag = 0;
		}
	}
}

void revolver_task_t::ADJUST_control()
{
	static fp32 angle_out = 0;

	for (int i = 0; i < 4; i++)
	{
		fric_motor[i].speed_set = 0;
		fric_motor[i].current_calculate();
	}

	if (syspoint()->adjust_mode == SLIPPER && slipper_motor.has_calibrated)
	{
		slipper_motor.SLIPPER_control();
	}
	else if (syspoint()->adjust_mode == CALIBRATE)
	{
		slipper_motor.CALIBRATE_control();
	}
	else
	{
		slipper_motor.speed_set = 0;
	}

	// if (slipper_motor.speed_set == 0)
	// {
	// 	if(slipper_motor.should_lock == 0)
	// 	{
	// 		slipper_motor.ecd_set = slipper_motor.accumulate_ecd;
	// 		slipper_motor.should_lock = 1;
	// 	}
	// 	angle_out = slipper_motor.position_pid.calc(slipper_motor.accumulate_ecd, slipper_motor.ecd_set);
	// 	slipper_motor.give_current = slipper_motor.speed_pid.calc(slipper_motor.motor_speed, angle_out);

	// }
	// else
	// {
	// 	slipper_motor.should_lock = 0;
		slipper_motor.give_current = slipper_motor.speed_pid.calc(slipper_motor.motor_speed, slipper_motor.speed_set);
	// }
}

void slipper_motor_t::SLIPPER_control()
{
	speed_set = (revolver.revolver_rc_ctrl->rc.ch[3]) * RC_TO_SLIPPER_SEPPD_SET;
	//底部限位缓冲
	if(motor_speed < 0)
	{
		// position_limit_buffer(slipper_position_ecd[0]);
	}
	//顶部限位缓冲
	else if(motor_speed > 0)
	{
		// position_limit_buffer(slipper_position_ecd[MAX_DART_NUM]);
	}

	//软件限位
	if(accumulate_ecd < slipper_position_ecd[0] && speed_set < 0)
	{
		speed_set = 0;
	}
	else if(accumulate_ecd > slipper_position_ecd[MAX_DART_NUM] && speed_set> 0)
	{
		speed_set = 0;
	}
}

void slipper_motor_t::position_limit_buffer(fp32 limit_point)
{
	if(abs(limit_point - accumulate_ecd) < POSITION_LIMIT_BUFFER_DISTANCE)
	{
		speed_set = speed_set * (abs(limit_point - accumulate_ecd) / POSITION_LIMIT_BUFFER_DISTANCE);
	}
}
int CALI;
void slipper_motor_t::CALIBRATE_control()
{
	static bool_t calibrate_begin = 0;
	static uint16_t rc_cmd_time = 0;
	static bool_t found_zero_point = 0; //是否找到零点

	//遥控器↘↙进行校准
	if((IF_LEFT_ROCKER_RIGHT_BOTTOM && IF_RIGHT_ROCKER_LEFT_BOTTOM) || CALI)
	{
		rc_cmd_time++;
	}
	else
	{
		rc_cmd_time = 0;
	}
	if(rc_cmd_time > 200)
	{
		calibrate_begin = 1;
	}

	if(calibrate_begin == 0)
	{
		speed_set = 0;
		return;
	}


	if(bottom_tick)
	{
		found_zero_point = 1;
	}

	if(found_zero_point == 0)
	{
		speed_set = CALIBRATE_DOWN_SPEED;
	}
	else if(found_zero_point == 1 && bottom_tick == 1)
	{
		speed_set = CALIBRATE_UP_SPEED;
	}
	else if(found_zero_point == 1 && bottom_tick == 0)
	{
		speed_set = 0;
		has_calibrated = 1;
		found_zero_point = 0;
		calibrate_begin = 0;

		for(int i = 0; i <= MAX_DART_NUM; i++)
		{
			slipper_position_ecd[i] = accumulate_ecd + i * ONE_DART_ECD;
		}
	}
}

void revolver_task_t::SHOOT_control()
{
	if(syspoint()->shoot_mode == READY)
	{
		READY_control();
	}
	else if(syspoint()->shoot_mode == SHOOTING)
	{
		SHOOTING_control();
	}

	for(int i = 0; i < 4; i++)
	{
		fric_motor[i].current_calculate();
	}
}

void revolver_task_t::READY_control()
{
	static uint16_t rc_cmd_time = 0;
	static fp32 angle_out = 0;

	//遥控器↗↖，开启摩擦轮
	if(IF_LEFT_ROCKER_RIGHT_TOP && IF_RIGHT_ROCKER_LEFT_TOP)
	{
		rc_cmd_time++;
	}
	else
	{
		rc_cmd_time = 0;
	}
	if(rc_cmd_time > 200)
	{
		is_fric_wheel_on = 1;
	}

	if(is_fric_wheel_on == 0)
	{
		for(int i = 0; i < 4; i++)
		{
			fric_motor[i].speed_set = 0;
		}
	}
	else if (is_fric_wheel_on == 1)
	{
		
		fric_motor[0].speed_set = -(BASE_SPEED + fric_speed_offset);
		fric_motor[1].speed_set =  (BASE_SPEED + fric_speed_offset);
		fric_motor[2].speed_set = -(BASE_SPEED + fric_speed_offset);
		fric_motor[3].speed_set =  (BASE_SPEED + fric_speed_offset);
		
	}

	angle_out = slipper_motor.position_pid.calc(slipper_motor.accumulate_ecd, slipper_motor.ecd_set);
	slipper_motor.give_current = slipper_motor.speed_pid.calc(slipper_motor.motor_speed, angle_out);
}

void revolver_task_t::SHOOTING_control()
{
	//摩擦轮转速设定
	if(is_fric_wheel_on == 0)
	{
		for(int i = 0; i < 4; i++)
		{
			fric_motor[i].speed_set = 0;
		}
	}
	else if (is_fric_wheel_on == 1)
	{
		
		fric_motor[0].speed_set = -(BASE_SPEED + fric_speed_offset);
		fric_motor[1].speed_set =  (BASE_SPEED + fric_speed_offset);
		fric_motor[2].speed_set = -(BASE_SPEED + fric_speed_offset);
		fric_motor[3].speed_set =  (BASE_SPEED + fric_speed_offset);
	}

	slipper_motor.SHOOTING_slipper_control();
	
}
int NUMADD = 0;
void slipper_motor_t::SHOOTING_slipper_control()
{
	static bool_t set_num_add_flag = 0;
	static bool_t num_add_flag = 0;
	static fp32 angle_out = 0;
	static uint16_t on_set_position_time = 0;
	static bool_t no_skip_num_add = 0;

	if(has_calibrated == 0)
	{
		return;
	}

	//遥控器↖每打一次，发射数量+1
	if (IF_LEFT_ROCKER_LEFT_TOP || NUMADD)
	{
		if_shoot_begin = 1;
		if (set_num_add_flag == 0)
		{
			bullet_num_set++;
			set_num_add_flag = 1;
		}
	}
	else
	{
		set_num_add_flag = 0;
	}

	//开始发射前，滑块位置锁死
	if (if_shoot_begin == 0)
	{
		angle_out = position_pid.calc(accumulate_ecd, ecd_set);
		give_current = speed_pid.calc(motor_speed, angle_out);
		on_set_position_time = 0;
		return;
	}

	//未到达指定位置时
	// if(bullet_num_set > bullet_num)
	// {
	// 	//未打完所有飞镖
	// 	if(bullet_num < MAX_DART_NUM)
	// 	{
	// 		//接近设定位置，改用角度环控制
	// 		if(ecd_set - accumulate_ecd < ANGLE_LOOP_SWITCH_DISTANCE)
	// 		{
	// 			angle_out = position_pid.calc(accumulate_ecd, ecd_set);
	// 			give_current = speed_pid.calc(motor_speed, angle_out);
	// 			on_set_position_time++;
	// 			//排除bullet_num = 0的情况，否则第一发飞镖在没有打出的时候计数就会+1
	// 			if (num_add_flag == 0)
	// 			{
	// 				bullet_num++;
	// 				num_add_flag = 1;
	// 			}
	// 			//暂停时间到，打出下一发
	// 			if (on_set_position_time > 500)
	// 			{
	// 				ecd_set = slipper_position_ecd[bullet_num + 1];
	// 				num_add_flag = 0;
	// 			}
	// 		}
	// 		//离设定位置较远，用速度环控制
	// 		else
	// 		{
	// 			give_current = speed_pid.calc(motor_speed, SLIPPER_SHOOTING_SPEED);
	// 			on_set_position_time = 0;
	// 		}
	// 	}
	// 	//打完所有飞镖，自动返回零点
	// 	else
	// 	{
	// 		ecd_set = slipper_position_ecd[0];
	// 		if(accumulate_ecd - ecd_set < ANGLE_LOOP_SWITCH_DISTANCE)
	// 		{
	// 			angle_out = position_pid.calc(accumulate_ecd, ecd_set);
	// 			give_current = speed_pid.calc(motor_speed, angle_out);
	// 		}
	// 		else
	// 		{
	// 			give_current = speed_pid.calc(motor_speed, SLIPPER_BACK_SPEED);
	// 		}
	// 	}
	// }
	// //到达指定位置，滑块锁定
	// else if(bullet_num_set == bullet_num)
	// {
	// 	angle_out = position_pid.calc(accumulate_ecd, ecd_set);
	// 	give_current = speed_pid.calc(motor_speed, angle_out);
	// }

	//未打完所有飞镖
	if (bullet_num < MAX_DART_NUM)
	{
		//未打出设定的发弹量
		if (bullet_num < bullet_num_set)
		{
			//接近设定位置，用角度环控制
			if (ecd_set - accumulate_ecd < ANGLE_LOOP_SWITCH_DISTANCE)
			{
				angle_out = position_pid.calc(accumulate_ecd, ecd_set);
				give_current = speed_pid.calc(motor_speed, angle_out);
				on_set_position_time++;
				if (no_skip_num_add == 1)
				{
					if (num_add_flag == 0)
					{
						bullet_num++;
						num_add_flag = 1;
					}
				}
				//暂停时间到，打出下一发
				if (on_set_position_time > 600)
				{
					ecd_set = slipper_position_ecd[bullet_num + 1];
					num_add_flag = 0;
					no_skip_num_add = 0;
				}
			}
			//离设定位置较远，用速度环控制
			else
			{
				give_current = speed_pid.calc(motor_speed, SLIPPER_SHOOTING_SPEED);
				on_set_position_time = 0;
			}
		}
		//打出设定的发弹量，滑块锁定
		else
		{
			angle_out = position_pid.calc(accumulate_ecd, ecd_set);
			give_current = speed_pid.calc(motor_speed, angle_out);
			no_skip_num_add = 1;
			on_set_position_time++;
		}
	}
	//打完所有飞镖，返回零点
	else
	{
		ecd_set = slipper_position_ecd[0];
		if (accumulate_ecd - ecd_set < ANGLE_LOOP_SWITCH_DISTANCE)
		{
			angle_out = position_pid.calc(accumulate_ecd, ecd_set);
			give_current = speed_pid.calc(motor_speed, angle_out);
		}
		else
		{
			give_current = speed_pid.calc(motor_speed, SLIPPER_BACK_SPEED);
		}
	}
}

void slipper_motor_t::bullet_num_cal()
{
	int i = 0, j = 0;
	fp32 ecd_offset[MAX_DART_NUM + 1];

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

void fric_motor_t::current_calculate()
{
	give_current = speed_pid.calc(motor_measure->speed_rpm, speed_set);
}
