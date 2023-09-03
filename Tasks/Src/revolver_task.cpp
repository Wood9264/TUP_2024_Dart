#include "system_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Revolver_task.h"
#include "judge_task.h"
#include "BSP_buzzer.h"

revolver_task_t  revolver; 

revolver_task_t* revolver_point(void)
{
	return &revolver;
}

fp32 SPEED[4];
fp32 TEMP[4];

/**
  * @brief          �����������
  * @param[in]      null
  */
void revolver_task(void const *pvParameters)
{
	//��ʱ
	vTaskDelay(REVOLVER_TASK_INIT_TIME);
	while(1)
	{
		//���ݸ���
		revolver.revolver_feedback_update();
		//���������
		revolver.control();
		//���͵���
		CAN1_200_cmd_motor(revolver.fric_motor[0].give_current, revolver.fric_motor[1].give_current, revolver.fric_motor[2].give_current, revolver.fric_motor[3].give_current);
		CAN2_200_cmd_motor(revolver.slipper_motor.give_current, 0, 0, 0);
		vTaskDelay(2);

		for(int i = 0; i < 4; i++)
		{
			SPEED[i] = revolver.fric_motor[i].motor_measure->speed_rpm;
			TEMP[i] = revolver.fric_motor[i].motor_measure->temperate;
		}
  }
}

/**
  * @brief          ������������ʼ��
  * @param[in]      null
  */
revolver_task_t::revolver_task_t()
{
	//��ʼ��PID
	fp32 slipper_speed_pid[3] = {SLIPPER_SPEED_PID_KP, SLIPPER_SPEED_PID_KI, SLIPPER_SPEED_PID_KD};
	fp32 slipper_position_pid[3] = {SLIPPER_POSITION_PID_KP, SLIPPER_POSITION_PID_KI, SLIPPER_POSITION_PID_KD};
	fp32 Fric_speed_pid[3] = {FRIC_SPEED_PID_KP, FRIC_SPEED_PID_KI, FRIC_SPEED_PID_KD};

	slipper_motor.speed_pid.init(PID_POSITION, slipper_speed_pid,SLIPPER_SPEED_PID_MAX_OUT, SLIPPER_SPEED_PID_MAX_IOUT);
	slipper_motor.position_pid.init(PID_POSITION, slipper_position_pid,SLIPPER_POSITION_PID_MAX_OUT, SLIPPER_POSITION_PID_MAX_IOUT);

	for(int i = 0; i < 4; i++)
	{
		fric_motor[i].speed_pid.init(PID_POSITION, Fric_speed_pid,FRIC_SPEED_PID_MAX_OUT, FRIC_SPEED_PID_MAX_IOUT);
	}
	
	slipper_motor.calibrate_begin = 0;
	slipper_motor.has_calibrated = 0;
	slipper_motor.if_shoot_begin = 0;

	//���ָ��
	slipper_motor.motor_measure = get_motor_measure_class(SLIPPER_MOTOR);
	fric_motor[0].motor_measure = get_motor_measure_class(FL);
	fric_motor[1].motor_measure = get_motor_measure_class(FR);
	fric_motor[2].motor_measure = get_motor_measure_class(BL);
	fric_motor[3].motor_measure = get_motor_measure_class(BR);

	revolver_rc_ctrl = get_remote_control_point();
}

/**
  * @brief          ����������ݸ���
  * @param[in]      null
  */
void revolver_task_t::revolver_feedback_update()
{
	static fp32 speed_fliter_1 = 0.0f;
	static fp32 speed_fliter_2 = 0.0f;
	static fp32 speed_fliter_3 = 0.0f;

	//�������ٶ��˲�
	static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};
	//���׵�ͨ�˲�
	speed_fliter_1 = speed_fliter_2;
	speed_fliter_2 = speed_fliter_3;
	speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (slipper_motor.motor_measure->speed_rpm * Motor_RMP_TO_SPEED) * fliter_num[2];
	slipper_motor.motor_speed = speed_fliter_3;
	//�ۼƱ���ֵ
	slipper_motor.accumulate_ecd = slipper_motor.motor_measure->num * 8192 + slipper_motor.motor_measure->ecd;

	slipper_motor.bottom_tick = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11);

	static int16_t recali_time = 0;
	//����У׼��ʱ������ʱ�򻬿鲻�������ײ����㿪�أ��������˵��У׼�����ˣ�Ҫ����У׼
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
 * @brief	���������
*/
void revolver_task_t::control()
{
	if(syspoint()->sys_mode == ZERO_FORCE)
	{
		ZERO_FORCE_control();
		//Ħ����ת�ٲ���
		fric_speed_offset_control();
		//ң������ȡĦ����ת��
		fric_speed_buzzer();
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

/**
 * @brief	����ģʽ
*/
void revolver_task_t::ZERO_FORCE_control()
{
	for(int i = 0; i < 4; i++)
	{
		fric_motor[i].speed_set = RAMP_float(0, fric_motor[i].speed_set, FRIC_RAMP_BUFF);
		fric_motor[i].current_calculate();
	}
	slipper_motor.give_current = 0;
}

/**
 * @brief	ң������Ħ���ֵ�ת��
*/
void revolver_task_t::fric_speed_offset_control()
{
	static bool_t offset_flag = 0;

	//ң������ҡ�˨J���е���
	if (IF_RIGHT_ROCKER_RIGHT_TOP)
	{
		//��ҡ���Ҳ�һ��+100�� ��һ��-100
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

/**
 * @brief	����ģʽ�������ֶ����������λ��
*/
void revolver_task_t::ADJUST_control()
{
	static fp32 angle_out = 0;

	for (int i = 0; i < 4; i++)
	{
		fric_motor[i].speed_set = RAMP_float(0, fric_motor[i].speed_set, FRIC_RAMP_BUFF);
		fric_motor[i].current_calculate();
	}

	//У׼������������ֶ���������λ��
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

/**
 * @brief	�ֶ���������λ��
*/
void slipper_motor_t::SLIPPER_control()
{
	speed_set = (revolver.revolver_rc_ctrl->rc.ch[3]) * RC_TO_SLIPPER_SEPPD_SET;
	//�ײ���λ����
	if(motor_speed < 0)
	{
		// position_limit_buffer(slipper_position_ecd[0]);
	}
	//������λ����
	else if(motor_speed > 0)
	{
		// position_limit_buffer(slipper_position_ecd[MAX_DART_NUM]);
	}

	//�����λ
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

/**
 * @brief	У׼ģʽ���趨��ײ�Ϊ��������
*/
void slipper_motor_t::CALIBRATE_control()
{
	static uint16_t rc_cmd_time = 0;
	static bool_t found_zero_point = 0; //�Ƿ��ҵ����

	//ң�����K�L��ʼУ׼
	if(IF_LEFT_ROCKER_RIGHT_BOTTOM && IF_RIGHT_ROCKER_LEFT_BOTTOM)
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


	//���㿪�ر�ѹ�£��ҵ����
	if(bottom_tick)
	{
		found_zero_point = 1;
	}

	//��������
	if(found_zero_point == 0)
	{
		speed_set = CALIBRATE_DOWN_SPEED;
	}
	//ѹ�´��㿪�غ�����
	else if(found_zero_point == 1 && bottom_tick == 1)
	{
		speed_set = CALIBRATE_UP_SPEED;
	}
	//�����뿪���㿪�أ�У׼���
	else if(found_zero_point == 1 && bottom_tick == 0)
	{
		speed_set = 0;
		has_calibrated = 1;
		found_zero_point = 0;
		calibrate_begin = 0;

		//���÷��ڷ���ʱ����ÿ��ͣ����λ��
		for(int i = 0; i <= MAX_DART_NUM; i++)
		{
			slipper_position_ecd[i] = accumulate_ecd + i * ONE_DART_ECD + CALIBRATE_OFFSET;
		}
	}
}

/**
 * @brief	����ģʽ
*/
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

/**
 * @brief	׼�����䣬�ڴ�ģʽ�²��ܿ���Ħ����
*/
void revolver_task_t::READY_control()
{
	static uint16_t rc_cmd_time = 0;
	static fp32 angle_out = 0;

	//ң�����J�I������Ħ����
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
		fric_motor[0].speed_set = RAMP_float( (BASE_SPEED + fric_speed_offset), fric_motor[0].speed_set, FRIC_RAMP_BUFF);
		fric_motor[1].speed_set = RAMP_float(-(BASE_SPEED + fric_speed_offset), fric_motor[1].speed_set, FRIC_RAMP_BUFF);
		fric_motor[2].speed_set = RAMP_float( (BASE_SPEED + fric_speed_offset), fric_motor[2].speed_set, FRIC_RAMP_BUFF);
		fric_motor[3].speed_set = RAMP_float(-(BASE_SPEED + fric_speed_offset), fric_motor[3].speed_set, FRIC_RAMP_BUFF);
	}

	//����λ������
	angle_out = slipper_motor.position_pid.calc(slipper_motor.accumulate_ecd, slipper_motor.ecd_set);
	slipper_motor.give_current = slipper_motor.speed_pid.calc(slipper_motor.motor_speed, angle_out);
}

/**
 * @brief	��ʼ����
*/
void revolver_task_t::SHOOTING_control()
{
	//Ħ����ת���趨
	if(is_fric_wheel_on == 0)
	{
		for(int i = 0; i < 4; i++)
		{
			fric_motor[i].speed_set = 0;
		}
	}
	else if (is_fric_wheel_on == 1)
	{
		fric_motor[0].speed_set =  (BASE_SPEED + fric_speed_offset);
		fric_motor[1].speed_set = -(BASE_SPEED + fric_speed_offset);
		fric_motor[2].speed_set =  (BASE_SPEED + fric_speed_offset + 1000);
		fric_motor[3].speed_set = -(BASE_SPEED + fric_speed_offset + 1000);
	}

	slipper_motor.SHOOTING_slipper_control();
	
}

/**
 * @brief	����ʱ����Ŀ���
*/
void slipper_motor_t::SHOOTING_slipper_control()
{
	static bool_t set_num_add_flag = 0;
	static bool_t num_add_flag = 0;
	static fp32 angle_out = 0;

	//���δУ׼����������
	if (has_calibrated == 0)
	{
		angle_out = position_pid.calc(accumulate_ecd, ecd_set);
		give_current = speed_pid.calc(motor_speed, angle_out);
		return;
	}

	//ң������ҡ�˨Iÿ��һ�Σ���������+1
	if (IF_LEFT_ROCKER_LEFT_TOP)
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

	//��ʼ����ǰ������λ������
	if (if_shoot_begin == 0)
	{
		angle_out = position_pid.calc(accumulate_ecd, ecd_set);
		give_current = speed_pid.calc(motor_speed, angle_out);
		return;
	}

	//δ�������з���
	if (bullet_num_set <= MAX_DART_NUM)
	{
		ecd_set = slipper_position_ecd[bullet_num_set];
		angle_out = position_pid.DLcalc(accumulate_ecd, ecd_set, SLIPPER_SHOOTING_SPEED);
		give_current = speed_pid.calc(motor_speed, angle_out);
	}
	//����������з��ڣ��Զ��ص����
	else
	{
		ecd_set = slipper_position_ecd[0];
		angle_out = position_pid.DLcalc(accumulate_ecd, ecd_set, SLIPPER_BACK_SPEED);
		give_current = speed_pid.calc(motor_speed, angle_out);
	}
}

/**
 * @brief	���㻬������ĸ�ͣ�����������ȡ�ĸ������Ѿ�����ķ�����
*/
void slipper_motor_t::bullet_num_cal()
{
	int i = 0, j = 0;
	fp32 ecd_offset[MAX_DART_NUM + 1]; //��ǰ����λ�õı���ֵ��ÿ��ͣ�������ֵ�Ĳ�

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
 * @brief	Ħ���ֵ�������
*/
void fric_motor_t::current_calculate()
{
	give_current = speed_pid.calc(motor_measure->speed_rpm, speed_set);
}

/**
 * @brief	ң������ȡĦ����ת��
*/
void revolver_task_t::fric_speed_buzzer()
{
	static uint8_t show_num = 0;

	if (IF_RIGHT_ROCKER_RIGHT_BOTTOM)
	{
		//ǧλ
		if (IF_LEFT_ROCKER_RIGHT_TOP)
		{
			show_num = (BASE_SPEED + fric_speed_offset) / 1000;
			buzzer_warn_error(show_num);

		}
		//��λ
		else if (IF_LEFT_ROCKER_LEFT_TOP)
		{
			show_num = ((int)(BASE_SPEED + fric_speed_offset) / 100) % 10;
			buzzer_warn_error(show_num);
		}
		else
		{
			buzzer_off();
		}
	}
	else
	{
		buzzer_off();
	}
}
