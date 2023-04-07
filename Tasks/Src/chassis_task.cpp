#include "chassis_task.h"
#include "struct_typedef.h"
#include "Bsp_can.h"
#include "user_lib.h"
#include "arm_math.h"
#include "gimbal_task.h"
#include "task.h"

chassis_t chassis_move;

/**
  * @brief          ������ָ��
  * @param[in]      null
  * @retval         ���̶���
  */
chassis_t* chassispoint(void)
{
	return &chassis_move;
}

/**
  * @brief          ��������
  * @param[in]      none
  * @retval         none
  */
void chassis_task(void const * pvParameters)
{
	 //����һ��ʱ��
  vTaskDelay(CHASSIS_TASK_INIT_TIME);
	uint32_t currentTime;
	while(1)
	{
		
		currentTime = xTaskGetTickCount();//��ǰϵͳʱ��
    chassis_move.Get_info();
	  chassis_move.Control();
//		CAN1_200_cmd_motor(1000,1000,1000,1000);
		CAN1_200_cmd_motor(chassis_move.chassis_motor[0].give_current,chassis_move.chassis_motor[1].give_current,
		                      chassis_move.chassis_motor[2].give_current,chassis_move.chassis_motor[3].give_current);
	  vTaskDelayUntil(&currentTime, 2);
	}
}



/**
  * @brief          ���캯����ʼ��
  * @param[in]      none
  * @retval         none
  */
chassis_t::chassis_t()
{
  //��ʼ�����̵��pid ��ת��pid
  fp32 chassis_yaw_pid[3] = {CHASSIS_MOTOR_YAW_PID_KP,CHASSIS_MOTOR_YAW_PID_KI,CHASSIS_MOTOR_YAW_PID_KD};  
	chassis_angle_pid.init(PID_POSITION,chassis_yaw_pid, MOTOR_YAW_PID_MAX_OUT, MOTOR_YAW_PID_MAX_IOUT);
	
	fp32 motor_speed_pid[3] ={CHASSIS_MOTOR_SPEED_PID_KP, CHASSIS_MOTOR_SPEED_PID_KI, CHASSIS_MOTOR_SPEED_PID_KD};
	for(int i=0;i<=3;i++)
  {
		chassis_motor[i].chassis_motor_measure=get_motor_measure_class(i);
	  chassis_motor_speed_pid[i].init(PID_POSITION,motor_speed_pid,M3508_MOTOR_SPEED_PID_MAX_OUT, M3508_MOTOR_SPEED_PID_MAX_IOUT);
	}
	//����vxvy�����С�ٶ�
	vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
	vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;
	vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
	vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

}

/**
  * @brief          �������ݶ�ȡ
  * @param[in]      none
  * @retval         none
  */
void chassis_t::Get_info()
{
	  //��ȡ��ǰģʽ
    chassis_mode=syspoint()->sys_mode;
	 //��ȡ��̨�������ָ��
	  chassis_yaw_motor = gimbal_point()->Yaw_motor;
	  //��ȡ����ٶȣ�������ķ������ٶȻ���ɵ��̵��ٶ�
	   for ( int i = 0; i <= 3; i++)
    {
			chassis_motor[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN *chassis_motor[i].chassis_motor_measure->speed_rpm;	   
		}
    //��ȡ����ǰ���ٶ� x�� ƽ���ٶ�y����ת�ٶ�wz������ϵΪ����ϵ
    vx = (-chassis_motor[0].speed +chassis_motor[1].speed 
		                           + chassis_motor[2].speed - chassis_motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    vy = (-chassis_motor[0].speed - chassis_motor[1].speed 
		                           + chassis_motor[2].speed + chassis_motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    wz = (-chassis_motor[0].speed - chassis_motor[1].speed 
		                           -chassis_motor[2].speed - chassis_motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;
}

/**
  * @brief          ���̿��������������
  * @param[in]      none
  * @retval         none
  */
void chassis_t::Control()
{
		if(syspoint()->sys_mode== INIT||syspoint()->sys_mode== ZERO_FORCE||syspoint()->sys_mode==DBUS_MISSING_CONTROL||syspoint()->sys_mode == AUTO)
		{
			chassis_zero_control_set();
		}else if(syspoint()->sys_mode== ABSOLUTE_ANGLE)
		{
			chassis_follow_gimbal_control_set();
		}else if(syspoint()->sys_mode== SPIN)
		{
			chassis_spin_control_set();
		}else if(syspoint()->sys_mode== NO_FOLLOW_YAW)
		{
		  chassis_no_follow_gimbal_control_set();
		}
		chassis_control_loop();
	
}
/**
  * @brief          ��������ģʽ����������
  * @param[in]      none
  * @retval         none
  */
void chassis_t::chassis_zero_control_set()
{
    vx_set = 0.0f;
    vy_set = 0.0f;
    wz_set = 0.0f;
    syspoint()->chassis_cmd_slow_set_vx.out=0;
	  syspoint()->chassis_cmd_slow_set_vy.out=0;
}

/**
  * @brief          ���̸�����̨ģʽ����������
  * @param[in]      none
  * @retval         none
  */

void chassis_t::chassis_follow_gimbal_control_set()
{
    syspoint()->Chassis_value_calc();
    vx_follow_set=syspoint()->chassis_cmd_slow_set_vx.out;
	  vy_follow_set=syspoint()->chassis_cmd_slow_set_vy.out;
    fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
		//��ת���Ƶ����ٶȷ��򣬱�֤ǰ����������̨�����������˶�ƽ��
		sin_yaw = arm_sin_f32(-chassis_yaw_motor.relative_angle);
		cos_yaw = arm_cos_f32(-chassis_yaw_motor.relative_angle);
		vx_set = cos_yaw * vx_follow_set + sin_yaw * vy_follow_set;
		vy_set = -sin_yaw * vx_follow_set + cos_yaw * vy_follow_set;
		//���ÿ��������̨�Ƕ�
	  angle_set=0;
		chassis_relative_angle_set = rad_format(angle_set);
		chassis_angle_pid.out=chassis_angle_pid.calc(chassis_yaw_motor.relative_angle,chassis_relative_angle_set);
	  wz_set=chassis_angle_pid.out;
		//�ٶ��޷�
		vx_set = fp32_constrain(vx_set, vx_min_speed, vx_max_speed);
		vy_set = fp32_constrain(vy_set, vy_min_speed, vy_max_speed);
    
}

/**
  * @brief          ���̲�������̨����������
  * @param[in]      none
  * @retval         none
  */
void chassis_t::chassis_no_follow_gimbal_control_set()
{
    syspoint()->Chassis_value_calc();
    vx_set=syspoint()->chassis_cmd_slow_set_vx.out;
	  vy_set=syspoint()->chassis_cmd_slow_set_vy.out;
	  angle_set= -CHASSIS_WZ_RC_SEN * syspoint()->system_rc_ctrl->rc.ch[CHASSIS_WZ_CHANNEL];

	  wz_set=angle_set;
		//�ٶ��޷�
    vx_set = fp32_constrain(vx_set, vx_min_speed, vx_max_speed);
		vy_set = fp32_constrain(vy_set, vy_min_speed, vy_max_speed);

}

/**
  * @brief          ����С����ģʽ���������� 
  * @param[in]      none
  * @retval         none
  */
void chassis_t::chassis_spin_control_set()
{
    
	  fp32 p=0.7;
    vx_follow_set *=p;
		vy_follow_set *=p;
    syspoint()->Chassis_value_calc();
    vx_follow_set=syspoint()->chassis_cmd_slow_set_vx.out;
	  vy_follow_set=syspoint()->chassis_cmd_slow_set_vy.out;
    angle_set = 1.5 * (8 - fabs(vx_follow_set)- fabs(vy_follow_set));  //����ģʽ
	
  	fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
		//��ת���Ƶ����ٶȷ��򣬱�֤ǰ����������̨�����������˶�ƽ��
		sin_yaw = arm_sin_f32(-chassis_yaw_motor.relative_angle);
		cos_yaw = arm_cos_f32(-chassis_yaw_motor.relative_angle); 

		vx_set = cos_yaw * vx_follow_set + sin_yaw  * vy_follow_set;
		vy_set = -sin_yaw * vx_follow_set + cos_yaw * vy_follow_set;
	
		wz_set = angle_set;
	   //�ٶ��޷�
		vx_set = fp32_constrain(vx_set, vx_min_speed, vx_max_speed);
		vy_set = fp32_constrain(vy_set, vy_min_speed, vy_max_speed);

}


/**
  * @brief          ���̿���������
  * @param[in]      none
  * @retval         none
  */
void chassis_t::chassis_control_loop()
{
	
		fp32 max_vector = 0.0f, vector_rate = 0.0f;
		fp32 temp = 0.0f;
		if(chassis_mode== INIT||chassis_mode== ZERO_FORCE)
		{
			 for(int i=0;i<=3;i++)
			{
			  chassis_motor_speed_pid[i].out=0;
			  chassis_motor[i].give_current=0;
			}
			 return;
		}
		chassis_vector_to_mecanum_wheel_speed(vx_set,vy_set,wz_set,wheel_speed);
		//�������ӿ�������ٶȣ�������������ٶ�i
		for (int i = 0; i <= 3; i++)
		{
				chassis_motor[i].speed_set = wheel_speed[i];
				temp = fabs(chassis_motor[i].speed_set);
				if (max_vector < temp)
				{
						max_vector = temp;
				}
		}

		if (max_vector > MAX_WHEEL_SPEED)
		{
				vector_rate = MAX_WHEEL_SPEED / max_vector;
				for (int i = 0; i <= 3; i++)
				{
						chassis_motor[i].speed_set *= vector_rate;
				}
		}
	
	  for (int i = 0; i <=3; i++)
    {
        chassis_motor_speed_pid[i].out=chassis_motor_speed_pid[i].calc( chassis_motor[i].speed,chassis_motor[i].speed_set);
    }
    
		//��ֵ����ֵ
    for (int i = 0; i <= 3; i++)
    {
         chassis_motor[i].give_current = (int16_t)( chassis_motor_speed_pid[i].out);
    }
}

/**
  * @brief          �������ݽ���
  * @param[in]      none
  * @retval         none
  */
void chassis_t::chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])	
{   
	
		wheel_speed[0] = (-vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set);
		wheel_speed[1] = (vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set);
		wheel_speed[2] = (-vx_set - vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set);
		wheel_speed[3] = (vx_set - vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set);
	
//	if(chassis_move.chassis_mode == SPIN)
//	{
//    wheel_speed[0] = (-vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set);
//		wheel_speed[1] = (vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set);
//		wheel_speed[2] = (vx_set - vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set);
//		wheel_speed[3] = (-vx_set - vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set);
//	}
	
}


//void chassis_t::chassis_power_limit()
//{


//}

