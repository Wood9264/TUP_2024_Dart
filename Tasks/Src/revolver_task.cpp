#include "system_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Revolver_task.h"
#include "judge_task.h"
Revolver_Task  revolver; 
/**
  * @brief          ������������ʼ��
  * @param[in]      null
  */
void Revolver_Task::REVOLVER_Init()

{
	Friction_wheel_set.Friction_Wheel_behaviour_init();
	Dial_set.Dial_mode_init();
	//��ʼ��PID
	fp32 Revolver_speed_pid[3] = {REVOLVER_SPEED_PID_KP, REVOLVER_SPEED_PID_KI, REVOLVER_SPEED_PID_KD};
	fp32 Revolver_position_pid[3] = {REVOLVER_POSITION_PID_KP, REVOLVER_POSITION_PID_KI, REVOLVER_POSITION_PID_KD};
	fp32 Firc_speed_pid[3] = {FIRC_SPEED_PID_KP, FIRC_SPEED_PID_KI, FIRC_SPEED_PID_KD};	 
	revolver_motor_speed_pid.init( PID_POSITION, Revolver_speed_pid,REVOLVER_SPEED_PID_MAX_OUT, REVOLVER_SPEED_PID_MAX_IOUT);
	revolver_motor_position_pid.init(PID_POSITION, Revolver_position_pid,REVOLVER_POSITION_PID_MAX_OUT, REVOLVER_POSITION_PID_MAX_IOUT);

	Firc_R_speed_pid.init(PID_POSITION, Firc_speed_pid,FIRC_SPEED_PID_MAX_OUT, FIRC_SPEED_PID_MAX_IOUT);
	Firc_L_speed_pid.init(PID_POSITION, Firc_speed_pid,FIRC_SPEED_PID_MAX_OUT, FIRC_SPEED_PID_MAX_IOUT);

	//б�º�����������λ��
	Dial_buff_ramp = AN_BULLET/80;
	//���ָ��
	revolver_motor_measure = get_motor_measure_class(TRIGGER);
	Firc_L_firc3508_motor_measure = get_motor_measure_class(FireL);
	Firc_R_firc3508_motor_measure = get_motor_measure_class(FireR);

	angle_out=0;
	//ģʽ��Ƕȳ�ʼ��
	Revolver_Feedback_Update();
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
}



/**
  * @brief          ����������ݸ���
  * @param[in]      null
  */

void Revolver_Task::Revolver_Feedback_Update()
{
	static fp32 speed_fliter_1 = 0.0f;
	static fp32 speed_fliter_2 = 0.0f;
	static fp32 speed_fliter_3 = 0.0f;

	//�����ֵ���ٶ��˲�һ��
	static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

	//���׵�ͨ�˲�
	speed_fliter_1 = speed_fliter_2;
	speed_fliter_2 = speed_fliter_3;
	speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (revolver_motor_measure->speed_rpm * Motor_RMP_TO_SPEED) * fliter_num[2];
	Revolver_speed = speed_fliter_3;
	firc_l = Firc_L_firc3508_motor_measure->speed_rpm*Motor_RMP_TO_SPEED;
	firc_r = Firc_R_firc3508_motor_measure->speed_rpm*Motor_RMP_TO_SPEED;
	//���������Ƕ�
	Dial_set.angle = revolver_motor_measure->num*-8192 + revolver_motor_measure->ecd;
	
Dial_set.heat_max=JUDGE_usGetHeatLimit_id1_17mm();
Dial_set.heating = JUDGE_usGetRemoteHeat_id1_17mm();
bullet_speed=JUDGE_usGetSpeedHeat();
//Dial_set.heat_max=100;
//Dial_set.heating = 20;
//bullet_speed=JUDGE_usGetSpeedHeat();
}

/**
  * @brief          �������Ħ���ֿ���������
  * @param[in]      null
  */

void Revolver_Task::Friction_wheel_calculating()
{

		if(switch_is_down(Friction_wheel_set.Friction_wheel_RC->rc.s[RevChannel]) || switch_is_up(Friction_wheel_set.Friction_wheel_RC->rc.s[RevChannel]) 
			|| switch_is_down(Friction_wheel_set.Friction_wheel_RC->rc.s[0]))
		{
			Firc_L_give_current = Firc_L_speed_pid.calc( Firc_L_firc3508_motor_measure->speed_rpm,Friction_wheel_set.Firc_L.speed_set);
			Firc_R_give_current = Firc_R_speed_pid.calc( Firc_R_firc3508_motor_measure->speed_rpm,Friction_wheel_set.Firc_R.speed_set);

			Firc_L_speed_ramp_set = 0;
		  Firc_R_speed_ramp_set = 0;

		} 
		else if((switch_is_mid(Friction_wheel_set.Friction_wheel_RC->rc.s[RevChannel]) &&switch_is_mid(Friction_wheel_set.Friction_wheel_RC->rc.s[0]))
			   ||(switch_is_mid(Friction_wheel_set.Friction_wheel_RC->rc.s[RevChannel]) &&switch_is_up(Friction_wheel_set.Friction_wheel_RC->rc.s[0])))
		{
			Firc_L_speed_ramp_set = RAMP_float(Friction_wheel_set.Firc_L.speed_set,Firc_L_firc3508_motor_measure->speed_rpm,75);
			Firc_R_speed_ramp_set = RAMP_float(Friction_wheel_set.Firc_R.speed_set, Firc_R_firc3508_motor_measure->speed_rpm,75);			
			
			// �������ֵ
			Firc_L_give_current	=Firc_L_speed_pid.calc( Firc_L_firc3508_motor_measure->speed_rpm,Firc_L_speed_ramp_set);
			Firc_R_give_current	=	Firc_R_speed_pid.calc(Firc_R_firc3508_motor_measure->speed_rpm,Firc_R_speed_ramp_set);
		}		
		else 
		{
		  Firc_L_give_current=0;
	   	Firc_R_give_current=0;
			Firc_L_speed_ramp_set = 0;
		  Firc_R_speed_ramp_set = 0; 
		
		}
}

/**
  * @brief          ����������̿���������
  * @param[in]      null
  */
void Revolver_Task::Dial_calculating()
{
		if(Dial_set_ramp_angle != Dial_set.set_angle)//����ת��ȥ
		{
			Dial_set_ramp_angle = RAMP_float(Dial_set.set_angle, Dial_set_ramp_angle, Dial_buff_ramp);
		}
		//�ǶȻ����ٶȻ�����pid���ԣ������������ڲ������ۼ�ʱ�Ѿ���ɣ�
		angle_out = revolver_motor_position_pid.calc( Dial_set.angle, Dial_set_ramp_angle);
		Dial_give_current = revolver_motor_speed_pid.calc(Revolver_speed, angle_out);

		if(switch_is_down(Dial_set.Dial_RC->rc.s[RevChannel]))
		{
			Dial_give_current=0;
		}

}

/**
  * @brief          �����������
  * @param[in]      null
  */
void revolver_task(void const *pvParameters)
{
	//��ʱ
	vTaskDelay(REVOLVER_TASK_INIT_TIME);
	//��������ʼ��
	revolver.REVOLVER_Init();
	
	while(1)
	{
		//���ݸ���
		revolver.Revolver_Feedback_Update(); 
		//����ֵ�趨
		revolver.Friction_wheel_set.Friction_wheel_mode_set();
		revolver.Dial_set.Dial_behaviour_set();
		//�趨ֵ����
		revolver.Friction_wheel_calculating();
		revolver.Dial_calculating();
		//�����ķ���

		CAN2_200_cmd_motor(revolver.Firc_L_give_current,revolver.Firc_R_give_current,revolver.Dial_give_current,0);
		vTaskDelay(2);
  }
}


