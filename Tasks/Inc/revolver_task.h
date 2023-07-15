#ifndef _REVOLVER_TASK_H
#define _REVOLVER_TASK_H
#ifdef __cplusplus
#include "system_task.h"
#include "pid.h"
#include "BSP_can.h"
#include "tim.h"
//����˫��PID ע�ⶶ��
#define SLIPPER_SPEED_PID_KP 700.0f // 780.0f
#define SLIPPER_SPEED_PID_KI 0
#define SLIPPER_SPEED_PID_KD 800 // 2
#define SLIPPER_SPEED_PID_MAX_OUT 9500.0f
#define SLIPPER_SPEED_PID_MAX_IOUT 5000.0f

#define SLIPPER_POSITION_PID_KP 0.025f // 0.040f //0.00080f//0.00072f//0.025
#define SLIPPER_POSITION_PID_KI 0
#define SLIPPER_POSITION_PID_KD 0.080f		// 0.060f//0.000001f//0.08
#define SLIPPER_POSITION_PID_MAX_OUT 60.0f // 45.0f
#define SLIPPER_POSITION_PID_MAX_IOUT 10.0f

//Ħ�����ٶȻ�PID
#define FRIC_SPEED_PID_KP 15.50f
#define FRIC_SPEED_PID_KI 0.000f
#define FRIC_SPEED_PID_KD 5.50f
#define FRIC_SPEED_PID_MAX_OUT 16384.0f
#define FRIC_SPEED_PID_MAX_IOUT 5000.0f

#define BASE_SPEED 6000 //�ĸ�Ħ���ֵĻ���ת��

#define ONE_DART_ECD (8192 * 19 * 2) //ÿ�����ڵĻ���������ֵ��������������
#define MAX_DART_NUM 2 //����ܿ�װ�������������

#define RC_TO_SLIPPER_SEPPD_SET (5.0f / 660) //ң����ͨ���������ٶ��趨ֵ�ı���
#define SLIPPER_LOCK_SPEED 0.1 //�������ٶ��趨Ϊ0����ʵ���ٶȵ������ֵʱ������ýǶȻ�����λ��

#define Motor_RMP_TO_SPEED 0.00290888208665721596153948461415f

#define POSITION_LIMIT_BUFFER_DISTANCE (8192 * 19 / 19) //����ӽ���λ��ʼ���ٻ���ʱ����λ�ľ���
#define ANGLE_LOOP_SWITCH_DISTANCE (8192 * 10 / 360.0f) //�ٶȻ��л����ǶȻ�ʱ���趨����ֵ��ʵ�ʱ���ֵ�ľ���

#define CALIBRATE_DOWN_SPEED (-3) //У׼ʱ�������Ƶ��ٶ�
#define CALIBRATE_UP_SPEED 2 //У׼ʱ�������Ƶ��ٶ�
#define SLIPPER_SHOOTING_SPEED 5 //����ʱ�������Ƶ��ٶ�
#define SLIPPER_BACK_SPEED -5 //�����Զ��������ʱ���ٶ�

#define REVOLVER_TASK_INIT_TIME 200
#ifdef __cplusplus
extern "C"
{
#endif

class fric_motor_t
	{
		public:
		const motor_t *motor_measure;
		PID_t speed_pid;

		fp32 speed_set;		

		int16_t give_current;

		void current_calculate();
	};

	class slipper_motor_t
	{
		public:
		const motor_t *motor_measure;
		fp32 motor_speed;
		fp32 speed_set;
		int64_t accumulate_ecd;
		uint16_t bullet_num;
		uint16_t bullet_num_set;

		fp32 ecd_set;
		fp32 slipper_position_ecd[MAX_DART_NUM + 1]; //���ڷ���ʱ����ͣ����λ��
		
		PID_t speed_pid;
		PID_t position_pid;

		int16_t give_current;

		bool_t has_calibrated;
		bool_t bottom_tick;
		bool_t if_shoot_begin;
		bool_t should_lock;

		void SLIPPER_control();
		void position_limit_buffer(fp32 limit_point);
		void CALIBRATE_control();
		void bullet_num_cal();
		void SHOOTING_slipper_control();
	};
	
	class revolver_task_t
	{
	public:
		revolver_task_t();

		const RC_ctrl_t *revolver_rc_ctrl;

		fric_motor_t fric_motor[4];
		slipper_motor_t slipper_motor;

		fp32 fric_speed_offset;
		bool_t is_fric_wheel_on;

		void revolver_feedback_update();
		void control();
		void ZERO_FORCE_control();
		void fric_speed_offset_control();
		void ADJUST_control();
		void SHOOT_control();
		void READY_control();
		void SHOOTING_control();
	};

	

	extern revolver_task_t revolver;
	extern revolver_task_t* revolver_point(void);
#endif
	extern void revolver_task(void const *pvParameters);

#ifdef __cplusplus
}
#endif
#endif
