#include "system_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "judge_task.h"
#include "tim.h"
#include "revolver_task.h"
#include "bsp_buzzer.h"
#include "revolver_task.h"

system_t sys;

/*****************ȫ�ֱ���*************************/
int steering_mode = 1; //���ո�ģʽ 0Ϊ�رգ�1Ϊ��
int vision_mode = 1;   //��ģʽ1��1v1 �Զ�����  3��3v3 �ֶ�����
int spin_mode = 0;	   //�Ƿ���С����

fp32 x_coordinate, y_coordinate;		 //����λ��ui
fp32 pre_x_coordinate, pre_y_coordinate; //����Ԥ��λ��ui
fp32 follow_radius, pre_radius;			 //����װ�װ�뾶

/***************************************************/
/**
 * @brief          ϵͳ������
 * @param[in]      none
 */
void system_task(void const *pvParameters)
{
	vTaskDelay(201);
	uint32_t currentTime;
	while (1)
	{
		currentTime = xTaskGetTickCount(); //��ǰϵͳʱ��
		sys.mode_set();
		sys.Transit();
		vTaskDelayUntil(&currentTime, 1);
	}
}

/**
 * @brief 		ȡϵͳ��ָ��
 * @retval   ϵͳ����
 */
system_t *syspoint(void)
{
	return &sys;
}

/**
 * @brief          ���캯����ʼ��
 * @param[in]      null
 */
system_t::system_t()
{
	system_rc_ctrl = get_remote_control_point();
	sys_mode = ZERO_FORCE;
	adjust_mode = SLIPPER;
	shoot_mode = READY;
}

/**
 * @brief          ģʽ����
 * @param[in]      null
 * @retval         null
 */
void system_t::mode_set()
{
	last_sys_mode = sys_mode;
	last_shoot_mode = shoot_mode;

	if (IF_RC_SW0_DOWN /*|| toe_is_error(DBUS_TOE)*/)
	{
		sys_mode = ZERO_FORCE;
	}
	else if (IF_RC_SW0_MID)
	{
		sys_mode = ADJUST;

		if (IF_RC_SW1_DOWN || IF_RC_SW1_MID)
		{
			adjust_mode = SLIPPER;
		}
		else if (IF_RC_SW1_UP)
		{
			adjust_mode = CALIBRATE;
		}
	}
	else if (IF_RC_SW0_UP)
	{
		sys_mode = SHOOT;

		if (IF_RC_SW1_DOWN)
		{
			shoot_mode = READY;
		}
		else if (IF_RC_SW1_MID || IF_RC_SW1_UP)
		{
			shoot_mode = SHOOTING;
		}
	}
}

/**
 * @brief          ��̨ģʽת��
 * @param[in]      null
 * @retval         null
 */
void system_t::Transit()
{
	if (sys_mode == ADJUST && last_sys_mode != ADJUST)
	{
		revolver_point()->slipper_motor.ecd_set = revolver_point()->slipper_motor.accumulate_ecd;
		revolver_point()->slipper_motor.speed_set = 0;
		revolver_point()->slipper_motor.calibrate_begin = 0;
	}
	if (sys_mode == SHOOT && last_sys_mode != SHOOT)
	{
		revolver_point()->is_fric_wheel_on = 0;
		revolver_point()->slipper_motor.speed_set = 0;
		revolver_point()->slipper_motor.ecd_set = revolver_point()->slipper_motor.accumulate_ecd;
		revolver_point()->slipper_motor.bullet_num_cal();
		revolver_point()->slipper_motor.bullet_num_set = revolver_point()->slipper_motor.bullet_num;
		revolver_point()->slipper_motor.if_shoot_begin = 0;
	}
}
