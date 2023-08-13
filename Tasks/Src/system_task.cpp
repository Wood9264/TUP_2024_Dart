#include "system_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "judge_task.h"
#include "tim.h"
#include "revolver_task.h"
#include "bsp_buzzer.h"
#include "revolver_task.h"

system_t sys;

/*****************全局变量*************************/
int steering_mode = 1; //弹舱盖模式 0为关闭，1为打开
int vision_mode = 1;   //打弹模式1：1v1 自动开火  3：3v3 手动开火
int spin_mode = 0;	   //是否开启小陀螺

fp32 x_coordinate, y_coordinate;		 //自瞄位置ui
fp32 pre_x_coordinate, pre_y_coordinate; //自瞄预测位置ui
fp32 follow_radius, pre_radius;			 //自瞄装甲板半径

/***************************************************/
/**
 * @brief          系统主任务
 * @param[in]      none
 */
void system_task(void const *pvParameters)
{
	vTaskDelay(201);
	uint32_t currentTime;
	while (1)
	{
		currentTime = xTaskGetTickCount(); //当前系统时间
		sys.mode_set();
		sys.Transit();
		vTaskDelayUntil(&currentTime, 1);
	}
}

/**
 * @brief 		取系统类指针
 * @retval   系统对象
 */
system_t *syspoint(void)
{
	return &sys;
}

/**
 * @brief          构造函数初始化
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
 * @brief          模式设置
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
 * @brief          云台模式转换
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
