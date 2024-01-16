#include "system_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "judge_task.h"
#include "tim.h"
#include "revolver_task.h"
#include "bsp_buzzer.h"
#include "revolver_task.h"
#include "load_task.h"

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
	last_sys_mode = ZERO_FORCE;
}

/**
 * @brief          模式设置
 * @param[in]      null
 * @retval         null
 */
void system_t::mode_set()
{
	last_sys_mode = sys_mode;

	if (IF_RC_SW0_DOWN || toe_is_error(DBUS_TOE))
	{
		sys_mode = ZERO_FORCE;
	}
	else if (IF_RC_SW0_MID)
	{
		sys_mode = CALIBRATE;
	}
	else if (IF_RC_SW0_UP)
	{
		sys_mode = SHOOT;
	}
}

/**
 * @brief          云台模式转换
 * @param[in]      null
 * @retval         null
 */
void system_t::Transit()
{
	if (sys_mode == CALIBRATE && last_sys_mode != CALIBRATE)
	{
		revolver_point()->yaw_motor.ecd_set = revolver_point()->yaw_motor.accumulate_ecd;
		load_point()->loader_motor.ecd_set = load_point()->loader_motor.accumulate_ecd;
		load_point()->loader_motor.speed_set = 0;
		load_point()->loader_motor.has_calibrate_begun = 0;
		load_point()->loader_motor.ecd_set = load_point()->loader_motor.accumulate_ecd;
		load_point()->rotary_motor.relative_angle_set = load_point()->rotary_motor.relative_angle;
	}
	if (sys_mode == SHOOT && last_sys_mode != SHOOT)
	{
		revolver_point()->is_fric_wheel_on = 0;
		revolver_point()->yaw_motor.ecd_set = revolver_point()->yaw_motor.accumulate_ecd;
		revolver_point()->yaw_motor.has_shoot_init_finished = 0;
		load_point()->loader_motor.ecd_set = load_point()->loader_motor.accumulate_ecd;
		load_point()->loader_motor.has_shoot_init_finished = 0;
		load_point()->rotary_motor.final_relative_angle_set = load_point()->rotary_motor.relative_angle;
		load_point()->rotary_motor.relative_angle_set = load_point()->rotary_motor.relative_angle;
		load_point()->rotary_motor.has_shoot_init_finished = 0;
	}
}
