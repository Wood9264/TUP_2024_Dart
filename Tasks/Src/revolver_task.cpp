/**
 * @file    revolver_task.cpp
 * @author  Yang Maolin (1831051389@qq.com)
 * @brief   发射机构相关代码。发射机构包括四个摩擦轮和yaw轴电机，摩擦轮用于发射飞镖，yaw轴用于瞄准目标，没有pitch轴。
 */
#include "system_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Revolver_task.h"
#include "judge_task.h"
#include "BSP_buzzer.h"
#include "monitor_task.h"
#include "load_task.h"
#include "screen_task.h"

revolver_task_t revolver;

/**
 * @brief   返回发射机构任务指针
 * @retval  发射机构任务指针
 */
revolver_task_t *revolver_point(void)
{
    return &revolver;
}

fp32 SPEED[4];
fp32 TEMP[4];

/**
 * @brief   发射机构任务
 * @param   pvParameters
 */
void revolver_task(void const *pvParameters)
{
    //初始化延时
    vTaskDelay(REVOLVER_TASK_INIT_TIME);
    uint32_t currentTime;
    while (1)
    {
        //获取当前系统时间
        currentTime = xTaskGetTickCount();
        //数据更新
        revolver.data_update();
        //发射机构控制
        revolver.control();
        //发送电流
        CAN1_200_cmd_motor(revolver.fric_wheel_group.fric_motor[0].give_current, revolver.fric_wheel_group.fric_motor[1].give_current,
                           revolver.fric_wheel_group.fric_motor[2].give_current, revolver.fric_wheel_group.fric_motor[3].give_current);
        CAN2_200_cmd_motor(revolver.yaw_motor.give_current, 0, 0, 0);
        //绝对延时1ms
        vTaskDelayUntil(&currentTime, 1);

        for (int i = 0; i < 4; i++)
        {
            SPEED[i] = revolver.fric_wheel_group.fric_motor[i].motor_measure->speed_rpm;
            TEMP[i] = revolver.fric_wheel_group.fric_motor[i].motor_measure->temperate;
        }
    }
}

/**
 * @brief   发射机构任务初始化
 */
revolver_task_t::revolver_task_t()
{
    // yaw轴初始化
    fp32 yaw_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
    fp32 yaw_position_pid[3] = {YAW_POSITION_PID_KP, YAW_POSITION_PID_KI, YAW_POSITION_PID_KD};
    yaw_motor.speed_pid.init(PID_POSITION, yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);
    yaw_motor.position_pid.init(PID_POSITION, yaw_position_pid, YAW_POSITION_PID_MAX_OUT, YAW_POSITION_PID_MAX_IOUT);
    yaw_motor.has_calibrated = 0;
    yaw_motor.motor_measure = get_motor_measure_class(YAW_MOTOR);

    //摩擦轮初始化
    fp32 Fric_speed_pid[3] = {FRIC_SPEED_PID_KP, FRIC_SPEED_PID_KI, FRIC_SPEED_PID_KD};
    for (uint8_t i = 0; i < 4; i++)
    {
        fric_wheel_group.fric_motor[i].speed_pid.init(PID_POSITION, Fric_speed_pid, FRIC_SPEED_PID_MAX_OUT, FRIC_SPEED_PID_MAX_IOUT);
    }
    fric_wheel_group.fric_motor[0].motor_measure = get_motor_measure_class(FL);
    fric_wheel_group.fric_motor[1].motor_measure = get_motor_measure_class(FR);
    fric_wheel_group.fric_motor[2].motor_measure = get_motor_measure_class(BL);
    fric_wheel_group.fric_motor[3].motor_measure = get_motor_measure_class(BR);
}

/**
 * @brief          发射机构数据更新
 */
void revolver_task_t::data_update()
{
    yaw_motor.accumulate_ecd = yaw_motor.motor_measure->num * 8192 + yaw_motor.motor_measure->ecd;

    for (uint8_t i = 0; i < 4; i++)
    {
        yaw_motor.outpost_offset_num[i] = screen_point()->outpost_yaw_offset_num[i];
        yaw_motor.base_offset_num[i] = screen_point()->base_yaw_offset_num[i];
        fric_wheel_group.outpost_speed[i] = screen_point()->outpost_speed[i];
        fric_wheel_group.base_speed[i] = screen_point()->base_speed[i];
    }
}

/**
 * @brief	发射机构控制
 */
void revolver_task_t::control()
{
    //右拨杆下，无力模式
    if (syspoint()->sys_mode == ZERO_FORCE)
    {
        ZERO_FORCE_control();
    }
    //右拨杆中，校准模式
    else if (syspoint()->sys_mode == CALIBRATE)
    {
        CALIBRATE_control();
    }
    //右拨杆上，发射模式
    else if (syspoint()->sys_mode == SHOOT)
    {
        SHOOT_control();
    }
}

/**
 * @brief	无力模式
 */
void revolver_task_t::ZERO_FORCE_control()
{
    //摩擦轮转动惯量大，需要主动停转
    fric_wheel_group.slow_stop();
    fric_wheel_group.current_calculate();
    yaw_motor.give_current = 0;
}

/**
 * @brief   摩擦轮缓停
 */
void fric_wheel_group_t::slow_stop()
{
    for (int i = 0; i < 4; i++)
    {
        fric_motor[i].speed_set = RAMP_float(0, fric_motor[i].speed_set, FRIC_RAMP_BUFF);
    }
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
 * @brief	校准模式
 */
void revolver_task_t::CALIBRATE_control()
{
    //摩擦轮缓停
    fric_wheel_group.slow_stop();
    fric_wheel_group.current_calculate();

    //左拨杆下，调整yaw轴
    if (syspoint()->sub_mode == CALIBRATE_ADJUST_POSITION)
    {
        yaw_motor.adjust_position();
    }
    //左拨杆中，校准yaw轴
    else if (syspoint()->sub_mode == CALIBRATE_LOADER_AND_YAW)
    {
        yaw_motor.calibrate();
    }
    //左拨杆上，检查校准结果
    else if (syspoint()->sub_mode == CALIBRATE_CHECK)
    {
        yaw_motor.check_calibrate_result();
    }
}

/**
 * @brief   调整yaw轴位置
 */
void yaw_motor_t::adjust_position()
{
    //遥控器左摇杆左右调整yaw轴位置
    ecd_set += revolver.revolver_rc_ctrl->rc.ch[2] * RC_TO_YAW_ECD_SET;
    current_calculate();
}

/**
 * @brief	校准yaw轴电机零点
 */
void yaw_motor_t::calibrate()
{
    //遥控器↖↗校准yaw轴
    if (RC_double_held_single_return(LEFT_ROCKER_LEFT_TOP, RIGHT_ROCKER_RIGHT_TOP, 400))
    {
        calibrated_point = accumulate_ecd;
        has_calibrated = 1;
        buzzer_warn(0, 0, 3, 10000);
    }
    //电流设为0，可以用手转动电机调整位置
    give_current = 0;
}

/**
 * @brief	检查yaw轴校准结果
 */
void yaw_motor_t::check_calibrate_result()
{
    //未校准时不进行检查
    if (has_calibrated == 0)
    {
        current_calculate();
        return;
    }

    //遥控器↗↖回校准点，检查yaw轴校准结果
    if (RC_double_held_single_return(LEFT_ROCKER_RIGHT_TOP, RIGHT_ROCKER_LEFT_TOP, 400))
    {
        has_back_to_zero_started = 1;
    }
    if (has_back_to_zero_started)
    {
        ecd_set = RAMP_float(calibrated_point, ecd_set, YAW_RAMP_BUFF);
    }
    current_calculate();
}

/**
 * @brief	发射模式
 */
void revolver_task_t::SHOOT_control()
{
    //左拨杆下，发射初始化
    if (syspoint()->sub_mode == SHOOT_INIT)
    {
        yaw_motor.shoot_init();
        fric_wheel_group.shoot_init();
    }
    //左拨杆中，手动发射
    else if (syspoint()->sub_mode == SHOOT_MANUAL)
    {
        yaw_motor.shooting();
        fric_wheel_group.shooting();
    }
}

/**
 * @brief	    摩擦轮发射初始化
 * @attention   为方便调试，本函数没有做初始化保护，yaw轴和装填电机未校准时摩擦轮也能转动
 */
void fric_wheel_group_t::shoot_init()
{
    //遥控器↖↗，开启摩擦轮
    if (RC_double_held_single_return(LEFT_ROCKER_LEFT_TOP, RIGHT_ROCKER_RIGHT_TOP, 400))
    {
        is_fric_wheel_on = 1;
    }

    if (is_fric_wheel_on == 0)
    {
        slow_stop();
    }
    else if (is_fric_wheel_on == 1)
    {
        fric_motor[0].speed_set = RAMP_float(-(outpost_speed[syspoint()->active_dart_index] + (SPEED_DIFFERENCE / 2)), fric_motor[0].speed_set, FRIC_RAMP_BUFF);
        fric_motor[1].speed_set = RAMP_float((outpost_speed[syspoint()->active_dart_index] + (SPEED_DIFFERENCE / 2)), fric_motor[1].speed_set, FRIC_RAMP_BUFF);
        fric_motor[2].speed_set = RAMP_float(-(outpost_speed[syspoint()->active_dart_index] - (SPEED_DIFFERENCE / 2)), fric_motor[2].speed_set, FRIC_RAMP_BUFF);
        fric_motor[3].speed_set = RAMP_float((outpost_speed[syspoint()->active_dart_index] - (SPEED_DIFFERENCE / 2)), fric_motor[3].speed_set, FRIC_RAMP_BUFF);
    }

    current_calculate();
}

/**
 * @brief	yaw电机发射初始化
 */
void yaw_motor_t::shoot_init()
{
    // yaw轴未校准时不能初始化
    if (has_calibrated == 0)
    {
        current_calculate();
        return;
    }

    //标志位被置一时开始初始化
    if (has_shoot_init_started)
    {
        //打击目标默认初始化为前哨站
        ecd_set = RAMP_float(calibrated_point + outpost_offset_num[syspoint()->active_dart_index] * 8192, ecd_set, YAW_RAMP_BUFF);
        has_shoot_init_finished = 0;
    }

    // yaw轴电机初始化结束
    if (fabs(ecd_set - accumulate_ecd) < YAW_ECD_TOLERANCE && has_shoot_init_started == 1)
    {
        has_shoot_init_started = 0;
        has_shoot_init_finished = 1;
    }

    current_calculate();
}

/**
 * @brief	摩擦轮发射时的控制
 */
void fric_wheel_group_t::shooting()
{
    //发射完最后一个飞镖后，摩擦轮停止转动
    if (syspoint()->active_dart_index == 4 && loader_motor_point()->has_shoot_down_finished)
    {
        is_fric_wheel_on = 0;
    }

    //摩擦轮转速设定
    if (is_fric_wheel_on == 0)
    {
        slow_stop();
    }
    else if (is_fric_wheel_on == 1)
    {
        if (syspoint()->strike_target == OUTPOST)
        {
            fric_motor[0].speed_set = RAMP_float(-((loader_motor_point()->has_shoot_down_finished ? outpost_speed[syspoint()->active_dart_index] : outpost_speed[syspoint()->active_dart_index - 1]) + (SPEED_DIFFERENCE / 2)),
                                                 fric_motor[0].speed_set, FRIC_RAMP_BUFF);
            fric_motor[1].speed_set = RAMP_float(((loader_motor_point()->has_shoot_down_finished ? outpost_speed[syspoint()->active_dart_index] : outpost_speed[syspoint()->active_dart_index - 1]) + (SPEED_DIFFERENCE / 2)),
                                                 fric_motor[1].speed_set, FRIC_RAMP_BUFF);
            fric_motor[2].speed_set = RAMP_float(-((loader_motor_point()->has_shoot_down_finished ? outpost_speed[syspoint()->active_dart_index] : outpost_speed[syspoint()->active_dart_index - 1]) - (SPEED_DIFFERENCE / 2)),
                                                 fric_motor[2].speed_set, FRIC_RAMP_BUFF);
            fric_motor[3].speed_set = RAMP_float(((loader_motor_point()->has_shoot_down_finished ? outpost_speed[syspoint()->active_dart_index] : outpost_speed[syspoint()->active_dart_index - 1]) - (SPEED_DIFFERENCE / 2)),
                                                 fric_motor[3].speed_set, FRIC_RAMP_BUFF);
        }
        else if (syspoint()->strike_target == BASE)
        {
            fric_motor[0].speed_set = RAMP_float(-((loader_motor_point()->has_shoot_down_finished ? base_speed[syspoint()->active_dart_index] : base_speed[syspoint()->active_dart_index - 1]) + (SPEED_DIFFERENCE / 2)),
                                                 fric_motor[0].speed_set, FRIC_RAMP_BUFF);
            fric_motor[1].speed_set = RAMP_float(((loader_motor_point()->has_shoot_down_finished ? base_speed[syspoint()->active_dart_index] : base_speed[syspoint()->active_dart_index - 1]) + (SPEED_DIFFERENCE / 2)),
                                                 fric_motor[1].speed_set, FRIC_RAMP_BUFF);
            fric_motor[2].speed_set = RAMP_float(-((loader_motor_point()->has_shoot_down_finished ? base_speed[syspoint()->active_dart_index] : base_speed[syspoint()->active_dart_index - 1]) - (SPEED_DIFFERENCE / 2)),
                                                 fric_motor[2].speed_set, FRIC_RAMP_BUFF);
            fric_motor[3].speed_set = RAMP_float(((loader_motor_point()->has_shoot_down_finished ? base_speed[syspoint()->active_dart_index] : base_speed[syspoint()->active_dart_index - 1]) - (SPEED_DIFFERENCE / 2)),
                                                 fric_motor[3].speed_set, FRIC_RAMP_BUFF);
        }
    }

    current_calculate();
}

/**
 * @brief	yaw发射时的控制
 */
void yaw_motor_t::shooting()
{
    // yaw轴未初始化时不能发射
    if (has_shoot_init_finished == 0)
    {
        current_calculate();
        return;
    }

    //装填电机已经下移出锁定区且没打完飞镖时，改变设定值
    if (loader_motor_point()->has_shoot_up_finished && !rotary_motor_point()->should_lock && syspoint()->active_dart_index < 4)
    {
        if (syspoint()->strike_target == OUTPOST)
        {
            ecd_set = RAMP_float(calibrated_point + outpost_offset_num[syspoint()->active_dart_index] * 8192, ecd_set, YAW_RAMP_BUFF);
        }
        else if (syspoint()->strike_target == BASE)
        {
            ecd_set = RAMP_float(calibrated_point + base_offset_num[syspoint()->active_dart_index] * 8192, ecd_set, YAW_RAMP_BUFF);
        }
    }

    //目标值和实际值之差小于一定值，可认为转到位
    if (fabs(ecd_set - accumulate_ecd) < YAW_ECD_TOLERANCE)
    {
        has_move_to_next_finished = 1;
    }

    current_calculate();
}

/**
 * @brief	摩擦轮电流计算
 */
void fric_wheel_group_t::current_calculate()
{
    for (int i = 0; i < 4; i++)
    {
        fric_motor[i].give_current = fric_motor[i].speed_pid.calc(fric_motor[i].motor_measure->speed_rpm, fric_motor[i].speed_set);
    }
}

/**
 * @brief	yaw轴电流计算
 */
void yaw_motor_t::current_calculate()
{
    speed_set = position_pid.calc(accumulate_ecd, ecd_set);
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
