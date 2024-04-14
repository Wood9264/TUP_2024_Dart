/**
 * @file system_task.cpp
 * @author Yang Maolin (1831051389@qq.com)
 * @brief 系统任务相关代码。包含系统模式和子模式的设定、转换，以及发射任务和装填任务共用数据的控制。
 * @attention 本文件中包含了少量与飞镖系统无关的代码，因与其他模块耦合度较高难以清理，暂时保留。无关代码已做标注。
 */
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

/*****************无用代码*************************/
int steering_mode = 1; //弹舱盖模式 0为关闭，1为打开
int vision_mode = 1;   //打弹模式1：1v1 自动开火  3：3v3 手动开火
int spin_mode = 0;     //是否开启小陀螺

fp32 x_coordinate, y_coordinate;         //自瞄位置ui
fp32 pre_x_coordinate, pre_y_coordinate; //自瞄预测位置ui
fp32 follow_radius, pre_radius;          //自瞄装甲板半径

/***************************************************/

/**
 * @brief 系统任务
 * @param pvParameters
 */
void system_task(void const *pvParameters)
{
    //初始化延时
    vTaskDelay(201);
    uint32_t currentTime;
    while (1)
    {
        //获取当前系统时间
        currentTime = xTaskGetTickCount();
        //模式设置
        sys.mode_set();
        //模式转换
        sys.mode_transit();
        //系统任务控制
        sys.control();
        //绝对延时1ms
        vTaskDelayUntil(&currentTime, 1);
    }
}

/**
 * @brief   返回系统类指针
 */
system_t *syspoint(void)
{
    return &sys;
}

/**
 * @brief          构造函数初始化
 */
system_t::system_t()
{
    sys_mode = ZERO_FORCE;
    last_sys_mode = ZERO_FORCE;
}

/**
 * @brief   系统模式及子模式设定。右拨杆设置系统模式，左拨杆设置子模式
 */
void system_t::mode_set()
{
    last_sys_mode = sys_mode;
    last_sub_mode = sub_mode;

    if (IF_RC_SW0_DOWN || toe_is_error(DBUS_TOE))
    {
        sys_mode = ZERO_FORCE;
    }
    else if (IF_RC_SW0_MID)
    {
        sys_mode = CALIBRATE;

        if (IF_RC_SW1_DOWN)
        {
            sub_mode = CALIBRATE_ADJUST_POSITION;
        }
        else if (IF_RC_SW1_MID)
        {
            sub_mode = CALIBRATE_LOADER_AND_YAW;
        }
        else if (IF_RC_SW1_UP)
        {
            sub_mode = CALIBRATE_CHECK;
        }
    }
    else if (IF_RC_SW0_UP)
    {
        sys_mode = SHOOT;

        if (IF_RC_SW1_DOWN)
        {
            sub_mode = SHOOT_INIT;
        }
        else if (IF_RC_SW1_MID || IF_RC_SW1_UP)
        {
            sub_mode = SHOOT_MANUAL;
        }
    }
}

/**
 * @brief   模式转换时的数据初始化
 */
void system_t::mode_transit()
{
    //主模式转换时的数据初始化
    if (sys_mode == CALIBRATE && last_sys_mode != CALIBRATE)
    {
        revolver_point()->yaw_motor.ecd_set = revolver_point()->yaw_motor.accumulate_ecd;

        load_point()->loader_motor.ecd_set = load_point()->loader_motor.accumulate_ecd;
        load_point()->loader_motor.has_auto_calibrate_begun = 0;

        load_point()->rotary_motor.relative_angle_set = load_point()->rotary_motor.relative_angle;
    }
    else if (sys_mode == SHOOT && last_sys_mode != SHOOT)
    {
        revolver_point()->fric_wheel_group.is_fric_wheel_on = 0;

        revolver_point()->yaw_motor.ecd_set = revolver_point()->yaw_motor.accumulate_ecd;
        revolver_point()->yaw_motor.has_shoot_init_started = 0;
        revolver_point()->yaw_motor.has_shoot_init_finished = 0;

        load_point()->loader_motor.ecd_set = load_point()->loader_motor.accumulate_ecd;
        load_point()->loader_motor.has_shoot_init_started = 0;
        load_point()->loader_motor.has_shoot_init_finished = 0;

        load_point()->rotary_motor.final_relative_angle_set = load_point()->rotary_motor.relative_angle;
        load_point()->rotary_motor.relative_angle_set = load_point()->rotary_motor.relative_angle;
        load_point()->rotary_motor.has_shoot_init_started = 0;
        load_point()->rotary_motor.has_shoot_init_finished = 0;

        has_index_added = 0;
        strike_target = OUTPOST;
        active_dart_index = 0;
    }

    //子模式转换时的数据初始化
    if (sub_mode == CALIBRATE_ADJUST_POSITION && last_sub_mode != CALIBRATE_ADJUST_POSITION)
    {
        revolver_point()->yaw_motor.ecd_set = revolver_point()->yaw_motor.accumulate_ecd;
        load_point()->loader_motor.ecd_set = load_point()->loader_motor.accumulate_ecd;
        load_point()->loader_motor.has_auto_calibrate_begun = 0;
    }
    else if (sub_mode == CALIBRATE_CHECK && last_sub_mode != CALIBRATE_CHECK)
    {
        revolver_point()->yaw_motor.ecd_set = revolver_point()->yaw_motor.accumulate_ecd;
        revolver_point()->yaw_motor.has_back_to_zero_started = 0;
        load_point()->loader_motor.ecd_set = load_point()->loader_motor.accumulate_ecd;
    }
    else if (sub_mode == SHOOT_INIT && last_sub_mode != SHOOT_INIT)
    {
        revolver_point()->fric_wheel_group.is_fric_wheel_on = 0;

        revolver_point()->yaw_motor.ecd_set = revolver_point()->yaw_motor.accumulate_ecd;
        revolver_point()->yaw_motor.has_shoot_init_started = 0;
        revolver_point()->yaw_motor.has_shoot_init_finished = 0;

        load_point()->loader_motor.ecd_set = load_point()->loader_motor.accumulate_ecd;
        load_point()->loader_motor.has_shoot_init_started = 0;
        load_point()->loader_motor.has_shoot_init_finished = 0;
        //下移标志位置一，防止三元运算符中读取转速补偿时数组越界
        load_point()->loader_motor.has_shoot_down_finished = 1;

        load_point()->rotary_motor.final_relative_angle_set = load_point()->rotary_motor.relative_angle;
        load_point()->rotary_motor.relative_angle_set = load_point()->rotary_motor.relative_angle;
        load_point()->rotary_motor.has_shoot_init_started = 0;
        load_point()->rotary_motor.has_shoot_init_finished = 0;

        has_index_added = 0;
        strike_target = OUTPOST;
        active_dart_index = 0;
    }
}

/**
 * @brief   系统任务控制
 */
void system_t::control()
{
    if (sys_mode == SHOOT)
    {
        SHOOT_control();
    }
}

/**
 * @brief   发射模式
 */
void system_t::SHOOT_control()
{
    if (IF_RC_SW1_DOWN)
    {
        //发射初始化
        shoot_init();
    }
    else if (IF_RC_SW1_MID || IF_RC_SW1_UP)
    {
        //遥控器控制打下一发飞镖
        dart_index_add();
        //切换打击目标
        switch_strike_target();
    }
}

/**
 * @brief   发射初始化
 */
void system_t::shoot_init()
{
    if (RC_held_continuous_return(RIGHT_ROCKER_LEFT_TOP, 100))
    {
        //↗↖初始化为1号弹体位置
        if (RC_held_continuous_return(LEFT_ROCKER_RIGHT_TOP, 100))
        {
            rotary_motor_point()->has_shoot_init_started = 1;
            loader_motor_point()->has_shoot_init_started = 1;
            revolver_point()->yaw_motor.has_shoot_init_started = 1;
            syspoint()->active_dart_index = 0;
        }
        //↖↖初始化为2号弹体位置
        else if (RC_held_continuous_return(LEFT_ROCKER_LEFT_TOP, 100))
        {
            rotary_motor_point()->has_shoot_init_started = 1;
            loader_motor_point()->has_shoot_init_started = 1;
            revolver_point()->yaw_motor.has_shoot_init_started = 1;
            syspoint()->active_dart_index = 1;
        }
        //↙↖初始化为3号弹体位置
        else if (RC_held_continuous_return(LEFT_ROCKER_LEFT_BOTTOM, 100))
        {
            rotary_motor_point()->has_shoot_init_started = 1;
            loader_motor_point()->has_shoot_init_started = 1;
            revolver_point()->yaw_motor.has_shoot_init_started = 1;
            syspoint()->active_dart_index = 2;
        }
        //↘↖初始化为4号弹体位置
        else if (RC_held_continuous_return(LEFT_ROCKER_RIGHT_BOTTOM, 100))
        {
            rotary_motor_point()->has_shoot_init_started = 1;
            loader_motor_point()->has_shoot_init_started = 1;
            revolver_point()->yaw_motor.has_shoot_init_started = 1;
            syspoint()->active_dart_index = 3;
        }
    }
}

/**
 * @brief   打下一发飞镖
 */
void system_t::dart_index_add()
{
    static uint16_t settled_time = 0;

    //装填电机下移完毕且转盘电机转到位且yaw轴转到位一定时间后，可打下一发飞镖
    if (loader_motor_point()->has_shoot_down_finished && rotary_motor_point()->has_move_to_next_finished && revolver_point()->yaw_motor.has_move_to_next_finished)
    {
        settled_time++;
    }
    else
    {
        settled_time = 0;
    }

    if (settled_time > 500)
    {
        //左摇杆↖开始打下一发飞镖
        if (RC_held_continuous_return(LEFT_ROCKER_LEFT_TOP, 0) && syspoint()->active_dart_index < 4)
        {
            active_dart_index++;
            has_index_added = 1;
            loader_motor_point()->has_shoot_up_finished = 0;
            loader_motor_point()->has_shoot_down_finished = 0;
            rotary_motor_point()->has_move_to_next_finished = 0;
        }
        else
        {
            has_index_added = 0;
        }
    }
}

/**
 * @brief   切换打击目标
 */
void system_t::switch_strike_target()
{
    //↙↙切换打击目标为前哨站
    if (RC_double_held_single_return(LEFT_ROCKER_LEFT_BOTTOM, RIGHT_ROCKER_LEFT_BOTTOM, 400))
    {
        strike_target = OUTPOST;
    }
    //↘↘切换打击目标为基地
    else if (RC_double_held_single_return(LEFT_ROCKER_RIGHT_BOTTOM, RIGHT_ROCKER_RIGHT_BOTTOM, 400))
    {
        strike_target = BASE;
    }
}
