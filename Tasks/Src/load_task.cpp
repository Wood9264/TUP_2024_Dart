#include "load_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "remote_control.h"
#include "BSP_can.h"
#include "system_task.h"
#include "monitor_task.h"

load_task_t load;

loader_motor_t *loader_motor_point()
{
    return &load.loader_motor;
}

rotary_motor_t *rotary_motor_point()
{
    return &load.rotary_motor;
}

/**
 * @brief   装填任务
 * @param[in]   none
 */
void load_task(void const *pvParameters)
{
    //延时
    vTaskDelay(LOAD_TASK_INIT_TIME);
    uint32_t currentTime;
    while (1)
    {
        //获取当前系统时间
        currentTime = xTaskGetTickCount();
        //装填任务数据更新
        load.data_update();
        //分任务控制
        load.control();

        vTaskDelayUntil(&currentTime, 1);
    }
}

/**
 * @brief   装填任务初始化
 */
load_task_t::load_task_t()
{
    //初始化装填电机
    fp32 loader_speed_pid[3] = {LOADER_SPEED_PID_KP, LOADER_SPEED_PID_KI, LOADER_SPEED_PID_KD};
    fp32 loader_position_pid[3] = {LOADER_POSITION_PID_KP, LOADER_POSITION_PID_KI, LOADER_POSITION_PID_KD};
    loader_motor.speed_pid.init(PID_POSITION, loader_speed_pid, LOADER_SPEED_PID_MAX_OUT, LOADER_SPEED_PID_MAX_IOUT);
    loader_motor.position_pid.init(PID_POSITION, loader_position_pid, LOADER_POSITION_PID_MAX_OUT, LOADER_POSITION_PID_MAX_IOUT);
    loader_motor.give_current = 0;
    loader_motor.motor_measure = get_motor_measure_class(LOADER_MOTOR);
    loader_motor.has_calibrated = 0;

    //初始化转盘电机
    rotary_motor.LADRC_FDW.init(ROTARY_LADRC_WC, ROTARY_LADRC_B0, ROTARY_LADRC_W0, ROTARY_LADRC_MAXOUT, ROTARY_LADRC_W, ROTARY_LADRC_GAIN);
    rotary_motor.relative_angle = 0;
    rotary_motor.last_relative_angle = 0;
    rotary_motor.give_current = 0;
    rotary_motor.motor_measure = get_motor_measure_class(ROTARY_MOTOR);

    //初始化遥控器
    load_rc_ctrl = get_remote_control_point();
}

/**
 * @brief   装填任务数据更新
 */
void load_task_t::data_update()
{
    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;
    //装填电机转速二阶低通滤波
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (loader_motor.motor_measure->speed_rpm * LOADER_MOTOR_RMP_TO_FILTER_SPEED) * fliter_num[2];
    loader_motor.motor_speed = speed_fliter_3;
    //累计编码值
    loader_motor.accumulate_ecd = loader_motor.motor_measure->num * 8192 + loader_motor.motor_measure->ecd;

    //更新转盘电机数据
    rotary_motor.last_relative_angle = rotary_motor.relative_angle;
    rotary_motor.motor_ecd_to_relative_angle();
    rotary_motor.acceleration_update();
}

/**
 * @brief   计算ecd与zero_point_ecd之间的相对角度
 * @retval  none
 */
void rotary_motor_t::motor_ecd_to_relative_angle()
{
    //计算相对角度
    fp32 delta_angle = (motor_measure->ecd - ROTARY_ZERO_POINT_ECD) * MOTOR_ECD_TO_RAD;
    //角度限幅
    if (delta_angle > PI)
    {
        delta_angle -= 2 * PI;
    }
    else if (delta_angle < -PI)
    {
        delta_angle += 2 * PI;
    }
    relative_angle = delta_angle;
}

/**
 * @brief   计算转盘电机加速度
 */
void rotary_motor_t::acceleration_update()
{
    acceleration = (relative_angle - last_relative_angle) * 1000;
}

/**
 * @brief   装填任务标志位更新
 */
void load_task_t::flag_update()
{
    //更新装填电机标志位
    loader_motor.flag_update();
    //更新转盘电机标志位
    rotary_motor.flag_update();
}

/**
 * @brief   装填电机标志位更新
 */
void loader_motor_t::flag_update()
{
    //更新限位开关状态
    bottom_tick = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11);

    //转盘电机处在四个角度区间时，装填电机受限
    if ((rotary_motor_point()->relative_angle > ROTARY_HALF_MOVABLE_ANGLE && rotary_motor_point()->relative_angle < PI / 2 - ROTARY_HALF_MOVABLE_ANGLE) ||
        (rotary_motor_point()->relative_angle > PI / 2 + ROTARY_HALF_MOVABLE_ANGLE && rotary_motor_point()->relative_angle < PI - ROTARY_HALF_MOVABLE_ANGLE) ||
        (rotary_motor_point()->relative_angle > -PI + ROTARY_HALF_MOVABLE_ANGLE && rotary_motor_point()->relative_angle < -PI / 2 - ROTARY_HALF_MOVABLE_ANGLE) ||
        (rotary_motor_point()->relative_angle > -PI / 2 + ROTARY_HALF_MOVABLE_ANGLE && rotary_motor_point()->relative_angle < -ROTARY_HALF_MOVABLE_ANGLE))
    {
        is_restricted_state = 1;
    }
    else
    {
        is_restricted_state = 0;
    }
}

/**
 * @brief   转盘电机标志位更新
 */
void rotary_motor_t::flag_update()
{
    //更新转盘电机锁定状态
    if (loader_motor_point()->accumulate_ecd > loader_motor_point()->restrict_point_ecd)
    {
        should_lock = 1;
    }
    else
    {
        should_lock = 0;
    }
}

/**
 * @brief   分任务控制
 */
void load_task_t::control()
{
    if (syspoint()->sys_mode == ZERO_FORCE)
    {
        ZERO_FORCE_control();
    }
    else if (syspoint()->sys_mode == CALIBRATE)
    {
        CALIBRATE_control();
    }
    else if (syspoint()->sys_mode == SHOOT)
    {
        // SHOOT_control();
    }
}

/**
 * @brief   无力模式
 */
void load_task_t::ZERO_FORCE_control()
{
    loader_motor.give_current = 0;
    rotary_motor.give_current = 0;
}

/**
 * @brief   校准模式
 */
void load_task_t::CALIBRATE_control()
{
    if (IF_RC_SW1_DOWN)
    {
        loader_motor.adjust_position();
    }
    if (IF_RC_SW1_MID && IF_RC_SW1_UP)
    {
        loader_motor.calibrate();
    }
}

/**
 * @brief   调整装填电机位置
 */
void loader_motor_t::adjust_position()
{
    //未校准时不能调整位置
    if (!has_calibrated)
    {
        speed_set = position_pid.calc(accumulate_ecd, ecd_set);
        give_current = speed_pid.calc(motor_speed, speed_set);
        return;
    }

    ecd_set += load.load_rc_ctrl->rc.ch[1] * RC_TO_LOADER_MOTOR_ECD_SET;
    //限位
    if (ecd_set > max_point_ecd)
    {
        ecd_set = max_point_ecd;
    }
    else if (ecd_set > restrict_point_ecd && is_restricted_state)
    {
        ecd_set = restrict_point_ecd;
    }
    else if (ecd_set < zero_point_ecd)
    {
        ecd_set = zero_point_ecd;
    }

    speed_set = position_pid.calc(accumulate_ecd, ecd_set);
    give_current = speed_pid.calc(motor_speed, speed_set);
}

/**
 * @brief   校准装填电机
 */
void loader_motor_t::calibrate()
{
    //遥控器↙↘开始自动校准
    if (RC_double_held_single_return(LEFT_ROCKER_LEFT_BOTTOM, RIGHT_ROCKER_RIGHT_BOTTOM, 400))
    {
        calibrate_begin = 1;
    }

    //遥控器↘↙手动校准，适用于触点开关失效的情况
    if (RC_double_held_single_return(LEFT_ROCKER_RIGHT_BOTTOM, RIGHT_ROCKER_LEFT_BOTTOM, 400))
    {
        manual_calibrate();
    }

    if (calibrate_begin == 1)
    {
        auto_calibrate();
    }
    else
    {
        speed_set = position_pid.calc(accumulate_ecd, ecd_set);
        give_current = speed_pid.calc(motor_speed, speed_set);
    }
}

/**
 * @brief	自动校准
 */
void loader_motor_t::auto_calibrate()
{
    static bool_t found_zero_point = 0; //是否找到零点

    //触点开关被压下，找到零点
    if (bottom_tick)
    {
        found_zero_point = 1;
    }

    //滑块下移
    if (found_zero_point == 0)
    {
        ecd_set -= CALIBRATE_DOWN_PER_LENGTH;
        speed_set = position_pid.calc(accumulate_ecd, ecd_set);
        give_current = speed_pid.calc(motor_speed, speed_set);
    }
    //压下触点开关后上移
    else if (found_zero_point == 1 && bottom_tick == 1)
    {
        ecd_set += CALIBRATE_DOWN_PER_LENGTH;
        speed_set = position_pid.calc(accumulate_ecd, ecd_set);
        give_current = speed_pid.calc(motor_speed, speed_set);
    }
    //滑块离开触点开关，校准完毕
    else if (found_zero_point == 1 && bottom_tick == 0)
    {
        speed_set = 0;
        has_calibrated = 1;
        found_zero_point = 0;
        calibrate_begin = 0;

        //设置零点、受限点和最大点
        zero_point_ecd = accumulate_ecd;
        restrict_point_ecd = accumulate_ecd + LOADER_RESTRICT_FORWARD_ECD;
        max_point_ecd = accumulate_ecd + LOADER_FORWARD_ECD;
    }
}

/**
 * @brief	手动校准，适用于触点开关失效的情况
 */
void loader_motor_t::manual_calibrate()
{
    speed_set = 0;
    has_calibrated = 1;
    calibrate_begin = 0;

    //设置零点和最大点
    zero_point_ecd = accumulate_ecd;
    max_point_ecd = accumulate_ecd + LOADER_FORWARD_ECD;
    buzzer_warn(0, 0, 3, 10000);
}

/**
 * @brief   调整转盘电机位置
 */
void rotary_motor_t::adjust_position()
{
    static fp32 add_angle = 0.0f;
    
    //装填电机未校准时或转盘电机锁定时不能调整位置
    if (!loader_motor_point()->has_calibrated || should_lock)
    {
        LADRC_FDW.FDW_calc(relative_angle, relative_angle_set, acceleration);
        return;
    }

    add_angle = load.load_rc_ctrl->rc.ch[0] * RC_TO_ROTARY_MOTOR_ANGLE_SET;
    relative_angle_set = rad_format(relative_angle_set + add_angle);
    LADRC_FDW.FDW_calc(relative_angle, relative_angle_set, acceleration);
}
