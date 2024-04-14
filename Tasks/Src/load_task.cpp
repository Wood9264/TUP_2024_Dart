/**
 * @file load_task.cpp
 * @author Yang Maolin (1831051389@qq.com)
 * @brief 装填机构相关代码。装填机构包括装填电机和转盘电机，装填电机负责推弹，转盘电机负责转动弹仓
 */
#include "load_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "remote_control.h"
#include "BSP_can.h"
#include "system_task.h"
#include "monitor_task.h"
#include "revolver_task.h"

load_task_t load;

/**
 * @brief   返回装填任务指针
 */
load_task_t *load_point()
{
    return &load;
}

/**
 * @brief   返回装填电机指针
 */
loader_motor_t *loader_motor_point()
{
    return &load.loader_motor;
}

/**
 * @brief   返回转盘电机指针
 */
rotary_motor_t *rotary_motor_point()
{
    return &load.rotary_motor;
}

/**
 * @brief   装填任务
 * @param[in]   none
 */

/**
 * @brief   装填机构任务
 * @param   pvParameters
 */
void load_task(void const *pvParameters)
{
    //初始化延时
    vTaskDelay(LOAD_TASK_INIT_TIME);
    uint32_t currentTime;
    while (1)
    {
        //获取当前系统时间
        currentTime = xTaskGetTickCount();
        //装填任务数据更新
        load.data_update();
        //装填任务标志位更新
        load.flag_update();
        //装填机构控制
        load.control();
        //发送电流
        CAN2_1FF_cmd_motor(load.rotary_motor.give_current, 0, 0, 0);
        //绝对延时1ms
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
    fp32 rotary_speed_pid[3] = {ROTARY_SPEED_PID_KP, ROTARY_SPEED_PID_KI, ROTARY_SPEED_PID_KD};
    fp32 rotary_position_pid[3] = {ROTARY_POSITION_PID_KP, ROTARY_POSITION_PID_KI, ROTARY_POSITION_PID_KD};
    rotary_motor.speed_pid.init(PID_POSITION, rotary_speed_pid, ROTARY_SPEED_PID_MAX_OUT, ROTARY_SPEED_PID_MAX_IOUT);
    rotary_motor.speed_pid.differ_init(ROTARY_SPEED_PID_BANDWIDTH, ROTARY_POSITION_PID_TIME_CONS);
    rotary_motor.position_pid.init(PID_POSITION, rotary_position_pid, ROTARY_POSITION_PID_MAX_OUT, ROTARY_POSITION_PID_MAX_IOUT);
    rotary_motor.position_pid.differ_init(ROTARY_POSITION_PID_BANDWIDTH, ROTARY_POSITION_PID_TIME_CONS);
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
    //装填电机转速滤波
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (loader_motor.motor_measure->speed_rpm * LOADER_MOTOR_RMP_TO_FILTER_SPEED) * fliter_num[2];
    loader_motor.filtered_speed = speed_fliter_3;
    //累计编码值
    loader_motor.accumulate_ecd = loader_motor.motor_measure->num * 8192 + loader_motor.motor_measure->ecd;

    //更新转盘电机数据
    rotary_motor.last_relative_angle = rotary_motor.relative_angle;
    rotary_motor.motor_ecd_to_relative_angle();
    // rotary_motor.acceleration_update();
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

// /**
//  * @brief   计算转盘电机加速度
//  */
// void rotary_motor_t::acceleration_update()
// {
//     acceleration = (relative_angle - last_relative_angle) * 1000;
// }

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

    static int16_t recali_time = 0;
    //除了校准的时候，其它时候装填电机不会碰到底部触点开关；如果碰到说明校准出错了，要重新校准
    if (has_calibrated == 1 && bottom_tick == 1)
    {
        recali_time++;
        if (recali_time > 20)
        {
            has_calibrated = 0;
            buzzer_warn(0, 0, 3, 10000);
        }
    }
    else
    {
        recali_time = 0;
    }

    //转盘电机处在四个角度区间时，装填电机前进会撞到载弹架，装填电机受限
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
    //装填电机位于载弹架中时，转盘电机锁定不能转动
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
 * @brief   装填机构控制
 */
void load_task_t::control()
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
    //左拨杆下，调整装填电机位置和转盘位置
    if (syspoint()->sub_mode == CALIBRATE_ADJUST_POSITION)
    {
        loader_motor.adjust_position();
        rotary_motor.adjust_position();
    }
    //左拨杆中，校准装填电机
    else if (syspoint()->sub_mode == CALIBRATE_LOADER_AND_YAW)
    {
        loader_motor.calibrate();
        rotary_motor.calibrate();
    }
    //左拨杆上，检查校准结果
    else if (syspoint()->sub_mode == CALIBRATE_CHECK)
    {
        loader_motor.check_calibrate_result();
        rotary_motor.check_calibrate_result();
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
        current_calculate(NULL);
        return;
    }

    //右摇杆前后控制装填电机前进后退
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

    current_calculate(NULL);
}

/**
 * @brief   校准装填电机
 */
void loader_motor_t::calibrate()
{
    //遥控器↙↘开始自动校准
    if (RC_double_held_single_return(LEFT_ROCKER_LEFT_BOTTOM, RIGHT_ROCKER_RIGHT_BOTTOM, 400))
    {
        has_auto_calibrate_begun = 1;
        ecd_set = accumulate_ecd;
    }

    //遥控器↘↙手动校准，适用于触点开关失效的情况
    if (RC_double_held_single_return(LEFT_ROCKER_RIGHT_BOTTOM, RIGHT_ROCKER_LEFT_BOTTOM, 400))
    {
        manual_calibrate();
    }

    //自动校准计算电流
    if (has_auto_calibrate_begun == 1)
    {
        auto_calibrate();
    }
    current_calculate(NULL);
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

    //装填电机下移
    if (found_zero_point == 0)
    {
        ecd_set -= LOADER_CALIBRATE_DOWN_PER_LENGTH;
    }
    //压下触点开关后上移
    else if (found_zero_point == 1 && bottom_tick == 1)
    {
        ecd_set += LOADER_CALIBRATE_UP_PER_LENGTH;
    }
    //装填电机离开触点开关，校准完毕
    else if (found_zero_point == 1 && bottom_tick == 0)
    {
        has_calibrated = 1;
        found_zero_point = 0;
        has_auto_calibrate_begun = 0;

        //设置零点、受限点和最大点
        zero_point_ecd = accumulate_ecd + ZERO_POINT_OFFSET;
        restrict_point_ecd = zero_point_ecd + LOADER_RESTRICT_FORWARD_ECD;
        max_point_ecd = accumulate_ecd + LOADER_FORWARD_ECD;
    }
}

/**
 * @brief	手动校准，适用于触点开关失效的情况
 */
void loader_motor_t::manual_calibrate()
{
    has_calibrated = 1;

    //设置零点和最大点
    zero_point_ecd = accumulate_ecd + ZERO_POINT_OFFSET;
    restrict_point_ecd = accumulate_ecd + LOADER_RESTRICT_FORWARD_ECD;
    max_point_ecd = accumulate_ecd + LOADER_FORWARD_ECD;
    buzzer_warn(0, 0, 3, 10000);
}

/**
 * @brief   调整转盘电机位置
 */
void rotary_motor_t::adjust_position()
{
    static fp32 add_angle = 0.0f;

    // 装填电机未校准时或转盘电机锁定时不能调整位置
    if (!loader_motor_point()->has_calibrated || should_lock)
    {
        current_calculate();
        return;
    }

    //右摇杆左右控制转盘电机角度
    add_angle = load.load_rc_ctrl->rc.ch[0] * RC_TO_ROTARY_MOTOR_ANGLE_SET;
    relative_angle_set = rad_format(relative_angle_set + add_angle);
    current_calculate();
}

/**
 * @brief   校准时转盘电机不转动，但仍需计算电流
 */
void rotary_motor_t::calibrate()
{
    current_calculate();
}

/**
 * @brief   检查校准结果时装填电机不动，但仍需计算电流
 */
void loader_motor_t::check_calibrate_result()
{
    current_calculate(NULL);
}

/**
 * @brief   检查校准结果时转盘电机不转动，但仍需计算电流
 */
void rotary_motor_t::check_calibrate_result()
{
    current_calculate();
}

/**
 * @brief   发射模式
 */
void load_task_t::SHOOT_control()
{
    //左拨杆下，发射初始化
    if (syspoint()->sub_mode == SHOOT_INIT)
    {
        rotary_motor.shoot_init();
        loader_motor.shoot_init();
    }
    //左拨杆中，手动发射
    if (syspoint()->sub_mode == SHOOT_MANUAL)
    {
        shooting();
    }
}

/**
 * @brief   转盘电机发射初始化
 */
void rotary_motor_t::shoot_init()
{
    //装填电机未校准时不能初始化
    if (!loader_motor_point()->has_calibrated)
    {
        current_calculate();
        return;
    }

    //标志位被置一时开始初始化
    if (has_shoot_init_started)
    {
        final_relative_angle_set = rad_format((PI / 2) * syspoint()->active_dart_index);
        has_shoot_init_finished = 0;
    }

    //锁定时设定值不变化，不锁定时设定值逐渐向final增加
    if (!should_lock)
    {
        relative_angle_set = RAMP_float_loop_constrain(final_relative_angle_set, relative_angle_set, ROTARY_SHOOT_INIT_RAMP_BUFF);
    }

    current_calculate();

    //角度差小于一定值时初始化结束
    if (fabs(final_relative_angle_set - relative_angle) < ROTARY_ANGLE_TOLERANCE && has_shoot_init_started == 1)
    {
        has_shoot_init_started = 0;
        has_shoot_init_finished = 1;
    }
}

/**
 * @brief   装填电机发射初始化
 */
void loader_motor_t::shoot_init()
{
    //装填电机未校准时不能初始化
    if (!has_calibrated)
    {
        current_calculate(NULL);
        return;
    }

    //标志位被置一时开始初始化
    if (has_shoot_init_started)
    {
        ecd_set = zero_point_ecd;
        has_shoot_init_finished = 0;
    }

    current_calculate(LOADER_SHOOT_INIT_SPEED);

    //装填电机初始化结束
    if (fabs(ecd_set - accumulate_ecd) < LOADER_ECD_TOLERANCE && has_shoot_init_started == 1)
    {
        has_shoot_init_started = 0;
        has_shoot_init_finished = 1;
    }
}

/**
 * @brief   发射
 */
void load_task_t::shooting()
{
    //装填电机未校准或装填电机和转盘电机未初始化时不能发射
    if (!loader_motor.has_calibrated || !loader_motor.has_shoot_init_finished || !rotary_motor.has_shoot_init_finished)
    {
        loader_motor.current_calculate(NULL);
        rotary_motor.current_calculate();
        return;
    }

    //装填电机上移
    loader_motor.shoot_move_up();
    //装填电机上移完毕后下移
    loader_motor.shoot_move_down();
    //转盘转到下一个位置
    rotary_motor.shoot_move_to_next();
}

/**
 * @brief   装填电机上移
 */
void loader_motor_t::shoot_move_up()
{
    //未上移完毕时计算电流和标志位，上移完毕不再执行
    if (!has_shoot_up_finished)
    {
        //索引增加时，设置目标位置为最大点
        if (syspoint()->has_index_added)
        {
            ecd_set = max_point_ecd;
        }

        current_calculate(LOADER_SHOOT_UP_SPEED);

        //目标值和实际值之差小于一定值，可认为上移完毕
        if (fabs(ecd_set - accumulate_ecd) < LOADER_ECD_TOLERANCE)
        {
            has_shoot_up_finished = 1;
        }
    }
}

/**
 * @brief   装填电机下移
 */
void loader_motor_t::shoot_move_down()
{
    //上移完毕后计算电流和标志位，未上移完毕时不执行
    if (has_shoot_up_finished)
    {
        ecd_set = zero_point_ecd;

        current_calculate(LOADER_SHOOT_DOWN_SPEED);

        //目标值和实际值之差小于一定值，可认为下移完毕
        if (fabs(ecd_set - accumulate_ecd) < LOADER_ECD_TOLERANCE)
        {
            has_shoot_down_finished = 1;
        }
    }
}

/**
 * @brief   转盘电机转到下一个位置
 */
void rotary_motor_t::shoot_move_to_next()
{
    //设定最终角度
    if (syspoint()->active_dart_index < 4)
    {
        final_relative_angle_set = rad_format((PI / 2) * syspoint()->active_dart_index);
    }

    //装填电机上移完毕且转盘电机不锁定时，设定值逐渐向final增加
    if (loader_motor_point()->has_shoot_up_finished && !should_lock)
    {
        relative_angle_set = RAMP_float_loop_constrain(final_relative_angle_set, relative_angle_set, ROTARY_SHOOT_RAMP_BUFF);
    }

    current_calculate();

    //目标值和实际值之差小于一定值，可认为转到位
    if (fabs(final_relative_angle_set - relative_angle) < ROTARY_ANGLE_TOLERANCE)
    {
        has_move_to_next_finished = 1;
    }
}

/**
 * @brief       计算电流
 * @param[in]   max_out 速度环输出限幅，为NULL时使用初始化时的限幅
 */
void loader_motor_t::current_calculate(fp32 max_out)
{
    if (max_out == NULL)
    {
        speed_set = position_pid.calc(accumulate_ecd, ecd_set);
        give_current = speed_pid.calc(filtered_speed, speed_set);
    }
    else
    {
        speed_set = position_pid.DLcalc(accumulate_ecd, ecd_set, max_out);
        give_current = speed_pid.calc(filtered_speed, speed_set);
    }
}

/**
 * @brief   计算电流
 */
void rotary_motor_t::current_calculate()
{
    speed_set = position_pid.relative_angle_use_differ_calc(relative_angle, relative_angle_set);
    give_current = speed_pid.relative_angle_use_differ_calc(motor_measure->speed_rpm, speed_set);
}
