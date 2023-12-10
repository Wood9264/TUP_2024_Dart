#include "load_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "remote_control.h"
#include "BSP_can.h"
#include "system_task.h"
#include "monitor_task.h"

load_task_t load;

load_task_t *load_point()
{
    return &load;
}

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
        //装填任务标志位更新
        load.flag_update();
        //分任务控制
        load.control();
        //发送电流
        CAN2_1FF_cmd_motor(load.rotary_motor.give_current, load.loader_motor.give_current, 0, 0);
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
    rotary_motor.position_pid.init(PID_POSITION, rotary_position_pid, ROTARY_POSITION_PID_MAX_OUT, ROTARY_POSITION_PID_MAX_IOUT);
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
    if (IF_RC_SW1_DOWN)
    {
        loader_motor.adjust_position();
        rotary_motor.adjust_position();
    }
    else if (IF_RC_SW1_MID || IF_RC_SW1_UP)
    {
        loader_motor.calibrate();
        rotary_motor.calibrate();
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
        has_calibrate_begun = 1;
    }

    //遥控器↘↙手动校准，适用于触点开关失效的情况
    if (RC_double_held_single_return(LEFT_ROCKER_RIGHT_BOTTOM, RIGHT_ROCKER_LEFT_BOTTOM, 400))
    {
        manual_calibrate();
    }

    if (has_calibrate_begun == 1)
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
        ecd_set -= LOADER_CALIBRATE_DOWN_PER_LENGTH;
        speed_set = position_pid.calc(accumulate_ecd, ecd_set);
        give_current = speed_pid.calc(motor_speed, speed_set);
    }
    //压下触点开关后上移
    else if (found_zero_point == 1 && bottom_tick == 1)
    {
        ecd_set += LOADER_CALIBRATE_UP_PER_LENGTH;
        speed_set = position_pid.calc(accumulate_ecd, ecd_set);
        give_current = speed_pid.calc(motor_speed, speed_set);
    }
    //滑块离开触点开关，校准完毕
    else if (found_zero_point == 1 && bottom_tick == 0)
    {
        speed_set = 0;
        has_calibrated = 1;
        found_zero_point = 0;
        has_calibrate_begun = 0;

        //设置零点、受限点和最大点
        zero_point_ecd = accumulate_ecd + ZERO_POINT_OFFSET;
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
    has_calibrate_begun = 0;

    //设置零点和最大点
    zero_point_ecd = accumulate_ecd + ZERO_POINT_OFFSET;
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
        calculate_current();
        return;
    }

    add_angle = load.load_rc_ctrl->rc.ch[0] * RC_TO_ROTARY_MOTOR_ANGLE_SET;
    relative_angle_set = rad_format(relative_angle_set + add_angle);
    calculate_current();
}

/**
 * @brief   校准模式下转盘电机不转动，但仍需计算电流
 */
void rotary_motor_t::calibrate()
{
    calculate_current();
}

/**
 * @brief   发射模式
 */
void load_task_t::SHOOT_control()
{
    if (IF_RC_SW1_DOWN)
    {
        rotary_motor.shoot_init();
        loader_motor.shoot_init();
    }
    if (IF_RC_SW1_MID || IF_RC_SW1_UP)
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
        calculate_current();
        return;
    }

    if (RC_held_continuous_return(RIGHT_ROCKER_LEFT_TOP, 100))
    {
        //↗↖初始化为1号弹体位置
        if (RC_held_continuous_return(LEFT_ROCKER_RIGHT_TOP, 100))
        {
            final_relative_angle_set = 0;
            has_shoot_init_started = 1;
            has_shoot_init_finished = 0;
            syspoint()->active_dart_index = 1;
        }
        //↖↖初始化为2号弹体位置
        else if (RC_held_continuous_return(LEFT_ROCKER_LEFT_TOP, 100))
        {
            final_relative_angle_set = PI / 2;
            has_shoot_init_started = 1;
            has_shoot_init_finished = 0;
            syspoint()->active_dart_index = 2;
        }
        //↙↖初始化为3号弹体位置
        else if (RC_held_continuous_return(LEFT_ROCKER_LEFT_BOTTOM, 100))
        {
            final_relative_angle_set = PI;
            has_shoot_init_started = 1;
            has_shoot_init_finished = 0;
            syspoint()->active_dart_index = 3;
        }
        //↘↖初始化为4号弹体位置
        else if (RC_held_continuous_return(LEFT_ROCKER_RIGHT_BOTTOM, 100))
        {
            final_relative_angle_set = -PI / 2;
            has_shoot_init_started = 1;
            has_shoot_init_finished = 0;
            syspoint()->active_dart_index = 4;
        }
    }

    //锁定时转盘不动
    if (should_lock)
    {
        calculate_current();
    }
    //不锁定时设定值逐渐向final增加
    else
    {
        relative_angle_set = RAMP_float_loop_constrain(final_relative_angle_set, relative_angle_set, ROTARY_SHOOT_INIT_RAMP_BUFF);
        calculate_current();
    }

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
        speed_set = position_pid.calc(accumulate_ecd, ecd_set);
        give_current = speed_pid.calc(motor_speed, speed_set);
        return;
    }

    //转盘初始化时装填电机一起初始化
    if (rotary_motor_point()->has_shoot_init_started)
    {
        ecd_set = zero_point_ecd;
        has_shoot_init_started = 1;
        has_shoot_init_finished = 0;
    }

    speed_set = position_pid.DLcalc(accumulate_ecd, ecd_set, LOADER_SHOOT_INIT_SPEED);
    give_current = speed_pid.calc(motor_speed, speed_set);

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
    //装填电机未校准时或装填电机和转盘电机未初始化时不能发射
    if (!loader_motor.has_calibrated || !loader_motor.has_shoot_init_finished || !rotary_motor.has_shoot_init_finished)
    {
        loader_motor.speed_set = loader_motor.position_pid.calc(loader_motor.accumulate_ecd, loader_motor.ecd_set);
        loader_motor.give_current = loader_motor.speed_pid.calc(loader_motor.motor_speed, loader_motor.speed_set);
        rotary_motor.calculate_current();
        return;
    }

    //装填电机上移
    if (loader_motor.shoot_move_up())
    {
        //装填电机上移完毕后下移，同时转盘转到下一个位置
        if (loader_motor.shoot_move_down() && rotary_motor.shoot_move_to_next())
        {
            //装填电机下移完毕且转盘转到位后，可用遥控器控制打下一发飞镖
            dart_index_add();
        }
        //不执行dart_index_add时将标志位置零
        else
        {
            has_index_added = 0;
        }
    }
}

/**
 * @brief   装填电机上移
 * @retval  1:上移完毕 0:未上移完毕
 */
bool_t loader_motor_t::shoot_move_up()
{
    static bool_t has_shoot_up_finished = 0;

    if (load_point()->has_index_added)
    {
        ecd_set = max_point_ecd;
        has_shoot_up_finished = 0;
    }

    if (fabs(ecd_set - accumulate_ecd) < LOADER_ECD_TOLERANCE)
    {
        has_shoot_up_finished = 1;
    }

    if (has_shoot_up_finished)
    {
        return 1;
    }
    else
    {
        speed_set = position_pid.DLcalc(accumulate_ecd, ecd_set, LOADER_SHOOT_UP_SPEED);
        give_current = speed_pid.calc(motor_speed, speed_set);
        return 0;
    }
}

/**
 * @brief   装填电机下移
 * @retval  1:下移完毕 0:未下移完毕
 */
bool_t loader_motor_t::shoot_move_down()
{
    ecd_set = zero_point_ecd;
    speed_set = position_pid.DLcalc(accumulate_ecd, ecd_set, LOADER_SHOOT_DOWN_SPEED);
    give_current = speed_pid.calc(motor_speed, speed_set);
    return (fabs(accumulate_ecd - ecd_set) < LOADER_ECD_TOLERANCE);
}

/**
 * @brief   转盘电机转到下一个位置
 * @retval  1:转到位 0:未转到位
 */
bool_t rotary_motor_t::shoot_move_to_next()
{
    //根据当前发射的飞镖编号计算最终角度设定值
    final_relative_angle_set = rad_format((PI / 2) * (syspoint()->active_dart_index - 1));

    //转盘电机锁定时不能发射
    if (should_lock)
    {
        calculate_current();
        return 0;
    }

    relative_angle_set = RAMP_float_loop_constrain(final_relative_angle_set, relative_angle_set, ROTARY_SHOOT_RAMP_BUFF);
    calculate_current();

    return (fabs(final_relative_angle_set - relative_angle) < ROTARY_ANGLE_TOLERANCE);
}

/**
 * @brief   打下一发飞镖
 */
void load_task_t::dart_index_add()
{
    //左摇杆↖打出下一发飞镖
    if (RC_held_continuous_return(LEFT_ROCKER_LEFT_TOP, 0) && has_index_added == 0 && syspoint()->active_dart_index <= 4)
    {
        syspoint()->active_dart_index++;
        has_index_added = 1;
    }
}

/**
 * @brief   计算电流
*/
void rotary_motor_t::calculate_current()
{
    speed_set = position_pid.relative_angle_calc(relative_angle, relative_angle_set);
    give_current = speed_pid.calc(motor_measure->speed_rpm, speed_set);
}
