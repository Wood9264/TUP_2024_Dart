#include "load_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "remote_control.h"
#include "BSP_can.h"

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

    //初始化转盘电机
    rotary_motor.LADRC_FDW.init(ROTARY_LADRC_WC, ROTARY_LADRC_B0, ROTARY_LADRC_W0, ROTARY_LADRC_MAXOUT, ROTARY_LADRC_W, ROTARY_LADRC_GAIN);
    rotary_motor.give_current = 0;
    rotary_motor.zero_point_ecd = ROTARY_ZERO_POINT_ECD;
    rotary_motor.motor_measure = get_motor_measure_class(ROTARY_MOTOR);
}

/**
 * @brief   装填任务数据更新
 */
void load_task_t::data_update()
{
    //更新装填电机数据
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
    rotary_motor.motor_ecd_to_relative_angle(rotary_motor.motor_measure->ecd, rotary_motor.zero_point_ecd);
}

/**
 * @brief   计算ecd与zero_point_ecd之间的相对角度
 * @param[in]   ecd 当前ecd
 * @param[in]   zero_point_ecd 初始ecd
 * @retval      相对角度，单位rad
 */
fp32 rotary_motor_t::motor_ecd_to_relative_angle(uint16_t ecd, uint16_t zero_point_ecd)
{
    //计算相对角度
    fp32 relative_angle = (ecd - zero_point_ecd) * MOTOR_ECD_TO_RAD;
    //角度限幅
    if (relative_angle > PI)
    {
        relative_angle -= 2 * PI;
    }
    else if (relative_angle < -PI)
    {
        relative_angle += 2 * PI;
    }
    return relative_angle;
}
