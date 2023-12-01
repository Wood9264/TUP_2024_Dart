#ifndef LOAD_TASK_H
#define LOAD_TASK_H

#include "motor.h"
#include "pid.h"
#include "ladrc_feedforward.h"
#include "arm_math.h"
#include "remote_control.h"

#define LOAD_TASK_INIT_TIME 200

//装填电机速度环PID参数
#define LOADER_SPEED_PID_KP 0.0f
#define LOADER_SPEED_PID_KI 0.0f
#define LOADER_SPEED_PID_KD 0.0f
#define LOADER_SPEED_PID_MAX_OUT 0.0f
#define LOADER_SPEED_PID_MAX_IOUT 0.0f
//装填电机位置环PID参数
#define LOADER_POSITION_PID_KP 1.0f
#define LOADER_POSITION_PID_KI 0.0f
#define LOADER_POSITION_PID_KD 0.0f
#define LOADER_POSITION_PID_MAX_OUT 0.0f
#define LOADER_POSITION_PID_MAX_IOUT 0.0f

//装填电机转速滤波参数
#define LOADER_MOTOR_RMP_TO_FILTER_SPEED 0.00290888208665721596153948461415f
//电机编码值转化成角度值
#define MOTOR_ECD_TO_RAD (2 * PI / 8192)
//遥控器通道到装填电机位置增量的比例
#define RC_TO_LOADER_MOTOR_ECD_SET 0.1f

//转盘电机LADRC参数
#define ROTARY_LADRC_WC 0.0f
#define ROTARY_LADRC_B0 0.0f
#define ROTARY_LADRC_W0 0.0f
#define ROTARY_LADRC_MAXOUT 0.0f
#define ROTARY_LADRC_W 0.0f
#define ROTARY_LADRC_GAIN 0.0f

//转盘电机零点编码值
#define ROTARY_ZERO_POINT_ECD 0
//装填电机前进距离的编码值
#define LOADER_FORWARD_ECD 100000

#define CALIBRATE_DOWN_PER_LENGTH 5 //校准时装填电机每次下移的编码值
#define CALIBRATE_UP_PER_LENGTH 5   //校准时装填电机每次上移的编码值
#define SLIPPER_SHOOTING_SPEED 10   //发射时滑块上移的速度
#define SLIPPER_BACK_SPEED 5        //滑块自动返回零点时的速度

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus

    class loader_motor_t
    {
    public:
        const motor_t *motor_measure;
        fp32 motor_speed;
        int64_t accumulate_ecd;

        fp32 speed_set;
        fp32 ecd_set;

        PID_t speed_pid;
        PID_t position_pid;

        int16_t give_current;

        int64_t zero_point_ecd;
        int64_t max_point_ecd;

        bool_t calibrate_begin;
        bool_t has_calibrated;
        bool_t bottom_tick;
        void adjust_position();
        void calibrate();
        void auto_calibrate();
        void manual_calibrate();
    };

    class rotary_motor_t
    {
    public:
        const motor_t *motor_measure;
        fp32 motor_acceleration;
        fp32 relative_angle;
        fp32 last_relative_angle;

        fp32 relative_angle_set;

        LADRC_FDW_t LADRC_FDW;

        int16_t give_current;
        void motor_ecd_to_relative_angle();
        void acceleration_update();
    };

    class load_task_t
    {
    public:
        const RC_ctrl_t *load_rc_ctrl;
        loader_motor_t loader_motor;
        rotary_motor_t rotary_motor;

        load_task_t();
        void data_update();
        void control();
        void ZERO_FORCE_control();
        void CALIBRATE_control();
        void SHOOT_control();
    };

#endif

    extern void load_task(void const *pvParameters);
#ifdef __cplusplus
}
#endif

#endif
