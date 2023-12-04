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

//转盘电机LADRC参数
#define ROTARY_LADRC_WC 0.0f
#define ROTARY_LADRC_B0 0.0f
#define ROTARY_LADRC_W0 0.0f
#define ROTARY_LADRC_MAXOUT 0.0f
#define ROTARY_LADRC_W 0.0f
#define ROTARY_LADRC_GAIN 0.0f

//装填电机转速滤波参数
#define LOADER_MOTOR_RMP_TO_FILTER_SPEED 0.00290888208665721596153948461415f
//电机编码值转化成角度值
#define MOTOR_ECD_TO_RAD (2 * PI / 8192)
//遥控器通道到装填电机位置增量的比例
#define RC_TO_LOADER_MOTOR_ECD_SET 0.1f
//遥控器通道到转盘电机角度设定值增量的比例
#define RC_TO_ROTARY_MOTOR_ANGLE_SET 0.00001f

//转盘电机发射初始化时角度的斜坡增加量
#define ROTARY_SHOOT_INIT_RAMP_BUFF 0.007f

//转盘电机判定为初始化完毕时允许的角度误差，单位为弧度
#define ROTARY_INIT_ANGLE_ERROR 0.01f
//装填电机判定为初始化完毕时允许的编码值误差
#define LOADER_INIT_ECD_ERROR 100

//转盘电机零点编码值
#define ROTARY_ZERO_POINT_ECD 0
//装填机构处在载弹架中时转盘电机可转动角度的一半，单位为弧度
#define ROTARY_HALF_MOVABLE_ANGLE 0.17f
//装填电机最大前进距离的编码值
#define LOADER_FORWARD_ECD 100000
//装填电机受限时最大前进距离的编码值
#define LOADER_RESTRICT_FORWARD_ECD 80000

#define LOADER_CALIBRATE_DOWN_PER_LENGTH 5 //校准时装填电机每次下移的编码值
#define LOADER_CALIBRATE_UP_PER_LENGTH 5   //校准时装填电机每次上移的编码值
#define LOADER_SHOOT_INIT_SPEED 5          //发射初始化时装填电机下移的速度
#define SLIPPER_SHOOTING_SPEED 10          //发射时滑块上移的速度
#define SLIPPER_BACK_SPEED 5               //滑块自动返回零点时的速度

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
        int64_t restrict_point_ecd;

        bool_t has_calibrate_begun;
        bool_t has_calibrated;
        bool_t bottom_tick;
        bool_t is_restricted_state;
        bool_t has_shoot_init_finished;

        void flag_update();
        void adjust_position();
        void calibrate();
        void auto_calibrate();
        void manual_calibrate();
        void shoot_init();
    };

    class rotary_motor_t
    {
    public:
        const motor_t *motor_measure;
        fp32 acceleration;
        fp32 relative_angle;
        fp32 last_relative_angle;

        fp32 relative_angle_set;
        fp32 final_relative_angle_set;

        LADRC_FDW_t LADRC_FDW;

        int16_t give_current;

        bool_t should_lock;
        bool_t has_shoot_init_started;
        bool_t has_shoot_init_finished;

        void motor_ecd_to_relative_angle();
        void acceleration_update();
        void flag_update();
        void adjust_position();
        void shoot_init();
    };

    class load_task_t
    {
    public:
        const RC_ctrl_t *load_rc_ctrl;
        loader_motor_t loader_motor;
        rotary_motor_t rotary_motor;

        load_task_t();
        void data_update();
        void flag_update();
        void control();
        void ZERO_FORCE_control();
        void CALIBRATE_control();
        void SHOOT_control();
    };

    extern load_task_t *load_point();
    loader_motor_t *loader_motor_point();
    rotary_motor_t *rotary_motor_point();
#endif

    extern void load_task(void const *pvParameters);

#ifdef __cplusplus
}
#endif

#endif
