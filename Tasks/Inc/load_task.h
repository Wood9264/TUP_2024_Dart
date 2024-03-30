#ifndef LOAD_TASK_H
#define LOAD_TASK_H

#include "motor.h"
#include "pid.h"
#include "ladrc_feedforward.h"
#include "arm_math.h"
#include "remote_control.h"

//任务初始化延时时间
#define LOAD_TASK_INIT_TIME 200

//装填电机速度环PID参数
#define LOADER_SPEED_PID_KP 700.0f
#define LOADER_SPEED_PID_KI 0.0f
#define LOADER_SPEED_PID_KD 3000.0f
#define LOADER_SPEED_PID_MAX_OUT 9000.0f
#define LOADER_SPEED_PID_MAX_IOUT 0.0f
//装填电机位置环PID参数
#define LOADER_POSITION_PID_KP 0.001f
#define LOADER_POSITION_PID_KI 0.0f
#define LOADER_POSITION_PID_KD 0.01f
#define LOADER_POSITION_PID_MAX_OUT 10.0f
#define LOADER_POSITION_PID_MAX_IOUT 0.0f

//转盘电机速度环PID参数
#define ROTARY_SPEED_PID_KP 60.0f
#define ROTARY_SPEED_PID_KI 1.0f
#define ROTARY_SPEED_PID_KD 90.0f
#define ROTARY_SPEED_PID_MAX_OUT 25000.0f
#define ROTARY_SPEED_PID_MAX_IOUT 25000.0f
//转盘电机位置环PID参数
#define ROTARY_POSITION_PID_KP 100.0f
#define ROTARY_POSITION_PID_KI 0.0f
#define ROTARY_POSITION_PID_KD 100.0f
#define ROTARY_POSITION_PID_MAX_OUT 300.0f
#define ROTARY_POSITION_PID_MAX_IOUT 250.0f

//装填电机转速滤波参数
#define LOADER_MOTOR_RMP_TO_FILTER_SPEED 0.00290888208665721596153948461415f

//转盘电机发射初始化时角度的斜坡增加量
#define ROTARY_SHOOT_INIT_RAMP_BUFF 0.0008f
//转盘电机发射时角度的斜坡增加量
#define ROTARY_SHOOT_RAMP_BUFF 0.0008f

//转盘电机零点编码值
#define ROTARY_ZERO_POINT_ECD 6683
//装填机构处在载弹架中时转盘电机可转动角度的一半，单位为弧度
#define ROTARY_HALF_MOVABLE_ANGLE 0.166f
//装填电机最大前进距离的编码值
#define LOADER_FORWARD_ECD 328256
//装填电机受限时最大前进距离的编码值
#define LOADER_RESTRICT_FORWARD_ECD 16385
//校准时零点编码值的补偿量。防止装填电机回退时因为超调碰到触点开关
#define ZERO_POINT_OFFSET 2048

//校准时装填电机下移的单位编码值
#define LOADER_CALIBRATE_DOWN_PER_LENGTH 300
//校准时装填电机上移的单位编码值
#define LOADER_CALIBRATE_UP_PER_LENGTH 300
//发射初始化时装填电机下移的速度
#define LOADER_SHOOT_INIT_SPEED 5
//发射时装填电机上移的速度
#define LOADER_SHOOT_UP_SPEED 5
//发射时装填电机下移的速度
#define LOADER_SHOOT_DOWN_SPEED 8

//转盘电机运动到指定位置时允许的角度误差，单位为弧度
#define ROTARY_ANGLE_TOLERANCE 0.01f
//装填电机运动到指定位置时允许的编码值误差
#define LOADER_ECD_TOLERANCE 8192

//电机编码值转化成角度值
#define MOTOR_ECD_TO_RAD (2 * PI / 8192)
//遥控器通道到装填电机位置增量的比例
#define RC_TO_LOADER_MOTOR_ECD_SET 0.8f
//遥控器通道到转盘电机角度设定值增量的比例
#define RC_TO_ROTARY_MOTOR_ANGLE_SET 0.00001f

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus

    //装填电机类
    class loader_motor_t
    {
    public:
        const motor_t *motor_measure;
        fp32 filtered_speed; //滤波后的速度
        int64_t accumulate_ecd;

        fp32 speed_set;
        fp32 ecd_set;

        PID_t speed_pid;
        PID_t position_pid;

        int16_t give_current;

        int64_t zero_point_ecd;
        int64_t max_point_ecd;
        int64_t restrict_point_ecd; //受限点编码值

        bool_t has_auto_calibrate_begun;
        bool_t has_calibrated;
        bool_t bottom_tick;         //底部限位开关状态
        bool_t is_restricted_state; //装填电机受限状态
        bool_t has_shoot_init_started;
        bool_t has_shoot_init_finished;
        bool_t has_shoot_up_finished;
        bool_t has_shoot_down_finished;

        void flag_update();
        void adjust_position();
        void calibrate();
        void check_calibrate_result();
        void auto_calibrate();
        void manual_calibrate();
        void shoot_init();
        void shoot_move_up();
        void shoot_move_down();
        void current_calculate(fp32 max_out);
    };

    //转盘电机类
    class rotary_motor_t
    {
    public:
        const motor_t *motor_measure;

        fp32 relative_angle;
        fp32 last_relative_angle;

        fp32 relative_angle_set;
        fp32 final_relative_angle_set; //最终角度设定值，用于给斜坡函数设定最终值
        fp32 speed_set;

        PID_t speed_pid;
        PID_t position_pid;

        int16_t give_current;

        bool_t should_lock; //锁定状态。装填电机位于载弹架中时，转盘电机锁定不能转动
        bool_t has_shoot_init_started;
        bool_t has_shoot_init_finished;
        bool_t has_move_to_next_finished;

        void motor_ecd_to_relative_angle();
        void acceleration_update();
        void flag_update();
        void adjust_position();
        void calibrate();
        void check_calibrate_result();
        void shoot_init();
        void current_calculate();
        void shoot_move_to_next();
    };

    //装填任务类
    class load_task_t
    {
    public:
        const RC_ctrl_t *load_rc_ctrl;
        loader_motor_t loader_motor; //装填电机
        rotary_motor_t rotary_motor; //转盘电机

        load_task_t();
        void data_update();
        void flag_update();
        void control();
        void ZERO_FORCE_control();
        void CALIBRATE_control();
        void SHOOT_control();
        void shooting();
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
