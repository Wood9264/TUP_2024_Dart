#ifndef LOAD_TASK_H
#define LOAD_TASK_H

#include "motor.h"
#include "pid.h"
#include "ladrc_feedforward.h"
#include "arm_math.h"

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

//转盘电机LADRC参数
#define ROTARY_LADRC_WC 0.0f
#define ROTARY_LADRC_B0 0.0f
#define ROTARY_LADRC_W0 0.0f
#define ROTARY_LADRC_MAXOUT 0.0f
#define ROTARY_LADRC_W 0.0f
#define ROTARY_LADRC_GAIN 0.0f

//转盘电机零点编码值
#define ROTARY_ZERO_POINT_ECD 0

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
    };

    class rotary_motor_t
    {
    public:
        const motor_t *motor_measure;
        fp32 motor_acceleration;
        fp32 relative_angle;
        
        uint16_t zero_point_ecd;

        fp32 relative_angle_set;

        LADRC_FDW_t LADRC_FDW;

        int16_t give_current;
        fp32 motor_ecd_to_relative_angle(uint16_t ecd, uint16_t zero_point_ecd);
    };

    class load_task_t
    {
    public:
        loader_motor_t loader_motor;
        rotary_motor_t rotary_motor;
        load_task_t();
        void data_update();
    };

#endif

    extern void load_task(void const *pvParameters);
#ifdef __cplusplus
}
#endif

#endif
