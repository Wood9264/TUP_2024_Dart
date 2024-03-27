#ifndef _REVOLVER_TASK_H
#define _REVOLVER_TASK_H
#ifdef __cplusplus
#include "system_task.h"
#include "pid.h"
#include "BSP_can.h"
#include "tim.h"

//任务初始化延时时间
#define REVOLVER_TASK_INIT_TIME 200

//摩擦轮速度环PID
#define FRIC_SPEED_PID_KP 30.0f
#define FRIC_SPEED_PID_KI 0.0f
#define FRIC_SPEED_PID_KD 0.0f
#define FRIC_SPEED_PID_MAX_OUT 12000.0f
#define FRIC_SPEED_PID_MAX_IOUT 5000.0f

// yaw轴速度环PID
#define YAW_SPEED_PID_KP 8.0f
#define YAW_SPEED_PID_KI 0.0f
#define YAW_SPEED_PID_KD 5.0f
#define YAW_SPEED_PID_MAX_OUT 9000.0f
#define YAW_SPEED_PID_MAX_IOUT 5000.0f

// yaw轴位置环PID
#define YAW_POSITION_PID_KP 1.0f
#define YAW_POSITION_PID_KI 0
#define YAW_POSITION_PID_KD 11.5f
#define YAW_POSITION_PID_MAX_OUT 6000.0f
#define YAW_POSITION_PID_MAX_IOUT 10.0f

//前后摩擦轮的转速差，正数为前轮快，负数为后轮快
#define SPEED_DIFFERENCE 1000

//摩擦轮启停时的斜坡增加量
#define FRIC_RAMP_BUFF 5

//发射初始化时yaw轴电机的转速
#define YAW_SHOOT_INIT_SPEED 1000
//发射时yaw轴电机的转速
#define YAW_SHOOT_SPEED 1000

// yaw轴电机运动到指定位置时允许的编码值误差
#define YAW_ECD_TOLERANCE 100

//遥控器通道到yaw轴位置增量的比例
#define RC_TO_YAW_ECD_SET 1

#ifdef __cplusplus
extern "C"
{
#endif

    //摩擦轮电机类
    class fric_motor_t
    {
    public:
        const motor_t *motor_measure;
        PID_t speed_pid;
        fp32 speed_set;
        int16_t give_current;
    };

    //摩擦轮组类
    class fric_wheel_group_t
    {
    public:
        fric_motor_t fric_motor[4]; //四个摩擦轮电机

        int16_t outpost_speed[4]; //打击前哨站时的转速
        int16_t base_speed[4];    //打击基地时的转速

        bool_t is_fric_wheel_on; //摩擦轮启动标志位

        void slow_stop();
        void current_calculate();
        void shoot_init();
        void shooting();
    };

    // yaw轴电机类
    class yaw_motor_t
    {
    public:
        const motor_t *motor_measure;
        PID_t speed_pid;
        PID_t position_pid;

        int64_t accumulate_ecd; //累计编码值
        fp32 ecd_set;           //设定编码值
        fp32 speed_set;

        int16_t give_current;

        int64_t calibrated_point;   //校准点编码值
        fp32 outpost_offset_num[4]; //打击前哨站时相对校准点的偏移量
        fp32 base_offset_num[4];    //打击基地时相对校准点的偏移量

        bool_t has_calibrated;            //已校准标志位
        bool_t has_shoot_init_started;    //开始发射初始化标志位
        bool_t has_shoot_init_finished;   //发射初始化完成标志位
        bool_t has_move_to_next_finished; //移动到下一个位置完成标志位

        void calibrate();
        void adjust_position();
        void check_calibrate_result();
        void shoot_init();
        void shooting();
        void current_calculate();
    };

    //发射机构类
    class revolver_task_t
    {
    public:
        revolver_task_t();

        const RC_ctrl_t *revolver_rc_ctrl; //遥控器信息

        fric_wheel_group_t fric_wheel_group; //摩擦轮组
        yaw_motor_t yaw_motor;               // yaw轴电机

        void data_update();
        void control();
        void ZERO_FORCE_control();
        void CALIBRATE_control();
        void SHOOT_control();
    };

    extern revolver_task_t revolver;
    extern revolver_task_t *revolver_point(void);
#endif
    extern void revolver_task(void const *pvParameters);

#ifdef __cplusplus
}
#endif
#endif
