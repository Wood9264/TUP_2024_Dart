/**
 * @file        system_task.h
 * @author      Yang Maolin (1831051389@qq.com)
 * @attention   本文件中包含了大量与飞镖系统无关的代码，因与其他模块耦合度较高难以清理，暂时保留。
 *              与飞镖系统有关的部分为：从系统模式枚举开始到system_t类定义结束（不包括system_t类中特别注明的部分）。
 */
#ifndef __SYSTEM_TASK_H
#define __SYSTEM_TASK_H

#include "remote_control.h"
#include "detect_task.h"
#include "ladrc_feedforward.h"
#include "user_lib.h"
#include "vision.h"
// yaw,pitch控制通道以及状态开关通道
#define YAW_CHANNEL 2
#define PITCH_CHANNEL 3
#define GIMBAL_MODE_CHANNEL 0
#define REVOLVER_MODE_CHANNEL 1
//前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 1
//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 0

#define GIMBAL_CALC 0
#define CHASSIS_CALC 1

#define YAW_MOUSE_SEN 0.00005f
#define PITCH_MOUSE_SEN 0.00005f

//遥控器前后左右摇杆（max 660）转化成车体前后速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.005f
#define CHASSIS_VY_RC_SEN 0.005f

#define CHASSIS_VX_RE_SEN 0.65f
#define CHASSIS_VY_RE_SEN 0.65f

#define CHASSIS_WZ_RE_SEN 0.7f
//遥控器摇杆（max 660）转化成车体yaw pitch速度（m/s）的比例
#define YAW_RC_SEN -0.00001f
#define PITCH_RC_SEN -0.00000422f // 0.005

// YAW左右转控制按钮
#define GIMBAL_LEFT_KEY KEY_PRESSED_OFFSET_Q
#define GIMBAL_RIGHT_KEY KEY_PRESSED_OFFSET_E

//底盘前后左右控制按键
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D

//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_DEADBAND 0
//摇杆死区（底盘）
#define CHASSIS_RC_DEADLINE 50

//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 5.0f
//底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 3.0f

#define HEAT_ERROR 30 //热量限制

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus

    //系统模式
    enum system_mode_e
    {
        ZERO_FORCE, //无力模式
        CALIBRATE,  //校准模式
        SHOOT,      //发射模式
    };

    //子模式
    enum sub_mode_e
    {
        CALIBRATE_ADJUST_POSITION, //调整位置
        CALIBRATE_LOADER_AND_YAW,  //校准装填电机和yaw轴
        CALIBRATE_CHECK,           //检查校准结果
        SHOOT_INIT,                //发射初始化
        SHOOT_MANUAL,              //手动发射
    };

    //打击目标
    enum strike_target_e
    {
        OUTPOST, //前哨站
        BASE,    //基地
    };

    //系统类
    class system_t
    {
    public:
        const RC_ctrl_t *system_rc_ctrl;

        system_mode_e sys_mode; //系统模式
        system_mode_e last_sys_mode;
        sub_mode_e sub_mode; //子模式
        sub_mode_e last_sub_mode;
        strike_target_e strike_target; //打击目标
        uint8_t active_dart_index;     //当前正在发射的飞镖编号，范围0~4
        bool_t has_index_added;

        /*下面的变量都没用*/
        lpf_type_def yaw_lpf;
        lpf_type_def pitch_lpf;

        fp32 rc_add_yaw;
        fp32 rc_add_pit;
        uint8_t mode_change_type;
        bool_t vision_flag;
        bool_t vision_last_flag;
        bool_t vision_auto_flag;
        bool_t head_flag;
        bool_t head_last_flag;
        bool_t head_go_flag;
        bool init_state;

        //    first_order_filter_type_t chassis_cmd_slow_set_vx;  //use first order filter to slow set-point.使用一阶低通滤波减缓设定值
        //    first_order_filter_type_t chassis_cmd_slow_set_vy;  //use first order filter to slow set-point.使用一阶低通滤波减缓设定值
        //
        fp32 chassis_vx_set_temp;
        fp32 chassis_vy_set_temp;
        fp32 vx_max_speed; // max forward speed, unit m/s.前进方向最大速度 单位m/s
        fp32 vx_min_speed; // max backward speed, unit m/s.后退方向最大速度 单位m/s
        fp32 vy_max_speed; // max letf speed, unit m/s.左方向最大速度 单位m/s
        fp32 vy_min_speed; // max right speed, unit m/s.右方向最大速度 单位m/s

        system_t();

        void mode_set();
        void mode_transit();
        void control();
        void SHOOT_control();
        void shoot_init();
        void dart_index_add();
        void switch_strike_target();

        /*下面的方法都没用*/
        void Set_control();

        void Rc_vision();
        void Rc_head();
        int Gimbal_mode_change_transit();
        void Gimbal_value_calc();
        void Chassis_value_calc();
        void Key_set();
    };

    extern system_t *syspoint(void);

//射速环测试
#define SPEED_PID_KP 15
#define SPEED_PID_KI 0
#define SPEED_PID_KD 1
#define SPEED_PID_MAX_OUT 30
#define SPEED_PID_MAX_IOUT 10

//摩擦轮转数设定
#define FRICTION_L1_SPEED 4600 // 15
#define FRICTION_L2_SPEED 4800 // 18
#define FRICTION_L3_SPEED 7200 // 30

#define REVOL_STEP0 0
#define REVOL_STEP1 1
#define REVOL_STEP2 2

#define RevChannel 1

#define AN_BULLET 36864 // 38000//33903.370320855614973262032085562	//36864//33903.370320855614973262032085562		//单个子弹电机位置增加值

    typedef enum
    {
        shoot_unready = 0,
        shoot_ready = 1,
    } Key_ShootFlag;

    typedef enum
    {
        FRICTION_WHEEL_OFF,
        FRICTION_WHEEL_ON,
    } Friction_Wheel_Mode;

    typedef struct
    {
        fp32 speed_set;
        fp32 speed_ramp_set;
    } Firc3508_Motor_set;

    class Friction_wheel_Mode_set
    {
    public:
        Friction_Wheel_Mode friction_wheel_mode;
        void Friction_Wheel_behaviour_init();
        const RC_ctrl_t *Friction_wheel_RC; //摩擦轮使用的遥控器指针
        void Friction_wheel_mode_set();     //摩擦轮模式设置
        void Friction_wheel_mode_off_set();
        void Friction_wheel_mode_on_set();
        void User_fix(); //转速手动补偿
        void Temp_Fix_30S();
        void Temp_Fix_18S();
        void Temp_Fix_15S();
        void Temp_fix_save();
        Firc3508_Motor_set Firc_L;
        Firc3508_Motor_set Firc_R;
        uint16_t speedlimit;
        int fix_mode;
        int fix_last_mode;
        int fix_times;
        fp32 fixed;
        int16_t v_fic_set;
        fp32 firc_mode;
    };

    typedef enum
    {
        shoot_unfinish = 0,
        shoot_finish = 1,
    } Shoot_IFfinish;
    typedef enum
    {
        stuck_unfinish = 0,
        stuck_finish = 1,
    } Stuck_IFfinish;
    typedef enum
    {
        REVOL_POSI_MODE = 0,
        REVOL_SPEED_MODE = 1,
        REVOL_REVERSAL_MODE = 2,
    } eRevolverCtrlMode;

    class Dial_mode_set
    {
    public:
        void Dial_mode_init();
        void Dial_behaviour_set();
        void real_firc_info_update();
        const RC_ctrl_t *Dial_RC; //拨盘使用的遥控器指针
        uint8_t Revolver_Switch;
        uint8_t Revolver_last_Switch;
        int16_t speed_time;
        Key_ShootFlag key_ShootFlag;
        Shoot_IFfinish ifshoot_finsh;
        Stuck_IFfinish ifstuck_finish;
        eRevolverCtrlMode Revolver_mode;
        eRevolverCtrlMode last_Revolver_mode;

        //裁判系统数据
        uint16_t heat_max;
        int16_t heating;
        fp32 angle;
        int64_t set_angle;
        int64_t last_angle;
        int16_t angle_out;
        int16_t unfinish_time;
        bool_t if_stucked;
        bool_t stuck_mode;

        fp32 bullet_last_speed;
        fp32 bullet_speed;
    };

    extern fp32 x_coordinate, y_coordinate;
    extern fp32 pre_x_coordinate, pre_y_coordinate;
    extern fp32 follow_radius, pre_radius;
#endif
    extern void system_task(void const *argument);
// extern void system_t_task_init();
#ifdef __cplusplus
}
#endif

#endif
