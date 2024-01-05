#ifndef _REVOLVER_TASK_H
#define _REVOLVER_TASK_H
#ifdef __cplusplus
#include "system_task.h"
#include "pid.h"
#include "BSP_can.h"
#include "tim.h"
//单发双环PID 注意抖动
#define SLIPPER_SPEED_PID_KP 700.0f // 780.0f
#define SLIPPER_SPEED_PID_KI 0.0f
#define SLIPPER_SPEED_PID_KD 3000.0f // 2
#define SLIPPER_SPEED_PID_MAX_OUT 9000.0f
#define SLIPPER_SPEED_PID_MAX_IOUT 5000.0f

#define SLIPPER_POSITION_PID_KP 0.001f   //0.025f // 0.040f //0.00080f//0.00072f//0.025
#define SLIPPER_POSITION_PID_KI 0
#define SLIPPER_POSITION_PID_KD 0.01f		// 0.060f//0.000001f//0.08
#define SLIPPER_POSITION_PID_MAX_OUT 10.0f // 45.0f
#define SLIPPER_POSITION_PID_MAX_IOUT 10.0f

//yaw轴速度环PID
#define YAW_SPEED_PID_KP 8.0f
#define YAW_SPEED_PID_KI 0.0f
#define YAW_SPEED_PID_KD 5.0f
#define YAW_SPEED_PID_MAX_OUT 9000.0f
#define YAW_SPEED_PID_MAX_IOUT 5000.0f

//yaw轴位置环PID
#define YAW_POSITION_PID_KP 1.0f
#define YAW_POSITION_PID_KI 0
#define YAW_POSITION_PID_KD 11.5f
#define YAW_POSITION_PID_MAX_OUT 6000.0f
#define YAW_POSITION_PID_MAX_IOUT 10.0f

//摩擦轮速度环PID
#define FRIC_SPEED_PID_KP 8.0f
#define FRIC_SPEED_PID_KI 0.0f
#define FRIC_SPEED_PID_KD 5.0f
#define FRIC_SPEED_PID_MAX_OUT 8192.0f
#define FRIC_SPEED_PID_MAX_IOUT 5000.0f

#define FRIC_RAMP_BUFF 20	//摩擦轮启动时的斜坡增加量

#define BASE_SPEED 500 //四个摩擦轮的基础转速

#define ONE_DART_ECD 134985 //每发飞镖的滑块电机编码值增加量
#define MAX_DART_NUM 2 //发射架可装填的最大飞镖数量
#define CALIBRATE_OFFSET 8192	//校准时编码值的补偿量。防止滑块回退时因为超调碰到触点开关

#define RC_TO_SLIPPER_SEPPD_SET (7.0f / 660) //遥控器通道到滑块速度设定值的比例
#define SLIPPER_LOCK_SPEED 0.1 //滑块电机速度设定为0，且实际速度低于这个值时，电机用角度环锁定位置

#define Motor_RMP_TO_SPEED 0.00290888208665721596153948461415f

#define POSITION_LIMIT_BUFFER_DISTANCE (8192 * 36 / 36) //滑块接近限位开始减速缓冲时与限位的距离
#define ANGLE_LOOP_SWITCH_DISTANCE (8192) //速度环切换到角度环时，设定编码值与实际编码值的距离

// YAW轴电机运动到指定位置时允许的编码值误差
#define YAW_ECD_TOLERANCE 100

#define YAW_SHOOT_INIT_SPEED 1000 //发射初始化时YAW轴电机的转速
#define YAW_SHOOT_SPEED 1000 //发射时YAW轴电机的转速

// #define CALIBRATE_DOWN_SPEED (-5) //校准时滑块下移的速度
// #define CALIBRATE_UP_SPEED 5 //校准时滑块上移的速度
// #define SLIPPER_SHOOTING_SPEED 10 //发射时滑块上移的速度
// #define SLIPPER_BACK_SPEED 5 //滑块自动返回零点时的速度

#define RC_TO_YAW_ECD_SET 1 //遥控器通道到yaw轴位置增量的比例

#define REVOLVER_TASK_INIT_TIME 200
#ifdef __cplusplus
extern "C"
{
#endif

class fric_motor_t
	{
		public:
		const motor_t *motor_measure;
		PID_t speed_pid;
		fp32 speed_set;		
		int16_t give_current;

		

		void current_calculate();
	};

class yaw_motor_t
	{
		public:
		const motor_t *motor_measure;
		PID_t speed_pid;
		PID_t position_pid;

		int64_t accumulate_ecd;
		fp32 ecd_set;
		fp32 speed_set;
		fp32 final_ecd_set;

		int16_t give_current;

		int64_t calibrated_point;
		fp32 offset_num[5];

		bool_t has_calibrated;
		bool_t has_shoot_init_started;
		bool_t has_shoot_init_finished;
		bool_t has_shoot_move_finished;
		bool_t has_move_to_next_finished;

		void calibrate();
		void init();
		void shooting();
	};

	class slipper_motor_t
	{
		public:
		const motor_t *motor_measure;
		fp32 motor_speed;
		fp32 speed_set;
		int64_t accumulate_ecd;
		uint16_t bullet_num;	//已经打出的飞镖数量
		uint16_t bullet_num_set;

		fp32 ecd_set;
		fp32 slipper_position_ecd[MAX_DART_NUM + 1]; //飞镖发射时滑块每次停留的位置
		
		PID_t speed_pid;
		PID_t position_pid;

		int16_t give_current;

		bool_t calibrate_begin;
		bool_t has_calibrated;
		bool_t bottom_tick;
		bool_t if_shoot_begin;	//开始发射标志位
		bool_t should_lock;

		void SLIPPER_control();
		void position_limit_buffer(fp32 limit_point);
		void CALIBRATE_control();
		void auto_calibrate();
		void manual_calibrate();
		void bullet_num_cal();
		void SHOOTING_slipper_control();
	};
	
	class revolver_task_t
	{
	public:
		revolver_task_t();

		const RC_ctrl_t *revolver_rc_ctrl;

		fric_motor_t fric_motor[4];
		yaw_motor_t yaw_motor;
		

		int16_t outpost_speed_offset[4];
		int16_t base_speed_offset[4];

		fp32 fric_speed_offset;
		bool_t is_fric_wheel_on;

		void data_update();
		void control();
		void ZERO_FORCE_control();
		void fric_speed_offset_control();
		void CALIBRATE_control();
		void SHOOT_control();
		void READY_control();
		void SHOOTING_control();
		void fric_motor_init();
		void fric_motor_shooting();
		void fric_speed_buzzer(); 
		



		
		//将要移除的内容
		slipper_motor_t slipper_motor;
	};

	

	extern revolver_task_t revolver;
	extern revolver_task_t* revolver_point(void);
#endif
	extern void revolver_task(void const *pvParameters);

#ifdef __cplusplus
}
#endif
#endif
