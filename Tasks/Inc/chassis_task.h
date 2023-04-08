#ifndef CHASSISTASK_H
#define CHASSISTASK_H

#include "system_task.h"	
#include "pid.h"
#include "motor.h"
#include "user_lib.h"
#include "gimbal_task.h"
#include "cmsis_os.h"
#include "judge.h"
#include "Bsp_can.h"
#include "judge_task.h"

#define CHASSIS_TASK_INIT_TIME 357 //任务开始空闲一段时间

#define CHASSIS_CONTROL_TIME_MS 2  //底盘任务控制间隔 2ms

#define M3508_MOTOR_SPEED_PID_MAX_OUT    16000.0f
#define M3508_MOTOR_SPEED_PID_MAX_IOUT   2000.0f
#define CHASSIS_MOTOR_SPEED_PID_KP 5000.0f
#define CHASSIS_MOTOR_SPEED_PID_KI 0.0f	
#define CHASSIS_MOTOR_SPEED_PID_KD 2.5f
		
		
#define MOTOR_YAW_PID_MAX_OUT    20.0f
#define MOTOR_YAW_PID_MAX_IOUT   0.2f

#define CHASSIS_MOTOR_YAW_PID_KP -16.8f
#define CHASSIS_MOTOR_YAW_PID_KI   0.0f	
#define CHASSIS_MOTOR_YAW_PID_KD -18.0f

		
		
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.3f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.3f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.3f
	


#define CHASSIS_WZ_RC_SEN      0.007f      //不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
#define CHASSIS_WZ_CHANNEL     2           //在特殊模式下，可以通过遥控器控制旋转
#define CHASSIS_WZ_SET_SCALE -0.072f//底盘设置旋转速度，设置前后左右轮不同设定速度的比例分权 0为在几何中心，不需要补偿
#define MOTOR_DISTANCE_TO_CENTER 0.22f
#define MAX_WHEEL_SPEED 6.0f	//底盘电机最大速度



#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f //m3508转化成底盘速度(m/s)的比例，做两个宏 是因为可能换电机需要更换比例
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus
typedef struct
{
  const motor_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  fp32 power;
	fp32 current_set;
	fp32 current;
	fp32 current_scale;
	int16_t give_current;
}chassis_motor_t;


typedef struct
{
  fp32 BusV;
  fp32 Current;
  fp32 Power;
} ina226_t;

class chassis_t
{
	private:
	system_mode_t chassis_mode;
	gimbal_motor_t chassis_yaw_motor;   //底盘使用到yaw云台电机的相对角度来计算底盘的欧拉角
  gimbal_motor_t chassis_pitch_motor; //底盘使用到pitch云台电机的相对角度来计算底盘的欧拉角	
	
	
	PID_t chassis_motor_speed_pid[4];
  PID_t chassis_angle_pid;              //底盘跟随角度pid
	PID_t chassis_current_pid[4]; 
	
	first_order_filter_type_t chassis_cmd_slow_set_vx;
  first_order_filter_type_t chassis_cmd_slow_set_vy;

	fp32 vx;                         //底盘速度 前进方向 前为正，单位 m/s
  fp32 vy;                         //底盘速度 左右方向 左为正  单位 m/s
  fp32 wz;                         //底盘旋转角速度，逆时针为正 单位 rad/s
	
  fp32 vx_set;                     //底盘设定速度 前进方向 前为正，单位 m/s
  fp32 vy_set;                     //底盘设定速度 左右方向 左为正，单位 m/s
	fp32 wz_set;                     //底盘旋转角速度，逆时针为正 单位 rad/s
	
	
	fp32 vx_follow_set;                     //底盘设定速度 前进方向 前为正，单位 m/s
  fp32 vy_follow_set;                     //底盘设定速度 左右方向 左为正，单位 m/s
	
	fp32 vx_max_speed;               //前进方向最大速度 单位m/s 
  fp32 vx_min_speed;               //前进方向最小速度 单位m/s
  fp32 vy_max_speed;               //左右方向最大速度 单位m/s
  fp32 vy_min_speed;               //左右方向最小速度 单位m/s
	 
  fp32 chassis_relative_angle_set;

	
	fp32 angle_set;
	fp32 wheel_speed[4]; 
	public:
	fp32 chassis_real_power;
	fp32 chassis_power;
	fp32 cap_absorb;
	int shift_flag;
	fp32 chassis_power_buffer;
	uint16_t chassis_max_power;
  ina226_t ina226_data;	
	const Chassis_Power_t *chassis_cap_masure;
	chassis_motor_t chassis_motor[4];
	chassis_t();
	void Get_info();
	void Control();
	

	void  chassis_zero_control_set(void);
	void  chassis_follow_gimbal_control_set(void);
	void  chassis_no_follow_gimbal_control_set(void);
	void  chassis_spin_control_set(void);
	
	void  chassis_control_loop(void);
	
	void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4]);
  void chassis_power_limit();
	
};

chassis_t* chassispoint(void);	
#endif
extern void chassis_task(void const * pvParameters);
#ifdef __cplusplus
}	
#endif
#endif

