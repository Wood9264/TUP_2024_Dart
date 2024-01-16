#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H

#include "system_task.h"
#include "BSP_can.h"
#include "pid.h"
#include "ladrc.h"
#include "ladrc_feedforward.h"
#include "INS_task.h"
#include "arm_math.h"
#include "kalman.h"
#include "filter.h"
#include "communicate.h"
#include "user_lib.h"


//任务初始化 空闲一段时间
#define GIMBAL_TASK_INIT_TIME 201

#define Motor_Ecd_to_Rad 0.000766990394f //      2*  PI  /8192


//掉头180 按键
#define TURN_KEYBOARD KEY_PRESSED_OFFSET_Z&&KEY_PRESSED_OFFSET_CTRL
//掉头云台速度
#define TURN_SPEED    0.01f

//电机码盘值最大以及中值
#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8191

//云台初始化回中值，允许的误差,并且在误差范围内停止一段时间以及最大时间6s后解除初始化状态，
#define GIMBAL_INIT_ANGLE_ERROR     0.01f
#define GIMBAL_INIT_STOP_TIME       100
#define GIMBAL_INIT_TIME            1000
#define GIMBAL_CALI_REDUNDANT_ANGLE 0.1f
//云台初始化回中值的速度以及控制到的角度
#define GIMBAL_INIT_PITCH_SPEED     0.03f
#define GIMBAL_INIT_YAW_SPEED       0.025f

#define INIT_YAW_SET    0.0f
#define INIT_PITCH_SET  0.0f
#define INIT_Relative_PITCH_SET -0.1f
//电机编码值转化成角度值
#ifndef MOTOR_ECD_TO_RAD
// #define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192
#define MOTOR_ECD_TO_RAD_DOUBLE  0.000383495197 //2*  PI  /8192*2

#endif
#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus
	
class gimbal_motor_t
{
	public:
		const motor_t *get_gimbal_motor;
	  LADRC_FDW_t LADRC_FDW;	
	  LADRC_FDW_t AUTO_LADRC_FDW;
	  uint16_t offset_ecd;
		int64_t ecd_angle; //计算角度差值
    fp32 max_relative_angle; //rad
    fp32 min_relative_angle; //rad

    fp32 relative_angle;     //rad
    fp32 relative_angle_set; //rad
    fp32 absolute_angle;     //rad
    fp32 absolute_angle_set; //rad
    fp32 motor_gyro;         //rad/s
    fp32 motor_speed;

    fp32 current_set;
    int16_t given_current;
	
		fp32 yaw_error;
		void Absolute_angle_limit(fp32 add);
		void Absolute_angle_nolimit(fp32 add);
		void Relative_anlge_limit(fp32 add);

		void Absolute_adrc_control();
		void Absolute_auto_adrc_control();
		void Relative_adrc_control();
};	

class Gimbal_Kalman_t
{
	public:
	//根据视觉发送的误差值加上当前值的目标值 卡尔曼
  extKalman_t Gimbal_Pitch_Accle_Kalman;
	extKalman_t Gimbal_Pitch_Gyro_Kalman;
	extKalman_t Gimbal_Yaw_Accle_Kalman;
	extKalman_t Gimbal_Yaw_Gyro_Kalman;
	
  //云台角度误差卡尔曼
	extKalman_t Vision_Distance_Kalman;
	extKalman_t Yaw_Error_Vis_Kalman;//视觉误差Kalman
	extKalman_t Pitch_Error_Vis_Kalman;//
	extKalman_t Yaw_Set_Gim_Kalman;//给定set kalman
	extKalman_t Pitch_Set_Gim_Kalman;//

	float Auto_Error_Yaw[2];//    now/last
	float Auto_Error_Pitch[2];
	float Auto_Distance;//距离单目

	uint32_t Vision_Time[2];// NOW/LAST
	
};
	
class gimbal_control_t
{
	public:
		fp32 add_yaw;
	  fp32 add_pit;
  	bool_t vision_flag;
		bool_t vision_last_flag;
    uint16_t  attack_switch_ramp;
	
    gimbal_motor_t Yaw_motor;
	  gimbal_motor_t Pitch_motor;
	  gimbal_motor_t Roll_motor;
    Gimbal_Kalman_t gimbal_kalman;	

	  const fp32 *gimbal_INT_angle_point;
	  const fp32 *gimbal_INT_gyro_point;

	  gimbal_control_t();
	
    void Get_info();
	  void Control();
	
	  fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
  
	  void Zero_force_control();
	  void Absolute_angle_control();
	  void Spin_control();
	  void Auto_control();
	  void Init_control();
		void Dbus_missing_control();

    void Init_angle_set();
		void Keyboard_angle_set();
	  void Gimbal_auto_angle_get();
		
		void Judge_init_state();
};

extern gimbal_control_t *gimbal_point(void);


#endif
extern void gimbal_task(void const *pvParameters);

#ifdef __cplusplus
}	
#endif

#endif
