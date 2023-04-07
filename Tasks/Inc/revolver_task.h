#ifndef _REVOLVER_TASK_H
#define _REVOLVER_TASK_H
#ifdef __cplusplus
#include "system_task.h"
#include "pid.h"
#include "BSP_can.h"
#include "tim.h"

#define REVOLVER_SPEED_PID_KP 650.0f//780.0f
#define REVOLVER_SPEED_PID_KI 0
#define REVOLVER_SPEED_PID_KD  800//2
#define REVOLVER_SPEED_PID_MAX_OUT 9000.0f
#define REVOLVER_SPEED_PID_MAX_IOUT 5000.0f

#define REVOLVER_POSITION_PID_KP 0.025f//0.040f //0.00080f//0.00072f
#define REVOLVER_POSITION_PID_KI 0
#define REVOLVER_POSITION_PID_KD 0.080f//0.060f//0.000001f
#define REVOLVER_POSITION_PID_MAX_OUT 60.0f//45.0f
#define REVOLVER_POSITION_PID_MAX_IOUT 10.0f

//¡¨∑¢≤¶≈Ãpid
#define REVOLVER_FULL_SPEED_PID_KP           650.0f//1800.0f         
#define REVOLVER_FULL_SPEED_PID_KI           0.0f
#define REVOLVER_FULL_SPEED_PID_KD           0.0f
#define REVOLVER_FULL_SPEED_PID_MAX_OUT      9000.0f
#define REVOLVER_FULL_SPEED_PID_MAX_IOUT     5000.0f

#define REVOLVER_FULL_POSITION_PID_KP        0.008f//0.0008f
#define REVOLVER_FULL_POSITION_PID_KI        0.0f
#define REVOLVER_FULL_POSITION_PID_KD        0.01f
#define REVOLVER_FULL_POSITION_PID_MAX_OUT   60.0f
#define REVOLVER_FULL_POSITION_PID_MAX_IOUT  10.0f

#define Motor_RMP_TO_SPEED 0.00290888208665721596153948461415f

#define REVOLVER_TASK_INIT_TIME 200
#ifdef __cplusplus
extern "C"{
#endif

class Revolver_Task
{
	public:
		void REVOLVER_Init();
		void Revolver_Feedback_Update();
		void Friction_wheel_calculating();
		void Dial_calculating();
	
		Dial_mode_set Dial_set;
		Friction_wheel_Mode_set  Friction_wheel_set;
		PID_t revolver_motor_speed_pid;
		PID_t revolver_motor_position_pid;
		PID_t Firc_R_speed_pid;
		PID_t Firc_L_speed_pid;
		float Firc_R_give_current;
		float Firc_L_give_current;
		float Dial_give_current;
		const motor_t *Firc_L_firc3508_motor_measure;
		const motor_t *Firc_R_firc3508_motor_measure;
		const motor_t *revolver_motor_measure;
		float firc_l;
    float firc_r;
		float Revolver_speed;
		float bullet_speed;
		float last_bullet_speed;
		float Firc_L_speed_ramp_set;
		float Firc_R_speed_ramp_set;
		float Dial_set_ramp_angle;
		float Dial_buff_ramp;
		float angle_out;
};






	extern Revolver_Task  revolver; 


#endif
extern void revolver_task(void const *pvParameters);

#ifdef __cplusplus
}	
#endif
#endif

