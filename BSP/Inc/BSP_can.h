#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "struct_typedef.h"
#include "motor.h"
#include "detect_task.h"
//#include "pm01.h"
#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2
#ifdef __cplusplus
extern "C"{

#endif

#ifdef __cplusplus
enum CAN1_msg_ID
{
	CAN1_3508_RB_ID = 0x201,
	CAN1_3508_LB_ID = 0x202,
	CAN1_3508_LF_ID = 0x203,
	CAN1_3508_RF_ID = 0x204,
	CAN1_YAW_MOTOR_ID = 0x206,
	
};

enum CAN2_msg_ID
{
	CAN2_FireL_Motor_ID = 0x201,
	CAN2_FireR_Motor_ID = 0x202,
	CAN2_TRIGGER_MOTOR_ID = 0X203,
	CAN2_PITCH_Motor_ID = 0x206,
};

//可在此处加入所需电机编号
enum motor_ID
{
	RB,
	LB,
	LF,
	RF,
	YAW,
	FireL,
	FireR,
	TRIGGER,
	PITCH,
};

typedef struct
{   
	fp32 Cap_input_vol;
	fp32 Cap_voltage;
	fp32 Cap_current;
	uint16_t Cap_power;
} Chassis_Power_t;

extern void CAN1_200_cmd_motor(int16_t can1_motor1, int16_t can1_motor2, int16_t can1_motor3, int16_t can1_motor4);
extern void CAN1_1FF_cmd_motor(int16_t can1_motor5, int16_t can1_motor6, int16_t can1_motor7, int16_t can1_motor8);
extern void CAN2_200_cmd_motor(int16_t can2_motor1, int16_t can2_motor2, int16_t can2_motor3, int16_t can2_motor4);
extern void CAN2_1FF_cmd_motor(int16_t can2_motor5, int16_t can2_motor6, int16_t can2_motor7, int16_t can2_motor8);
extern void CAN_cmd_chassis_reset_ID(void);
extern void CAN_CMD_CAP(uint16_t motor1);
extern const motor_t *get_motor_measure_class(uint16_t type);
extern const Chassis_Power_t *get_cap_measure_point(void);

#endif	
extern void can_filter_init(void);	
#ifdef __cplusplus
}
#endif

#endif
