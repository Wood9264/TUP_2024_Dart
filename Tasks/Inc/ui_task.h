#ifndef __UI_H
#define __UI_H

#include "chassis_task.h"
#include "BSP_can.h"

#define YES TRUE
#define NO  FALSE

#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus	
typedef struct
{
  uint16_t lib;          
  uint8_t Auto;                 
  uint8_t Spin;  
} UI_order;
	
typedef struct
{
  UI_order UI_send_measure;
	fp32 V_Cap_measure;
}UI_Measure_t;

typedef struct
{
  fp32 SPIN;
  uint8_t CLIP;
  uint8_t AUTO;
  uint8_t BlockMuch;
  uint8_t Shoot_heat_limit;
  uint8_t Bullet_Warning;
	UI_Measure_t UI_measure;
  float Vcap_show;
	uint8_t COVER;
	uint8_t FIRC;
	uint8_t STUCK;
	uint8_t SHIFT;
  uint8_t ENEMY;
		fp32 PITCH;
	fp32 YAW;
	uint8_t CUFF;
	uint8_t VUFF;

}User_CMD_t;


typedef struct
{
  bool_t IF_Init_Over;
  User_CMD_t User;
}UI_Info_t;

typedef struct
{
	int global_cover;
	int global_spin;
	int global_auto;
	int global_firc;
	int global_stuck;
	int global_shift;
	int global_enemy;
		fp32 global_pitch;
	fp32 global_yaw;
	int global_cuff;
	int global_vuff;
	uint16_t global_spcp;
	int global_fix;
}	global_flag_t;








void Startjudge_task(void);


#endif
extern void ui_task(void *argumt);
#ifdef __cplusplus
}	
#endif

#endif


