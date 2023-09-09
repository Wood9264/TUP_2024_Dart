#ifndef MONITOR_TASK_H
#define MONITOR_TASK_H

#include "bsp_led.h"
#include "vision.h"
#include "gimbal_Task.h"
#include "struct_typedef.h"
#include "detect_task.h"

#define BUZZER_MAX_OFF_TICK 10	//调用buzzer_warn后蜂鸣器停止的tick数，buzzer_warn未被调用的时间超过这个tick数后蜂鸣器自动停止

#ifdef __cplusplus
extern "C"{
#endif

	typedef struct
	{
		uint8_t buzzer_off_tick;	   //蜂鸣器启动持续的tick数
		uint16_t buzzer_tick;		   //蜂鸣器响、停持续的tick计数
		uint8_t buzzer_warn_num;	   //蜂鸣器报警的次数
		uint8_t buzzer_warn_num_set;   //蜂鸣器报警的设定次数
		uint16_t buzzer_warn_interval; //蜂鸣器响的间隔时间
		uint16_t buzzer_psc;		   //蜂鸣器的分频系数
		uint16_t buzzer_pwm;		   //蜂鸣器的重载值
	} buzzer_t;

#ifdef __cplusplus
/**
  * @brief          监视任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
#endif
extern void	monitor_task(void const *argument);
extern void buzzer_warn(uint8_t num_set, uint16_t interval, uint16_t psc, uint16_t pwm);
#ifdef __cplusplus
}
#endif



#endif



