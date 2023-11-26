#ifndef __COMMUNICATE_TASK_H
#define __COMMUNICATE_TASK_H

#include "user_lib.h"
#include "vision.h"
#include "gimbal_task.h"


#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus
typedef struct
{
    float YawGet_KF;
    float YawTarget_KF;

    float PitchGet_KF;
    float PitchTarget_KF;

    float DistanceGet_KF;
    float DistanceTarget_KF;
} Visual_TypeDef;

class Vision_process_t
{
	public:
		QueueObj speed_queue;
    QueueObj accel_queue;
    QueueObj dis_queue;
    Visual_TypeDef  data_kal;
	  
	  float predict_angle;
    float feedforwaurd_angle;
	  float speed_get;       
    float accel_get;
    float distend_get;
	  
	  float Yaw_error;
	  float Pitch_error;//实际视觉给定角度
	  float YawTarget_now;
	  float PitchTarget_now;//实际视觉给定角度
	  float update_cloud_yaw ;
	  float update_cloud_pitch;	/*记录视觉更新数据时的云台数据，给下次接收用*/
    float lastupdate_cloud_yaw;
	  float lastupdate_cloud_pitch; /*前两帧的数据*/
	
	  //UI葡萄射手
		fp32 fx,fy;  //相机焦距参数
	  fp32 cx,cy;  //相机光心——图像中心坐标（960，540）
	  fp32 x_value,y_value,z_value;
	  fp32 predict_x_value,predict_y_value,predict_z_value;
		fp32 f;
		uint16_t vision_mode;
	  Vision_process_t();
    
		void Ui_follow_calc();
	  void Transmit_info();
	  void Get_info();
	  void Pridict_info();
		void Control();
};
	
extern Vision_process_t *Vision_process_point(void);

#endif
extern void communi_task(void const *pvParameters);
#ifdef __cplusplus
}	
#endif

#endif
