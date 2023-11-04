#ifndef __VISION_H
#define __VISION_H

#include "struct_typedef.h"
#include "crc_check.h"
#include "INS_task.h"
#include "arm_math.h"
#include "gimbal_task.h"

/*--------------------------------暂定协议-------------------------------------*/

#define    VISION_LENGTH        22     		 //暂定22字节,头3字节,数据17字节,尾2字节

//起始字节,协议固定为0xA5
#define    VISION_SOF         (0xA5)
#define    VISION_SENTRY1_SOF         (0xB5)
#define    VISION_SENTRY2_SOF         (0xC5)

//长度根据协议定义,数据段长度为n需要根据帧头第二字节来获取
#define    VISION_LEN_HEADER    3         //帧头长
#define    VISION_LEN_DATA      59       //数据段长度
#define    VISION_LEN_TAIL      2	      //帧尾CRC16
#define    VISION_LEN_PACKED    64        //数据包长度

#define    VISION_OFF         		(0x00)
#define    VISION_RED           	(0x01)
#define    VISION_BLUE          	(0x02)
#define    VISION_RBUFF_ANTI   	 	(0x03)//红逆大符
#define    VISION_BBUFF_ANTI   		(0x04)//蓝逆大符
#define    VISION_RBUFF_CLOCKWISE   (0x05)//红顺大符
#define    VISION_BBUFF_CLOCKWISE   (0x06)//蓝顺大符
#define    VISION_RBUFF_STAND   	(0x07)//红小符
#define    VISION_BBUFF_STAND   	(0x08)//蓝小符

#define NOW  0
#define LAST 1


#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus

	typedef struct
{
	uint8_t   SOF;			//帧头起始位,暂定0xA5
	uint8_t   CmdID;		//指令
	uint8_t   CRC8;			//帧头CRC校验,保证发送的指令是正确的
}extVisionSendhand_t;
//STM32发送
typedef struct
{
		/* 数据 */
	float    INS_quat_send[4];
	float    INS_gyro_send[3];
	float    INS_accel_send[3];
	float    revolver_speed;
  float    firc_error;
//
	//float    temp;
  //float    firc_speed;


	uint16_t   Vision_empty_time;		//预留
	
	/* 尾 */
	uint16_t  CRC16;
	
}extVisionSendDataFir_t;


typedef struct
{
	/* 数据 */
	float		robots_position[14];
	/* 尾 */
	uint16_t  CRC16;
	
}extVisionSendDataSed_t;


typedef struct
{
	/* 数据 */
	float		robots_position[6];
  uint16_t robots_hp[10];
	uint16_t stage_remain_time;
	/* 尾 */
	uint16_t  CRC16;
	
}extVisionSendDataThi_t;



//STM32接收
typedef __packed struct
{
	/* 头 */
	uint8_t   SOF;			//帧头起始位,暂定0xA5
	uint8_t   CmdID;		//指令
	uint8_t   CRC8;			//帧头CRC校验,保证发送的指令是正确的
	
	/* 数据 */
	float     pitch_angle;
	float     yaw_angle;
	float     distance;			//距离
	uint8_t   is_switched;
	uint8_t   is_findtarget;
	uint8_t   is_spinning;
	uint8_t   is_predictl;
  uint8_t   is_shooting;//取决于视觉那边是否加入自动开火标志位
	float     x_info;  //三轴数据
	float     y_info;
	float     z_info;
	float     predict_x_info;  //预测三轴数据
	float     predict_y_info;
	float     predict_z_info;
//	uint8_t   is_middle;
	
	
	/* 尾 */
	uint16_t  CRC16;  
}extVisionRecvDataFir_t;	

typedef __packed struct
{
	/* 头 */
	uint8_t   SOF;			//帧头起始位,暂定0xA5
	uint8_t   CmdID;		//指令
	uint8_t   CRC8;			//帧头CRC校验,保证发送的指令是正确的
	
	/* 数据 */
	float     target_linear[3];
	float     target_angular[3];
	
	/* 尾 */
	uint16_t  CRC16;       
	
}extVisionRecvDataSed_t;	

typedef __packed struct
{
	/* 头 */
	uint8_t   SOF;			//帧头起始位,暂定0xA5
	uint8_t   CmdID;		//指令
	uint8_t   CRC8;			//帧头CRC校验,保证发送的指令是正确的
	
	/* 数据 */
	float     theta_gimbal;
	float     theta_chassis;
	
	/* 尾 */
	uint16_t  CRC16;       
	
}extVisionRecvDataThi_t;	
	
//计算帧率
typedef struct
{
	bool		    rx_data_update;		// 接收数据是否更新
	uint32_t 		rx_time_prev;		// 接收数据的前一时刻
	uint32_t 		rx_time_now;		// 接收数据的当前时刻
	uint16_t 		rx_time_fps;		// 帧率

} Vision_State_t;

class vision_info_t
{
	public:
		extVisionRecvDataFir_t   RxPacketFir;
	  extVisionRecvDataSed_t   RxPacketSed;
	  extVisionRecvDataThi_t   RxPacketThi;
	
    extVisionSendDataFir_t   TxPacketFir;  //陀螺仪，弹速，yaw
	  extVisionSendDataSed_t   TxPacketSed;	 //机器人位置（哨兵）
    extVisionSendDataThi_t   TxPacketThi;
	
	  extVisionSendhand_t TxHandFir;
	  extVisionSendhand_t TxHandSed;
	  extVisionSendhand_t TxHandThi;
	
	  Vision_State_t Fir_State;
	  Vision_State_t Sed_State;
	  Vision_State_t Thi_State;
	  uint16_t Vision_empty_time;
	
	  void Vision_Send_Fir_Data(uint8_t CmdID );
	  void Vision_Send_Sed_Data(uint8_t CmdID);
	  void Vision_Send_Thi_Data(uint8_t CmdID);
	
	  float Vision_Error_Angle_Yaw(float *error);
	  float Vision_Error_Angle_Pitch(float *error);
	  void Vision_Get_Distance(float *distance);
		void Vision_Get_coordinate_system(float *x_axis,float *y_axis,float *z_axis);
	  void Vision_Get_predict_coordinate(float *predict_x,float *predict_y,float *predict_z);
};	
	
vision_info_t* vision_info_point(void);	
	
#endif
extern  void Vision_Read_Data(uint8_t *ReadFromUsart);
#ifdef __cplusplus
}	
#endif

#endif
