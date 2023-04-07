#ifndef __JUDGE_H
#define __JUDGE_H


#include "struct_typedef.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "stdbool.h"
#define    FALSE    0
#define    TRUE     1
#define    BLUE     0
#define    RED      1
//2021裁判系统官方接口协议
//通信方式是串口，配置为波特率 115200， 8 位数据位， 1 位停止位，无硬件流控，无校验位
//命令码 0x0001~ 0x020A  官方下发比赛数据
//命令码 0x0301 机器人间交互数据 可进行机器人间数据交互 ui绘制
//命令码 0x0302 自定义控制器交互数据接口
//命令码 0x0303 客户端小地图交互数据 云台手通过客户端向机器人下发信息
//命令码 0x0304 键盘、鼠标信息，通过图传串口发送
//命令码 0x0305 客户端小地图接收信息  接收雷达发送的坐标信息

//长度根据协议定义,数据段长度为n需要根据帧头第二字节来获取
#define    LEN_HEADER    5        //帧头长
#define    LEN_CMDID     2        //命令码长度
//#define    LEN_DATA      32        //数据长度
#define    LEN_TAIL      2	      //帧尾CRC16

//起始字节,协议固定为0xA5
#define    JUDGE_FRAME_HEADER         (0xA5)

#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus
//通信协议偏移位置
typedef enum 
{
	FRAME_HEADER         = 0,
	CMD_ID               = 5,
	DATA                 = 7,
	
}JudgeFrameOffset;


//5字节帧头,偏移位置
typedef enum
{
	SOF          = 0,//起始位
	DATA_LENGTH  = 1,//帧内数据长度,根据这个来获取数据长度
	SEQ          = 3,//包序号
	CRC8         = 4 //CRC8
}FrameHeaderOffset;


typedef __packed struct
{
	uint8_t  SOF;
	uint16_t DataLength;
	uint8_t  Seq;
	uint8_t  CRC8;
	
} xFrameHeader;


enum judge_robot_ID{
	hero_red       = 1,
	engineer_red   = 2,
	infantry3_red  = 3,
	infantry4_red  = 4,
	infantry5_red  = 5,
	plane_red      = 6,
	
	hero_blue      = 101,
	engineer_blue  = 102,
	infantry3_blue = 103,
	infantry4_blue = 104,
	infantry5_blue = 105,
	plane_blue     = 106,
};

/***************命令码ID********************/

/* 
   ID: 0x0001  Byte:  11    比赛状态数据，                  1Hz 周期发送      
   ID: 0x0002  Byte:  1     比赛结果数据，                  比赛结束后发送      
   ID: 0x0003  Byte:  32//原28    比赛机器人血量数据，            1Hz 周期发送  
   ID: 0x0004  Byte:  3     飞镖发射状态，                  飞镖发射后发送
   ID: 0x0005  Byte:  11    人工智能挑战赛加成与惩罚区状态   1Hz 周期发送 
   ID: 0X0101  Byte:  4    场地事件数据，                  1Hz 周期发送 
   ID: 0X0102  Byte:  3    场地补给站动作标识数据，        动作发生后发送
   ID: 0x0103  Byte:  2    	请求补给站补弹数据	          由参赛队发送，上限 10Hz。（RM 对抗赛尚未开放）   
   ID: 0X0104  Byte:  3//原2    裁判警告数据，                  警告发生后发送        
   ID: 0x0105  Byte:  1    飞镖发射口倒计时，              1Hz 周期发送


   ID: 0x0202  Byte:  16//原14   实时功率热量数据，              50Hz 周期发送
   ID: 0x0203  Byte:  16   机器人位置数据，                10Hz 发送
   ID: 0x0204  Byte:  1    机器人增益数据，                1Hz 周期发送
   ID: 0x0205  Byte:  3    空中机器人能量状态数据，        10Hz 周期发送，只有空中机器人主控发送
   ID: 0x0206  Byte:  1    伤害状态数据，                  伤害发生后发送
   ID: 0x0207  Byte:  7//原6    实时射击数据，                  子弹发射后发送
	 ID: 0x0208  Byte: 6//原2    弹丸剩余发射数，仅空中机器人，哨兵机器人以及 ICRA 机器人发送  1Hz 周期发送
	 ID: 0x0209  Byte:  4    机器人 RFID 状态，              1Hz 周期发送
	 ID: 0x020A  Byte:  12   飞镖机器人客户端指令数据，      10Hz 周期发送
	 ID: 0x0301  Byte:  n    机器人间交互数据，              发送方触发发送，
	 ID: 0x0302  Byte:  n    自定义控制器交互数据接口,      通过客户端触发发送，上限 30Hz
	 ID: 0x0303  Byte:  15   客户端小地图交互数据,  		   触发发送
	 ID: 0x0304  Byte:  12	 键盘、鼠标信息,					通过图传串口发送
	 ID: 0X0305  Byte:  10	 客户端小地图接收信息		最大接收频率： 10Hz 雷达站发送的坐标信息可以被所有己方操作手在第一视角小地图看到。	

*/


//命令码ID,用来判断接收的是什么数据
typedef enum
{ 
	ID_game_status                 = 0x0001,//比赛状态数据,1Hz周期发送
	ID_game_result 	               = 0x0002,//比赛结果数据，比赛结束后发送
	ID_robot_HP                    = 0x0003,//比赛机器人血量数据,1Hz周期发送
	ID_dart_status                 = 0x0004,//飞镖发射状态，飞镖发射时发送
	ID_ICRA_buff_debuff_zone_status= 0x0005,
	ID_event_data                  = 0x0101,//场地事件数据,1Hz 周期发送 
	ID_supply_projectile_action    = 0x0102,//场地补给站动作标识数据,动作发生后发送
	ID_referee_warning             = 0x0104,//裁判警告数据，警告发生后发送 
	ID_dart_remaining_time         = 0x0105,//飞镖发射口倒计时，1Hz周期发送
  ID_robot_status                = 0x0201,//机器人状态数据,10Hz周期发送 
	ID_heat_data                   = 0x0202,//实时功率热量数据,50Hz周期发送 
	ID_robot_pos                   = 0x0203,//机器人位置数据,10Hz发送
	ID_buff                        = 0x0204,//机器人增益数据,1Hz 周期发送 
	ID_aerial_robot_energy         = 0x0205,//空中机器人能量状态数据,10Hz周期发送,只有空中机器人主控发送
	ID_robot_hurt                  = 0x0206,//伤害状态数据,，伤害发生后发送
	ID_shoot_data                  = 0x0207,//实时射击数据,子弹发射后发送 
	ID_bullet_remaining            = 0x0208,//弹丸剩余发射数，仅空中机器人，哨兵机器人以及 ICRA 机器人发送，1Hz 周期发送
	ID_rfid_status                 = 0x0209,//机器人 RFID 状态，1Hz 周期发送 
	ID_dart_client_cmd             = 0x020A,//飞镖机器人客户端指令数据，10Hz 周期发送 
	ID_robot_interactive_header_data      = 0x0301, //机器人间交互数据，发送方触发发送
	ID_customize_controller_interaction_data_interface     = 0x0302, //自定义控制器交互数据接口，通过客户端触发发送，上限 30Hz
  ID_minimap_interactive_data    = 0x0303, //客户端小地图交互数据，触发发送
	ID_keybord_and_mouse_massage   = 0x0304, //键盘、鼠标信息，通过图传串口发送
	ID_client_map_command     	   = 0x0305, //雷达站发送的坐标信息可以被所有己方操作手在第一视角小地图看到

} CmdID;

//命令码数据段长,根据官方协议来定义长度
typedef enum
{
		/* Std */
	LEN_FRAME_HEAD 	                 = 5,	// 帧头长度
	LEN_CMD_ID 		                   = 2,	// 命令码长度
	LEN_FRAME_TAIL 	                 = 2,	// 帧尾CRC16
	
	LEN_game_status                  =  11,	//0x0001
	LEN_game_result                  =  1,	//0x0002
	LEN_robot_HP                     =  32,	//0x0003 28
	LEN_dart_status                  =  3,	//0x0004
	LEN_ICRA_buff_debuff_zone_status =  11, //0x0005
	LEN_event_data                   =  4,	//0x0101
	LEN_supply_projectile_action     =  4,	//0x0102   3
	LEN_referee_warning              =  3,	//0x0104   2
	LEN_dart_remaining_time          =  1,	//0x0105
	LEN_robot_status                 = 27,	//0x0201
	LEN_heat_data                    = 16,	//0x0202  14
	LEN_robot_pos                    = 16,	//0x0203
	LEN_buff                         =  1,	//0x0204
	LEN_aerial_robot_energy          =  3,	//0x0205    
	LEN_robot_hurt                   =  1,	//0x0206  
	LEN_shoot_data                   =  7,	//0x0207  
	LEN_bullet_remaining             =  6,	//0x0208  2
	LEN_rfid_status                  =  4,	//0x0209 
	LEN_dart_client_cmd              = 12,	//0x020A 
	LEN_minimap_interactive_data 	 = 15,   //0x0303
	LEN_keybord_and_mouse_massage 	 = 12,   //0x0304
	LEN_client_map_command    		 = 10,  //0x0305
	
} JudgeDataLength;



/*比赛状态数据： 0x0001      发送范围：所有机器人。*/
typedef __packed struct
{
//语法：在自定义的变量后面加上：数字 表明该变量所占的位数
uint8_t game_type : 4;
uint8_t game_progress : 4;
uint16_t stage_remain_time;
	
uint64_t SyncTimeStamp;	
} ext_game_status_t;
/* 0-3 bit： 比赛类型
	 1： RoboMaster 机甲大师赛；
	 2： RoboMaster 机甲大师单项赛；
	 3： ICRA RoboMaster 人工智能挑战赛
	 4： RoboMaster 联盟赛 3V3
	 5： RoboMaster 联盟赛 1V1
	 4-7 bit： 当前比赛阶段
	 0： 未开始比赛；
	 1： 准备阶段；
	 2： 自检阶段；
	 3： 5s 倒计时；
	 4： 对战中；
   5： 比赛结算中
*/

/*比赛结果数据： 0x0002     发送范围：所有机器人。*/
typedef __packed struct
{
uint8_t winner;
} ext_game_result_t;

/*机器人血量数据 0x0003     发送范围：所有机器人。*/
typedef __packed struct
{
uint16_t red_1_robot_HP;
uint16_t red_2_robot_HP;
uint16_t red_3_robot_HP;
uint16_t red_4_robot_HP;
uint16_t red_5_robot_HP;
uint16_t red_7_robot_HP;
uint16_t red_outpost_HP;
uint16_t red_base_HP;
uint16_t blue_1_robot_HP;
uint16_t blue_2_robot_HP;
uint16_t blue_3_robot_HP;
uint16_t blue_4_robot_HP;
uint16_t blue_5_robot_HP;
uint16_t blue_7_robot_HP;
uint16_t blue_outpost_HP;
uint16_t blue_base_HP;
} ext_game_robot_HP_t;

/*飞镖发射状态： 0x0004          发送范围：所有机器人。*/
typedef __packed struct
{
uint8_t dart_belong;
uint16_t stage_remaining_time;
} ext_dart_status_t;


/*人工智能挑战赛加成与惩罚区状态： 0x0005   发送范围：所有机器人。*/
typedef __packed struct
{
uint8_t F1_zone_status:1;
uint8_t F1_zone_buff_debuff_status:3;
uint8_t F2_zone_status:1;
uint8_t F2_zone_buff_debuff_status:3;
uint8_t F3_zone_status:1;
uint8_t F3_zone_buff_debuff_status:3;
uint8_t F4_zone_status:1;
uint8_t F4_zone_buff_debuff_status:3;
uint8_t F5_zone_status:1;
uint8_t F5_zone_buff_debuff_status:3;
uint8_t F6_zone_status:1;
uint8_t F6_zone_buff_debuff_status:3;
uint16_t red1_bullet_left;
uint16_t red2_bullet_left;
uint16_t blue1_bullet_left;
uint16_t blue2_bullet_left;
} ext_ICRA_buff_debuff_zone_status_t;

/*场地事件数据： 0x0101      发送范围： 己方机器人*/
typedef __packed struct
{
uint32_t event_type;
} ext_event_data_t;

/*补给站动作标识： 0x0102                发送范围：己方机器人*/
typedef __packed struct
{
uint8_t supply_projectile_id;
uint8_t supply_robot_id;
uint8_t supply_projectile_step;
uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

/*裁判警告信息： cmd_id      0x0104         发送范围：己方机器人*/
typedef __packed struct
{
uint8_t level;
uint8_t foul_robot_id;
} ext_referee_warning_t;

/*飞镖发射口倒计时： cmd_id   0x0105      发送范围：己方机器人*/
typedef __packed struct
{
uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;

/*比赛机器人状态： 0x0201                  发送范围：单一机器人*/
typedef __packed struct
{
uint8_t robot_id;
uint8_t robot_level;
uint16_t remain_HP;
uint16_t max_HP;
uint16_t shooter_id1_17mm_cooling_rate;
uint16_t shooter_id1_17mm_cooling_limit;
uint16_t shooter_id1_17mm_speed_limit;
uint16_t shooter_id2_17mm_cooling_rate;
uint16_t shooter_id2_17mm_cooling_limit;
uint16_t shooter_id2_17mm_speed_limit;
uint16_t shooter_id1_42mm_cooling_rate;
uint16_t shooter_id1_42mm_cooling_limit;
uint16_t shooter_id1_42mm_speed_limit;
uint16_t chassis_power_limit;
uint8_t mains_power_gimbal_output : 1;
uint8_t mains_power_chassis_output : 1;
uint8_t mains_power_shooter_output : 1;
} ext_game_robot_status_t;

/*实时功率热量数据： 0x0202                发送范围：单一机器人*/
typedef __packed struct
{
uint16_t chassis_volt;
uint16_t chassis_current;
float chassis_power;
uint16_t chassis_power_buffer;
uint16_t shooter_id1_17mm_cooling_heat;
uint16_t shooter_id2_17mm_cooling_heat;
uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;

/*机器人位置： 0x0203                     发送范围：单一机器人。*/
typedef __packed struct
{
float x;
float y;
float z;
float yaw;
} ext_game_robot_pos_t;

/*机器人增益： 0x0204                      发送范围：单一机器人。*/
typedef __packed struct
{
uint8_t power_rune_buff;
}ext_buff_t;

/*空中机器人能量状态： 0x0205              发送范围：单一机器人*/
typedef __packed struct
{
uint8_t attack_time;
} aerial_robot_energy_t;

/*伤害状态： 0x0206。                      发送范围：单一机器人。*/
typedef __packed struct
{
uint8_t armor_id : 4;
uint8_t hurt_type : 4;
} ext_robot_hurt_t;

/*实时射击信息： 0x0207                    发送范围：单一机器人。*/
typedef __packed struct
{
uint8_t bullet_type;
uint8_t shooter_id;
uint8_t bullet_freq;
float bullet_speed;
} ext_shoot_data_t;

/*子弹剩余发射数： 0x0208                  发送范围：单一机器人。空中机器人，哨兵机器人以及 ICRA 机器人,主控发送*/
typedef __packed struct
{
uint16_t bullet_remaining_num_17mm;
uint16_t bullet_remaining_num_42mm;
uint16_t coin_remaining_num;
} ext_bullet_remaining_t;


/*机器人 RFID 状态： 0x0209                发送范围：单一机器人*/
typedef __packed struct
{
uint32_t rfid_status;
} ext_rfid_status_t;

/*飞镖机器人客户端指令数据： 0x020A        发送范围：单一机器人*/
typedef __packed struct
{
uint8_t dart_launch_opening_status;
uint8_t dart_attack_target;
uint16_t target_change_time;
uint16_t operate_launch_cmd_time;
} ext_dart_client_cmd_t;



/*机器人间交互数据   0x0301*/

/* 
	交互数据包括一个统一的数据段头结构。
  数据段包含了内容ID，发送者以及接收者的ID和内容数据段，
  整个交互数据的包总共长最大为 128 个字节，
  减去 frame_header,cmd_id 和 frame_tail 共 9 个字节
  以及数据段头结构的 6 个字节，故而发送的内容数据段最大为 113

	机器人 ID：                                                                                                                                                                                                                                                                                                                                             
	1，英雄(红)；
	2，工程(红)；
	3/4/5，步兵(红)；
	6，空中(红)；
	7，哨兵(红)；
  9，雷达站（红）
	101，英雄(蓝)；
	102，工程(蓝)；
	103/104/105，步兵(蓝)；
	106，空中(蓝)；
	107，哨兵(蓝)：
  109，雷达站（蓝）	
	
	客户端 ID： 
	0x0101 为英雄操作手客户端( 红) ；
	0x0102 ，工程操作手客户端 ((红 )；
	0x0103/0x0104/0x0105，步兵操作手客户端(红)；
	0x0106，空中操作手客户端((红)； 
	0x0165，英雄操作手客户端(蓝)；
	0x0166，工程操作手客户端(蓝)；
	0x0167/0x0168/0x0169，操作手客户端步兵(蓝)；
	0x016A，空中操作手客户端(蓝)。 
*/

//数据内容id
enum
{
	//0x200-0x02ff 	队伍自定义命令 格式  INTERACT_ID_XXXX
	INTERACT_ID_delete_graphic 			= 0x0100,	/*客户端删除图形*/
	INTERACT_ID_draw_one_graphic 		= 0x0101,	/*客户端绘制一个图形*/
	INTERACT_ID_draw_two_graphic 		= 0x0102,	/*客户端绘制2个图形*/
	INTERACT_ID_draw_five_graphic 	= 0x0103,	/*客户端绘制5个图形*/
	INTERACT_ID_draw_seven_graphic 	= 0x0104,	/*客户端绘制7个图形*/
	INTERACT_ID_draw_char_graphic 	= 0x0110,	/*客户端绘制字符图形*/
	INTERACT_ID_bigbome_num					= 0x02ff
};

/* 交互数据段头  */
typedef __packed struct
{
uint16_t data_cmd_id;
uint16_t send_ID;
uint16_t receiver_ID;
}ext_student_interactive_header_data_t;

/*客户端删除图形 */
typedef __packed struct
{
uint8_t operate_type;
uint8_t layer;
} ext_client_custom_graphic_delete_t;

/*图形数据*/
typedef __packed struct
{
uint8_t graphic_name[3]; 		 //图形名
uint32_t operate_tpye:3;
uint32_t graphic_tpye:3;
uint32_t layer:4;
uint32_t color:4;
uint32_t start_angle:9;
uint32_t end_angle:9;
uint32_t width:10;
uint32_t start_x:11;
uint32_t start_y:11;
uint32_t radius:10;
uint32_t end_x:11;
uint32_t end_y:11;
} graphic_data_struct_t;


/*绘制图形类型*/
typedef enum
{
	LINE      = 0,//直线
	RECTANGLE = 1,//矩形
	CIRCLE    = 2,//正圆
	OVAL      = 3,//椭圆
	ARC       = 4,//圆弧
	FLOAT     = 5,//浮点数
	INT       = 6,//整型数
	CHAR      = 7 //字符
}Graphic_Type;

/*绘制字符串*/
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct;
	uint8_t data[30];
} ext_client_string_t;

/*对图形进行操作*/
typedef enum
{
	NONE   = 0,/*空操作*/
	ADD    = 1,/*增加图层*/
	MODIFY = 2,/*修改图层*/
	DELETE = 3,/*删除图层*/
}Graphic_Operate;

/*图形颜色*/
typedef enum
{
	RED_BLUE  = 0,//红蓝主色	
	YELLOW    = 1,
	GREEN     = 2,
	ORANGE    = 3,
	FUCHSIA   = 4,	/*紫红色*/
	PINK      = 5,
	CYAN_BLUE = 6,	/*青色*/
	BLACK     = 7,
	WHITE     = 8
}Graphic_Color;

/*绘制浮点数*/
typedef __packed struct
{                          
	uint8_t graphic_name[3]; 
	uint32_t operate_tpye:3; 
	uint32_t graphic_tpye:3; 
	uint32_t layer:4;        
	uint32_t color:4;        
	uint32_t start_angle:9;  
	uint32_t end_angle:9;    
	uint32_t width:10;       
	uint32_t start_x:11;    
	uint32_t start_y:11;     
  float number;       
} Float_data_struct_t;

/*绘制整型数*/
typedef __packed struct
{                          
	uint8_t graphic_name[3]; 
	uint32_t operate_tpye:3; 
	uint32_t graphic_tpye:3; 
	uint32_t layer:4;        
	uint32_t color:4;        
	uint32_t start_angle:9;  
	uint32_t end_angle:9;    
	uint32_t width:10;       
	uint32_t start_x:11;    
	uint32_t start_y:11;     
  int number;       
} Int_data_struct_t;

/*  以上为数据段结构体
    以下为完整数据包（帧头+命令码+数据段头结构+数据段+帧尾）
*/

//接收机器人交互信息 
typedef __packed struct
{
//	xFrameHeader   					txFrameHeader;//帧头
//	uint16_t								CmdID;//命令码
//	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
//	robot_interactive_data_t  interactData;//数据段
//	uint16_t		 						FrameTail;//帧尾
	uint8_t data[10];
}ext_CommunatianData_t;

//发送机器人交互信息
typedef __packed struct
{
	xFrameHeader   					txFrameHeader;//帧头
	uint16_t								CmdID;//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	uint8_t data[10];         //数据段
	uint16_t		 						FrameTail;//帧尾
}ext_Communatian_send_Data_t;

//固定数据段长度数据包
typedef __packed struct
{
	xFrameHeader txFrameHeader;			//帧头
	uint16_t  CmdID;										//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	ext_client_string_t clientData;//数据段
	uint16_t	FrameTail;								//帧尾
}ext_charstring_data_t;

//删除图层
typedef __packed struct
{
	xFrameHeader txFrameHeader;			
	uint16_t  CmdID;										
	ext_student_interactive_header_data_t   dataFrameHeader;
	ext_client_custom_graphic_delete_t clientData;		
	uint16_t	FrameTail;								
}ext_deleteLayer_data_t;

//客户端绘制一个图形
typedef __packed struct
{
	xFrameHeader txFrameHeader;			//帧头
	uint16_t  CmdID;										//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	graphic_data_struct_t clientData;		//数据段
	uint16_t	FrameTail;								//帧尾
}ext_graphic_one_data_t;

//客户端绘制两个图形
typedef __packed struct
{
	xFrameHeader txFrameHeader;			
	uint16_t  CmdID;										
	ext_student_interactive_header_data_t   dataFrameHeader;
	graphic_data_struct_t clientData[2];		
	uint16_t	FrameTail;								
}ext_graphic_two_data_t;

//客户端绘制五个图形
typedef __packed struct
{
	xFrameHeader txFrameHeader;			
	uint16_t  CmdID;										
	ext_student_interactive_header_data_t   dataFrameHeader;
	graphic_data_struct_t clientData[5];		
	uint16_t	FrameTail;								
}ext_graphic_five_data_t;

//客户端绘制七个图形
typedef __packed struct
{
	xFrameHeader txFrameHeader;			
	uint16_t  CmdID;										
	ext_student_interactive_header_data_t   dataFrameHeader;
	graphic_data_struct_t clientData[7];		
	uint16_t	FrameTail;								
}ext_graphic_seven_data_t;

//客户端绘制两个浮点数
typedef __packed struct
{
	xFrameHeader txFrameHeader;			
	uint16_t  CmdID;										
	ext_student_interactive_header_data_t   dataFrameHeader;
	Float_data_struct_t clientData[2];		
	uint16_t	FrameTail;								
}ext_float_two_data_t;

//客户端绘制两个整型数
typedef __packed struct
{
	xFrameHeader txFrameHeader;			
	uint16_t  CmdID;										
	ext_student_interactive_header_data_t   dataFrameHeader;
	Int_data_struct_t clientData[2];		
	uint16_t	FrameTail;								
}ext_int_two_data_t;

/*以上为0x0301有关结构体*/

/*
	自定义控制器数据包括一个统一的数据段头结构。数据段为内容数据段，整个交互数据的包总共长最大为
	39 个字节，减去 frame_header,cmd_id 和 frame_tail 共 9 个字节，故而发送的内容数据段最大为 30 字
	节。整个交互数据 0x0302 的包下行行频率为 30Hz。
*/

//交互数据接收信息： 0x0302。 发送频率：上限 30Hz
//typedef __packed struct
//{
//uint8_t data[9];
//} robot_interactive_data_t;


/*小地图交互信息标识： 0x0303。 发送频率：触发时发送。
		客户端下发信息
		机器人 ID： 机器人 ID： 1，英雄(红)； 2，工程(红)； 3/4/5，步兵(红)； 6，空中(红)； 7，哨兵(红)； 9，雷
		达站（红）； 10，前哨站（红）； 11，基地（红）； 101，英雄(蓝)； 102，工程(蓝)； 103/104/105，步兵
		(蓝)； 106，空中(蓝)； 107，哨兵(蓝)； 109，雷达站（蓝）； 110，前哨站（蓝）； 111，基地（蓝） 。
*/
typedef __packed struct
{
float target_position_x;
float target_position_y;
float target_position_z;
uint8_t commd_keyboard;
uint16_t target_robot_ID;
} ext_robot_command_t;

/*图传遥控信息标识： 0x0304。发送频率： 30Hz。
   客户端下发信息
   字节偏移量      大小          		说明
	  0			   2			鼠标 X 轴信息
	  2 		   2			鼠标 Y 轴信息
	  4			   2			鼠标滚轮信息
	  6			   1			鼠标左键
	  7			   1     		鼠标右键按下
	  8			   2			键盘信息
		bit 0：键盘 W 是否按下
		bit 1：键盘 S 是否按下
		bit 2：键盘 A 是否按下
		bit 3：键盘 D 是否按下
		bit 4：键盘 SHIFT 是否按下
		bit 5：键盘 CTRL 是否按下
		bit 6：键盘 Q 是否按下
		bit 7：键盘 E 是否按下
		bit 8：键盘 R 是否按下
		bit 9：键盘 F 是否按下
		bit 10：键盘 G 是否按下
		bit 11：键盘 Z 是否按下
		bit 12：键盘 X 是否按下
		bit 13：键盘 C 是否按下
		bit 14：键盘 V 是否按下
		bit 15：键盘 B 是否按下
*/

//自己更改过结构体名称	原   ext_robot_command_t;
typedef __packed struct
{
int16_t mouse_x;
int16_t mouse_y;
int16_t mouse_z;
int8_t left_button_down;
int8_t right_button_down;
uint16_t keyboard_value;
uint16_t reserved;
} ext_robot_command_t_change;

/*小地图接收信息标识： 0x0305。 最大接收频率： 10Hz。
	雷达站发送的坐标信息可以被所有己方操作手在第一视角小地图看到
*/
typedef __packed struct
{ 
uint16_t target_robot_ID;
float target_position_x;
float target_position_y;
} ext_client_map_command_t;

/**********************以上为裁判系统手册提及，使用的结构体**************************************************/

//机器人id号信息
typedef struct{
	uint16_t teammate_hero;
	uint16_t teammate_engineer;
	uint16_t teammate_infantry3;
	uint16_t teammate_infantry4;
	uint16_t teammate_infantry5;
	uint16_t teammate_plane;
	uint16_t teammate_sentry;
	
	uint16_t client_hero;
	uint16_t client_engineer;
	uint16_t client_infantry3;
	uint16_t client_infantry4;
	uint16_t client_infantry5;
	uint16_t client_plane;
} ext_interact_id_t;

//记录裁判系统数据
typedef struct judge_info_struct {
	xFrameHeader							FrameHeader;				// 帧头信息
	
	ext_game_status_t 							GameState;				// 0x0001           比赛状态数据
	ext_game_result_t 							GameResult;				// 0x0002         比赛结果数据
	ext_game_robot_HP_t 						GameRobotHP;			// 0x0003         机器人血量数据
	ext_dart_status_t								GameRobotmissile;				// 0x0004         飞镖发射状态
	ext_ICRA_buff_debuff_zone_status_t	Game_ICRA_buff;      //                人工智能挑战赛加成与惩罚区状态
	
	ext_event_data_t								EventData;					// 0x0101         场地事件数据
	ext_supply_projectile_action_t	SupplyProjectileAction;		// 0x0102 补给站动作标识
	ext_referee_warning_t						RefereeWarning;		// 0x0104         裁判警告信息
	ext_dart_remaining_time_t				dart_remaining_time;// 0x0105         飞镖发射口倒计时
	
	ext_game_robot_status_t					GameRobotStat;	// 0x0201         比赛机器人状态
	ext_power_heat_data_t						PowerHeatData;		// 0x0202         实时功率热量数据
	ext_game_robot_pos_t						GameRobotPos;			// 0x0203         机器人位置
	ext_buff_t											Buff;								// 0x0204     机器人增益
	aerial_robot_energy_t				    AerialRobotEnergy;// 0x0205             空中机器人能量状态
	ext_robot_hurt_t								RobotHurt;					// 0x0206         伤害状态
	ext_shoot_data_t								ShootData;					// 0x0207         实时射击信息(射频  射速  子弹信息)
	ext_bullet_remaining_t					bullet_remaining;		// 0x0208	        子弹剩余发射数
	ext_rfid_status_t								rfid_status;				// 0x0209	        RFID信息
	ext_dart_client_cmd_t           dart_client;        // 0x020A         飞镖客户端
	
	ext_interact_id_t								ids;								//与本机交互的机器人id
	uint16_t                        self_client;        //本机客户端
	bool	 		IF_REF_ONL;

} Referee_info_t;



  bool Judge_Read_Data(uint8_t *ReadFormUsart);//主循环4ms调用一次
	void JUDGE_Show_Data(void);
	void Send_to_Teammate(void);
	bool is_red_or_blue(void);
	void Determine_ID(void);//判断自己是哪个队伍
	bool Judge_If_Chassis(void);

	/*****裁判数据辅助判断函数********/
	bool JUDGE_sGetDataState(void);    //判断是否接受到裁判系统
	float JUDGE_fGetChassisPower(void);
	uint16_t JUDGE_fGetRemainEnergy(void);
	uint8_t JUDGE_ucGetRobotLevel(void);
	
	//各枪口热量读取枪口热量
	uint16_t JUDGE_usGetRemoteHeat_id1_17mm(void);
	uint16_t JUDGE_usGetRemoteHeat_id2_17mm(void);
	uint16_t JUDGE_usGetRemoteHeat_id1_42mm(void);
	
	//读取射速
	float JUDGE_usGetSpeedHeat(void);
	
	//统计发弹量
	//ATTENTION！！！！！！！！！！！！！双枪管可能会出错！！！！！！！！！！！！！！！
	void JUDGE_ShootNumCount_17mm(void);
	void JUDGE_ShootNumCount_42mm(void);
	//读取发弹量
	//ATTENTION！！！！！！！！！！！！！双枪管可能会出错！！！！！！！！！！！！！！！
	uint16_t JUDGE_usGetShootNum_17mm(void);
	uint16_t JUDGE_usGetShootNum_42mm(void);
	//发弹量清零
	//ATTENTION！！！！！！！！！！！！！双枪管可能会出错！！！！！！！！！！！！！！！
	void JUDGE_ShootNum_Clear_17mm(void);
	void JUDGE_ShootNum_Clear_42mm(void);
	//各枪口热量读取枪口热量上限
	uint16_t JUDGE_usGetHeatLimit_id1_17mm(void);
	uint16_t JUDGE_usGetHeatLimit_id2_17mm(void);
	uint16_t JUDGE_usGetHeatLimit_id1_42mm(void);
	//各枪口热量读取枪口热量每秒冷却值
	uint16_t JUDGE_usGetShootCold_id1_17mm(void);
	uint16_t JUDGE_usGetShootCold_id2_17mm(void);
	uint16_t JUDGE_usGetShootCold_id1_42mm(void);

	uint16_t JUDGE_usGetSpeedLimit(void);
	//补给站口
	uint8_t JUDGE_usGetSupply_Id(void);
	//补给机器人 ID
	uint8_t JUDGE_usGetSupply_Robo_Id(void);
	//出弹口开闭状态 
	uint8_t JUDGE_usGetSupply_Mode(void);
	//返回当前补弹数量（为字符型变量）
	uint8_t JUDGE_usGetSupply_Num(void);
	//小地图接收信息目标机器人 ID(雷达站)
	uint16_t JUDGE_usGetRadar_Station_to_robo_ID(void);
	//小地图接收信息目标机器人 x 位置坐标
	float JUDGE_usGetRadar_Station_to_robo_posX(void);
	//小地图接收信息目标机器人 y 位置坐标
	float JUDGE_usGetRadar_Station_to_robo_posy(void);

	//当前机器人血量
	uint16_t JUDGE_usGetRadar_Robo_HP(void);
	//当前机器人最大血量
	uint16_t JUDGE_usGetRadar_Robo_Max_HP(void);
	//17mm 子弹剩余发射数量
	uint16_t JUDGE_usGetRadar_Robo_remain_17mm(void);
	//shooter电源输出
	uint8_t JUDGE_usGetJudge_shooter_Power(void);
	float JUDGE_fGetChassisVolt(void);

	bool JUDGE_IfArmorHurt(void);
	bool Judge_If_Death(void);

	//底盘功率函数
	uint16_t JUDGE_usGetChassisPowerLimit(void);//获取当前等级底盘功率上限
	uint8_t JUDGE_ucRobotLevel(void);//获取机器人等级数据

	uint8_t Judge_armor_id(void);//装甲板受打击ID
	uint8_t Judge_hurt_mode(void);//伤害类型
	void JUDGE_Show_Graph(void); //上传客户端图形


	void Client_graphic_Init(void);
	void Client_graphic_Info_update(void);
	//英雄
	void Client_aim_line(void);//
	void _high_aim_(void);
	void _lowshort_aim_2(void);
	void _lowshort_aim_3(void);
	void _lowshortstem_aim_4(void);
	void _lowlong_aim_(void);

	void Client_aim_update(void);//准心
	void Client_supercapacitor_update(void);//超级电容
	void Client_gimbal_angle_update(void);//吊射角度
	void Client_bullet_int_update(void);//弹丸信息


#endif

#ifdef __cplusplus
}
#endif

#endif 
