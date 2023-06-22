#include "judge.h"
#include "ui_task.h"
#include "bsp_usart.h"
#include "cmsis_os.h"
#include "main.h"
#include "stdbool.h"
#include "string.h"
#include "crc8_crc16.h"
#include "revolver_task.h"
#include "usart.h"
#include "stdbool.h"
#include "stdio.h"


	ext_game_status_t                   Game_status;                 //0x0001;
	ext_game_result_t                   Game_result;                 //0x0002;
	ext_game_robot_HP_t                 Robot_HP;                    //0x0003
	ext_dart_status_t                   Dart_status;                 //0x0004
	ext_ICRA_buff_debuff_zone_status_t  ICRA_buff_debuff_zone_status;//0x0005
	ext_event_data_t                    Event_data;                  //0x0101
	ext_supply_projectile_action_t      Supply_projectile_action;    //0x0102
	ext_referee_warning_t               Referee_warning;             //0x0104
	ext_dart_remaining_time_t           Dart_remaining_time;         //0x0105
	ext_game_robot_status_t             Robot_status;                //0x0201
	ext_power_heat_data_t               Power_heat_data;             //0x0202
	ext_game_robot_pos_t                robot_position;              //0x0203
	ext_buff_t                          buff;                        //0x0204
	aerial_robot_energy_t         			Aerial_robot_energy;         //0x0205
	ext_robot_hurt_t                    Robot_hurt;                  //0x0206
	ext_shoot_data_t                    Shoot_data;                  //0x0207
	ext_bullet_remaining_t               Bullet_remaining;           //0x0208
	ext_rfid_status_t                   Rfid_status;                 //0x0209
	ext_dart_client_cmd_t               Dart_client_cmd;             //0x020A
	ext_CommunatianData_t           		CommunatianData_cmd;         //0x0301
																													         //0x0302 自定义遥控器
	ext_robot_command_t					        Minimap_robot_command;	     //0x0303
	ext_robot_command_t_change			    Transfer_image_robot_command;//0x0304
	ext_client_map_command_t			      Client_map_command_t;        //0x0305

	xFrameHeader                        FrameHeader;//发送帧头信息
  Referee_info_t                      REF;
	
	
/****************************************************/
bool Judge_Data_TF = FALSE;//裁判数据是否可用,辅助函数调用
uint8_t Judge_Self_ID;//当前机器人的ID
uint16_t Judge_SelfClient_ID;//发送者机器人对应的客户端ID

/**************裁判系统数据辅助****************/
uint16_t ShootNum_17mm;//统计发弹量,0x0003触发一次则认为发射了一颗
uint16_t ShootNum_42mm;//统计发弹量,0x0003触发一次则认为发射了一颗
bool Hurt_Data_Update = FALSE;//装甲板伤害数据是否更新,每受一次伤害置TRUE,然后立即置FALSE,给底盘闪避用




/**
  * @brief  读取裁判数据,loop中循环调用此函数来读取数据
  * @param  缓存数据
  * @retval 是否对正误判断做处理
  * @attention  在此判断帧头和CRC校验,无误再写入数据
  */
bool Judge_Read_Data(uint8_t *ReadFromUsart)
{
	bool retval_tf = FALSE;//数据正确与否标志,每次调用读取裁判系统数据函数都先默认为错误
	
	uint16_t judge_length;//统计一帧数据长度 
	
	int CmdID = 0;//数据命令码解析
	
	/***------------------*****/
	//无数据包，则不作任何处理
	if (ReadFromUsart == NULL)
	{
		return -1;
	}
	
	//写入帧头数据,用于判断是否开始存储裁判数据
	memcpy(&FrameHeader, ReadFromUsart, LEN_HEADER);
	memcpy(&REF.FrameHeader,ReadFromUsart,LEN_HEADER);   //储存帧头数据
	
		//判断帧头数据是否为0xA5
	if(ReadFromUsart[ SOF ] == JUDGE_FRAME_HEADER)
	{
		//帧头CRC8校验
		if (verify_CRC8_check_sum( ReadFromUsart, LEN_HEADER ) == TRUE)
		{
			//统计一帧数据长度,用于CR16校验
			judge_length = ReadFromUsart[ DATA_LENGTH ] + LEN_HEADER + LEN_CMDID + LEN_TAIL;
			
			//帧尾CRC16校验
			if(verify_CRC16_check_sum(ReadFromUsart,judge_length) == TRUE)
			{
				retval_tf = TRUE;//都校验过了则说明数据可用
				CmdID = (ReadFromUsart[6] << 8 | ReadFromUsart[5]);
				//解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
			
				switch(CmdID)
				{
					case ID_game_status:         //0x0001
						memcpy(&Game_status, (ReadFromUsart + DATA), LEN_game_status);
						memcpy(&REF.GameState, (ReadFromUsart + DATA), LEN_game_status);
					break;
					
					case ID_game_result:          //0x0002
						memcpy(&Game_result, (ReadFromUsart + DATA), LEN_game_result);		
						memcpy(&REF.GameResult, (ReadFromUsart + DATA), LEN_game_result);					
					break;
					
					case ID_robot_HP:             //0x0003
						memcpy(&Robot_HP, (ReadFromUsart + DATA), LEN_robot_HP);
						memcpy(&REF.GameRobotHP, (ReadFromUsart + DATA), LEN_robot_HP);
					break;
					
					case ID_dart_status:          //0x0004
						memcpy(&Dart_status, (ReadFromUsart + DATA), LEN_dart_status);
						memcpy(&REF.GameRobotmissile, (ReadFromUsart + DATA), LEN_dart_status);
					break;
					
					case ID_ICRA_buff_debuff_zone_status:          //0x0005
						memcpy(&ICRA_buff_debuff_zone_status, (ReadFromUsart + DATA), LEN_ICRA_buff_debuff_zone_status);
						memcpy(&REF.Game_ICRA_buff, (ReadFromUsart + DATA), LEN_ICRA_buff_debuff_zone_status);
					break;
					
					case ID_event_data:   //0x0101
						memcpy(&Event_data, (ReadFromUsart + DATA), LEN_event_data);
						memcpy(&REF.EventData, (ReadFromUsart + DATA), LEN_event_data);
					break;
					
					case ID_supply_projectile_action:      //0x0102
						memcpy(&Supply_projectile_action, (ReadFromUsart + DATA), LEN_supply_projectile_action);
						memcpy(&REF.SupplyProjectileAction, (ReadFromUsart + DATA), LEN_supply_projectile_action);
					break;
					
					case ID_referee_warning:      //0x0104
						memcpy(&Referee_warning, (ReadFromUsart + DATA), LEN_referee_warning);
						memcpy(&REF.RefereeWarning, (ReadFromUsart + DATA), LEN_referee_warning);
					break;
					
					case ID_dart_remaining_time:      //0x0105
						memcpy(&Dart_remaining_time, (ReadFromUsart + DATA), LEN_dart_remaining_time);
						memcpy(&REF.dart_remaining_time, (ReadFromUsart + DATA), LEN_dart_remaining_time);
					break;
					
					case ID_robot_status:      //0x0201
						memcpy(&Robot_status, (ReadFromUsart + DATA), LEN_robot_status);
						memcpy(&REF.GameRobotStat, (ReadFromUsart + DATA), LEN_robot_status);
						Determine_ID();
					break;
					
					case ID_heat_data:      //0x0202
						memcpy(&Power_heat_data, (ReadFromUsart + DATA), LEN_heat_data);
						memcpy(&REF.PowerHeatData, (ReadFromUsart + DATA), LEN_heat_data);
					break;
					
					case ID_robot_pos:      //0x0203
						memcpy(&robot_position, (ReadFromUsart + DATA), LEN_robot_pos);
						memcpy(&REF.GameRobotPos, (ReadFromUsart + DATA), LEN_robot_pos);
					break;
					
					case ID_buff:      //0x0204
						memcpy(&buff, (ReadFromUsart + DATA), LEN_buff);
						memcpy(&REF.Buff, (ReadFromUsart + DATA), LEN_buff);
					break;
					
					case ID_aerial_robot_energy:      //0x0205
						memcpy(&Aerial_robot_energy, (ReadFromUsart + DATA), LEN_aerial_robot_energy);
						memcpy(&REF.AerialRobotEnergy, (ReadFromUsart + DATA), LEN_aerial_robot_energy);
					break;
					
					case ID_robot_hurt:      //0x0206
						memcpy(&Robot_hurt, (ReadFromUsart + DATA), LEN_robot_hurt);
						memcpy(&REF.RobotHurt, (ReadFromUsart + DATA), LEN_robot_hurt);
					  if(Robot_hurt.hurt_type == 0)//非装甲板离线造成伤害
						{	Hurt_Data_Update = TRUE;	}//装甲数据每更新一次则判定为受到一次伤害
					break;
					
					case ID_shoot_data:      //0x0207
						memcpy(&Shoot_data, (ReadFromUsart + DATA), LEN_shoot_data);
					    JUDGE_ShootNumCount_17mm();//发弹量统计,不适用于双枪管,不准
//					    JUDGE_ShootNumCount_42mm();//发弹量统计,不适用于双枪管,不准
					break;
					
					case ID_bullet_remaining:      //0x0208
						memcpy(&Bullet_remaining, (ReadFromUsart + DATA), LEN_bullet_remaining);
					break;
					
					case ID_rfid_status:      //0x0209
						memcpy(&Rfid_status, (ReadFromUsart + DATA), LEN_rfid_status);
					break;
					
					case ID_dart_client_cmd:      //0x020A
						memcpy(&Dart_client_cmd, (ReadFromUsart + DATA), LEN_dart_client_cmd);
					break;
					
					
					case ID_robot_interactive_header_data://0x0301
						memcpy(&CommunatianData_cmd, (ReadFromUsart + DATA), 10);
					break;

					
					case ID_minimap_interactive_data:  //0x0303
						memcpy(&Minimap_robot_command, (ReadFromUsart + DATA), LEN_minimap_interactive_data);
					break;

					case ID_keybord_and_mouse_massage:  //0x0304
						memcpy(&Transfer_image_robot_command, (ReadFromUsart + DATA), LEN_keybord_and_mouse_massage);
					break;
					
					case ID_client_map_command:  //0x0305
						memcpy(&Client_map_command_t, (ReadFromUsart + DATA), LEN_client_map_command);
					break;
					
				}
				
				//首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,用来判断一个数据包是否有多帧数据
				if(*(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL) == 0xA5)
				{
					//如果一个数据包出现了多帧数据,则再次读取
					Judge_Read_Data(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL);
				}
			}					
		}		
	}
	if (retval_tf == TRUE)
	{
		Judge_Data_TF = TRUE;//辅助函数用
	}
	else		//只要CRC16校验不通过就为FALSE
	{
		Judge_Data_TF = FALSE;//辅助函数用
	}

	return retval_tf;//对数据正误做处理	

}

/**
  * @brief  判断自己红蓝方
  * @param  void
  * @retval RED   BLUE
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */
bool Color;
bool  is_red_or_blue(void)
{
	Judge_Self_ID = Robot_status.robot_id;//读取当前机器人ID
	
	if(Robot_status.robot_id > 10)
	{
		return BLUE;
	}
	else 
	{
		return RED;
	}
}

/**
  * @brief  判断自身ID，选择客户端ID
  * @param  void
  * @retval RED   BLUE
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */
void  Determine_ID(void)//判断自己是哪个队伍
{
	if(REF.GameRobotStat.robot_id < 10)//本机器人的ID，红方
	{ 
		REF.ids.teammate_hero 		 	= 1;
		REF.ids.teammate_engineer  = 2;
		REF.ids.teammate_infantry3 = 3;
		REF.ids.teammate_infantry4 = 4;
		REF.ids.teammate_infantry5 = 5;
		REF.ids.teammate_plane		 	= 6;
		REF.ids.teammate_sentry		= 7;
		
		REF.ids.client_hero 		 	= 0x0101;
		REF.ids.client_engineer  = 0x0102;
		REF.ids.client_infantry3 = 0x0103;
		REF.ids.client_infantry4 = 0x0104;
		REF.ids.client_infantry5 = 0x0105;
		REF.ids.client_plane			= 0x0106;
		
		if     (REF.GameRobotStat.robot_id == hero_red)//不断刷新放置在比赛中更改颜色
			REF.self_client = REF.ids.client_hero;
		else if(REF.GameRobotStat.robot_id == engineer_red)
			REF.self_client = REF.ids.client_engineer;
		else if(REF.GameRobotStat.robot_id == infantry3_red)
			REF.self_client = REF.ids.client_infantry3;
		else if(REF.GameRobotStat.robot_id == infantry4_red)
			REF.self_client = REF.ids.client_infantry4;
		else if(REF.GameRobotStat.robot_id == infantry5_red)
			REF.self_client = REF.ids.client_infantry5;
		else if(REF.GameRobotStat.robot_id == plane_red)
			REF.self_client = REF.ids.client_plane;
	}
	else //蓝方
	{
		REF.ids.teammate_hero 		 	= 101;
		REF.ids.teammate_engineer  = 102;
		REF.ids.teammate_infantry3 = 103;
		REF.ids.teammate_infantry4 = 104;
		REF.ids.teammate_infantry5 = 105;
		REF.ids.teammate_plane		 	= 106;
		REF.ids.teammate_sentry		= 107;
		
		REF.ids.client_hero 		 	= 0x0165;
		REF.ids.client_engineer  = 0x0166;
		REF.ids.client_infantry3 = 0x0167;
		REF.ids.client_infantry4 = 0x0168;
		REF.ids.client_infantry5 = 0x0169;
		REF.ids.client_plane			= 0x016A;
		
		if     (REF.GameRobotStat.robot_id == hero_blue)
			REF.self_client = REF.ids.client_hero;
		else if(REF.GameRobotStat.robot_id == engineer_blue)
			REF.self_client = REF.ids.client_engineer;
		else if(REF.GameRobotStat.robot_id == infantry3_blue)
			REF.self_client = REF.ids.client_infantry3;
		else if(REF.GameRobotStat.robot_id == infantry4_blue)
			REF.self_client = REF.ids.client_infantry4;
		else if(REF.GameRobotStat.robot_id == infantry5_blue)
			REF.self_client = REF.ids.client_infantry5;
		else if(REF.GameRobotStat.robot_id == plane_blue)
			REF.self_client = REF.ids.client_plane;
		
	}

}

/********************裁判数据辅助判断函数***************************/
/**
  * @brief  数据是否可用
  * @param  void
  * @retval  TRUE可用   FALSE不可用
  * @attention  在裁判读取函数中实时改变返回值
  */
bool JUDGE_sGetDataState(void)
{
	return Judge_Data_TF;
}

/**
  * @brief  读取瞬时功率
  * @param  void
  * @retval 实时功率值
  * @attention  
  */
float  JUDGE_fGetChassisPower(void)
{
	return (Power_heat_data.chassis_power);
}


float JUDGE_fGetChassisVolt(void)
{
	return (Power_heat_data.chassis_volt);
}
/**
  * @brief  读取剩余焦耳能量
  * @param  void
  * @retval 剩余缓冲焦耳能量(最大60)
  * @attention  
  */
uint16_t JUDGE_fGetRemainEnergy(void)
{
	return (Power_heat_data.chassis_power_buffer);
}
/**
  * @brief  读取当前等级
  * @param  void
  * @retval 当前等级
  * @attention  
  */
uint8_t  JUDGE_ucGetRobotLevel(void)
{
    return	Robot_status.robot_level;
}
/**
  * @brief  读取id1_17mm枪口热量
  * @param  void
  * @retval 17mm
  * @attention  
  */
uint16_t JUDGE_usGetRemoteHeat_id1_17mm(void)
{
	return Power_heat_data.shooter_id1_17mm_cooling_heat;
}

/**
  * @brief  读取id2_17mm枪口热量
  * @param  void
  * @retval 17mm
  * @attention  
  */
uint16_t  JUDGE_usGetRemoteHeat_id2_17mm(void)
{
	return Power_heat_data.shooter_id2_17mm_cooling_heat;
}

/**
  * @brief  读取id1_42mm枪口热量
  * @param  void
  * @retval 42mm
  * @attention  
  */
uint16_t JUDGE_usGetRemoteHeat_id1_42mm(void)
{
	return Power_heat_data.shooter_id1_42mm_cooling_heat;
}


/**
  * @brief  读取射速
  * @param  void
  * @retval 17mm
  * @attention  实时速度
  */
float JUDGE_usGetSpeedHeat(void)
{
	return Shoot_data.bullet_speed;
}


uint16_t  JUDGE_usGetSpeedLimit(void)
{
	return Robot_status.shooter_id1_17mm_speed_limit;

}

/**
  * @brief  统计17mm发弹量
  * @param  void
  * @retval void
  * @attention  
  */
portTickType shoot_time_17mm;//发射延时测试
portTickType shoot_ping_17mm;//计算出的最终发弹延迟

float Shoot_Speed_Now_17mm = 0;
float Shoot_Speed_Last_17mm = 0;

void   JUDGE_ShootNumCount_17mm(void)
{
	//检测到是小弹丸  bullet_type==1
	if(Shoot_data.bullet_type == 1)
	{
	Shoot_Speed_Now_17mm = Shoot_data.bullet_speed;
	if(Shoot_Speed_Last_17mm != Shoot_Speed_Now_17mm)//因为是float型，几乎不可能完全相等,所以速度不等时说明发射了一颗弹
	{
		ShootNum_17mm++;
		Shoot_Speed_Last_17mm = Shoot_Speed_Now_17mm;
	}
	shoot_time_17mm = xTaskGetTickCount();//获取弹丸发射时的系统时间
//	shoot_ping_17mm = shoot_time_17mm - REVOL_uiGetRevolTime();//计算延迟
	}
}

/**
  * @brief  统计42mm发弹量
  * @param  void
  * @retval void
  * @attention  
  */
portTickType shoot_time_42mm;//发射延时测试
portTickType shoot_ping_42mm;//计算出的最终发弹延迟

float Shoot_Speed_Now_42mm = 0;
float Shoot_Speed_Last_42mm = 0;

void JUDGE_ShootNumCount_42mm(void)
{
	//检测到是大弹丸  bullet_type==2
	if(Shoot_data.bullet_type == 2)
	{
	Shoot_Speed_Now_42mm = Shoot_data.bullet_speed;
	if(Shoot_Speed_Last_42mm != Shoot_Speed_Now_42mm)//因为是float型，几乎不可能完全相等,所以速度不等时说明发射了一颗弹
	{
		ShootNum_42mm++;
		Shoot_Speed_Last_42mm = Shoot_Speed_Now_42mm;
	}
	shoot_time_42mm = xTaskGetTickCount();//获取弹丸发射时的系统时间
//	shoot_ping_42mm = shoot_time_42mm - REVOL_uiGetRevolTime();//计算延迟
	}
}
/**
  * @brief  读取17mm发弹量
  * @param  void
  * @retval 发弹量
  * @attention 不适用于双枪管
  */
uint16_t  JUDGE_usGetShootNum_17mm(void)
{
	return ShootNum_17mm;
}

/**
  * @brief  读取42mm发弹量
  * @param  void
  * @retval 发弹量
  * @attention 不适用于双枪管
  */
uint16_t  JUDGE_usGetShootNum_42mm(void)
{
	return ShootNum_42mm;
}
/**
  * @brief  17mm发弹量清零
  * @param  void
  * @retval void
  * @attention 
  */
void JUDGE_ShootNum_Clear_17mm(void)
{
	ShootNum_17mm = 0;
}
/**
  * @brief  42mm发弹量清零
  * @param  void
  * @retval void
  * @attention 
  */
void   JUDGE_ShootNum_Clear_42mm(void)
{
	ShootNum_42mm = 0;
}

/**
  * @brief  读取id1_17mm枪口热量
  * @param  void
  * @retval 当前等级17mm热量上限
  * @attention  
  */
uint16_t  JUDGE_usGetHeatLimit_id1_17mm(void)
{
	return Robot_status.shooter_id1_17mm_cooling_limit;
}

/**
  * @brief  读取id2_17mm枪口热量
  * @param  void
  * @retval 当前等级17mm热量上限
  * @attention  
  */
uint16_t  JUDGE_usGetHeatLimit_id2_17mm(void)
{
	return Robot_status.shooter_id2_17mm_cooling_limit;
}

/**
  * @brief  读取id1_42mm枪口热量
  * @param  void
  * @retval 当前等级42mm热量上限
  * @attention  
  */
uint16_t  JUDGE_usGetHeatLimit_id1_42mm(void)
{
	return Robot_status.shooter_id1_42mm_cooling_limit;
}
/**
  * @brief  读取底盘功率最大值
  * @param  void
  * @retval 当前等级底盘功率上限
  * @attention  
  */
uint16_t  JUDGE_usGetChassisPowerLimit(void)
{
	return Robot_status.chassis_power_limit;
}

/**
  * @brief  读取机器人等级
  * @param  void
  * @retval 
  * @attention  
  */
uint8_t  JUDGE_ucRobotLevel(void)
{
	return Robot_status.robot_level;
}


/**
  * @brief  当前等级id1_17mm对应的枪口每秒冷却值
  * @param  void
  * @retval 当前等级17mm冷却速度
  * @attention  
  */
uint16_t  JUDGE_usGetShootCold_id1_17mm(void)
{
	return Robot_status. shooter_id1_17mm_cooling_rate;
}
/**
  * @brief  当前等级id2_17mm对应的枪口每秒冷却值
  * @param  void
  * @retval 当前等级17mm冷却速度
  * @attention  
  */
uint16_t  JUDGE_usGetShootCold_id2_17mm(void)
{
	return Robot_status.shooter_id1_17mm_cooling_rate;
}

/**
  * @brief  当前等级id1_42mm对应的枪口每秒冷却值
  * @param  void
  * @retval 当前等级17mm冷却速度
  * @attention  
  */
uint16_t  JUDGE_usGetShootCold_id1_42mm(void)
{
	return Robot_status. shooter_id1_42mm_cooling_rate;
}

/**
  * @brief  补给站口 ID：
  * @param  void
  * @retval 当前补给站口 ID：
  * @attention  为字符型变量
	1： 1 号补给口
	2： 2 号补给口
  */
uint8_t  JUDGE_usGetSupply_Id(void)
{
	return Supply_projectile_action.supply_projectile_id;
}

/**
  * @brief  补给机器人 ID：
  * @param  void
  * @retval 当前需补给机器人 ID：
  * @attention  为字符型变量
	补弹机器人 ID： 0 为当前无机器人补弹， 1 为红方英雄机器人补弹， 2 为红方工程机
	器人补弹， 3/4/5 为红方步兵机器人补弹， 101 为蓝方英雄机器人补弹， 102 为蓝方工
	程机器人补弹， 103/104/105 为蓝方步兵机器人补弹
  */
uint8_t  JUDGE_usGetSupply_Robo_Id(void)
{
	return Supply_projectile_action.supply_projectile_id;
}


/**
  * @brief  出弹口开闭状态 
  * @param  void
  * @retval 出弹口开闭状态 
  * @attention  为字符型变量
	0 为关闭，
	1 为子弹准备中，
	2 为子弹下落
  */
uint8_t  JUDGE_usGetSupply_Mode(void)
{
	return Supply_projectile_action.supply_projectile_step ;
}


/**
  * @brief  当前补弹数量
  * @param  void
  * @retval 当前补弹数量
  * @attention 为字符型变量
	50： 50 颗子弹；
	100： 100 颗子弹；
	150： 150 颗子弹；
	200： 200 颗子弹
*/ 

uint8_t  JUDGE_usGetSupply_Num(void)
{
	return Supply_projectile_action.supply_projectile_num;
}

/**
  * @brief  小地图接收信息目标机器人 ID(雷达站)
  * @param  void
  * @retval 当前补弹数量
  * @attention 

*/ 
uint16_t   JUDGE_usGetRadar_Station_to_robo_ID(void)
{
	return Client_map_command_t.target_robot_ID;
}

/**
  * @brief  小地图接收信息目标机器人 x 位置坐标，单位 m 当 x,y 超出界限时则不显示
			(雷达站)
  * @param  void
  * @retval 当前补弹数量
  * @attention 

*/ 
float  JUDGE_usGetRadar_Station_to_robo_posX(void)
{
	return	Client_map_command_t.target_position_x;
}

/**
  * @brief  小地图接收信息目标机器人 y 位置坐标，单位 m 当 x,y 超出界限时则不显示
			(雷达站)
  * @param  void
  * @retval 当前补弹数量
  * @attention 

*/ 
float  JUDGE_usGetRadar_Station_to_robo_posy(void)
{
	return	Client_map_command_t.target_position_y;
}

/**
  * @brief  当前机器人血量			
  * @param  void
  * @retval 当前机器人血量
  * @attention 

*/
uint16_t  JUDGE_usGetRadar_Robo_HP(void)
{
	return Robot_status.remain_HP;
}

/**
  * @brief  当前机器人最大血量			
  * @param  void
  * @retval 当前机器人血量
  * @attention 

*/
uint16_t  JUDGE_usGetRadar_Robo_Max_HP(void)
{
	return Robot_status.max_HP;
}

/**
  * @brief  当前机器人17mm 子弹剩余发射数量			
  * @param  void
  * @retval 当前机器17mm 子弹剩余发射数量
  * @attention 
17mm 子弹剩余发射数量
含义说明
					联盟赛 														对抗赛
步兵机器人 			全队步兵与英雄剩余可发射 17mm 弹丸总量			全队 17mm 弹丸剩余可兑换数量

英雄机器人 			全队步兵与英雄剩余可发射 17mm 弹丸总量			全队 17mm 弹丸剩余可兑换数量

空中机器人、哨兵机器人 该机器人剩余可发射 17mm 弹丸总量 				该机器人剩余可发射 17mm 弹丸总量

*/
uint16_t  JUDGE_usGetRadar_Robo_remain_17mm(void)
{
	return Bullet_remaining.bullet_remaining_num_17mm;
}

/**
  * @brief  shooter电源输出
  * @param  void
  * @retval 1 已输出   0没输出
  * @attention  
  */
uint8_t  JUDGE_usGetJudge_shooter_Power(void)
{
	return 	Robot_status.mains_power_shooter_output;
}



/****************底盘自动闪避判断用*******************/
/**
  * @brief  装甲板伤害数据是否更新
  * @param  void
  * @retval TRUE已更新   FALSE没更新
  * @attention  
  */
bool  JUDGE_IfArmorHurt(void)
{
	static portTickType ulCurrent = 0;
	static uint32_t ulDelay = 0;
	static bool IfHurt = FALSE;//默认装甲板处于离线状态

	
	ulCurrent = xTaskGetTickCount();

	if (Hurt_Data_Update == TRUE)//装甲板数据更新
	{
		Hurt_Data_Update = FALSE;//保证能判断到下次装甲板伤害更新
		ulDelay = ulCurrent + 200;//
		IfHurt = TRUE;
	}
	else if (ulCurrent > ulDelay)//
	{
		IfHurt = FALSE;
	}
	
	return IfHurt;
}

bool  Judge_If_Death(void)
{
	if(Robot_status.remain_HP == 0 && JUDGE_sGetDataState() == TRUE)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

bool  Judge_If_Chassis(void)
{
	if(Robot_status.mains_power_chassis_output == 1)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}
/**
  * @brief  装甲板受打击ID
  * @param  void
  * @retval   
  * @attention  
		bit 0-3： 当血量变化类型为装甲伤害，代表装甲 ID，其中数值为 0-4 号代表机器人
		的五个装甲片，其他血量变化类型，该变量数值为 0。

  */
uint8_t  Judge_armor_id(void)
{
	return Robot_hurt.armor_id;
}

/**
  * @brief  装甲板受伤模式
  * @param  void
  * @retval     
  * @attention  
		bit 4-7： 血量变化类型
			0x0 装甲伤害扣血；
			0x1 模块掉线扣血；
			0x2 超射速扣血；
			0x3 超枪口热量扣血；
			0x4 超底盘功率扣血；
			0x5 装甲撞击扣血
  */
uint8_t  Judge_hurt_mode(void)
{
	return Robot_hurt.hurt_type;
}				


/**
* @brief  机器人间信息交互
* @param  void
**/
uint8_t CliendTxBuffer[200];
#define Teammate_max_len 200 //200
unsigned char TeammateTxBuffer[Teammate_max_len];
char first_line[30]  = {"         COVER:"};//弹舱盖
char second_line[30] = {"         STUCK:"};//小陀螺
char third_line[30]  = {"         ENEMY:"};//自瞄
char fourth_line[30] = {"          FIRC:"};//热量限制
char fifth_line[30]  = {": CUFF"};//发弹量int
char sixth_line[30]  = {": VUFF"};
char seventh_line[30]= {"super capacitor:"};//超级电容剩余量,float
char empty_line[30] = {"                             "};
/*******************************开始绘字符串******************************/
ext_charstring_data_t tx_client_char;
uint8_t state_first_graphic;//0~7循环
void Char_Graphic(ext_client_string_t* graphic,//最终要发出去的数组中的数据段内容
									const char* name,
									uint32_t operate_tpye,
									uint32_t layer,
									uint32_t color,
									uint32_t size,
									uint32_t length,
									uint32_t width,
									uint32_t start_x,
									uint32_t start_y,
									const char *character)//外部放入的数组
{
	graphic_data_struct_t *data_struct = &graphic->grapic_data_struct;
	for(char i=0;i<3;i++)
		data_struct->graphic_name[i] = name[i];	//字符索引
	data_struct->operate_tpye = operate_tpye; //图层操作
	data_struct->graphic_tpye = CHAR;         //Char型
	data_struct->layer = layer;//都在第零层
	data_struct->color = color;//都是白色
	data_struct->start_angle = size;
	data_struct->end_angle = length;	
	data_struct->width = width;
	data_struct->start_x = start_x;
	data_struct->start_y = start_y;	
	
	data_struct->radius = 0;
	data_struct->end_x = 0;
	data_struct->end_y = 0;
	memcpy(graphic->data,empty_line,19);
  memcpy(graphic->data,character,length);
}

bool Send_Color = 0;
uint16_t send_time = 0;
static void  Draw_char()
{
	if(state_first_graphic == 0)//不知道什么时候进入客户端所以要不断更新
	{
		Char_Graphic(&tx_client_char.clientData,"CL1",ADD,0,GREEN,20,strlen(first_line),2,(50),(1080*9/12),first_line);//x1920/18
		state_first_graphic = 1;
	}
	else if(state_first_graphic == 1)
	{
		Char_Graphic(&tx_client_char.clientData,"CL2",ADD,0,GREEN,20,strlen(second_line),2,(50),(1080*8/12),second_line);
		state_first_graphic = 2;
	}
	else if(state_first_graphic == 2)
	{
		Char_Graphic(&tx_client_char.clientData,"CL3",ADD,0,GREEN,20,strlen(third_line),2,(50),(1080*7/12),third_line);
		state_first_graphic = 3;
	}
	else if(state_first_graphic == 3)
	{
		Char_Graphic(&tx_client_char.clientData,"CL4",ADD,0,GREEN,20,strlen(fourth_line),2,(50),(1080*6/12),fourth_line);
		state_first_graphic = 4;
	}
	else if(state_first_graphic == 4)
	{
		Char_Graphic(&tx_client_char.clientData,"CL5",ADD,0,GREEN,20,strlen(fifth_line),2,(1920-320),(1080*9/12),fifth_line);
		state_first_graphic = 5;
	}
	else if(state_first_graphic == 5)
	{
		Char_Graphic(&tx_client_char.clientData,"CL6",ADD,0,GREEN,20,strlen(sixth_line),2,(1920-320),(1080*8/12),sixth_line);
		state_first_graphic = 6;
	}
	else if(state_first_graphic == 6)
	{
		Char_Graphic(&tx_client_char.clientData,"CL7",ADD,0,GREEN,20,strlen(seventh_line),2,(1920-200-200),(1080*7/12),seventh_line);
		state_first_graphic = 7;
	}
}
/*******************************结束绘制字符串******************************/

//void  Client_graphic_Init()
//{
//	if(state_first_graphic>=7)
//	{
//		state_first_graphic = 0;
//	}
//		//帧头
//		tx_client_char.txFrameHeader.SOF = JUDGE_FRAME_HEADER;
//		tx_client_char.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_string_t);
//		tx_client_char.txFrameHeader.Seq = 0;//包序号
//		memcpy(CliendTxBuffer,&tx_client_char.txFrameHeader,sizeof(xFrameHeader));
//		append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//头校验
//	
//		//命令码
//		tx_client_char.CmdID = ID_robot_interactive_header_data;
//		
//		//数据段头结构
//		tx_client_char.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_char_graphic;
//		tx_client_char.dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
//		tx_client_char.dataFrameHeader.receiver_ID = REF.self_client;
//		
//		//数据段
//		Draw_char();
//		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_client_char.CmdID, LEN_CMD_ID+tx_client_char.txFrameHeader.DataLength);//加上命令码长度2
//		
//		//帧尾
//		append_CRC16_check_sum(CliendTxBuffer,sizeof(tx_client_char));
//		
//    HAL_UART_Transmit(&huart1,CliendTxBuffer,sizeof(tx_client_char),200);
//    
//}

//************************************绘制象形*******************************/
ext_graphic_seven_data_t tx_client_graphic_figure;
void Figure_Graphic(graphic_data_struct_t* graphic,//最终要发出去的数组的数据段内容
									const char* name,
									uint32_t operate_tpye,
									uint32_t graphic_tpye,//绘制什么图像
									uint32_t layer,
									uint32_t color,
									uint32_t start_angle,
									uint32_t end_angle,
									uint32_t width,
									uint32_t start_x,
									uint32_t start_y,
									uint32_t radius,
									uint32_t end_x,
									uint32_t end_y)							
{
	for(char i=0;i<3;i++)
		graphic->graphic_name[i] = name[i];	//字符索引
	graphic->operate_tpye = operate_tpye; //对进行图层操作
	graphic->graphic_tpye = graphic_tpye;         //Char型
	graphic->layer        = layer;//都在第一层
	graphic->color        = color;//变色
	graphic->start_angle  = start_angle;
	graphic->end_angle    = end_angle;	
	graphic->width        = width;
	graphic->start_x      = start_x;
	graphic->start_y      = start_y;	
	graphic->radius = radius;
	graphic->end_x  = end_x;
	graphic->end_y  = end_y;
}
int update_figure_flag;
extern global_flag_t global_flag;


static void stuck_second_figure(global_flag_t*global_stuck)//小陀螺打开为true
{
	if(global_flag.global_stuck == 1)//打开小陀螺为绿色
	{
		Figure_Graphic(&tx_client_graphic_figure.clientData[0],"GL2",update_figure_flag,CIRCLE,1,GREEN,0,0,5,  200+160,1080*8/12, 20,0,0);
	}
	else if(global_flag.global_stuck != 1)//没开小陀螺为紫红色
		
	{
		Figure_Graphic(&tx_client_graphic_figure.clientData[0],"GL2",update_figure_flag,CIRCLE,1,FUCHSIA,0,0,5,  200+160,1080*8/12, 20,0,0);
	}
}

static void auto_aim_third_figure(global_flag_t*global_aim)//自瞄打开为true
{
	if(global_flag.global_auto == 1)//打开自瞄为绿色
	{
		Figure_Graphic(&tx_client_graphic_figure.clientData[1],"GL3",update_figure_flag,CIRCLE,1,GREEN,0,0,5,  200+160,1080*7/12, 20,0,0);
	}
	else if(global_flag.global_auto != 1)//没开自瞄为紫红色
	{
		Figure_Graphic(&tx_client_graphic_figure.clientData[1],"GL3",update_figure_flag,CIRCLE,1,FUCHSIA,0,0,5,  200+160,1080*7/12, 20,0,0);
	}
}

static void cover_forth_figure(global_flag_t*global_cover)
{
	if(global_flag.global_cover==1)
	{
		Figure_Graphic(&tx_client_graphic_figure.clientData[2],"GL4",update_figure_flag,CIRCLE,1,GREEN,0,0,5,  200+160,1080*9/12, 20,0,0);
	}
	else if(global_flag.global_cover!=1)
	{
		Figure_Graphic(&tx_client_graphic_figure.clientData[2],"GL4",update_figure_flag,CIRCLE,1,FUCHSIA,0,0,5,  200+160,1080*9/12, 20,0,0);
	}
}

static void firc_fifth_figure(global_flag_t*global_firc)
{
	if(global_flag.global_firc==1)
	{
		Figure_Graphic(&tx_client_graphic_figure.clientData[3],"GL5",update_figure_flag,CIRCLE,1,GREEN,0,0,5,  200+160,1080*5.9/12, 20,0,0);
	}
	else if(global_flag.global_firc!=1)
	{
		Figure_Graphic(&tx_client_graphic_figure.clientData[3],"GL5",update_figure_flag,CIRCLE,1,FUCHSIA,0,0,5,  200+160,1080*5.9/12, 20,0,0);
	}
}


static void cuff_sixth_figure(global_flag_t*global_cuff)
{
	if(global_flag.global_cuff==1)
	{
		Figure_Graphic(&tx_client_graphic_figure.clientData[4],"GL6",update_figure_flag,CIRCLE,1,GREEN,0,0,5,  1920-370,1080*8.8/12, 20,0,0);
	}
	else if(global_flag.global_cuff!=1)
	{
		Figure_Graphic(&tx_client_graphic_figure.clientData[4],"GL6",update_figure_flag,CIRCLE,1,FUCHSIA,0,0,5, 1920-370,1080*8.8/12, 20,0,0);
	}
}


static void vuff_seventh_figure(global_flag_t*global_vuff)
{
  if(global_flag.global_vuff==1)
	{
		Figure_Graphic(&tx_client_graphic_figure.clientData[5],"GL7",update_figure_flag,CIRCLE,1,GREEN,0,0,5,  1920-370,1080*7.8/12, 20,0,0);
	}
	else if(global_flag.global_vuff!=1)
	{
		Figure_Graphic(&tx_client_graphic_figure.clientData[5],"GL7",update_figure_flag,CIRCLE,1,FUCHSIA,0,0,5,  1920-370,1080*7.8/12, 20,0,0);
	}
}

void EMPTY (global_flag_t*global_stuck ,global_flag_t*global_aim, global_flag_t*global_cover,global_flag_t*global_firc,global_flag_t*global_cuff,global_flag_t*global_vuff )
{
  
	  if   (global_flag.global_stuck == 1)//打开小陀螺为绿色
		Figure_Graphic(&tx_client_graphic_figure.clientData[0],"GL2",DELETE,CIRCLE,1,GREEN,0,0,5,  200+160,1080*8/12, 20,0,0);
		if   (global_flag.global_auto == 1)//打开自瞄为绿色
		Figure_Graphic(&tx_client_graphic_figure.clientData[1],"GL3",DELETE,CIRCLE,1,FUCHSIA,0,0,5,  200+160,1080*8/12, 20,0,0);
		if   (global_flag.global_cover == 1)
		Figure_Graphic(&tx_client_graphic_figure.clientData[2],"GL4",DELETE,CIRCLE,1,GREEN,0,0,5,  200+160,1080*9/12, 20,0,0);
		if   (global_flag.global_firc==1)
		Figure_Graphic(&tx_client_graphic_figure.clientData[3],"GL5",DELETE,CIRCLE,1,GREEN,0,0,5,  200+160,1080*9/12, 20,0,0);
    if   (global_flag.global_cuff==1)
		Figure_Graphic(&tx_client_graphic_figure.clientData[4],"GL6",DELETE,CIRCLE,1,GREEN,0,0,5,  200+160,1080*9/12, 20,0,0);
		if   (global_flag.global_vuff==1)
		Figure_Graphic(&tx_client_graphic_figure.clientData[5],"GL7",DELETE,CIRCLE,1,GREEN,0,0,5,  200+160,1080*9/12, 20,0,0);

//	  Figure_Graphic(&tx_client_graphic_figure.clientData[0],"GL2",DELETE,CIRCLE,1,GREEN,0,0,5,  200+160,1080*8/12, 20,0,0);
}

//!!!!!!!!!!!!!!!!!!!全局变量！！！！！

uint32_t global_sight_bead_x = 960,global_sight_bead_y = 720,global_supercapacitor_point=0;//[0,100]
float global_supercapacitor_remain = 77.3,//[0,100]
	    global_gimble_pitch,global_gimble_yaw;
int   global_bullet_speed,global_bullet_sum;
//extern chassis_move_t UI_chassis;
//extern chassis_move_t chassis_move;

static void Draw_Figure_bool()
{

	stuck_second_figure    (&global_flag);
	auto_aim_third_figure (&global_flag);
	cover_forth_figure    (&global_flag);
	firc_fifth_figure     (&global_flag);  
	cuff_sixth_figure    (&global_flag);
	vuff_seventh_figure   (&global_flag);
//	EMPTY(&global_flag,&global_flag,&global_flag,&global_flag,&global_flag,&global_flag);

  
}

//void  Client_graphic_Info_update()//七个图像一起更新
//{
//		//帧头
//		tx_client_graphic_figure.txFrameHeader.SOF = JUDGE_FRAME_HEADER;
//		tx_client_graphic_figure.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t)*7;
//		tx_client_graphic_figure.txFrameHeader.Seq = 0;//包序号
//		memcpy(CliendTxBuffer,&tx_client_graphic_figure.txFrameHeader,sizeof(xFrameHeader));
//		append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//头校验

//		//命令码
//		tx_client_graphic_figure.CmdID = ID_robot_interactive_header_data;

//		//数据段头结构
//		tx_client_graphic_figure.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_seven_graphic;
//		tx_client_graphic_figure.dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
//		tx_client_graphic_figure.dataFrameHeader.receiver_ID = REF.self_client;
//	
//		//数据段
//		Draw_Figure_bool();
//		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_client_graphic_figure.CmdID, LEN_CMD_ID+tx_client_graphic_figure.txFrameHeader.DataLength);//加上命令码长度2

//		//帧尾
//		append_CRC16_check_sum(CliendTxBuffer,sizeof(tx_client_graphic_figure));
//		
//    HAL_UART_Transmit(&huart1,CliendTxBuffer,sizeof(tx_client_graphic_figure),200);

//}

ext_graphic_two_data_t tx_aim_figure;//第三层放准心
int update_aim_flag;//1-add,3删除
static void sight_bead_figrue(uint32_t x,uint32_t y)//可移动准心，请强制转换成uint32_t 1920*1080有部分地区无法画出
{
	Figure_Graphic(&tx_aim_figure.clientData[0],"GR1",update_aim_flag,LINE,2,FUCHSIA,0,0,3,  x-20,y+20  ,0,  x+20,y-20);//graphic_Remove
	Figure_Graphic(&tx_aim_figure.clientData[1],"GR2",update_aim_flag,LINE,2,FUCHSIA,0,0,3,  x-20,y-20  ,0,  x+20,y+20);
}
void  Client_aim_update()//两个个图像一起更新
{
		//帧头
		tx_aim_figure.txFrameHeader.SOF = JUDGE_FRAME_HEADER;
		tx_aim_figure.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t)*2;
		tx_aim_figure.txFrameHeader.Seq = 0;//包序号
		memcpy(CliendTxBuffer,&tx_aim_figure.txFrameHeader,sizeof(xFrameHeader));
		append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//头校验

		//命令码
		tx_aim_figure.CmdID = ID_robot_interactive_header_data;

		//数据段头结构
		tx_aim_figure.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_two_graphic;
		tx_aim_figure.dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
		tx_aim_figure.dataFrameHeader.receiver_ID = REF.self_client;
	
		//数据段
		sight_bead_figrue(global_sight_bead_x,global_sight_bead_y);
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_aim_figure.CmdID, LEN_CMD_ID+tx_aim_figure.txFrameHeader.DataLength);//加上命令码长度2

		//帧尾
		append_CRC16_check_sum(CliendTxBuffer,sizeof(tx_aim_figure));
		
	HAL_UART_Transmit(&huart1,CliendTxBuffer,sizeof(tx_aim_figure),200);

}
//剩余电容只有一个图层
ext_graphic_one_data_t tx_supercapacitor_figure;
int update_supercapacitor_flag;
int superflag=0;
static void supercapacitor_figure(float remain_energy,uint32_t turning_point)//剩余超级电容（单位百分比），低于某百分比变红色
{
		superflag++;
	static uint32_t remaining = 0;
//	static uint32_t remaining_last =0;
//	remaining_last = remaining;
	remaining = (uint32_t)remain_energy;//强制转换
		
	if(remaining >= turning_point)//直线长度为3
		Figure_Graphic(&tx_supercapacitor_figure.clientData,"SR1",update_supercapacitor_flag,LINE,2,GREEN,0,0,10,  (1920-400),585  ,0,  (1920-400)+remaining,585);//585
	else if(remaining < turning_point)
		Figure_Graphic(&tx_supercapacitor_figure.clientData,"SR1",update_supercapacitor_flag,LINE,2,FUCHSIA,0,0,10,(1920-400),585  ,0,  (1920-400)+remaining,585);

//	switch (superflag)
//	{
//		
//		case  5:
//			Figure_Graphic(&tx_supercapacitor_figure.clientData,"SR1",DELETE,LINE,2,GREEN,0,0,10,                       (1920-400),585,  0,  (1920-400)+24,       585);
//		break;
//		case 10:
//			Figure_Graphic(&tx_supercapacitor_figure.clientData,"SR1",DELETE,LINE,2,GREEN,0,0,10,                       (1920-400),585,  0,  (1920-400)+24,       585);
//		break;	 
//}
//	if(superflag==11)
//		superflag=0;

}
void  Client_supercapacitor_update()//一个图像更新
{
		//帧头
//		tx_supercapacitor_figure.txFrameHeader.SOF = JUDGE_FRAME_HEADER;
//		tx_supercapacitor_figure.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t);
//		tx_supercapacitor_figure.txFrameHeader.Seq = 0;//包序号
//		memcpy(CliendTxBuffer,&tx_supercapacitor_figure.txFrameHeader,sizeof(xFrameHeader));
//		append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//头校验

//		//命令码
//		tx_supercapacitor_figure.CmdID = ID_robot_interactive_header_data;

//		//数据段头结构
//		tx_supercapacitor_figure.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_one_graphic;
//		tx_supercapacitor_figure.dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
//		tx_supercapacitor_figure.dataFrameHeader.receiver_ID = REF.self_client;
	
			tx_supercapacitor_figure.txFrameHeader.SOF = JUDGE_FRAME_HEADER;
		tx_supercapacitor_figure.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t);
		tx_supercapacitor_figure.txFrameHeader.Seq = 0;//包序号
		memcpy(CliendTxBuffer,&tx_supercapacitor_figure.txFrameHeader,sizeof(xFrameHeader));
		append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//头校验

		//命令码
		tx_supercapacitor_figure.CmdID = ID_robot_interactive_header_data;

		//数据段头结构
		tx_supercapacitor_figure.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_one_graphic;
		tx_supercapacitor_figure.dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
		tx_supercapacitor_figure.dataFrameHeader.receiver_ID = REF.self_client;
	
		//数据段
		supercapacitor_figure(global_supercapacitor_remain,global_supercapacitor_point);
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_supercapacitor_figure.CmdID, LEN_CMD_ID+tx_supercapacitor_figure.txFrameHeader.DataLength);//加上命令码长度2

		//帧尾
		append_CRC16_check_sum(CliendTxBuffer,sizeof(tx_supercapacitor_figure));
	
		//数据段
//		supercapacitor_figure(global_supercapacitor_remain,global_supercapacitor_point);
//		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_supercapacitor_figure.CmdID, LEN_CMD_ID+tx_supercapacitor_figure.txFrameHeader.DataLength);//加上命令码长度2

//		//帧尾
//		append_CRC16_check_sum(CliendTxBuffer,sizeof(tx_supercapacitor_figure));
		
	HAL_UART_Transmit(&huart1,CliendTxBuffer,sizeof(tx_supercapacitor_figure),200);
}
//******************绘制浮点数*************************/
//第五层图层
ext_float_two_data_t tx_gimbal_angle;
int update_float_flag;
void Float_Graphic(Float_data_struct_t* graphic,//最终要发出去的数组的数据段内容
									const char* name,
									uint32_t operate_tpye,
									uint32_t graphic_tpye,//绘制什么图像
									uint32_t layer,
									uint32_t color,
									uint32_t size,
									uint32_t decimal,
									uint32_t width,
									uint32_t start_x,
									uint32_t start_y,
									float number)							
{
	for(char i=0;i<3;i++)
		graphic->graphic_name[i] = name[i];	//字符索引
	graphic->operate_tpye = operate_tpye; //图层操作
	graphic->graphic_tpye = graphic_tpye;  
	graphic->layer        = layer;//
	graphic->color        = color;//变色
	graphic->start_angle  = size;
	graphic->end_angle    = decimal;//小数有效位	
	graphic->width        = width;
	graphic->start_x      = start_x;
	graphic->start_y      = start_y;	
	int32_t IntData = number * 1000;
	graphic->radius          = (IntData & 0x000003ff) >>  0;
	graphic->end_x           = (IntData & 0x001ffc00) >> 10;
	graphic->end_y           = (IntData & 0xffe00000) >> 21;

}
static void gimbal_angle_float(float gimble_pitch,float gimble_yaw)//当前云台角度
{
	//青色pitch第一行，黄色yaw第二行
		Float_Graphic(&tx_gimbal_angle.clientData[0],"FR1",update_float_flag,FLOAT,4,CYAN_BLUE,30,2,3,(1920*4/6),810  ,(float)gimble_pitch);
		Float_Graphic(&tx_gimbal_angle.clientData[1],"FR2",update_float_flag,FLOAT,4,YELLOW,   30,2,3,(1920*4/6),760  ,(float)gimble_yaw);		
}
void  Client_gimbal_angle_update()//两个图像更新
{
		//帧头
		tx_gimbal_angle.txFrameHeader.SOF = JUDGE_FRAME_HEADER;
		tx_gimbal_angle.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t)*2;
		tx_gimbal_angle.txFrameHeader.Seq = 0;//包序号
		memcpy(CliendTxBuffer,&tx_gimbal_angle.txFrameHeader,sizeof(xFrameHeader));
		append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//头校验

		//命令码
		tx_gimbal_angle.CmdID = ID_robot_interactive_header_data;

		//数据段头结构
		tx_gimbal_angle.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_two_graphic;
		tx_gimbal_angle.dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
		tx_gimbal_angle.dataFrameHeader.receiver_ID = REF.self_client;
	
		//数据段
	gimbal_angle_float(global_flag.global_pitch,global_flag.global_yaw);
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_gimbal_angle.CmdID, LEN_CMD_ID+tx_gimbal_angle.txFrameHeader.DataLength);//加上命令码长度2

		//帧尾
		append_CRC16_check_sum(CliendTxBuffer,sizeof(tx_gimbal_angle));
		
	HAL_UART_Transmit(&huart1,CliendTxBuffer,sizeof(tx_gimbal_angle),200);
}
//**********************绘制int类型***************************/
ext_int_two_data_t tx_bullet_int;
int update_int_flag;
void Int_Graphic(Int_data_struct_t* graphic,//最终要发出去的数组的数据段内容
									const char* name,
									uint32_t operate_tpye,
									uint32_t graphic_tpye,//绘制什么图像
									uint32_t layer,
									uint32_t color,
									uint32_t size,
									uint32_t zero,
									uint32_t width,
									uint32_t start_x,
									uint32_t start_y,
									int number)							
{
	for(char i=0;i<3;i++)
		graphic->graphic_name[i] = name[i];	//字符索引
	graphic->operate_tpye = operate_tpye; //图层操作
	graphic->graphic_tpye = graphic_tpye;        
	graphic->layer        = layer;//都在第一层
	graphic->color        = color;//变色
	graphic->start_angle  = size;
	graphic->end_angle    = zero;	
	graphic->width        = width;
	graphic->start_x      = start_x;
	graphic->start_y      = start_y;	
	graphic->number       = number;
}
static void bullet_int(int fix_time)//子弹射速和发弹量
{
	//总数量第一行，射速第二行
		Int_Graphic(&tx_bullet_int.clientData[0],"IR1",update_int_flag,INT,5,CYAN_BLUE,30,0,3,(1920*5/6),(710)  ,fix_time);
	//	Int_Graphic(&tx_bullet_int.clientData[1],"IR2",update_int_flag,INT,5,WHITE,30,0,3,(1920-280),(730)  ,(int)bullet_speed);		
}
void  Client_bullet_int_update()//两个图像更新
{
		//帧头
		tx_bullet_int.txFrameHeader.SOF = JUDGE_FRAME_HEADER;
		tx_bullet_int.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t)*2;
		tx_bullet_int.txFrameHeader.Seq = 0;//包序号
		memcpy(CliendTxBuffer,&tx_bullet_int.txFrameHeader,sizeof(xFrameHeader));
		append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//头校验

		//命令码
		tx_bullet_int.CmdID = ID_robot_interactive_header_data;

		//数据段头结构
		tx_bullet_int.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_two_graphic;
		tx_bullet_int.dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
		tx_bullet_int.dataFrameHeader.receiver_ID = REF.self_client;
	
		//数据段
		bullet_int(global_flag.global_fix);
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_bullet_int.CmdID, LEN_CMD_ID+tx_bullet_int.txFrameHeader.DataLength);//加上命令码长度2

		//帧尾
		append_CRC16_check_sum(CliendTxBuffer,sizeof(tx_bullet_int));
		
	HAL_UART_Transmit(&huart1,CliendTxBuffer,sizeof(tx_bullet_int),200);
}

//*****************************英雄需求*************************************/
//*****************************英雄需求*************************************/
ext_graphic_five_data_t aim_line_graphic;
uint32_t global_vertical_1,global_vertical_2,global_horizontal_1,global_horizontal_2,global_horizontal_3;
//绘图
static void Draw_Figure_aimline(uint32_t vertical_1,uint32_t vertical_2,//竖线范围在0~1920
	                              uint32_t horizontal_1,uint32_t horizontal_2,uint32_t horizontal_3)//水平线范围在0~1080
{
	//竖线
		Figure_Graphic(&aim_line_graphic.clientData[0],"LL1",ADD,LINE,6,WHITE,0,0,3,  vertical_1,1080, 0,vertical_1,0);
		Figure_Graphic(&aim_line_graphic.clientData[1],"LL2",ADD,LINE,6,WHITE,0,0,3,  vertical_2,1080, 0,vertical_2,0);
	//水平线
		Figure_Graphic(&aim_line_graphic.clientData[2],"LL3",ADD,LINE,6,WHITE,0,0,3,  0,horizontal_1, 0,1920,horizontal_1);
		Figure_Graphic(&aim_line_graphic.clientData[3],"LL4",ADD,LINE,6,WHITE,0,0,3,  0,horizontal_2, 0,1920,horizontal_2);
		Figure_Graphic(&aim_line_graphic.clientData[4],"LL5",ADD,LINE,6,WHITE,0,0,3,  0,horizontal_3, 0,1920,horizontal_3);
		

}
void  Client_aim_line()//五个图像不更新
{
		//帧头
		aim_line_graphic.txFrameHeader.SOF = JUDGE_FRAME_HEADER;
		aim_line_graphic.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t)*5;
		aim_line_graphic.txFrameHeader.Seq = 0;//包序号
		memcpy(CliendTxBuffer,&aim_line_graphic.txFrameHeader,sizeof(xFrameHeader));
		append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//头校验

		//命令码
		aim_line_graphic.CmdID = ID_robot_interactive_header_data;

		//数据段头结构
		aim_line_graphic.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_five_graphic;
		aim_line_graphic.dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
		aim_line_graphic.dataFrameHeader.receiver_ID = REF.self_client;
	
		//数据段
    Draw_Figure_aimline(global_vertical_1,global_vertical_2,global_horizontal_1,global_horizontal_2,global_horizontal_3);
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&aim_line_graphic.CmdID, LEN_CMD_ID+aim_line_graphic.txFrameHeader.DataLength);//加上命令码长度2

		//帧尾
		append_CRC16_check_sum(CliendTxBuffer,sizeof(aim_line_graphic));
		
	HAL_UART_Transmit(&huart1,CliendTxBuffer,sizeof(aim_line_graphic),200);
}

////三号准心图层
//void Figure_Graphic(graphic_data_struct_t* graphic,//最终要发出去的数组的数据段内容
//									const char* name,
//									uint32_t operate_tpye,
//									uint32_t graphic_tpye,//绘制什么图像
//									uint32_t layer,
//									uint32_t color,
//									uint32_t start_angle,
//									uint32_t end_angle,
//									uint32_t width,
//									uint32_t start_x,
//									uint32_t start_y,
//									uint32_t radius,
//									uint32_t end_x,
//									uint32_t end_y)		
ext_graphic_seven_data_t high_aim_figure;//操作手准心之上,不更新
ext_graphic_seven_data_t low_aim_shortfigure_1;//准心下的第一个短线
ext_graphic_seven_data_t low_aim_shortfigure_2;
ext_graphic_seven_data_t  low_aim_shortfigure_3;//五个短线两个纵线
ext_graphic_seven_data_t  vision_range_data;
ext_graphic_five_data_t  low_aim_longfigure;//准心下的长横线
//!!!!!!!!!!!!!!!!!!全局变量
uint32_t division_value = 10;
//division_value分度值,line_length短线长度的一半
static void aim_1(uint32_t division_value,uint32_t line_length)//准心上半部分的宽度"AH"--aim_high
{
	Figure_Graphic(&high_aim_figure.clientData[1],"AH2",ADD,LINE,1,GREEN,0,0,2,  498,8, 0,795,381);

	Figure_Graphic(&high_aim_figure.clientData[4],"AH5",ADD,LINE,1,GREEN,0,0,2,  1409,11,0,  1115,383);
}
void _high_aim_()
{
		//帧头
		high_aim_figure.txFrameHeader.SOF = JUDGE_FRAME_HEADER;
		high_aim_figure.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t)*7;
		high_aim_figure.txFrameHeader.Seq = 0;//包序号
		memcpy(CliendTxBuffer,&high_aim_figure.txFrameHeader,sizeof(xFrameHeader));
		append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//头校验

		//命令码
		high_aim_figure.CmdID = ID_robot_interactive_header_data;

		//数据段头结构
		high_aim_figure.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_seven_graphic;
		high_aim_figure.dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
		high_aim_figure.dataFrameHeader.receiver_ID = REF.self_client;
	
		//数据段
		aim_1(division_value,10);
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&high_aim_figure.CmdID, LEN_CMD_ID+high_aim_figure.txFrameHeader.DataLength);//加上命令码长度2

		//帧尾
		append_CRC16_check_sum(CliendTxBuffer,sizeof(high_aim_figure));
		
		HAL_UART_Transmit(&huart1,CliendTxBuffer,sizeof(high_aim_figure),200);
}

static void aim_lowshort_2(uint32_t division_value,uint32_t line_length)//准心上半部分的宽度"AL"--aim_low
{
	Figure_Graphic(&low_aim_shortfigure_1.clientData[0],"AL1",ADD,LINE,3,YELLOW,0,0,1,  950,540-170,0,  950,540-10);//graphic_Remove
}
void _lowshort_aim_2()
{
		//帧头
		low_aim_shortfigure_1.txFrameHeader.SOF = JUDGE_FRAME_HEADER;
		low_aim_shortfigure_1.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t)*7;
		low_aim_shortfigure_1.txFrameHeader.Seq = 0;//包序号
		memcpy(CliendTxBuffer,&low_aim_shortfigure_1.txFrameHeader,sizeof(xFrameHeader));
		append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//头校验

		//命令码
		low_aim_shortfigure_1.CmdID = ID_robot_interactive_header_data;

		//数据段头结构
		low_aim_shortfigure_1.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_seven_graphic;
		low_aim_shortfigure_1.dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
		low_aim_shortfigure_1.dataFrameHeader.receiver_ID = REF.self_client;
	
		//数据段
		aim_lowshort_2(division_value,10);
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&low_aim_shortfigure_1.CmdID, LEN_CMD_ID+low_aim_shortfigure_1.txFrameHeader.DataLength);//加上命令码长度2

		//帧尾
		append_CRC16_check_sum(CliendTxBuffer,sizeof(low_aim_shortfigure_1));
		
		HAL_UART_Transmit(&huart1,CliendTxBuffer,sizeof(low_aim_shortfigure_1),200);
}
static void aim_lowshort_3(uint32_t division_value,uint32_t line_length)//准心上半部分的宽度"AM"--aim_low_middle
{
	Figure_Graphic(&low_aim_shortfigure_2.clientData[0],"AM1",ADD,LINE,3,YELLOW,0,0,1,  950-line_length,540+60-division_value*9 ,0,  950+line_length,540+60-division_value*9 );//graphic_Remove	Figure_Graphic(&low_aim_shortfigure_2.clientData[1],"AM2",ADD,LINE,3,YELLOW,0,0,1,  960-line_length,540+30-division_value*10,0,  960+line_length,540+30-division_value*10);
	Figure_Graphic(&low_aim_shortfigure_2.clientData[1],"AM2",ADD,LINE,3,YELLOW,0,0,1,  950-line_length,540+60-division_value*10,0,  950+line_length,540+60-division_value*10);
  Figure_Graphic(&low_aim_shortfigure_2.clientData[2],"AM3",ADD,LINE,3,YELLOW,0,0,1,  950-line_length,540+60-division_value*12,0,  950+line_length,540+60-division_value*12);
	Figure_Graphic(&low_aim_shortfigure_2.clientData[3],"AM4",ADD,LINE,3,YELLOW,0,0,1,  950-line_length,540+60-division_value*13,0,  950+line_length,540+60-division_value*13);
	Figure_Graphic(&low_aim_shortfigure_2.clientData[4],"AM5",ADD,LINE,3,YELLOW,0,0,1,  950-line_length,540+60-division_value*14,0,  950+line_length,540+60-division_value*14);
	Figure_Graphic(&low_aim_shortfigure_2.clientData[5],"AM6",ADD,LINE,3,YELLOW,0,0,1,  950-line_length,540+60-division_value*20,0,  950+line_length,540+60-division_value*20);
	Figure_Graphic(&low_aim_shortfigure_2.clientData[6],"AM7",ADD,LINE,3,YELLOW,0,0,1,  950-line_length,540+60-division_value*21,0,  950+line_length,540+60-division_value*21);
}
void _lowshort_aim_3()
{
		//帧头
		low_aim_shortfigure_2.txFrameHeader.SOF = JUDGE_FRAME_HEADER;
		low_aim_shortfigure_2.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t)*7;
		low_aim_shortfigure_2.txFrameHeader.Seq = 0;//包序号
		memcpy(CliendTxBuffer,&low_aim_shortfigure_2.txFrameHeader,sizeof(xFrameHeader));
		append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//头校验

		//命令码
		low_aim_shortfigure_2.CmdID = ID_robot_interactive_header_data;

		//数据段头结构
		low_aim_shortfigure_2.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_seven_graphic;
		low_aim_shortfigure_2.dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
		low_aim_shortfigure_2.dataFrameHeader.receiver_ID = REF.self_client;
	
		//数据段
		aim_lowshort_3(division_value,20);
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&low_aim_shortfigure_2.CmdID, LEN_CMD_ID+low_aim_shortfigure_2.txFrameHeader.DataLength);//加上命令码长度2

		//帧尾
		append_CRC16_check_sum(CliendTxBuffer,sizeof(low_aim_shortfigure_2));
		
		HAL_UART_Transmit(&huart1,CliendTxBuffer,sizeof(low_aim_shortfigure_2),200);
}
//视觉瞄准范围
static void vision_range_1(uint32_t division_value,uint32_t line_length)//准心上半部分的宽度"AM"--aim_low_bottom,"AS"--aim_stem
{ 
	int start_x,start_y,length,height;
  start_x=620;
	start_y=800;
	length=680;
	height=520;
	Figure_Graphic(&vision_range_data.clientData[0],"AB1",ADD,LINE,3,YELLOW,0,0,1,   start_x,start_y,0,start_x+length,start_y);//graphic_Remove
	Figure_Graphic(&vision_range_data.clientData[1],"AB2",ADD,LINE,3,YELLOW,0,0,1,   start_x+length,start_y,0,start_x+length,start_y-height);
	Figure_Graphic(&vision_range_data.clientData[2],"AB3",ADD,LINE,3,YELLOW,0,0,1,   start_x+length,start_y-height,0,start_x,start_y-height);
	Figure_Graphic(&vision_range_data.clientData[3],"AB4",ADD,LINE,3,YELLOW,0,0,1,   start_x,start_y-height,0,start_x,start_y);

}
void vision_range()
{
		//帧头
		vision_range_data.txFrameHeader.SOF = JUDGE_FRAME_HEADER;
		vision_range_data.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t)*7;
		vision_range_data.txFrameHeader.Seq = 0;//包序号
		memcpy(CliendTxBuffer,&vision_range_data.txFrameHeader,sizeof(xFrameHeader));
		append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//头校验

		//命令码
		vision_range_data.CmdID = ID_robot_interactive_header_data;

		//数据段头结构
		vision_range_data.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_seven_graphic;
		vision_range_data.dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
		vision_range_data.dataFrameHeader.receiver_ID = REF.self_client;
	
		//数据段
		vision_range_1(division_value,10);
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&vision_range_data.CmdID, LEN_CMD_ID+vision_range_data.txFrameHeader.DataLength);//加上命令码长度2

		//帧尾
		append_CRC16_check_sum(CliendTxBuffer,sizeof(vision_range_data));
		
		HAL_UART_Transmit(&huart1,CliendTxBuffer,sizeof(vision_range_data),200);

}






static void aim_lowlong(uint32_t division_value,uint32_t line_length)//准心上半部分的宽度"AM"--aim_low_Long,"AS"--aim_stem
{ 
	Figure_Graphic(&low_aim_longfigure.clientData[0],"AL1",ADD,LINE,4,YELLOW,0,0,1,960-line_length-30,540-30-division_value*19,0,960+line_length+30,540-30-division_value*19);//graphic_Remove
	Figure_Graphic(&low_aim_longfigure.clientData[1],"AL2",ADD,LINE,4,YELLOW,0,0,1,960-line_length-40,540-30-division_value*15,0,960+line_length+40,540-30-division_value*15);
	Figure_Graphic(&low_aim_longfigure.clientData[2],"AL3",ADD,LINE,4,YELLOW,0,0,1,960-line_length-50,540-30-division_value*11,0,960+line_length+50,540-30-division_value*11);
	Figure_Graphic(&low_aim_longfigure.clientData[3],"AL4",ADD,LINE,4,YELLOW,0,0,1,960-line_length-60,540-30-division_value*7 ,0,960+line_length+60,540-30-division_value*7 );
	Figure_Graphic(&low_aim_longfigure.clientData[4],"AL5",ADD,LINE,4,YELLOW,0,0,1,960-line_length-70,540-30-division_value*3 ,0,960+line_length+70,540-30-division_value*3 );
	
}
void _lowlong_aim_()
{
		//帧头
		low_aim_longfigure.txFrameHeader.SOF = JUDGE_FRAME_HEADER;
		low_aim_longfigure.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t)*5;
		low_aim_longfigure.txFrameHeader.Seq = 0;//包序号
		memcpy(CliendTxBuffer,&low_aim_longfigure.txFrameHeader,sizeof(xFrameHeader));
		append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//头校验

		//命令码
		low_aim_longfigure.CmdID = ID_robot_interactive_header_data;

		//数据段头结构
		low_aim_longfigure.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_five_graphic;//0x0103 客户端绘制5个图形
		low_aim_longfigure.dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
		low_aim_longfigure.dataFrameHeader.receiver_ID = REF.self_client;
	
		//数据段
		aim_lowlong(division_value,10);
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&low_aim_longfigure.CmdID, LEN_CMD_ID+low_aim_longfigure.txFrameHeader.DataLength);//加上命令码长度2

		//帧尾
		append_CRC16_check_sum(CliendTxBuffer,sizeof(low_aim_longfigure));
		
		HAL_UART_Transmit(&huart1,CliendTxBuffer,sizeof(low_aim_longfigure),200);
}





/**************************************************************************/
void  Client_graphic_Init()
{
	if(state_first_graphic>=7)
	{
		state_first_graphic = 0;
	}
		//帧头
		tx_client_char.txFrameHeader.SOF = JUDGE_FRAME_HEADER;
		tx_client_char.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_string_t);
		tx_client_char.txFrameHeader.Seq = 0;//包序号
		memcpy(CliendTxBuffer,&tx_client_char.txFrameHeader,sizeof(xFrameHeader));
		append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//头校验
	
		//命令码
		tx_client_char.CmdID = ID_robot_interactive_header_data;
		
		//数据段头结构
		tx_client_char.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_char_graphic;
		tx_client_char.dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
		tx_client_char.dataFrameHeader.receiver_ID = REF.self_client;
		
		//数据段
		Draw_char();
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_client_char.CmdID, LEN_CMD_ID+tx_client_char.txFrameHeader.DataLength);//加上命令码长度2
		
		//帧尾
		append_CRC16_check_sum(CliendTxBuffer,sizeof(tx_client_char));
		
    HAL_UART_Transmit(&huart1,CliendTxBuffer,sizeof(tx_client_char),200);
    
}


void  Client_graphic_Info_update()//七个图像一起更新
{
		//帧头
		tx_client_graphic_figure.txFrameHeader.SOF = JUDGE_FRAME_HEADER;
		tx_client_graphic_figure.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t)*7;
		tx_client_graphic_figure.txFrameHeader.Seq = 0;//包序号
		memcpy(CliendTxBuffer,&tx_client_graphic_figure.txFrameHeader,sizeof(xFrameHeader));
		append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//头校验

		//命令码
		tx_client_graphic_figure.CmdID = ID_robot_interactive_header_data;

		//数据段头结构
		tx_client_graphic_figure.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_seven_graphic;
		tx_client_graphic_figure.dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
		tx_client_graphic_figure.dataFrameHeader.receiver_ID = REF.self_client;
	
		//数据段
		Draw_Figure_bool();
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_client_graphic_figure.CmdID, LEN_CMD_ID+tx_client_graphic_figure.txFrameHeader.DataLength);//加上命令码长度2

		//帧尾
		append_CRC16_check_sum(CliendTxBuffer,sizeof(tx_client_graphic_figure));
		
    HAL_UART_Transmit(&huart1,CliendTxBuffer,sizeof(tx_client_graphic_figure),200);

}