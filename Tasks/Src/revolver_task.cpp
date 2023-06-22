#include "system_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Revolver_task.h"
#include "judge_task.h"
#include "ui_task.h"

Revolver_Task  revolver; 

Revolver_Task* revolver_point(void)
{
	return &revolver;
}

/*********全局变量*************/

int stuck_mode_to_ui;
int v_firc_set;
extern int vision_mode;
bool auto_firc_flag_angle;
int num_firc;

/******************************/
/**
  * @brief          发射机构任务
  * @param[in]      null
  */
void revolver_task(void const *pvParameters)
{
	//延时
	vTaskDelay(REVOLVER_TASK_INIT_TIME);

	//射击任务初始化
	revolver.REVOLVER_Init();
	
	while(1)
	{
		//数据更新
		revolver.Revolver_Feedback_Update(); 

		//控制值设定
		revolver.Friction_wheel_set.Friction_wheel_mode_set();
		revolver.Dial_set.Dial_behaviour_set();

		//设定值处理
		revolver.Friction_wheel_calculating();
		revolver.Dial_calculating();

		//电流的发送
		CAN2_200_cmd_motor(revolver.Firc_L_give_current,revolver.Firc_R_give_current,revolver.Dial_give_current,0);
		//CAN2_200_cmd_motor(0,0,revolver.Dial_give_current,0);
		vTaskDelay(2);
  }
}

/**
  * @brief          发射机构任务初始化
  * @param[in]      null
  */

void Revolver_Task::REVOLVER_Init()

{
	Friction_wheel_set.Friction_Wheel_behaviour_init();
	Dial_set.Dial_mode_init();

	//初始化PID
	fp32 Revolver_speed_pid[3] = {REVOLVER_SPEED_PID_KP, REVOLVER_SPEED_PID_KI, REVOLVER_SPEED_PID_KD};
	fp32 Revolver_position_pid[3] = {REVOLVER_POSITION_PID_KP, REVOLVER_POSITION_PID_KI, REVOLVER_POSITION_PID_KD};
	fp32 Firc_speed_pid[3] = {FIRC_SPEED_PID_KP, FIRC_SPEED_PID_KI, FIRC_SPEED_PID_KD};	 
	revolver_motor_speed_pid.init( PID_POSITION, Revolver_speed_pid,REVOLVER_SPEED_PID_MAX_OUT, REVOLVER_SPEED_PID_MAX_IOUT);
	revolver_motor_position_pid.init(PID_POSITION, Revolver_position_pid,REVOLVER_POSITION_PID_MAX_OUT, REVOLVER_POSITION_PID_MAX_IOUT);

	Firc_R_speed_pid.init(PID_POSITION, Firc_speed_pid,FIRC_SPEED_PID_MAX_OUT, FIRC_SPEED_PID_MAX_IOUT);
	Firc_L_speed_pid.init(PID_POSITION, Firc_speed_pid,FIRC_SPEED_PID_MAX_OUT, FIRC_SPEED_PID_MAX_IOUT);

	//斜坡函数单次增加位置
	Dial_buff_ramp = AN_BULLET/40;
	//电机指针
	revolver_motor_measure = get_motor_measure_class(TRIGGER);
	Firc_L_firc3508_motor_measure = get_motor_measure_class(FireL);
	Firc_R_firc3508_motor_measure = get_motor_measure_class(FireR);

	angle_out=0;
	//模式与角度初始化
	Revolver_Feedback_Update();
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	//射速PID
	fp32 speed_pid[3]={SPEED_PID_KP,SPEED_PID_KI,SPEED_PID_KD};
	Speed_pid.init(PID_POSITION,speed_pid,SPEED_PID_MAX_OUT,SPEED_PID_MAX_IOUT);
	
}

float Firc_L_firc3508_motor_speed,Firc_R_firc3508_motor_speed;
float Firc_L_tempreate,Firc_R_tempreate,gloab_bullet_speed;
/**
  * @brief          发射机构数据更新
  * @param[in]      null
  */

void Revolver_Task::Revolver_Feedback_Update()
{
	static fp32 speed_fliter_1 = 0.0f;
	static fp32 speed_fliter_2 = 0.0f;
	static fp32 speed_fliter_3 = 0.0f;

	//拨弹轮电机速度滤波一下
	static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};
	//二阶低通滤波
	speed_fliter_1 = speed_fliter_2;
	speed_fliter_2 = speed_fliter_3;
	speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (revolver_motor_measure->speed_rpm * Motor_RMP_TO_SPEED) * fliter_num[2];
	Revolver_speed = speed_fliter_3;
	firc_l = Firc_L_firc3508_motor_measure->speed_rpm*Motor_RMP_TO_SPEED;
	firc_r = Firc_R_firc3508_motor_measure->speed_rpm*Motor_RMP_TO_SPEED;
	//计算输出轴角度
	Dial_set.angle = revolver_motor_measure->num*-8192 + revolver_motor_measure->ecd;
	

	Dial_set.heating = JUDGE_usGetRemoteHeat_id1_17mm();
	bullet_speed=JUDGE_usGetSpeedHeat();
	Dial_set.heat_max=JUDGE_usGetHeatLimit_id1_17mm();

	Dial_set.real_firc_info_update();
	judge_auto_firc();
}


/**
  * @brief          发射机构摩擦轮控制量计算
  * @param[in]      null
  */


/**
  * @brief          发射机构摩擦轮控制量计算
  * @param[in]      null
  */

void Revolver_Task::Friction_wheel_calculating()
{

		if(switch_is_down(Friction_wheel_set.Friction_wheel_RC->rc.s[RevChannel]) || switch_is_up(Friction_wheel_set.Friction_wheel_RC->rc.s[RevChannel]) 
			|| switch_is_down(Friction_wheel_set.Friction_wheel_RC->rc.s[0]))
		{
			Firc_L_give_current = Firc_L_speed_pid.calc( Firc_L_firc3508_motor_measure->speed_rpm,Friction_wheel_set.Firc_L.speed_set);
			Firc_R_give_current = Firc_R_speed_pid.calc( Firc_R_firc3508_motor_measure->speed_rpm,Friction_wheel_set.Firc_R.speed_set);

			Firc_L_speed_ramp_set = 0;
		  Firc_R_speed_ramp_set = 0;

		} 
		else if((switch_is_mid(Friction_wheel_set.Friction_wheel_RC->rc.s[RevChannel]) &&switch_is_mid(Friction_wheel_set.Friction_wheel_RC->rc.s[0]))
			   ||(switch_is_mid(Friction_wheel_set.Friction_wheel_RC->rc.s[RevChannel]) &&switch_is_up(Friction_wheel_set.Friction_wheel_RC->rc.s[0])))
		{
			Firc_L_speed_ramp_set = RAMP_float(Friction_wheel_set.Firc_L.speed_set,Firc_L_firc3508_motor_measure->speed_rpm,2500);
			Firc_R_speed_ramp_set = RAMP_float(Friction_wheel_set.Firc_R.speed_set, Firc_R_firc3508_motor_measure->speed_rpm,2500);			
			
			// 赋予电流值
			Firc_L_give_current	=Firc_L_speed_pid.calc( Firc_L_firc3508_motor_measure->speed_rpm,Firc_L_speed_ramp_set);
			Firc_R_give_current	=	Firc_R_speed_pid.calc(Firc_R_firc3508_motor_measure->speed_rpm,Firc_R_speed_ramp_set);
		}		
		else 
		{
		  Firc_L_give_current=0;
	   	Firc_R_give_current=0;
			Firc_L_speed_ramp_set = 0;
		  Firc_R_speed_ramp_set = 0; 
		
		}
}



/**
  * @brief          摩擦轮模式初始化
  * @param[in]      null
  */

void Friction_wheel_Mode_set::Friction_Wheel_behaviour_init()
{
	Friction_wheel_RC = get_remote_control_point();
	friction_wheel_mode = FRICTION_WHEEL_OFF;
	firc_mode=0;
	
}

/**
  * @brief          摩擦轮模式设置
  * @param[in]      null
  */
	
void Friction_wheel_Mode_set::Friction_wheel_mode_set()
{
	if((switch_is_down(Friction_wheel_RC->rc.s[0]) == 0||syspoint()->sys_mode==AUTO||syspoint()->sys_mode==SPIN_AUTO)&&((switch_is_down(Friction_wheel_RC->rc.s[1]) == 0))&&syspoint()->sys_mode!=DBUS_MISSING_CONTROL)//非左下右下 摩擦轮开启模式
	{
		friction_wheel_mode = FRICTION_WHEEL_ON;
		Friction_wheel_mode_on_set();
		firc_mode=1;
	}
	else
	{
		friction_wheel_mode = FRICTION_WHEEL_OFF;
		Friction_wheel_mode_off_set();
		firc_mode=0;
	}
}

/**
  * @brief          摩擦轮开启模式下设定值设置
  * @param[in]      null
  */

void Friction_wheel_Mode_set::Friction_wheel_mode_on_set()
{
	  speedlimit = JUDGE_usGetSpeedLimit();
		//speedlimit = 15;
		User_fix();

	//根据弹速上限进行弹速补偿
	if(revolver.bullet_last_speed<revolver.bullet_speed)
	{
		if(speedlimit==15)
		{
      revolver.speed_adapt(revolver.bullet_speed , 13.5 , 14.5 ,&revolver.fix_num , 5 , 5);
		}
    
		else if(speedlimit==18)
		{
      revolver.speed_adapt(revolver.bullet_speed , 16.5 , 17.5 ,&revolver.fix_num , 5 , 5);
		}

		else if(speedlimit==30)
		{
      revolver.speed_adapt(revolver.bullet_speed , 28.5 , 29.5 ,&revolver.fix_num , 5 , 5);
		}
		else 
		{
		  revolver.speed_adapt(revolver.bullet_speed , 13.5 , 14.5 ,&revolver.fix_num , 5 , 5);
		}
	}
	else
	{
		revolver.fix_num = 0;
	}


	if(speedlimit==15)
	{ 
		Temp_Fix_15S();
		Firc_L.speed_set = -abs(v_fic_set);
		Firc_R.speed_set = abs(v_fic_set);
	}
	else if(speedlimit==18)
	{	
		Temp_Fix_18S();
		Firc_L.speed_set = -abs(v_fic_set);
		Firc_R.speed_set = abs(v_fic_set);
	}
	else
	{
		Temp_Fix_30S();
		Firc_L.speed_set = -abs(v_fic_set);
		Firc_R.speed_set = abs(v_fic_set);
	}
}


/**
  * @brief          弹速自适应
  * @param[in]      null
  */

void Revolver_Task::speed_adapt(fp32 real_S , fp32 min_S, fp32 max_S,fp32 *fix , fp32 up_num , fp32 down_num)
{
	static uint8_t SpeedErr_cnt=0;
  if(real_S < min_S && real_S > 20)
    SpeedErr_cnt++;
  else if(real_S >= min_S && real_S <= max_S )
	  SpeedErr_cnt = 0;

  if(SpeedErr_cnt == 1)//射速偏低
  {
    SpeedErr_cnt = 0;
    *fix += up_num;
  }
  
  if(real_S > max_S)//射速偏高
    *fix -= down_num;
}

/**
  * @brief           摩擦轮关闭模式下设定值设置
  * @param[in]       null
  */

void Friction_wheel_Mode_set::Friction_wheel_mode_off_set()
{
		Firc_L.speed_set=0;
		Firc_R.speed_set=0;
		Firc_L.speed_ramp_set = 0;
		Firc_R.speed_ramp_set = 0;
}


/**
  * @brief           操作手手动补偿摩擦轮转速设置
  * @param[in]       null
  */
void Friction_wheel_Mode_set::User_fix()
{
	if(IF_KEY_PRESSED_C&&IF_KEY_PRESSED_CTRL)//c加 v减
	{
		fix_mode = 1;
	}
	else if(IF_KEY_PRESSED_V&&IF_KEY_PRESSED_CTRL)
	{
		fix_mode = 2;
	}
	else 
	{
		fix_mode = 0;
	}
	if(fix_mode == 1&&fix_last_mode != 1)
	{
		fixed += 50;
		fix_times++;
		
	}
	else if(fix_mode == 2&&fix_last_mode != 2)
	{
		fixed -= 50;
		fix_times--;
	}
	fix_last_mode = fix_mode;

	v_firc_set=v_fic_set;//测试用
	//save
	Temp_fix_save();
}


/**
  * @brief           摩擦轮手动补偿
  * @param[in]       null
  */

void Friction_wheel_Mode_set::Temp_Fix_15S()
{
   v_fic_set = FRICTION_L1_SPEED+fixed;	
}
/**
  * @brief           摩擦轮手动补偿
  * @param[in]       null
  */
void Friction_wheel_Mode_set::Temp_Fix_18S()
{
   v_fic_set = FRICTION_L2_SPEED+fixed;	
}
/**
  * @brief           摩擦轮手动补偿
  * @param[in]       null
  */
void Friction_wheel_Mode_set::Temp_Fix_30S()
{
   v_fic_set = FRICTION_L3_SPEED+fixed;	
	if(v_fic_set>FRICTION_L3_SPEED)
	{
			v_fic_set=FRICTION_L3_SPEED;
	}
}
/**
  * @brief           摩擦轮自动补偿/保护
  * @param[in]       null
  */
void Friction_wheel_Mode_set::Temp_fix_save()
{
		if(revolver.bullet_speed>speedlimit-0.3)
		{
				v_fic_set-=100;
		}
	

}
/**
  * @brief           拨盘模式初始化
  * @param[in]       null
  */

void Dial_mode_set::Dial_mode_init()
{
		Dial_RC = get_remote_control_point();
		Revolver_mode=REVOL_POSI_MODE;
		set_angle =angle;
}


/**
  * @brief           拨盘打弹延时
  * @param[in]       null
  */
bool real_shoot;
bool start_firc;
int delay_count;
float per_fir_delay;
bool To_zero_flag;

void Dial_mode_set::real_firc_info_update()
{
		bullet_speed = JUDGE_usGetSpeedHeat();
		if(bullet_speed != bullet_last_speed)
		{
				real_shoot=1;
		}
		else
		{
				real_shoot=0;
		}
		bullet_last_speed=bullet_speed;

    if(start_firc)
		{
				++delay_count;
		}

		if(real_shoot&&start_firc)
		{
				per_fir_delay=delay_count*2;
				if(per_fir_delay>200)
				{per_fir_delay=100;}
				//per_fir_delay-=10;
				start_firc=0;
				delay_count=0;
				real_shoot=0;
		}

}

void Revolver_Task::judge_auto_firc()
{
	auto_flag = vision_info_point()->RxPacketFir.is_shooting;
	if(auto_flag == 1 && last_auto_flag == 0)
		auto_firc = 1;
	else
		auto_firc = 0;

  last_auto_flag = auto_flag;
}
/**
  * @brief           拨盘模式设置及设定值设置
  * @param[in]       null
  */


void Dial_mode_set::Dial_behaviour_set()
{
		static int16_t speed_time;
		int auto_firc_flag=0;



if(vision_info_point()->RxPacketFir.is_spinning==1)
{
		vision_mode=1;					//小陀螺下自动开火
}
else{
		vision_mode=3;					//机动状态下手动开火
}


/*********************************自动开火*******************************/
		if(vision_mode==1)
		{
				if(((syspoint()->sys_mode==AUTO||syspoint()->sys_mode==SPIN_AUTO)&&
				(fabs(revolver.Firc_L_firc3508_motor_measure->speed_rpm-revolver.Friction_wheel_set.Firc_L.speed_set)<100)&&
				(fabs(revolver.Firc_R_firc3508_motor_measure->speed_rpm-revolver.Friction_wheel_set.Firc_R.speed_set)<100)))
				{
						if(vision_info_point()->RxPacketFir.is_shooting==1&&auto_firc_flag_angle)
//						if(revolver.auto_firc && auto_firc_flag_angle)
						{
						speed_time++;
						num_firc=1;
						Revolver_Switch = REVOL_STEP1;
						if(speed_time>30)//300
						{
							//	Revolver_Switch = REVOL_STEP1;	//单发测试
							Revolver_Switch = REVOL_STEP2;	//

						}		
						}
						else if(vision_info_point()->RxPacketFir.is_shooting==0)
						{
								Revolver_Switch = REVOL_STEP0;
								key_ShootFlag = shoot_unready;
								speed_time = 0;

						}
				}
				//正常开火
				if ( (switch_is_up(Dial_RC->rc.s[RevChannel])||Dial_RC->mouse.press_l==1)&&((switch_is_down(Dial_RC->rc.s[0])!=1)||syspoint()->sys_mode==AUTO||syspoint()->sys_mode==SPIN_AUTO))
				{
						speed_time++;
						num_firc=1;
						Revolver_Switch = REVOL_STEP1;
						if(speed_time>300)//300
						{
								Revolver_Switch = REVOL_STEP2;	
						}			
				}
				else 
				{
						Revolver_Switch = REVOL_STEP0;
						key_ShootFlag = shoot_unready;
						speed_time = 0;
				}
		}
else if(vision_mode==3)
{
		num_firc=1;
//左拨杆拨上一次或鼠标左键按下为STEP1  左拨杆一直在上或长按左键为STEP2
		if ( (switch_is_up(Dial_RC->rc.s[RevChannel])||Dial_RC->mouse.press_l==1)&&((switch_is_down(Dial_RC->rc.s[0])!=1)||syspoint()->sys_mode==AUTO||syspoint()->sys_mode==SPIN_AUTO))
		{
			speed_time++;
			Revolver_Switch = REVOL_STEP1;

			if(speed_time>300)//300
			{
				Revolver_Switch = REVOL_STEP2;	
			}			
		}
		else 
		{
				Revolver_Switch = REVOL_STEP0;
				key_ShootFlag = shoot_unready;
				speed_time = 0;
		}
}


		//STEP2切入速度环
		if(Revolver_Switch == REVOL_STEP2&&ifstuck_finish != stuck_unfinish)
				Revolver_mode =	REVOL_SPEED_MODE;
		else if((Revolver_Switch == REVOL_STEP1||Revolver_Switch == REVOL_STEP0)&&ifstuck_finish != stuck_unfinish)
			  Revolver_mode =	REVOL_POSI_MODE;
		


if(vision_mode==1)
{
//拨杆拨上一次或左键按下一次返回1 否则为0
		if((Revolver_Switch == REVOL_STEP1 && Revolver_last_Switch == REVOL_STEP0)||
        (vision_info_point()->RxPacketFir.is_shooting==1&&(syspoint()->sys_mode==AUTO||syspoint()->sys_mode==SPIN_AUTO)))
		{
			if(heating<heat_max-HEAT_ERROR|| heat_max == 0)//abs(revolver_control->set_angle- revolver_control->angle)<1000) || heat_max == 0)
			{
				key_ShootFlag = shoot_ready;
			}
		}
		else
		{
		key_ShootFlag = shoot_unready;
		}	
}
else
{
if((Revolver_Switch == REVOL_STEP1 && Revolver_last_Switch == REVOL_STEP0))
		{
			if(heating<heat_max-HEAT_ERROR|| heat_max == 0)//abs(revolver_control->set_angle- revolver_control->angle)<1000) || heat_max == 0)
			{
				key_ShootFlag = shoot_ready;
			}
		}
		else
		{
			key_ShootFlag = shoot_unready;
		}	
}
			
		Revolver_last_Switch = Revolver_Switch;

    //卡弹检测
		if(abs(set_angle - angle)> AN_BULLET/2)//这里应该是小于一半的的子弹位标记为卡弹未完成射击
		{
			ifshoot_finsh = shoot_unfinish;
			unfinish_time++;
		}
		else
		{
			unfinish_time = 0;
			ifshoot_finsh = shoot_finish;
			ifstuck_finish = stuck_finish;
		}
		if(unfinish_time>500)
		{
			Revolver_mode = REVOL_REVERSAL_MODE;
			ifstuck_finish = stuck_unfinish;
			if_stucked = 1;
			stuck_mode = 1;
			stuck_mode_to_ui=1;
		}



		//检测进入卡弹模式
		if(Revolver_mode == REVOL_REVERSAL_MODE&&last_Revolver_mode!=REVOL_REVERSAL_MODE)
		{
			set_angle += AN_BULLET*3/2;
		}

		else if(Revolver_mode == REVOL_POSI_MODE)//双环单发
		{
			if(key_ShootFlag == shoot_ready&&ifshoot_finsh != shoot_unfinish)//&&abs(revolver_control->set_angle - revolver_control->last_angle)<1000)
			{
					set_angle -= AN_BULLET*num_firc;	
//					set_angle -= AN_BULLET;
					start_firc=1;
//					--num_firc;
				 if(if_stucked == 1)
				{
					set_angle += AN_BULLET/2;
					if_stucked = 0;
					stuck_mode=0;
					stuck_mode_to_ui=0;
				}
			}
		}
		//赋予电流值（连发）
		else if(Revolver_mode == REVOL_SPEED_MODE)
		{
			//连发热量限制
			if( ifshoot_finsh != shoot_unfinish && (heating<heat_max-HEAT_ERROR|| heat_max ==0))
				{
					if(if_stucked == 1)
					{
						set_angle+= AN_BULLET/2;
						if_stucked = 0;
						stuck_mode = 0;
						stuck_mode_to_ui=0;
					}
					else
					{
					//双环连发
							set_angle-= AN_BULLET;
							start_firc=1;
					}
				}
		}


		if(auto_firc_flag==1)
		{
			//连发热量限制
			if( (heating<heat_max-HEAT_ERROR|| heat_max ==0))
				{
					if(if_stucked == 1)
					{
						set_angle+= AN_BULLET/2;
						if_stucked = 0;
						stuck_mode = 0;
						stuck_mode_to_ui=0;
					}
					else
					{
					  //双环连发
						set_angle-= AN_BULLET*num_firc;
						start_firc=1;
					}
				}
		}


		 last_Revolver_mode = Revolver_mode;
		if(switch_is_down(Dial_RC->rc.s[RevChannel]))
		{
		  set_angle=angle;
		}
}



/**
  * @brief          发射机构拨盘控制量计算
  * @param[in]      null
  */
void Revolver_Task::Dial_calculating()
{
		if(Dial_set_ramp_angle != Dial_set.set_angle)//缓慢转过去
		{
			Dial_set_ramp_angle = RAMP_float(Dial_set.set_angle, Dial_set_ramp_angle, Dial_buff_ramp);
		}
		//角度环，速度环串级pid调试（超热量限制在拨弹数累加时已经完成）
		angle_out = revolver_motor_position_pid.calc( Dial_set.angle, Dial_set_ramp_angle);
		Dial_give_current = revolver_motor_speed_pid.calc(Revolver_speed, angle_out);

		if(switch_is_down(Dial_set.Dial_RC->rc.s[RevChannel]))
		{
			Dial_give_current=0;
		}

}
