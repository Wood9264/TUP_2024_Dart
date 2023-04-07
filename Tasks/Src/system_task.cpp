#include "system_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "judge_task.h"
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

system_t sys;

uint16_t speedlimit;
/**
  * @brief          ϵͳ������
  * @param[in]      none
  */
void system_task(void const *pvParameters)
{
	vTaskDelay(201);
	uint32_t currentTime;
	while(1)
	{
		currentTime = xTaskGetTickCount();//��ǰϵͳʱ��
		sys.Mode_set();
		sys.Transit();

		vTaskDelayUntil(&currentTime, 1);
	}
}


/**
  * @brief 		ȡϵͳ��ָ��
  * @retval   ϵͳ����
  */
system_t *syspoint(void)
{
	return &sys;
}

/**
  * @brief          ���캯����ʼ��
  * @param[in]      null
  */
system_t::system_t()
{
	system_rc_ctrl = get_remote_control_point();
	rc_add_yaw =0.0f; 
	rc_add_pit = 0.0f;
	
	sys_mode = ZERO_FORCE;
}


/**
  * @brief          ��̨ģʽ����
  * @param[in]      null
  * @retval         null
  */
void system_t::Mode_set()
{	
	  Rc_vision();
		last_sys_mode=sys_mode;
	  if(init_state==1)
		{
		  return;
		}
		else
		{
			if(sys_mode != ZERO_FORCE && (toe_is_error(DBUS_TOE) == 1)) //��ǰ������ģʽ��ң��������ʱ����ʧ��ģʽ
			{
				sys_mode = DBUS_MISSING_CONTROL;
				return;
			}
			else if(system_rc_ctrl->mouse.press_r == 1 || vision_auto_flag ==1)//����     
			{
				sys_mode = AUTO;

			}
			else if(switch_is_down(system_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))  //����
			{
				sys_mode = ZERO_FORCE;
			}
			else if(switch_is_up(system_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))//С����//С���ݺ������޷����ʹ��
			{
				sys_mode = SPIN;
			}
			
			else if(switch_is_mid(sys.system_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))  //���̸�����̨
			{
				sys_mode = ABSOLUTE_ANGLE;
			}
			//��ʼ������
			if(last_sys_mode == ZERO_FORCE && sys_mode != ZERO_FORCE)
			{
				sys_mode = INIT;
				init_state=1;
			}
  	}
}



void system_t::Transit()
{
	 if(last_sys_mode != ABSOLUTE_ANGLE&&sys_mode ==ABSOLUTE_ANGLE)
		{
			gimbal_point()->Yaw_motor.absolute_angle_set =gimbal_point()->Yaw_motor.absolute_angle;
		  gimbal_point()->Pitch_motor.absolute_angle_set =gimbal_point()->Pitch_motor.absolute_angle;
		}
		else if(last_sys_mode != RELATIVE_ANGLE&&sys_mode ==RELATIVE_ANGLE)
		{
			gimbal_point()->Yaw_motor.relative_angle_set =gimbal_point()->Yaw_motor.relative_angle;
			gimbal_point()->Pitch_motor.relative_angle_set =gimbal_point()->Pitch_motor.relative_angle;
			
		}
		else if(last_sys_mode != SPIN&&sys_mode ==SPIN)
		{
			gimbal_point()->Yaw_motor.absolute_angle_set =gimbal_point()->Yaw_motor.absolute_angle;
			gimbal_point()->Pitch_motor.absolute_angle_set =gimbal_point()->Pitch_motor.absolute_angle;
		}	
		else if(last_sys_mode != AUTO&&sys_mode ==AUTO)
		{
			gimbal_point()->Yaw_motor.absolute_angle_set=gimbal_point()->Yaw_motor.absolute_angle;
			gimbal_point()->Pitch_motor.absolute_angle_set=INIT_PITCH_SET;
		
		}

}




/**
  * @brief          ң�����ж��Ƿ��������ģʽ
  * @param[in]      null
  * @retval         True or False
  */
void system_t::Rc_vision()
{
	static int16_t rc_vision_time;
	if (system_rc_ctrl->rc.ch[0] > RC_CALI_VALUE_HOLE &&  \
			system_rc_ctrl->rc.ch[1] > RC_CALI_VALUE_HOLE &&  \
		  system_rc_ctrl->rc.ch[2] < -RC_CALI_VALUE_HOLE && \
		  system_rc_ctrl->rc.ch[3] < -RC_CALI_VALUE_HOLE && \
		  switch_is_down(system_rc_ctrl->rc.s[0]) &&        \
		  switch_is_down(system_rc_ctrl->rc.s[1]) )
	  rc_vision_time++;
	else
		rc_vision_time = 0;
	
	if(rc_vision_time >500)
	{	
		vision_flag = 1;
		if(vision_flag == 1 && vision_last_flag == 0)
			vision_auto_flag = !vision_auto_flag;
	}
	else
	  vision_flag = 0;
	
  vision_last_flag = vision_flag;
	
}



/**
  * @brief          ��̨ͨ��ֵ����
  * @param[in]      null
  */
//fp32 mouse_yaw, mouse_pitch;

void system_t ::Gimbal_value_calc()
{
	static fp32 mouse_yaw, mouse_pitch;

  static int16_t yaw_channel = 0, pitch_channel = 0;
	//����ӦYAW,PITCH��������
//	mouse_yaw = LPF(&yaw_lpf ,0.002,system_rc_ctrl->mouse.x * YAW_MOUSE_SEN, 14);
//	mouse_pitch = LPF(&pitch_lpf ,0.002,system_rc_ctrl->mouse.y * PITCH_MOUSE_SEN, 30);


  mouse_yaw = LPF(&yaw_lpf ,0.002,system_rc_ctrl->mouse.x , 12);
	mouse_pitch = LPF(&pitch_lpf ,0.002,system_rc_ctrl->mouse.y , 12);
	//��ң���������ݴ������� int16_t yaw_channel,pitch_channel
	rc_deadband_limit(system_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
  rc_deadband_limit(system_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);

	rc_add_yaw = (yaw_channel * YAW_RC_SEN - 
		            mouse_yaw * YAW_MOUSE_SEN+//gimbal_control_set->gimbal_rc_ctrl->mouse.x * YAW_MOUSE_SEN + 
		           (system_rc_ctrl->key.v & GIMBAL_LEFT_KEY)*0.00003 - 
				  	   (system_rc_ctrl->key.v & GIMBAL_RIGHT_KEY)*0.000015)+vision_info_point()->RxPacketSed.target_angular[2]*CHASSIS_WZ_RE_SEN;
	
	rc_add_pit = (pitch_channel * PITCH_RC_SEN + 
								mouse_pitch* PITCH_MOUSE_SEN //gimbal_control_set->gimbal_rc_ctrl->mouse.y * PITCH_MOUSE_SEN
							 );
}

/**
  * @brief          ����ͨ��ֵ����
  * @param[in]      null
  */

void system_t::Chassis_value_calc()
{
	int16_t vx_channel, vy_channel;
	fp32 vx_set_channel, vy_set_channel;
	//�������ƣ���Ϊң�������ܴ��ڲ��� ҡ�����м䣬��ֵ��Ϊ0
  rc_deadband_limit(system_rc_ctrl->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
  rc_deadband_limit(system_rc_ctrl->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);
	//��������
	vx_set_channel = -vx_channel * CHASSIS_VX_RC_SEN+vision_info_point()->RxPacketSed.target_linear[0]*CHASSIS_VX_RE_SEN;
  vy_set_channel = vy_channel * CHASSIS_VY_RC_SEN+vision_info_point()->RxPacketSed.target_linear[1]*CHASSIS_VY_RE_SEN;
	
	if (system_rc_ctrl->key.v & CHASSIS_FRONT_KEY)
  {
    vx_set_channel = -NORMAL_MAX_CHASSIS_SPEED_X;
  }
  else if (system_rc_ctrl->key.v & CHASSIS_BACK_KEY)
  {
    vx_set_channel = NORMAL_MAX_CHASSIS_SPEED_X;
  }

  if (system_rc_ctrl->key.v & CHASSIS_LEFT_KEY)
  {
    vy_set_channel = -NORMAL_MAX_CHASSIS_SPEED_Y;
  }
  else if (system_rc_ctrl->key.v & CHASSIS_RIGHT_KEY)
  {
    vy_set_channel = NORMAL_MAX_CHASSIS_SPEED_Y;
  }
	
	chassis_cmd_slow_set_vx.out = RAMP_float(vx_set_channel,chassis_cmd_slow_set_vx.out,0.004);
	chassis_cmd_slow_set_vy.out = RAMP_float(vy_set_channel,chassis_cmd_slow_set_vy.out,0.004);
	//ֹͣ�źţ�����Ҫ�������٣�ֱ�Ӽ��ٵ���
	if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
  {
    chassis_cmd_slow_set_vx.out = 0.0f;
  }

  if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
  {
    chassis_cmd_slow_set_vy.out = 0.0f;
  }
	//��������
	if(IF_KEY_PRESSED_SHIFT)  //�򿪳�������
	{
		chassis_cmd_slow_set_vx.out = chassis_cmd_slow_set_vx.out*1.5f;
    chassis_cmd_slow_set_vy.out = chassis_cmd_slow_set_vy.out*1.5f;
	}
}


/**
  * @brief          Ħ����ģʽ��ʼ��
  * @param[in]      null
  */

void Friction_wheel_Mode_set::Friction_Wheel_behaviour_init()
{
	Friction_wheel_RC = get_remote_control_point();
	friction_wheel_mode = FRICTION_WHEEL_OFF;
}

/**
  * @brief          Ħ����ģʽ����
  * @param[in]      null
  */
	
void Friction_wheel_Mode_set::Friction_wheel_mode_set()
{
	if((switch_is_down(Friction_wheel_RC->rc.s[0]) == 0||syspoint()->sys_mode==AUTO)&&((switch_is_down(Friction_wheel_RC->rc.s[1]) == 0))&&syspoint()->sys_mode!=DBUS_MISSING_CONTROL)//���������� Ħ���ֿ���ģʽ
	{
		friction_wheel_mode = FRICTION_WHEEL_ON;
		Friction_wheel_mode_on_set();
	}
	else
	{
		friction_wheel_mode = FRICTION_WHEEL_OFF;
		Friction_wheel_mode_off_set();
		
	}
}

/**
  * @brief          Ħ���ֿ���ģʽ���趨ֵ����
  * @param[in]      null
  */

void Friction_wheel_Mode_set::Friction_wheel_mode_on_set()
{
	  speedlimit = JUDGE_usGetSpeedLimit();
		//speedlimit = 15;
		User_fix();

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
  * @brief           Ħ���ֹر�ģʽ���趨ֵ����
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
  * @brief           �������ֶ�����Ħ����ת������
  * @param[in]       null
  */
void Friction_wheel_Mode_set::User_fix()
{
	if(IF_KEY_PRESSED_C&&IF_KEY_PRESSED_CTRL)//c�� v��
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
	
}


/**
  * @brief           Ħ�����ֶ�����
  * @param[in]       null
  */

void Friction_wheel_Mode_set::Temp_Fix_15S()
{
   v_fic_set = FRICTION_L1_SPEED+fixed;	
}
/**
  * @brief           Ħ�����ֶ�����
  * @param[in]       null
  */
void Friction_wheel_Mode_set::Temp_Fix_18S()
{
   v_fic_set = FRICTION_L2_SPEED+fixed;	
}
/**
  * @brief           Ħ�����ֶ�����
  * @param[in]       null
  */
void Friction_wheel_Mode_set::Temp_Fix_30S()
{
   v_fic_set = FRICTION_L3_SPEED+fixed;	
}

/**
  * @brief           ����ģʽ��ʼ��
  * @param[in]       null
  */

void Dial_mode_set::Dial_mode_init()
{
		Dial_RC = get_remote_control_point();
		Revolver_mode=REVOL_POSI_MODE;
		set_angle =angle;
}

/**
  * @brief           ����ģʽ���ü��趨ֵ����
  * @param[in]       null
  */

void Dial_mode_set::Dial_behaviour_set()
{
		static int16_t speed_time;
	
		//�󲦸˲���һ�λ�����������ΪSTEP1  �󲦸�һֱ���ϻ򳤰����ΪSTEP2
		if ( (switch_is_up(Dial_RC->rc.s[RevChannel])||Dial_RC->mouse.press_l==1)&&((switch_is_down(Dial_RC->rc.s[0])!=1)||syspoint()->sys_mode==AUTO))
		{
			speed_time++;
			Revolver_Switch = REVOL_STEP1;
			
			if(speed_time>300)
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
	
		//STEP2�����ٶȻ�
		if(Revolver_Switch == REVOL_STEP2&&ifstuck_finish != stuck_unfinish)
				Revolver_mode =	REVOL_SPEED_MODE;
		else if((Revolver_Switch == REVOL_STEP1||Revolver_Switch == REVOL_STEP0)&&ifstuck_finish != stuck_unfinish)
			  Revolver_mode =	REVOL_POSI_MODE;
		
		//���˲���һ�λ��������һ�η���1 ����Ϊ0
		if(Revolver_Switch == REVOL_STEP1 && Revolver_last_Switch == REVOL_STEP0)
		{
			if(heating<heat_max|| heat_max == 0)//abs(revolver_control->set_angle- revolver_control->angle)<1000) || heat_max == 0)
			{
				key_ShootFlag = shoot_ready;
			}
		}
		else
		key_ShootFlag = shoot_unready;
		
		Revolver_last_Switch = Revolver_Switch;

    //�������
		if(abs(set_angle - angle)> AN_BULLET/2)//����Ӧ����С��һ��ĵ��ӵ�λ���Ϊ����δ������
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
		}
		//�����뿨��ģʽ
		if(Revolver_mode == REVOL_REVERSAL_MODE&&last_Revolver_mode!=REVOL_REVERSAL_MODE)
		{
			set_angle += AN_BULLET*3/2;
		}

		else if(Revolver_mode == REVOL_POSI_MODE)
		{
			if(key_ShootFlag == shoot_ready&&ifshoot_finsh != shoot_unfinish)//&&abs(revolver_control->set_angle - revolver_control->last_angle)<1000)
			{		
					set_angle -= AN_BULLET;	
				 if(if_stucked == 1)
				{
					set_angle += AN_BULLET/2;
					if_stucked = 0;
					stuck_mode=0;
				}
			}
		}
		//�������ֵ��������
		else if(Revolver_mode == REVOL_SPEED_MODE)
		{
			//������������
			if( ifshoot_finsh != shoot_unfinish && (heating<heat_max || heat_max ==0) )
				{
					//˫������
					set_angle-= AN_BULLET;
					if(if_stucked == 1)
					{
						set_angle+= AN_BULLET/2;
						if_stucked = 0;
						stuck_mode = 0;
					}
				}
		}
		 last_Revolver_mode = Revolver_mode;
		if(switch_is_down(Dial_RC->rc.s[RevChannel]))
		{
		  set_angle=angle;

		}
}

