#include "ui_task.h"
#include "main.h"
#include "struct_typedef.h"
#include "usart.h"
#include "judge.h"
#include "cmsis_os.h"
#include "main.h"
#include "string.h"
#include "stdbool.h"

#include "i2c.h"



#include "FreeRTOS.h"
//#include "vision.h"
//#include "Revolver_task.h"


UI_Info_t UI;
global_flag_t global_flag;
//extern volatile pm01_od_t pm01_od;
extern int update_figure_flag,update_aim_flag,update_float_flag,update_supercapacitor_flag,update_int_flag,steering_mode,firc_mode,stuck_mode,shift_flag,cuff_flag,vuff_flag;
//extern chassis_move_t chassis_move;
extern gimbal_control_t gimbal_control;
extern float global_supercapacitor_remain;//超电
//extVisionRecvData_t Vision_UI;
//extern Revolver_Control_t Revolver_control;

void UI_Init(void)
{  
    update_figure_flag = ADD;
    update_aim_flag = ADD;
    update_float_flag = ADD;
    update_supercapacitor_flag = ADD;    
    update_int_flag = ADD;     
}
void UI_DataUpdate(void)
{
//  global_flag.global_cover    =             steering_mode;
//  global_flag.global_spin     =   	chassis_move.SPIN_flag;
//  global_flag.global_auto     =	  gimbal_control.auto_mode;
//  global_flag.global_firc     = 								 firc_mode;
//  global_flag.global_stuck    =   							stuck_mode;
//  global_flag.global_cuff     =	 								 cuff_flag;
//	global_flag.global_vuff			=								   vuff_flag;
//	global_flag.global_spcp  		=  						 pm01_od.v_out;
//	global_flag.global_fix      =  Revolver_control.fix_times;
}	


void ui_task(void *argumt)
{  
	
	UI_Init();
  uint32_t currentTime;
	static int i = -1,J = 0;
  while(1)
	{	
    currentTime = xTaskGetTickCount();//当前系统时间
    
    i++;
    J++;
		UI_DataUpdate();

		switch(i)
		{   
			case 0:
        Client_graphic_Init();   //添加字符 cover等
        break;
			case 1:
		    
		    _lowlong_aim_(); 
        break;  
			case 2:
        _lowshortstem_aim_4();
      break;
			case 3:
       
			  Client_graphic_Info_update();//圆圈
      break;
    
			case 4:
        global_supercapacitor_remain = ( global_flag.global_spcp - 1400)*0.30 	;
        Client_supercapacitor_update();
        
        break;
    
			case 5:
        _high_aim_();
			  Client_bullet_int_update();
        break;
        
			case 6:
        _lowshort_aim_2();
        break;
			case 7:
        _lowshort_aim_3();
			  update_figure_flag = MODIFY;  
		  	update_supercapacitor_flag = MODIFY;
		  	update_int_flag = MODIFY;
        break;
			default:
        i = -1;
        break;
  }
  
  if(J == 100)
  {
    J = 0;

	}

    vTaskDelayUntil(&currentTime, 50);//绝对延时
		
  }
}







