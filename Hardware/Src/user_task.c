/**
  ****************************辽宁科技大学COD****************************
  * @file       user_task.c/h-雨落长安
  * @brief      一个普通程序，请大家友善对待，谢谢大家，写的demo，未来得及封装，凑合看吧
  ==============================================================================
  @endverbatim
  ****************************辽宁科技大学COD****************************
  */

#include "user_task.h"
#include "main.h"
#include "RM_Cilent_UI.h"
#include "string.h"
#include "CAN_Receive.h"//加载摩擦轮发射闭环真实转速
#define PI 3.1415936

Graph_Data G1,G2,G3,G4,G5,G6,G7,G8,G9,G10,G11;
String_Data CH_SHOOT;
String_Data CH_FLRB;
char shoot_arr[5]="shoot";
char flrb_arr[4]="FRBL";
void UserTask(void *pvParameters)
{
		memset(&G1,0,sizeof(G1));//中心垂线
		memset(&G2,0,sizeof(G2));//上击打线
		memset(&G3,0,sizeof(G3));//中心水平线
		memset(&G4,0,sizeof(G4));//枪管轴心线
		memset(&G5,0,sizeof(G5));//下击打线
		memset(&G6,0,sizeof(G6));//远距离击打线
		memset(&G7,0,sizeof(G7));//摩擦轮状态
		memset(&CH_SHOOT,0,sizeof(CH_SHOOT));//摩擦轮标识
	    memset(&G8,0,sizeof(G8));//前装甲板状态
	    memset(&G9,0,sizeof(G9));//左装甲板状态
	    memset(&G10,0,sizeof(G10));//右装甲板状态
	    memset(&G11,0,sizeof(G11));//后装甲板状态
	    memset(&CH_FLRB,0,sizeof(CH_FLRB));//装甲板标识

        Line_Draw(&G1,"091",UI_Graph_ADD,9,UI_Color_Purplish_red,1,960,330,960,620);
		Line_Draw(&G2,"092",UI_Graph_ADD,9,UI_Color_Purplish_red,1,880,580,1040,580);
		Line_Draw(&G3,"093",UI_Graph_ADD,9,UI_Color_Purplish_red,1,800,540,1120,540);
		Line_Draw(&G4,"094",UI_Graph_ADD,9,UI_Color_Purplish_red,1,880,500,1040,500);
		Line_Draw(&G5,"095",UI_Graph_ADD,9,UI_Color_Purplish_red,1,900,420,1020,420);
	    Line_Draw(&G6,"096",UI_Graph_ADD,9,UI_Color_Purplish_red,1,920,370,1000,370);
	    Circle_Draw(&G7,"097",UI_Graph_ADD,9,UI_Color_Yellow,15,230,770,15);
	    Char_Draw(&CH_SHOOT,"087",UI_Graph_ADD,8 ,UI_Color_Yellow,24,5,4,80,780,&shoot_arr[0]);
		UI_ReFresh(7,G1,G2,G3,G4,G5,G6,G7);                          //绘制图形	
        Char_ReFresh(CH_SHOOT);
		static double angle =0,heat=0,heat_door=0;//这里angle直接赋成差值
		static int flag_cnn1=0,flag_cnn2=0,heat_flag1=0,heat_flag2=0;;//计时1//计时2//受击打标志位//受击打备用标志位;
		Circle_Draw(&G8,"081",UI_Graph_ADD,8,UI_Color_Green,3,960+(int)340*sin((angle)*2*PI/360.0),540+(int)340*cos((angle)*2*PI/360.0),50);
		Circle_Draw(&G9,"082",UI_Graph_ADD,8,UI_Color_Green,3,960+(int)340*sin((angle+90)*2*PI/360.0),540+(int)340*cos((angle+90)*2*PI/360.0),50);
		Circle_Draw(&G10,"083",UI_Graph_ADD,8,UI_Color_Green,3,960+(int)340*sin((angle+180)*2*PI/360.0),540+(int)340*cos((angle+180)*2*PI/360.0),50);
		Circle_Draw(&G11,"084",UI_Graph_ADD,8,UI_Color_Green,3,960+(int)340*sin((angle+270)*2*PI/360.0),540+(int)340*cos((angle+270)*2*PI/360.0),50);
		UI_ReFresh(5,G7,G8,G9,G10,G11);//G7缺省   
		Char_Draw(&CH_FLRB,"077",UI_Graph_ADD,7 ,UI_Color_Yellow,24,1,4,960+(int)340*sin((angle)*2*PI/360.0),540+(int)340*cos((angle)*2*PI/360.0),&flrb_arr[0]);
		Char_ReFresh(CH_FLRB);
		Char_Draw(&CH_FLRB,"076",UI_Graph_ADD,7,UI_Color_Yellow,24,1,4,960+(int)340*sin((angle+90)*2*PI/360.0),540+(int)340*cos((angle+90)*2*PI/360.0),&flrb_arr[1]);
		Char_ReFresh(CH_FLRB);
		Char_Draw(&CH_FLRB,"075",UI_Graph_ADD,7 ,UI_Color_Yellow,24,1,4,960+(int)340*sin((angle+180)*2*PI/360.0),540+(int)340*cos((angle+180)*2*PI/360.0),&flrb_arr[2]);
		Char_ReFresh(CH_FLRB);
		Char_Draw(&CH_FLRB,"074",UI_Graph_ADD,7 ,UI_Color_Yellow,24,1,4,960+(int)340*sin((angle+270)*2*PI/360.0),540+(int)340*cos((angle+270)*2*PI/360.0),&flrb_arr[3]);
		Char_ReFresh(CH_FLRB);
  while(1)
	{
		//模拟装甲板受击判定||将装甲板ID对应angle，这段4个angle用旋转来模拟的装甲板移动与被击打！！！！！
		if(angle==50) heat=1;
		if(angle==80) heat=2;
		if(angle==85) heat=2;
		if(angle==90) heat=3;
		if(heat) heat_door=heat;
		if(heat_door)
		{		
			if(!heat_flag1){heat_flag1=heat;flag_cnn1=0;}//首次被击打
			else if(heat_flag1==heat) {flag_cnn1=0;}//二次被击打在同一装甲板
			else if(heat_flag1!=heat) 
				{ 
					if(!heat_flag2) {heat_flag2=heat;flag_cnn2=0;}
					if(heat_flag2==heat) {flag_cnn2=0;}
				  else if(heat){ if(flag_cnn1>=flag_cnn2){heat_flag1=heat;flag_cnn1=0;}else {heat_flag2=heat;flag_cnn2=0;}}
				}//二次击打位置不在同一装甲板
			if(flag_cnn1>=20) 
				{
					heat_flag1=0;flag_cnn1=0;
				}
			else if(heat_flag1) flag_cnn1+=1;//时间标志位1累加
			if(flag_cnn2>=20) 
				{
					heat_flag2=0;flag_cnn2=0;
				}
			else if(heat_flag2) flag_cnn2+=1;//时间标志位2累加
			heat=0;
		}
			//左右摩擦轮发射UI,初始化黄色，出错紫红色，正常绿色
		if(Shoot_Right_Date.V&&Shoot_Right_Date.V)
			Circle_Draw(&G7,"007",UI_Graph_Change,9,UI_Color_Green,15,230,770,15);
		else Circle_Draw(&G7,"007",UI_Graph_Change,9,UI_Color_Purplish_red,15,230,770,15);
		//装甲板旋转移动，正常绿色，受击紫红色闪烁，两s反应时间
		if(heat_flag1==1||heat_flag2==1)
			Circle_Draw(&G8,"081",UI_Graph_Change,8,UI_Color_Pink,10,960+(int)340*sin((angle)*2*PI/360.0),540+(int)340*cos((angle)*2*PI/360.0),50);
		else Circle_Draw(&G8,"081",UI_Graph_Change,8,UI_Color_Green,3,960+(int)340*sin((angle)*2*PI/360.0),540+(int)340*cos((angle)*2*PI/360.0),50);
		if(heat_flag1==2||heat_flag2==2) 
			Circle_Draw(&G9,"082",UI_Graph_Change,8,UI_Color_Pink,10,960+(int)340*sin((angle+90)*2*PI/360.0),540+(int)340*cos((angle+90)*2*PI/360.0),50);
		else Circle_Draw(&G9,"082",UI_Graph_Change,8,UI_Color_Green,3,960+(int)340*sin((angle+90)*2*PI/360.0),540+(int)340*cos((angle+90)*2*PI/360.0),50);
		if(heat_flag1==3||heat_flag2==3) 	
			Circle_Draw(&G10,"083",UI_Graph_Change,8,UI_Color_Pink,10,960+(int)340*sin((angle+180)*2*PI/360.0),540+(int)340*cos((angle+180)*2*PI/360.0),50);
		else Circle_Draw(&G10,"083",UI_Graph_Change,8,UI_Color_Green,3,960+(int)340*sin((angle+180)*2*PI/360.0),540+(int)340*cos((angle+180)*2*PI/360.0),50);
		if(heat_flag1==4||heat_flag2==4) 	
		  Circle_Draw(&G11,"084",UI_Graph_Change,8,UI_Color_Pink,10,960+(int)340*sin((angle+270)*2*PI/360.0),540+(int)340*cos((angle+270)*2*PI/360.0),50);
		else Circle_Draw(&G11,"084",UI_Graph_Change,8,UI_Color_Green,3,960+(int)340*sin((angle+270)*2*PI/360.0),540+(int)340*cos((angle+270)*2*PI/360.0),50);
		UI_ReFresh(5, G7, G8, G9, G10, G11);
		Char_Draw(&CH_FLRB, "077", UI_Graph_Change, 7, UI_Color_Yellow, 24, 1, 4, 960 + (int)340 * sin((angle) * 2 * PI / 360.0), 540 + (int)340 * cos((angle) * 2 * PI / 360.0), &flrb_arr[0]);
		Char_ReFresh(CH_FLRB);
		Char_Draw(&CH_FLRB, "076", UI_Graph_Change, 7, UI_Color_Yellow, 24, 1, 4, 960 + (int)340 * sin((angle + 90) * 2 * PI / 360.0), 540 + (int)340 * cos((angle + 90) * 2 * PI / 360.0), &flrb_arr[1]);
		Char_ReFresh(CH_FLRB);
		Char_Draw(&CH_FLRB, "075", UI_Graph_Change, 7, UI_Color_Yellow, 24, 1, 4, 960 + (int)340 * sin((angle + 180) * 2 * PI / 360.0), 540 + (int)340 * cos((angle + 180) * 2 * PI / 360.0), &flrb_arr[2]);
		Char_ReFresh(CH_FLRB);
		Char_Draw(&CH_FLRB, "074", UI_Graph_Change, 7, UI_Color_Yellow, 24, 1, 4, 960 + (int)340 * sin((angle + 270) * 2 * PI / 360.0), 540 + (int)340 * cos((angle + 270) * 2 * PI / 360.0), &flrb_arr[3]);
		Char_ReFresh(CH_FLRB);
		//频率控制10hz
		vTaskDelay(100);
	}		
}




