#ifndef __FILTER_H
#define __FILTER_H

#include "struct_typedef.h"

#define LPF_1_(hz,t,in,out) ((out) += ( 1 / ( 1 + 1 / ( (hz) *6.28f *(t) ) ) ) *( (in) - (out) ))
#define my_pow(a) ((a)*(a))
#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )
#define LPF_1(A,B,C,D) LPF_1_((A),(B),(C),*(D));
#define safe_div(numerator,denominator,safe_value) ( (denominator == 0) ? (safe_value) : ((numerator)/(denominator)) )
typedef struct
{
	float a;
	float b;
	float e_nr;
	float out;
} _filter_1_st;


typedef struct  //视觉目标速度测量
{
  int delay_cnt;//计算相邻两帧目标不变持续时间,用来判断速度是否为0
  int freq;
  int last_time;//上次受到目标角度的时间
  float last_position;//上个目标角度
  float speed;//速度
  float last_speed;//上次速度
  float processed_speed;//速度计算结果
}speed_calc_data_t;




void anotc_filter_1(float base_hz,float gain_hz,float dT,float in,_filter_1_st *f1);

void Moving_Average(float moavarray[],uint16_t len ,uint16_t *fil_cnt,float in,float *out);
int32_t Moving_Median(int32_t moavarray[],uint16_t len ,uint16_t *fil_p,int32_t in);
float Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position);



#define _xyz_f_t xyz_f_t
//void simple_3d_trans(_xyz_f_t *ref, _xyz_f_t *in, _xyz_f_t *out);

#endif
