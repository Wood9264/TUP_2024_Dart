#ifndef __LADRC_FEEDFORWARD_H
#define __LADRC_FEEDFORWARD_H

#include "struct_typedef.h"
#include "ladrc.h"

#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus
typedef struct
{   
	fp32 y[2];
	fp32 u[2];
} differ_type_def;	

typedef struct
{   
	fp32 y[2];
	fp32 u[2];
} lpf_type_def;

class	LADRC_FDW_t:public LADRC_t 
{
  private:
	  differ_type_def differ1;
	  differ_type_def differ2;
	  fp32 dif1;
	  fp32 dif2; 
	  fp32 own_w;//前馈带宽
	  fp32 own_gain;//前馈增益
	
	  //输入
	  fp32 own_set_last;
	public:
		void init(fp32 wc, fp32 b0 ,fp32 wo,fp32 max_out,fp32 w,fp32 gain);
	  fp32 FDW_calc(fp32 ref, fp32 set, fp32 gyro);
};
	
extern fp32 LPF(lpf_type_def *lpf ,fp32 time_cons,fp32 input,fp32 w);
#endif

#ifdef __cplusplus
}
#endif

#endif
