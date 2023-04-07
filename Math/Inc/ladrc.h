#ifndef __LADRC_H
#define __LADRC_H

#include "struct_typedef.h"

#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus
class LADRC_t
{
	public:
		fp32 time_cons; //时间常数
	
	  fp32 own_wo;//观测器带宽
		fp32 own_b0;//输出增益
		fp32 z1;
		fp32 z2;
	
	  fp32 own_wc;//控制器带宽
	
    fp32 own_max_out;  //最大输出
	  //输入
    fp32 own_set;//设定值
    fp32 own_fdb;//反馈值
		fp32 own_gyro;//角速度
		fp32 own_err;
	
		fp32 u;
	
		void init(fp32 wc,fp32 b0,fp32 wo,fp32 max_out);
	  fp32 calc(fp32 ref, fp32 set, fp32 gyro);
};
	
	
#endif

#ifdef __cplusplus
}
#endif

#endif

