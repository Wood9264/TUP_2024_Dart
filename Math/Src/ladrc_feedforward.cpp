#include "ladrc_feedforward.h"
#include "user_lib.h"
#include "math.h"
#include "arm_math.h"
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }
	

void LADRC_FDW_t::init(fp32 wc, fp32 b0 ,fp32 wo,fp32 max_out,fp32 w,fp32 gain)
{
	own_wc = wc;
	own_wo = wo;
	own_b0 = b0;
	own_gain = gain;
	own_w = w;
	
	own_max_out = max_out;
	own_fdb = u = own_set = own_gyro = 0.0f;
	z1 = z2 = 0.0f;
	time_cons = 0.002;//采样率
}

fp32 differentiator(differ_type_def *differ ,fp32 bandwidth,fp32 time_cons,fp32 input)
{ 
  //使用梯形法离散化
 
  differ->u[0] = differ->u[1];	
  differ->u[1] = input;
  differ->y[0] = differ->y[1];
  differ->y[1] = (bandwidth*(differ->u[1]-differ->u[0])-(bandwidth*time_cons/2-1)*differ->y[0])/(1+(bandwidth*time_cons)/2);
  
  return differ->y[1];
}

fp32 LADRC_FDW_t::FDW_calc(fp32 ref, fp32 set, fp32 gyro)
{
  own_err = rad_format(set - ref);
	own_set_last = own_set;
	own_set = set;
	own_fdb = ref;
	
	own_gyro = gyro;
	if(fabsf(own_set - own_set_last) > 3.0f)//如果发生了跳变,防止产生过大的前馈量
	{
		differ1.y[0] = 0;
		differ1.y[1] = 0;
		differ1.u[0] = own_set;
		differ1.u[1] = own_set;
		
		differ2.y[0] = 0;
		differ2.y[1] = 0;
		differ2.u[0] = 0;
		differ2.u[1] = 0;
	}
	dif1 = differentiator(&differ1,own_w,time_cons,set);
	dif2 = differentiator(&differ2,own_w,time_cons,dif1);
	
	//带前馈的ladrc算法，先把前馈增益置0，调好控制器，再调节前馈器，前馈增益一般不大于1.
	
	//零阶保持法离散化积分器
	z2 += time_cons * (own_wo * own_wo) * (own_gyro - z1);
	z1 += time_cons * ((own_b0 * u) + z2 + (2 * own_wo) * (own_gyro - z1));
	u = (own_wc * own_wc * own_err + 2 * own_wc * (own_gain * dif1 - z1) - z2 + own_gain * dif2) / own_b0;
	LimitMax(u,own_max_out);
	
	return u;
}

fp32 LPF(lpf_type_def *lpf ,fp32 time_cons,fp32 input,fp32 w)
{ 
  //使用梯形法离散化
 
//  lpf->u[0] = lpf->u[1];	
//  lpf->u[1] = input; 
//  lpf->y[0] = lpf->y[1];
//  lpf->y[1] = (((lpf->u[1]-lpf->u[0])*w)-(w*time_cons/2 -1)*lpf->y[0]) /(1+(w*time_cons)/2);
//  
  lpf->u[0] = lpf->u[1];	
  lpf->u[1] = input; 
  lpf->y[0] = lpf->y[1];
  lpf->y[1] = (lpf->u[1]+lpf->u[0])/(2/time_cons + w)*w - lpf->y[0]*(w - 2/time_cons)/(2/time_cons + w);
  
  return lpf->y[1];
}
