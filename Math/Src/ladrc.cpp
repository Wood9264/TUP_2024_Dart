#include "ladrc.h"
#include "user_lib.h"
#include "arm_math.h"
#include "math.h"
#include "main.h"

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
	
void LADRC_t::init(fp32 wc,fp32 b0,fp32 wo,fp32 max_out)
{
	own_wc = wc;
	own_wo = wo;
	own_b0 = b0;
	
	own_max_out = max_out;
	own_fdb = u = own_set = own_gyro = 0.0f;
	z1 = z2 = 0.0f;
	time_cons = 0.002;//采样率
}		

fp32 LADRC_t::calc(fp32 ref, fp32 set, fp32 gyro)
{
	fp32 err;
	err = set - ref;

	own_set = set;
	own_fdb = ref;
	own_err = rad_format(err);
	own_gyro = gyro;
	
	//零阶保持法离散化积分器
  z2 += time_cons * (own_wo * own_wo) * (own_gyro - z1);
	z1 += time_cons * ((own_b0 * u) + z2 + (2 * own_wo) * (own_gyro - z1));
	u = (own_wc * own_wc * own_err - 2 * own_wc * z1 - z2) / own_b0;
	LimitMax(u,own_max_out);
	
	return u;
}



