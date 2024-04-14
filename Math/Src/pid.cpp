#include "pid.h"
#include "cmath"
#include "user_lib.h"
#include "arm_math.h"
#include "ladrc_feedforward.h"
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

void PID_t::init(uint8_t mode,const fp32 PID[3],fp32 max_out,fp32 max_iout)
{
	own_mode = mode;
	own_Kp = PID[0];
	own_Ki = PID[1];
	own_Kd = PID[2];
	own_max_out = max_out;
	own_max_iout = max_iout;
	Dbuf[0] = Dbuf[1] = Dbuf[2] =0.0f;
	error[0] = error[1] =error[2] = Pout = Dout = Iout = out =0.0f;
}

/**
 * @brief 初始化微分器
 * @param init_bandwidth 微分器带宽
 * @param init_time_cons 采样时间
 */
void PID_t::differ_init(fp32 init_bandwidth, fp32 init_time_cons)
{
	bandwidth = init_bandwidth;
	time_cons = init_time_cons;
}

fp32 PID_t::calc(fp32 ref,fp32 set)
{
	error[2] = error[1];
	error[1] = error[0];
	own_set = set;
	own_fdb = ref;
	error[0] = set - ref;
	if(own_mode == PID_POSITION)
	{
		Pout = own_Kp * error[0];
		Iout += own_Ki * error[0];
		Dbuf[2] = Dbuf[1];
		Dbuf[1] = Dbuf[0];
		Dbuf[0] = error[0] - error[1];
		Dout = own_Kd *Dbuf[0];
		LimitMax(Iout,own_max_iout);
    if(isnan(Iout) || isinf(Iout))
			Iout=0;
		out = Pout + Iout + Dout;
    LimitMax(out,own_max_out);
	}
	else if(own_mode == PID_DELTA)
	{
		Pout = own_Kp * (error[0] - error[1]);
		Iout += own_Ki * error[0];
		Dbuf[2] = Dbuf[1];
		Dbuf[1] = Dbuf[0];
		Dbuf[0] = (error[0] - 2.0f *error[1] + error[2]);
		Dout = own_Kd * Dbuf[0];
		out = Pout + Iout + Dout;
		LimitMax(out,own_max_out);
	}
	return out;
}

void PID_t::clear(void)
{
	error[0] = error[1] = error[2] = 0.0f;
	Dbuf[0] = Dbuf[1] = Dbuf[2] = 0.0f;
	out = Pout = Iout = Dout =0.0f;
	own_fdb = own_set = 0.0f;

	differ.u[0] = differ.u[1] = 0.0f;
	differ.y[0] = differ.y[1] = 0.0f;
}

/**
 * @brief		PID动态限幅（dynamic limit）算法。调用此算法时使用指定的限幅，void PID_t::init初始化的限幅不生效
 * @param[in]	ref：当前值
 * @param[in]	set：设定值
 * @param[in]	max_out：限幅
*/
fp32 PID_t::DLcalc(fp32 ref, fp32 set, fp32 max_out)
{
	error[2] = error[1];
	error[1] = error[0];
	own_set = set;
	own_fdb = ref;
	error[0] = set - ref;
	if (own_mode == PID_POSITION)
	{
		Pout = own_Kp * error[0];
		Iout += own_Ki * error[0];
		Dbuf[2] = Dbuf[1];
		Dbuf[1] = Dbuf[0];
		Dbuf[0] = error[0] - error[1];
		Dout = own_Kd * Dbuf[0];
		LimitMax(Iout, own_max_iout);
		if (isnan(Iout) || isinf(Iout))
			Iout = 0;
		out = Pout + Iout + Dout;
		LimitMax(out, max_out);
	}
	else if (own_mode == PID_DELTA)
	{
		Pout = own_Kp * (error[0] - error[1]);
		Iout += own_Ki * error[0];
		Dbuf[2] = Dbuf[1];
		Dbuf[1] = Dbuf[0];
		Dbuf[0] = (error[0] - 2.0f * error[1] + error[2]);
		Dout = own_Kd * Dbuf[0];
		out = Pout + Iout + Dout;
		LimitMax(out, max_out);
	}
	return out;
}

/**
 * @brief		适用于相对角度的PID算法
 * @param[in]	ref：当前值
 * @param[in]	set：设定值
*/
fp32 PID_t::relative_angle_calc(fp32 ref, fp32 set)
{
	error[2] = error[1];
	error[1] = error[0];
	own_set = set;
	own_fdb = ref;
	error[0] = rad_format(set - ref);
	if(own_mode == PID_POSITION)
	{
		Pout = own_Kp * error[0];
		Iout += own_Ki * error[0];
		Dbuf[2] = Dbuf[1];
		Dbuf[1] = Dbuf[0];
		Dbuf[0] = error[0] - error[1];
		Dout = own_Kd *Dbuf[0];
		LimitMax(Iout,own_max_iout);
    if(isnan(Iout) || isinf(Iout))
			Iout=0;
		out = Pout + Iout + Dout;
    LimitMax(out,own_max_out);
	}
	else if(own_mode == PID_DELTA)
	{
		Pout = own_Kp * (error[0] - error[1]);
		Iout += own_Ki * error[0];
		Dbuf[2] = Dbuf[1];
		Dbuf[1] = Dbuf[0];
		Dbuf[0] = (error[0] - 2.0f *error[1] + error[2]);
		Dout = own_Kd * Dbuf[0];
		out = Pout + Iout + Dout;
		LimitMax(out,own_max_out);
	}
	return out;
}

/**
 * @brief 使用离散微分器的PID算法
 * @param ref 当前值
 * @param set 设定值
 * @return fp32 
 */
fp32 PID_t::use_differ_calc(fp32 ref,fp32 set)
{
	error[2] = error[1];
	error[1] = error[0];
	own_set = set;
	own_fdb = ref;
	error[0] = set - ref;
	if(own_mode == PID_POSITION)
	{
		Pout = own_Kp * error[0];
		Iout += own_Ki * error[0];
		Dbuf[2] = Dbuf[1];
		Dbuf[1] = Dbuf[0];
		Dbuf[0] = error[0] - error[1];
		Dout = own_Kd * differentiator(&differ,bandwidth,time_cons,error[0]);
		LimitMax(Iout,own_max_iout);
    if(isnan(Iout) || isinf(Iout))
			Iout=0;
		out = Pout + Iout + Dout;
    LimitMax(out,own_max_out);
	}
	else if(own_mode == PID_DELTA)
	{
		Pout = own_Kp * (error[0] - error[1]);
		Iout += own_Ki * error[0];
		Dbuf[2] = Dbuf[1];
		Dbuf[1] = Dbuf[0];
		Dbuf[0] = (error[0] - 2.0f *error[1] + error[2]);
		Dout = own_Kd * Dbuf[0];
		out = Pout + Iout + Dout;
		LimitMax(out,own_max_out);
	}
	return out;
}

/**
 * @brief 使用离散微分器的相对角度PID算法
 * @param ref 当前值
 * @param set 设定值
 * @return fp32 
 */
fp32 PID_t::relative_angle_use_differ_calc(fp32 ref,fp32 set)
{
	error[2] = error[1];
	error[1] = error[0];
	own_set = set;
	own_fdb = ref;
	error[0] = rad_format(set - ref);
	if(own_mode == PID_POSITION)
	{
		Pout = own_Kp * error[0];
		Iout += own_Ki * error[0];
		Dbuf[2] = Dbuf[1];
		Dbuf[1] = Dbuf[0];
		Dbuf[0] = error[0] - error[1];
		Dout = own_Kd * differentiator(&differ,bandwidth,time_cons,error[0]);
		LimitMax(Iout,own_max_iout);
    if(isnan(Iout) || isinf(Iout))
			Iout=0;
		out = Pout + Iout + Dout;
    LimitMax(out,own_max_out);
	}
	else if(own_mode == PID_DELTA)
	{
		Pout = own_Kp * (error[0] - error[1]);
		Iout += own_Ki * error[0];
		Dbuf[2] = Dbuf[1];
		Dbuf[1] = Dbuf[0];
		Dbuf[0] = (error[0] - 2.0f *error[1] + error[2]);
		Dout = own_Kd * Dbuf[0];
		out = Pout + Iout + Dout;
		LimitMax(out,own_max_out);
	}
	return out;
}
