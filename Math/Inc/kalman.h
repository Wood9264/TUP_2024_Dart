#ifndef __KALMAN_H
#define __KALMAN_H

#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus
class extKalman_t
{
	public:
		float X_last; //上一时刻的最优结果  X(k-|k-1)
    float X_mid;  //当前时刻的预测结果  X(k|k-1)
    float X_now;  //当前时刻的最优结果  X(k|k)
    float P_mid;  //当前时刻预测结果的协方差  P(k|k-1)
    float P_now;  //当前时刻最优结果的协方差  P(k|k)
    float P_last; //上一时刻最优结果的协方差  P(k-1|k-1)
    float kg;     //kalman增益
    float A;      //系统参数
	  float B;
    float Q;
    float R;
    float H;
	
	  void KalmanCreate(float T_Q,float T_R);
	  float KalmanFilter(float dat);
};	
	
#endif

#ifdef __cplusplus
}	
#endif

#endif
