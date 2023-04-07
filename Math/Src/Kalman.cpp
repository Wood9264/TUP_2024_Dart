/**    
  * @author  Liu heng
  * 一阶卡尔曼滤波器来自RoboMaster论坛  
  *   一维卡尔曼滤波器                     
  *   使用时先定义一个kalman指针，然后调用kalmanCreate()创建一个滤波器 
  *   每次读取到传感器数据后即可调用KalmanFilter()来对数据进行滤波
  *          使用示例                                             
  *          extKalman p;                  //定义一个卡尔曼滤波器结构体                                                 
  *          float SersorData;             //需要进行滤波的数据                                          
  *          KalmanCreate(&p,20,200);      //初始化该滤波器的Q=20 R=200参数                                                  
  *          while(1)                                                                
  *          {                                                                            
  *             SersorData = sersor();                     //获取数据                                           
  *             SersorData = KalmanFilter(&p,SersorData);  //对数据进行滤波                                                                            
  *          }                                                                            
  */

#include "kalman.h"

/**
  * @name   kalmanCreate
  * @brief  创建一个卡尔曼滤波器
  * @param  p:  滤波器
  *         T_Q:系统噪声协方差
  *         T_R:测量噪声协方差
  *         
  * @retval none
  * @attention R固定，Q越大，代表越信任侧量值，Q无穷代表只用测量值
  *		       	反之，Q越小代表越信任模型预测值，Q为零则是只用模型预测
  */
void extKalman_t::KalmanCreate(float T_Q,float T_R)
{
	X_last = (float)0;
	P_last = 0;
	Q = T_Q;
  R = T_R;
  A = 1;
	B = 0;
  H = 1;
  X_mid = X_last;
}

/**
  * @name   KalmanFilter
  * @brief  卡尔曼滤波器
  * @param  p:  滤波器
  *         dat:待滤波数据
  * @retval 滤波后的数据
  * @attention Z(k)是系统输入,即测量值   X(k|k)是卡尔曼滤波后的值,即最终输出
  *            A=1 B=0 H=1 I=1  W(K)  V(k)是高斯白噪声,叠加在测量值上了,可以不用管
  *            以下是卡尔曼的5个核心公式
  *            一阶H'即为它本身,否则为转置矩阵
  */
float extKalman_t::KalmanFilter(float dat)
{
	  X_mid = A * X_last;                     //百度对应公式(1)    x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
    P_mid = A * P_last + Q;               //百度对应公式(2)    p(k|k-1) = A*p(k-1|k-1)*A'+Q
    kg = P_mid / (P_mid + R);             //百度对应公式(4)    kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
    X_now = X_mid + kg * (dat - X_mid);     //百度对应公式(3)    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    P_now = (1 - kg) * P_mid;                //百度对应公式(5)    p(k|k) = (I-kg(k)*H)*P(k|k-1)
    P_last = P_now;                         //状态更新
    X_last = X_now;
    return X_now;							  //输出预测结果x(k|k)
	
}
