#include "user_lib.h"
#include "arm_math.h"

//快速开方
fp32 invSqrt(fp32 num)
{
    fp32 halfnum = 0.5f * num;
    fp32 y = num;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(fp32 *)&i;
    y = y * (1.5f - (halfnum * y * y));
    return y;
}

/**
  * @brief          斜波函数初始化
  * @author         RM
  * @param[in]      斜波函数结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      最大值
  * @param[in]      最小值
  * @retval         返回空
  */
void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min)
{
    ramp_source_type->frame_period = frame_period;
    ramp_source_type->max_value = max;
    ramp_source_type->min_value = min;
    ramp_source_type->input = 0.0f;
    ramp_source_type->out = 0.0f;
}

/**
  * @brief          斜波函数计算，根据输入的值进行叠加， 输入单位为 /s 即一秒后增加输入的值
  * @author         RM
  * @param[in]      斜波函数结构体
  * @param[in]      输入值
  * @param[in]      滤波参数
  * @retval         返回空
  */
void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input)
{
    ramp_source_type->input = input;
    ramp_source_type->out += ramp_source_type->input * ramp_source_type->frame_period;
    if (ramp_source_type->out > ramp_source_type->max_value)
    {
        ramp_source_type->out = ramp_source_type->max_value;
    }
    else if (ramp_source_type->out < ramp_source_type->min_value)
    {
        ramp_source_type->out = ramp_source_type->min_value;
    }
}
/**
  * @brief          一阶低通滤波初始化
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      滤波参数
  * @retval         返回空
  */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1])
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0] = num[0];
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

/**
  * @brief          一阶低通滤波计算
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @retval         返回空
  */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
        first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) *
	first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) 
	* first_order_filter_type->input;

}

//绝对限制
void abs_limit(fp32 *num, fp32 Limit)
{
    if (*num > Limit)
    {
        *num = Limit;
    }
    else if (*num < -Limit)
    {
        *num = -Limit;
    }
}

//判断符号位
fp32 sign(fp32 value)
{
    if (value >= 0.0f)
    {
        return 1.0f;
    }
    else
    {
        return -1.0f;
    }
}

//浮点死区
fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0.0f;
    }
    return Value;
}

//int26死区
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0;
    }
    return Value;
}

//限幅函数
fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

//限幅函数
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

//循环限幅函数   超过之后加减区间实现循环
fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        fp32 len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        fp32 len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

//弧度格式化为-PI~PI

//角度格式化为-180~180
fp32 theta_format(fp32 Ang)
{
    return loop_fp32_constrain(Ang, -180.0f, 180.0f);
}

/**
 * @brief       斜坡函数，使目标输出值缓慢等于期望值
 * @param[in]   final 期望最终输出
 * @param[in]   now 当前值
 * @param[in]   ramp 变化速度(越大越快)，必须为正数
 * @retval      当前输出
 */
float RAMP_float(float final, float now, float ramp)
{
    fp32 buffer = 0;

    buffer = final - now;

    if (buffer > 0)
    {
        if (buffer > ramp)
        {
            now += ramp;
        }
        else
        {
            now = final;
        }
    }
    else
    {
        if (buffer < -ramp)
        {
            now += -ramp;
        }
        else
        {
            now = final;
        }
    }

    return now;
}

/**
 * @brief       带循环限幅的斜坡函数,使目标输出值缓慢等于期望值
 * @param[in]   final 期望最终输出
 * @param[in]   now 当前值
 * @param[in]   ramp 变化速度(越大越快)，必须为正数
 * @retval      当前输出
 */
fp32 RAMP_float_loop_constrain(float final, float now, float ramp)
{
    fp32 buffer = 0;

    buffer = rad_format(final - now);

    if (buffer > 0)
    {
        if (buffer > ramp)
        {
            now += ramp;
        }
        else
        {
            now = final;
        }
    }
    else
    {
        if (buffer < -ramp)
        {
            now += -ramp;
        }
        else
        {
            now = final;
        }
    }

    return now;
}

/**
* @brief 获取目标的差分
* @param void
* @return void
*	以队列的逻辑
*/
float Get_Diff(uint8_t queue_len, QueueObj *Data,float add_data)
{
    if(queue_len>=Data->queueLength)
        queue_len=Data->queueLength;
    //防止溢出
    Data->queueTotal-=Data->queue[Data->nowLength];
    Data->queueTotal+=add_data;

    Data->queue[Data->nowLength]=add_data;
	
    Data->nowLength++;

    if(Data->full_flag==0)//初始队列未满
    {
        Data->aver_num=Data->queueTotal/Data->nowLength;
    }else if(Data->full_flag == 1)
	{
	    Data->aver_num=(Data->queueTotal)/queue_len;			
	}
    if(Data->nowLength>=queue_len)
    {
        Data->nowLength=0;
        Data->full_flag=1;
    }

    Data->Diff=add_data - Data->aver_num;
    return Data->Diff;
}
/**
* @brief 清空队列
* @param void
* @return void
*	以队列的逻辑
*/
void Clear_Queue(QueueObj* queue)
{
    for(uint16_t i=0; i<queue->queueLength; i++)
    {
        queue->queue[i]=0;
    }
    queue->nowLength = 0;
    queue->queueTotal = 0;
    queue->aver_num=0;
    queue->Diff=0;
    queue->full_flag=0;
}

