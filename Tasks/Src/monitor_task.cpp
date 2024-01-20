#include "monitor_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "task.h"
#include "bsp_buzzer.h"

#define RGB_FLOW_COLOR_CHANGE_TIME  1000
#define RGB_FLOW_COLOR_LENGHT   6
//blue-> green(dark)-> red -> blue(dark) -> green(dark) -> red(dark) -> blue
//蓝 -> 绿(灭) -> 红 -> 蓝(灭) -> 绿 -> 红(灭) -> 蓝 
uint32_t RGB_flow_color[RGB_FLOW_COLOR_LENGHT + 1] = {0xFF0000FF, 0x0000FF00, 0xFFFF0000, 0x000000FF, 0xFF00FF00, 0x00FF0000, 0xFF0000FF};
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

buzzer_t buzzer;

static void led_monitor(void);
static void buzzer_monitor(void);
void buzzer_warn_init(void);

/**
  * @brief          监视任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void monitor_task(void const *argument)
{
	vTaskDelay(1000);
	buzzer_warn_init();

	while (1)
	{
		led_monitor();
		buzzer_monitor();
		vTaskDelay(1);
	}
}

/**
  * @brief          led监控任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
static void led_monitor(void)
{
	static uint16_t tick = 0;

	//闪蓝灯，视觉通信正常
	if (vision_info_point()->RxPacketSed.SOF == 0xB5)
	{
		if (tick < 200)
		{
			aRGB_led_show(0xFF0000FF);
			tick++;
		}
		else if (tick < 400)
		{
			aRGB_led_show(0x000000FF);
			tick++;
		}
		else
		{
			tick = 0;
		}
	}
	//红灯快闪，CAN线设备掉线
	else if ((toe_is_error(CAN_TOE) == 1))
	{
		if (tick < 100)
		{
			aRGB_led_show(0xFFFF0000);
			tick++;
		}

		else if (tick < 200)
		{
			aRGB_led_show(0x00FF0000);
			tick++;
		}
		else
		{
			tick = 0;
		}
	}
	//闪绿灯，正常运行，无视觉
	else
	{
		if (tick < 200)
		{
			aRGB_led_show(0xFF00FF00);
			tick++;
		}
		else if (tick < 400)
		{
			aRGB_led_show(0x0000FF00);
			tick++;
		}
		else
		{
			tick = 0;
		}
	}
}

/**
  * @brief          蜂鸣器任务
  * @retval         none
  */
static void buzzer_monitor(void)
{
	//蜂鸣器停止的计数小于最大停止计数，则蜂鸣器响
	if (buzzer.buzzer_off_tick < BUZZER_MAX_OFF_TICK)
	{
		//报警次数设为0，则一直报警
		if(buzzer.buzzer_warn_num_set == 0)
		{
			buzzer_on(buzzer.buzzer_psc, buzzer.buzzer_pwm);
		}
		//报警次数小于设定值，则按照设定的报警次数报警
		else if (buzzer.buzzer_warn_num < buzzer.buzzer_warn_num_set)
		{
			if (buzzer.buzzer_tick < buzzer.buzzer_warn_interval)
			{
				buzzer_on(buzzer.buzzer_psc, buzzer.buzzer_pwm);
				buzzer.buzzer_tick++;
			}
			else if (buzzer.buzzer_tick < buzzer.buzzer_warn_interval * 2)
			{
				buzzer_off();
				buzzer.buzzer_tick++;
			}
			else
			{
				buzzer.buzzer_tick = 0;
				buzzer.buzzer_tick++;
				buzzer.buzzer_warn_num++;
			}
		}
		//报警次数达到设定值，则停止2倍的间隔时间，再次报警
		else
		{
			if (buzzer.buzzer_tick < buzzer.buzzer_warn_interval * 2)
			{
				buzzer_off();
				buzzer.buzzer_tick++;
			}
			else
			{
				buzzer.buzzer_tick = 0;
				buzzer.buzzer_warn_num = 0;
			}
		}

		buzzer.buzzer_off_tick++;
	}
	//蜂鸣器停止的计数达到最大停止计数，则蜂鸣器停止
	else
	{
		buzzer_off();
		buzzer.buzzer_warn_num = 0;
		buzzer.buzzer_warn_num_set = 0;
		buzzer.buzzer_tick = 0;
	}
}

/**
  * @brief          蜂鸣器报警
  * @param[in]      num_set: 设置报警次数，0为一直报警，1为报警一次，2为报警两次，以此类推
  * @param[in]      interval: 设置报警间隔，单位为ms；num_set为0时，该参数无效
  * @param[in]      psc: 蜂鸣器分频系数，越大频率越低
  * @param[in]      pwm: 蜂鸣器重载值，一般为10000
  * @retval         none
  */
void buzzer_warn(uint8_t num_set, uint16_t interval, uint16_t psc, uint16_t pwm)
{
	//如果报警次数大于等于已经设定的报警次数或者设置为一直报警，则更新数据
	if (num_set >= buzzer.buzzer_warn_num_set || num_set == 0)
	{
		buzzer.buzzer_warn_num_set = num_set;
		buzzer.buzzer_warn_interval = interval;
		buzzer.buzzer_psc = psc;
		buzzer.buzzer_pwm = pwm;
		//重置停止计数
		buzzer.buzzer_off_tick = 0;
	}
}

/**
  * @brief          蜂鸣器初始化
  * @retval         none
  */
void buzzer_warn_init(void)
{
	buzzer.buzzer_off_tick = BUZZER_MAX_OFF_TICK;
	buzzer.buzzer_tick = 0;
	buzzer.buzzer_warn_num = 0;
	buzzer.buzzer_warn_num_set = 0;
	buzzer.buzzer_warn_interval = 0;
	buzzer.buzzer_psc = 0;
	buzzer.buzzer_pwm = 0;
}
