#include "monitor_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "task.h"

#define RGB_FLOW_COLOR_CHANGE_TIME  1000
#define RGB_FLOW_COLOR_LENGHT   6
//blue-> green(dark)-> red -> blue(dark) -> green(dark) -> red(dark) -> blue
//蓝 -> 绿(灭) -> 红 -> 蓝(灭) -> 绿 -> 红(灭) -> 蓝 
uint32_t RGB_flow_color[RGB_FLOW_COLOR_LENGHT + 1] = {0xFF0000FF, 0x0000FF00, 0xFFFF0000, 0x000000FF, 0xFF00FF00, 0x00FF0000, 0xFF0000FF};
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

static void led_monitor(void);

/**
  * @brief          led rgb task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          led RGB任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void monitor_task(void const *argument)
{
	while (1)
	{
		led_monitor();
		vTaskDelay(2);
	}
}

static void led_monitor(void)
{
	static uint16_t tick = 0;

	//闪蓝灯，视觉通信正常
	if (vision_info_point()->RxPacketSed.SOF == 0xB5)
	{
		if (tick < 100)
		{
			aRGB_led_show(0xFF0000FF);
			tick++;
		}
		else if (tick < 200)
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
		if (tick < 50)
		{
			aRGB_led_show(0xFFFF0000);
			tick++;
		}

		else if (tick < 100)
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
		if (tick < 100)
		{
			aRGB_led_show(0xFF00FF00);
			tick++;
		}
		else if (tick < 200)
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
