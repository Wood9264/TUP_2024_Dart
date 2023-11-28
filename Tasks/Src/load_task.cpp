#include "load_task.h"
#include "FreeRTOS.h"
#include "task.h"

void load_task(void const *pvParameters)
{
	//延时
	vTaskDelay(LOAD_TASK_INIT_TIME);
	uint32_t currentTime;
	while(1)
	{
		//获取当前系统时间
		currentTime = xTaskGetTickCount();

		vTaskDelayUntil(&currentTime, 1);
  }
}
