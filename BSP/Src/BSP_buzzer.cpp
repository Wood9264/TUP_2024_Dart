#include "bsp_buzzer.h"
#include "main.h"
extern TIM_HandleTypeDef htim4;
void buzzer_on(uint16_t psc, uint16_t pwm)
{
    __HAL_TIM_PRESCALER(&htim4, psc);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);

}
void buzzer_off(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}

void buzzer_warn_error(int num)
{
	static int show_num = 0;
	static int stop_num = 300;
	if (show_num == 0 && stop_num == 0)
	{
		show_num = num;
		stop_num = 300;
	}
	else if (show_num == 0)
	{
		stop_num--;
		buzzer_off();
	}
	else
	{
		static int tick = 0;
		tick++;
		if (tick < 150)
		{
			buzzer_off();
		}
		else if (tick < 300)
		{
			buzzer_on(1, 10000);
		}
		else
		{
			tick = 0;
			show_num--;
		}
	}
}
