#include "motor.h"
#include "main.h"
motor_t motor[Total_amount];

motor_t::motor_t()
{
	ecd = 0;
	speed_rpm = 0;
	given_current = 0;
	temperate = 0;
	last_ecd = 0;
	num = 0;
}

void get_motor_measure(uint16_t number,uint8_t *rxdata)
{
	motor[number].last_ecd = motor[number].ecd;
	motor[number].ecd = (uint16_t)(rxdata[0] << 8 | rxdata[1]);
	motor[number].speed_rpm = (uint16_t)(rxdata[2] << 8 | rxdata[3]);
	motor[number].given_current = (uint16_t)(rxdata[4] << 8 | rxdata[5]);
	motor[number].temperate = rxdata[6];
	
	if(abs(motor[number].ecd - motor[number].last_ecd) > 4096)
	{
		if(motor[number].ecd >= 6500 && motor[number].last_ecd <= 1000)
		{
			motor[number].num += 1;
		}
		else if(motor[number].ecd <= 1000 && motor[number].last_ecd >= 6500)
		{
			motor[number].num -= 1;
		}

	}		
	
}


