#ifndef __MOTOR_H
#define __MOTOR_H

#include "struct_typedef.h"

#define Total_amount 9

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus

	class motor_t
	{
	public:
		uint16_t ecd;
		int16_t speed_rpm;
		int16_t given_current;
		uint8_t temperate;

		int16_t last_ecd;
		int64_t num;

		motor_t();

		friend void get_motor_measure(uint16_t number, uint8_t *rxdata);
	};

	extern motor_t motor[Total_amount];
	extern void get_motor_measure(uint16_t number, uint8_t *rxdata);

#endif

#ifdef __cplusplus
}
#endif

#endif
