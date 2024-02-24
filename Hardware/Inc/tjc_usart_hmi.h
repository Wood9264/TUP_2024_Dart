#ifndef __TJCUSARTHMI_H
#define __TJCUSARTHMI_H

#include "struct_typedef.h"

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
	typedef struct
	{
		char *variable_name;
		//	uint8_t
	} TJC_Usart_Hmi_Send_t;
	void TJCPrintf(const char *str, ...);
	void initRingBuff(void);
	void writeRingBuff(uint8_t data);
	void deleteRingBuff(uint16_t size);
	uint16_t getRingBuffLenght(void);
	uint8_t read1BFromRingBuff(uint16_t position);
	void USART1_Init(uint32_t bound);
	void USART1_printf(char *fmt, ...); //串口1的专用printf函数

#define RINGBUFF_LEN (500)		   //环形缓冲区的长度
// #define HMI_USART_RX_BUF_LENGHT 9 //每次接收的数据长度

#define usize getRingBuffLenght()
#define code_c() initRingBuff()
#define udelete(x) deleteRingBuff(x)
#define u(x) read1BFromRingBuff(x)

	typedef struct
	{
		uint16_t Head;
		uint16_t Tail;
		uint16_t Lenght;
		uint8_t Ring_data[RINGBUFF_LEN];
	} RingBuff_t;

	extern uint8_t RxBuff[1];

#endif
#ifdef __cplusplus
}
#endif

#endif
