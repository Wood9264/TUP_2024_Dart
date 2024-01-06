#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "bsp_usart.h"
#include "tjc_usart_hmi.h"
#include "stm32f4xx_hal_uart.h"
#include "usart.h"
#define STR_LENGTH 100

//发送已修改，未修改接收
//Copy还未修改
typedef struct
{
    uint16_t Head;
    uint16_t Tail;
    uint16_t Lenght;
    uint8_t  Ring_data[RINGBUFF_LEN];
}RingBuff_t;

RingBuff_t ringBuff;	//创建一个ringBuff的缓冲区
uint8_t RxBuff[1];



/********************************************************
函数名：  	TJCPrintf
作者：    	wwd
日期：    	2022.10.08
功能：    	向串口打印数据,使用前请将USART_SCREEN_write这个函数改为你的单片机的串口发送单字节函数
输入参数：		参考printf
返回值： 		打印到串口的数量
修改记录：
**********************************************************/
//void USART1_printf (char *fmt, ...){ 
//	char buffer[STR_LENGTH+1];  // 数据长度
//	u8 i = 0;	
//	va_list arg_ptr;
//	va_start(arg_ptr, fmt);  
//	vsnprintf(buffer, STR_LENGTH+1, fmt, arg_ptr);
//	while ((i < STR_LENGTH) && (i < strlen(buffer))){
//        USART_SendData(USART1, (u8) buffer[i++]);
//        while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET); 
//	}
//	va_end(arg_ptr);
//	
//}

uint8_t frame_tail[] = {0xff,0xff,0xff};
void TJCPrintf (const char *str, ...){ 
	uint8_t buffer[STR_LENGTH+1];  // 数据长度
	uint8_t i = 0;	
	va_list arg_ptr;
	va_start(arg_ptr, str);  
	vsnprintf((char *)buffer, STR_LENGTH+1, str, arg_ptr);
	va_end(arg_ptr);
	while ((i < STR_LENGTH) && (i < strlen((char *)buffer)))
	{
		HAL_UART_Transmit(&huart1,&buffer[i++],1,100);
        while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) == RESET); 
	}
	HAL_UART_Transmit(&huart1,frame_tail,3,100);
	while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) == RESET); 
}




/********************************************************
函数名：  	USART1_IRQHandler
作者：
日期：    	2022.10.08
功能：    	串口接收中断,将接收到的数据写入环形缓冲区
输入参数：
返回值： 		void
修改记录：
**********************************************************/
//void USART1_IRQHandler(void)
//{
//	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
//	{
//		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
//		RxBuff[0] = USART_ReceiveData(USART1);
//		writeRingBuff(RxBuff[0]);
//		
//		
//		
//		
//	}
//}



/********************************************************
函数名：  	initRingBuff
作者：
日期：    	2022.10.08
功能：    	初始化环形缓冲区
输入参数：
返回值： 		void
修改记录：
**********************************************************/
void initRingBuff(void)
{
  //初始化相关信息
  ringBuff.Head = 0;
  ringBuff.Tail = 0;
  ringBuff.Lenght = 0;
}



/********************************************************
函数名：  	writeRingBuff
作者：
日期：    	2022.10.08
功能：    	往环形缓冲区写入数据
输入参数：
返回值： 		void
修改记录：
**********************************************************/
void writeRingBuff(uint8_t data)
{
  if(ringBuff.Lenght >= RINGBUFF_LEN) //判断缓冲区是否已满
  {
    return ;
  }
  ringBuff.Ring_data[ringBuff.Tail]=data;
  ringBuff.Tail = (ringBuff.Tail+1)%RINGBUFF_LEN;//防止越界非法访问
  ringBuff.Lenght++;

}




/********************************************************
函数名：  	deleteRingBuff
作者：
日期：    	2022.10.08
功能：    	删除串口缓冲区中相应长度的数据
输入参数：		要删除的长度
返回值： 		void
修改记录：
**********************************************************/
void deleteRingBuff(uint16_t size)
{
	if(size >= ringBuff.Lenght)
	{
	    initRingBuff();
	    return;
	}
	for(int i = 0; i < size; i++)
	{

		if(ringBuff.Lenght == 0)//判断非空
		{
		initRingBuff();
		return;
		}
		ringBuff.Head = (ringBuff.Head+1)%RINGBUFF_LEN;//防止越界非法访问
		ringBuff.Lenght--;

	}

}



/********************************************************
函数名：  	read1BFromRingBuff
作者：
日期：    	2022.10.08
功能：    	从串口缓冲区读取1字节数据
输入参数：		position:读取的位置
返回值： 		所在位置的数据(1字节)
修改记录：
**********************************************************/
uint8_t read1BFromRingBuff(uint16_t position)
{
	uint16_t realPosition = (ringBuff.Head + position) % RINGBUFF_LEN;

	return ringBuff.Ring_data[realPosition];
}




/********************************************************
函数名：  	getRingBuffLenght
作者：
日期：    	2022.10.08
功能：    	获取串口缓冲区的数据数量
输入参数：
返回值： 		串口缓冲区的数据数量
修改记录：
**********************************************************/
uint16_t getRingBuffLenght()
{
	return ringBuff.Lenght;
}


/********************************************************
函数名：  	isRingBuffOverflow
作者：
日期：    	2022.10.08
功能：    	判断环形缓冲区是否已满
输入参数：
返回值： 		1:环形缓冲区已满 , 2:环形缓冲区未满
修改记录：
**********************************************************/
uint8_t isRingBuffOverflow()
{
	return ringBuff.Lenght == RINGBUFF_LEN;
}



////初始化IO 串口1 
////bound:波特率
//void USART1_Init(uint32_t bound){ 
//	
//	//串口1初始化并启动
//	//GPIO端口设置
//	GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;	 
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
//	 //USART1_TX   PA.9
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
//	GPIO_Init(GPIOA, &GPIO_InitStructure);  
//	//USART1_RX	  PA.10
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
//	GPIO_Init(GPIOA, &GPIO_InitStructure); 
//	//Usart1 NVIC 配置
//	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
//	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器 
//	//USART 初始化设置
//	USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
//	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
//	USART_Init(USART1, &USART_InitStructure); //初始化串口
//	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启ENABLE/关闭DISABLE中断
//	USART_Cmd(USART1, ENABLE);                    //使能串口 
//}


