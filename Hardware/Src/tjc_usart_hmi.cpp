#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "bsp_usart.h"
#include "tjc_usart_hmi.h"
#include "stm32f4xx_hal_uart.h"
#include "usart.h"
#include "stm32f4xx_hal_dma.h"
#include "judge_task.h"

#define STR_LENGTH 100

extern DMA_HandleTypeDef hdma_usart1_rx;

RingBuff_t ringBuff; //创建一个ringBuff的缓冲区
uint8_t RxBuff[1] = {0};

/********************************************************
函数名：  	TJCPrintf
作者：    	wwd
日期：    	2022.10.08
功能：    	向串口打印数据,使用前请将USART_SCREEN_write这个函数改为你的单片机的串口发送单字节函数
输入参数：		参考printf
返回值： 		打印到串口的数量
修改记录：
**********************************************************/
// void USART1_printf (char *fmt, ...){
//	char buffer[STR_LENGTH+1];  // 数据长度
//	u8 i = 0;
//	va_list arg_ptr;
//	va_start(arg_ptr, fmt);
//	vsnprintf(buffer, STR_LENGTH+1, fmt, arg_ptr);
//	while ((i < STR_LENGTH) && (i < strlen(buffer))){
//         USART_SendData(USART1, (u8) buffer[i++]);
//         while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
//	}
//	va_end(arg_ptr);
//
// }

uint8_t frame_tail[] = {0xff, 0xff, 0xff};

/**
 * @brief   串口打印函数
 * @param   str 要打印的字符串
 * @param   ... 可变参数
 */
void TJCPrintf(const char *str, ...)
{
    uint8_t buffer[STR_LENGTH + 1]; // 数据长度
    uint8_t i = 0;
    va_list arg_ptr;
    va_start(arg_ptr, str);
    vsnprintf((char *)buffer, STR_LENGTH + 1, str, arg_ptr);
    va_end(arg_ptr);
    while ((i < STR_LENGTH) && (i < strlen((char *)buffer)))
    {
        HAL_UART_Transmit(&huart1, &buffer[i++], 1, 100);
        while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) == RESET)
            ;
    }
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
extern "C"
{
    /**
     * @brief   串口1接收中断
     * @note    先后读取SR和DR寄存器后，中断标志位会自动清除
     */
    void USART1_IRQHandler(void)
    {
        if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) == SET)
        {
            RxBuff[0] = USART1->DR;
            writeRingBuff(RxBuff[0]);
        }
    }
}

extern "C"
{
    // //串口1接收中断
    // void USART1_IRQHandler(void)
    // {
    //     if (USART1->SR & UART_FLAG_IDLE)
    //     {
    //         __HAL_UART_CLEAR_PEFLAG(&huart1);
    //     }
    //     __HAL_DMA_DISABLE(huart1.hdmarx);

    //     __HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx, DMA_FLAG_TCIF1_5 | DMA_FLAG_HTIF1_5);

    //     __HAL_DMA_SET_COUNTER(&hdma_usart1_rx, HMI_USART_RX_BUF_LENGHT);

    //     __HAL_DMA_ENABLE(huart1.hdmarx);
    //     HAL_UART_Receive_DMA(&huart1, RxBuff, HMI_USART_RX_BUF_LENGHT);

    //     for (int i = 0; i < HMI_USART_RX_BUF_LENGHT && RxBuff[i] != '\0'; i++)
    //     {
    //         writeRingBuff(RxBuff[i]);
    //     }
    // }
}
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
    if (ringBuff.Lenght >= RINGBUFF_LEN) //判断缓冲区是否已满
    {
        return;
    }
    ringBuff.Ring_data[ringBuff.Tail] = data;
    ringBuff.Tail = (ringBuff.Tail + 1) % RINGBUFF_LEN; //防止越界非法访问
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
    if (size >= ringBuff.Lenght)
    {
        initRingBuff();
        return;
    }
    for (int i = 0; i < size; i++)
    {

        if (ringBuff.Lenght == 0) //判断非空
        {
            initRingBuff();
            return;
        }
        ringBuff.Head = (ringBuff.Head + 1) % RINGBUFF_LEN; //防止越界非法访问
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

/**
 * @brief       从串口缓冲区读取n字节数据
 * @param[out]  data 读取的数据
 * @param[in]   position 读取的位置
 * @param[in]   n 读取的长度
 */
void readNBFromRingBuff(uint8_t *data, uint16_t position, uint16_t n)
{
    for (uint16_t i = 0; i < n; i++)
    {
        data[i] = read1BFromRingBuff(position + i);
    }
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
// void USART1_Init(uint32_t bound){
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
// }
