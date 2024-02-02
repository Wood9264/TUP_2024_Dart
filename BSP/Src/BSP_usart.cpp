#include "BSP_usart.h"
#include "usart.h"
#include "cmsis_os.h"
#include "string.h"
#include "stm32f4xx_hal.h"
#include "judge_task.h"
#include "tjc_usart_hmi.h"

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart1;

void usart1_init()
{

  //   //使能 DMA 串口接收
  //   SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
  //   SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);

    
  //   //使能空闲中断
  //   __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	
  // 	//enable DMA
  //   //失效DMA
  //   __HAL_DMA_DISABLE(&hdma_usart1_rx);
	
  //  while(hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
  //   {
  //       __HAL_DMA_DISABLE(&hdma_usart1_rx);
  //   }
		
	// __HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx,DMA_FLAG_TCIF1_5 
	// 	| DMA_FLAG_HTIF1_5);
		
	// hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART1->DR);
	// hdma_usart1_rx.Instance->M0AR = (uint32_t)(RxBuff);
	
	
	// __HAL_DMA_SET_COUNTER(huart1.hdmarx, USART_RX_BUF_LENGHT);
	// __HAL_DMA_ENABLE(&hdma_usart1_rx);

}




