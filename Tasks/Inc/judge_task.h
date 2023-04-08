#ifndef __JUDGE_TASK_H
#define __JUDGE_TASK_H


#include "bsp_usart.h"
#include "judge.h"

#define REF_PROTOCOL_FRAME_MAX_SIZE         128
#ifdef __cplusplus
extern "C" {
#endif
#ifdef __cplusplus
#include "chassis_task.h"	
typedef  struct
{
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
} frame_header_struct_t;

typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;


	typedef struct
{
  frame_header_struct_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
  unpack_step_e  unpack_step;
  uint16_t       index;
} unpack_data_t;
#define VisionBufferLength_SH       150
#define JUDGE_BUFFER_LEN          200
#define	USART_RX_BUF_LENGHT 		512
#define REFEREE_FIFO_BUF_LENGTH 1024
extern void Cap_Update_Task(void const * argumt);
#endif
extern void Judge_Task(void const * argumt);
#ifdef __cplusplus
}
#endif

#endif 
