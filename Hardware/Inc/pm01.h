/*
 *
 * pm01_api.h
 *
 * Created on: 2021年07月08日
 *     Author: hepeng
 *
 * Copyright (c) 2021, 柳州启明电气科技有限公司 All rights reserved. 
 *
 */
 
#ifndef __PM01_H
#define __PM01_H

#include "can.h"
#include "stdint.h"


typedef union 
{

  uint16_t all;
  struct {
      uint16_t rdy:   1;  /*!< bit0    就绪     */
      uint16_t run:   1;  /*!< bit1    运行     */
      uint16_t alm:   1;  /*!< bit2    报警     */
      uint16_t pwr:   1;  /*!< bit3    电源开关 */
      uint16_t load:  1;  /*!< bit4    负载开关 */
      uint16_t cc:    1;  /*!< bit5    恒流     */
      uint16_t cv:    1;  /*!< bit6    恒压     */
      uint16_t cw:    1;  /*!< bit7    恒功率   */
      uint16_t revd:  7;  /*!< bit8-14 保留     */
      uint16_t flt:   1;  /*!< bit15   故障     */
  }bit;

}csr_t;

typedef struct mb_reg_type{

   uint16_t ccr;         /*!< 8000H 控制寄存器     */
   uint16_t p_set;       /*!< 8001H 输入功率限制   */  
   uint16_t v_set;       /*!< 8002H 输出电压设置   */
   uint16_t i_set;       /*!< 8003H 输出电流限制   */
   csr_t    sta_code;    /*!< 8100H 状态标志位     */  
   uint16_t err_code;    /*!< 8101H 故障代码       */
   int16_t  v_in;        /*!< 8102H 输入电压       */
   int16_t  i_in;        /*!< 8103H 输入电流       */
   int16_t  p_in;        /*!< 8104H 输入功率       */
   int16_t  v_out;       /*!< 8105H 输出电压       */
   int16_t  i_out;       /*!< 8106H 输出电流       */
   int16_t  p_out;       /*!< 8107H 输出功率       */
   int16_t  temp;        /*!< 8108H 温度           */
   uint16_t total_time;  /*!< 8109H 累计时间       */
   uint16_t run_time;    /*!< 810AH 运行时间       */
	
}pm01_od_t;
extern void pm01_cmd_send         ( uint16_t new_cmd, uint8_t save_flg );
extern void pm01_voltage_set      ( uint16_t new_voltage, uint8_t save_flg );
extern void pm01_current_set      ( uint16_t new_voltage, uint8_t save_flg );
extern void pm01_power_set        ( uint16_t new_power, uint8_t save_flg );
extern void pm01_access_poll      ( void );
extern void pm01_response_handle  ( CAN_RxHeaderTypeDef *can_rx_header, uint8_t *can_rx_data );
	
#endif

/*
 *  [] END OF FILE 
 */


