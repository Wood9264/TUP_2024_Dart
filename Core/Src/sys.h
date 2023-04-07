#ifndef _SYS_H
#define _SYS_H
#include "stm32f4xx.h"

#define GPIOF_ODR_Addr    (GPIOF_BASE+20) //0x40021414   
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
#define GPIOF_IDR_Addr    (GPIOF_BASE+16) //0x40021410 
#define PHout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //Êä³ö 
#define PHin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //ÊäÈë





#endif




