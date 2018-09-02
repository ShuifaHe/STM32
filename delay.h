#ifndef __DELAY_H
#define __DELAY_H 			   
#include "sys.h"  
	 
void delay_init(void);
void delay_ms(u32 nms);
void delay_us(u32 nus);

//纯软件延时子程序: 实测值约5500为1个毫秒,5.5为一个微秒 
void SoftDelay(vu32 nCount);
void SoftDelay_ms(u16 nms);
void SoftDelay_us(u16 nus);

u32 GetTicks(void); 		//获取SysTick的总累计次数

#endif





























