#include "delay.h"
#include "key.h"

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//按键驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2018/8/29
//版本：V1.0
//Made by warship									  
//////////////////////////////////////////////////////////////////////////////////   

//使用SysTick的计数模式对延迟进行管理（适合STM32F10x系列）
//包括delay_us,delay_ms, 不影响SysTick中断系统
//延时数不受SysTick中计数器24位的限制,一次可延时达47万毫秒,高达1个小时

//本文件中还包括了SysTick的中断服务程序SysTick_Handler()

//2018/5/10

//////////////////////////////////////////////////////////////////////////////////  
static u8  fac_us=9;							//us延时倍乘数			   
//static u32 fac_ms=0;	//ms延时倍乘数,在ucos下,代表每个节拍的ms数



//延时初始化: 主程序入口处直接调用该函数即可,由此即实现了全系统的时钟配置
//本函数实质上只是选择了SysTick的时钟源, 并设定SysTick为1毫秒中断一次
//系统RCC基础时钟的初始化SystemInit由启动代码完成(请查验并确认工程中选用的启动文件有相关动作)
//SYSTICK的时钟固定为HCLK时钟的1/8
//中断时间time  =  ( SysTick->LOAD + 1 ) / f                        f = AHB或AHB/8            （9000-1+1）/9M=1ms
void delay_init(void)
{
  SysTick->LOAD=9000-1;    //装载值设定为9*1000-1=8999  即每ms中断一次 
	SysTick->CTRL|=3;        //开启SYSTICK并允许中断	
}

//微秒延时函数:
//本函数仅反复读取SYSTICK的当前值,直到给定的延时时间到达, 不影响SYSTICK的自动重装,也不影响其中断
//即使在延时过程中发生中断,只要中断服务耗时不超出1ms, 都不会影响本函数的延时精度
//nus:0~2^32/fac_us=4294967296/9=477,218,588 高达4.7亿微秒                                                                             
void delay_us(u32 nus)
{                
	static u32 ticks;
	static u32 told,tnow,tcnt=0;
	static u32 reload;
	static u32 OldLongTicks;
	told=SysTick->VAL;     //取刚进入时的计数器值
	OldLongTicks=GetTicks();  //取刚进入时的系统滴嗒计数值并保存在
	reload=SysTick->LOAD;   //取系统滴嗒的重装载值（即每滴嗒的ticks数）                     
	ticks=nus*fac_us;       //计算本函数需要延时的ticks数(每us占fac_us个ticks)                           

	while(1)
	{
		tnow=SysTick->VAL;        //取当前计数值
		tcnt=(GetTicks()-OldLongTicks)*reload-tnow+told;
		if(tcnt>=ticks)break;   //累计节拍数大于/等于需要延迟的节拍数,即延时时间到达, 则退出本函数.
	}        
}


//毫秒延时函数:
//将给定的延时毫秒数乘以1000后直接调用微秒延时函数
//nms:0---2^32/1000/fac_us=47万毫秒  
void delay_ms(u32 nms)
{                                     
  delay_us((u32)(nms*1000));                                        //直接调用delay_us函数进行延时  
}


//纯软件延时子程序: 实测值约5500为1个毫秒,5.5为一个微秒
void SoftDelay(vu32 nCount)
{
  for(; nCount != 0; nCount--);
}


//纯软件延时子程序: 实测值约5500为1个毫秒,5.5为一个微秒
void SoftDelay_ms(u16 nms)
{
	u32 nCount=(u32)(5500*nms);
  for(; nCount != 0; nCount--);
}


//纯软件延时子程序: 实测值约5500为1个毫秒,5.5为一个微秒
void SoftDelay_us(u16 nus)
{
	u32 nCount=(u32)(5*nus+nus/2);
  for(; nCount != 0; nCount--);
}


/*******************************************************************************
* Function Name  : SysTickHandler
* Description    : 系统每1毫秒产生一次中断，进入此服务程序
注意: 本函数与延时函数不冲突
*******************************************************************************/
//nTicks为一个32位的静态全局变量,用于累计SysTick总次数
static u32 nTicks = 0;  //静态全局变量只在本文件有作用
u32 GetTicks()
{
	return nTicks;
}

extern u8 Flag300ms;
/* SysTick中断服务函数 */
void SysTick_Handler(void)
{
	nTicks++;
	if(!(nTicks%300)) Flag300ms=1;

	if (( nTicks % TICKS_INTERVAL) == 0 )
	 {
		Key_Scan_Stick(); //每5ms扫键一次
		if ( nTicks % (TICKS_INTERVAL*12) == 0 ) 
			 GetAndSaveKey();//每60ms分析一次键值
	 }
}





