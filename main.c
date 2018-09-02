#include "led.h"
#include "delay.h"

#include "sys.h"
#include "beep.h"

#include "key.h"
 
u8 Flag300ms=0;//300ms节拍标志

 int main(void)
 {
 	vu8 key=0;	
	delay_init();	    	 //延时函数初始化	  
 	LED_Init();			     //LED端口初始化
	KEY_Init();          //初始化与按键连接的硬件接口
	BEEP_Init();     	//初始化蜂鸣器端口
	LED0=0;					//先点亮红灯
	while(1)
	{
 		key=Read_A_Key();	//得到键值
	  				   
		switch(key)
		{				 
      //常规一般按键测试（按下键就起作用）：
			case KEY_EVENT(KB_KEY2,PRESS_DOWN):	//KEY2按下即有效，控制LED0翻转	
				LED0=!LED0;
				break;
			case KEY_EVENT(KB_KEY0,PRESS_DOWN):	//KEY0按下即有效，控制LED1翻转	 
				LED1=!LED1; //控制两灯灭
				break;
			
			//下面可自由增加其它按键测试，比如（仅举数例）：
			case KEY_EVENT(KB_KEY1,SINGLE_CLICK):	//KEY1键单击（快速按下再松开）	 
				LED0=!LED0;
				break;
			case KEY_EVENT(KB_WKUP,DOUBLE_CLICK):	//WKUP键双击（连续单击两次）
				BEEP=!BEEP; //控制蜂鸣器鸣叫或停止
				break;
			case WKUP_PLUSKEY0_PRES:	//WKUP+KEY0组合按键（先按下WKUP再按下KEY0）
				LED0=!LED0; //控制两灯同时翻转
				LED1=!LED1;
				break;
			case KEY_EVENT(KB_WKUP,LONG_RRESS_START):	//长按键WKUP
				LED1=!LED1; //控制LED1翻转
				break;
			case KEY_EVENT(KB_KEY1,LONG_PRESS_HOLD):	//长按键KEY1后，每300ms处理一次（实现类似电子表的长按"+"时连续快速调整时间的功能）
				if(Flag300ms){Flag300ms=0;LED0=!LED0;} //控制LED0每300ms翻转一次
				break;
		}
	 delay_ms(50);//这个延时值可以模拟CPU干其它工作的时间，可试着增大这个值，感受按键因来不及处理被缓存的效果 
	}	 
}

