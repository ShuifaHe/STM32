#include "key.h"
#include "led.h"
#include "sys.h" 
#include "delay.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//按键驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2018/8/30
//版本：V2.2
//Made by warship									  
//////////////////////////////////////////////////////////////////////////////////  
								    
//按键初始化函数
void KEY_Init(void) //IO初始化
{ 
 	GPIO_InitTypeDef GPIO_InitStructure;
 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOE,ENABLE);//使能PORTA,PORTE时钟

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4;//KEY0-KEY2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //设置成上拉输入
 	GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化GPIOE2,3,4

	//初始化 WK_UP-->GPIOA.0	  下拉输入
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PA0设置成输入，默认下拉	  
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.0
}

//硬件按键编码（为应用三行读键程序而准备）
//以战舰版的四键为例（最大暂支持16键，KeyS_Type定义为u32则可支持32键）
KeyS_Type GetHalKeyCode(void)
{
	KeyS_Type ktmp=0;
	if(!KEY0_IN) 	ktmp|=1<<KB_KEY0;
	if(!KEY1_IN) 	ktmp|=1<<KB_KEY1;
	if(!KEY2_IN) 	ktmp|=1<<KB_KEY2;
	if(WKUP_IN) 	ktmp|=1<<KB_WKUP;//注意本键为高电平有效 
	return ktmp;
}

//********************************************************************************
static KeyS_Type KeyStable=0; //存有稳定(消除抖动后)的键态(读键前)

KeyS_Type Trg=0;  		//全局变量：存有本次读键时的按键触发状态
KeyS_Type Cont=0; 		//全局变量：存有本次读键时的实时键态
u16 KeyTime=0;  //全局变量：存有本次读键时当前键态持续的时长

/******************** 用户应用程序按键判断接口函数(请根据具体需求修改) *********************************/
//最终由GetAndSaveKey()在SYSTICK中调用，存入按键队列，主循环调用时请使用Read_A_Key()
//返回稳定的键值，用户程序直接处理事先定义的键值即可。

//可适应的按键类型如下：
//普通：按下即有效，不用等按键释放
//单击：按下再松开后有效
//双击：快速单击两次（两次单击之间时间不超过SHORT_TICKS）
//长按：按下超过规定的时间LONG_TICKS，超过后可连续输出，通过软件可设置间隔一定时间输出一次键值
//组合：双键组合（其实多键组合也可同理实现）
/**********************************************************************************/
//不使用组合按键等条件判断时，以下宏定义可删除
#define KEY0_ON 						(0x0001<<KB_KEY0)  //宏定义：按键未释放值
#define KEY1_ON 						(0x0001<<KB_KEY1)
#define KEY2_ON 						(0x0001<<KB_KEY2)
#define WKUP_ON 						(0x0001<<KB_WKUP)
#define KEY0_PRESSED 				(Trg==KEY0_ON)  //宏定义：按键触发值
#define KEY1_PRESSED 				(Trg==KEY1_ON)
#define KEY2_PRESSED 				(Trg==KEY2_ON)
#define WKUP_PRESSED 				(Trg==WKUP_ON)

u8 Get_Key(void)
{
	u8 i,keyp=0;
/*按键的判断条件设定技巧：
	全局变量Trg中体现了对应按键的触发状态，在某按键被按下后有且只有一次读取到对应位为1;
	全局变量Cont则体现了当前按键的状态，处于按下的对应位为1，处于松开的对应位为0;
	而全局变量KeyTime里面，记录了当前键态持续的时间
*/
	
	//以下是按键判断，用户可根据需要随意添加或删改（注释掉的部分也可根据需要作为参考语句使用）
	
//注意：排在前面的判断条件具有高的优先级，一旦条件满足即刻返回，不再执行后续语句。
	if((Cont==(WKUP_ON+KEY0_ON)) && KEY0_PRESSED)	{ //WKUP+KEY0组合按键（先按下WKUP再按下KEY0）
		Get_Key_State(KB_CLR); //复位状态机，防止本按键对其干扰(本按键与状态机有冲突时请调用此句)
		return WKUP_PLUSKEY0_PRES;} 

//以下是使用状态机得到判断单击、双击、长按、保持等键码	
	for(i=0;i<KeyNumMax;i++)
	  {
			keyp=Get_Key_State(i);	
			if(keyp) return keyp; 
		}
	return keyp;
}


//
//按键预处理程序:  允许对有强实时性需要的紧要按键无须排队优先执行，其效果有点儿类似回调函数
//本函数根据实际项目需求可以有三种具体实现模式选择：
//模式一：如果按键处理占用时间短（无延时语句、长循环等），按键要求强实时处理，则可以把所有的按键处理都放在这里
//        这样主循环就无须处理按键了（相当于使用中断服务的方式处理全部按键）
//模式二：对按键处理实时性要求不高，能够忍受一定程序的按键响应时延，可以把所有按键处理放在主循环中查询响应，
//        此时本函数可以简化为return Get_Key();
//模式三（前两种的折中方案）：强实时性需要紧急处理的键，直接在这里写执行代码，其它允许延时的键留待主循环查询处理，形式如下例所示。
u8 Key_PrePro(void)
{
	//return Get_Key(); //模式二时，本函数简化到只须这一句，以下可全部删除。
	u8 newkeytmp,ret=0;
	newkeytmp=Get_Key();
	switch(newkeytmp)
	{
		case KEY_EVENT(KB_KEY1,DOUBLE_CLICK)://KEY1双击，执行两灯同时翻转（仅作为示例）
			LED0=!LED0;LED1=!LED1; //控制两灯翻转
      break;
    default:
			ret=newkeytmp;
	}
	return ret;
}






//**********************  以下为通用函数，一般不要修改  ****************************
u8 New_KeyBuff[KEYBUFFSIZE];
u8 pNewKey=0;

void GetAndSaveKey(void)//本函数由SYSTICK调用，在后台读键，如果有键值则存入按键缓冲区
{
	u8 newkeytmp;
	if(KeyTime>=LONG_TICKS && KEY_RELEASED)
		{//键盘长时间闲置，直接返回（绝大部分时间基本都是这种状态，此举将大大节省CPU资源）
			KeyTime=LONG_TICKS;//此句防止KeyTime溢出(KeyTime由扫键程序累增)
			return; 
	  }
	Trg=KeyStable & (KeyStable ^ Cont); //调用三行读键方法,其实核心只有此行，使得Trg在某按键被按下后有且只有一次读取到对应位为1;
	Cont=KeyStable;
	newkeytmp=Key_PrePro();//从键预处理程序中读键值
	if(newkeytmp)//如果有新的键值
	{
		New_KeyBuff[pNewKey++]=newkeytmp;//存入按键缓冲区(pNewKey永远指向下一空位置)
		if(pNewKey==KEYBUFFSIZE)pNewKey=0;//按键缓冲区循环使用
	}
}

//读取按键值：由主循环调用。
//从按键缓存中读取按键值，无键则返回0
u8 Read_A_Key(void)
{
	static u8 pReadKey=0;//读键指针
	if(pReadKey==KEYBUFFSIZE)pReadKey=0;//按键缓冲区循环使用
	if(pReadKey==pNewKey) return 0;//键已经取尽，返回0
	return New_KeyBuff[pReadKey++];
}


//按键扫描函数：一般由Systick中断服务程序以5ms一次的时间节拍调用此函数
//采用了键盘自适应变频扫描措施，在键盘正常稳定期间（非消抖期间）扫描频率降低以减少CPU资源占用
//该函数将影响全局变量：消除抖动后的稳定键态值KeyStable及累计时长KeyTime
void Key_Scan_Stick(void)
{
	KeyS_Type KeyValTemp;
	static KeyS_Type KeyValTempOld=0;
	static u16 debounce_cnt=0;
	static u16 debouncing=0;
	
	KeyTime++;//在稳定键态（包括无键）状态下，全局变量KeyTime是持续增加的
	if((!debouncing) && (KeyTime%NORMAL_SCAN_FREQ))//非消抖期间且累计计时不是6的倍数(即6*5＝30ms才扫描一次)
		return;	//则不扫描键盘直接返回，这里可调整NORMAL_SCAN_FREQ为其它数，估计最大到40即120ms扫描一次都问题不大的。
	
	KeyValTemp=GetHalKeyCode();//扫描键盘，得到实时键值（合并），可存16个键值（按下相应位为1松开为0）;
	
	if(KeyValTemp!=KeyStable) //如果当前值不等于旧存值，即键值有变化
	{
		debouncing=1;//标示为消抖期
		if(!(KeyValTemp^KeyValTempOld))//如果临时值不稳定（即新键值有变化）
		{
			debounce_cnt=0;
			KeyValTempOld=KeyValTemp;
		}
		else//临时值稳定
		{
		 if(++debounce_cnt >= DEBOUNCE_TICKS) 
		 {
			KeyStable = KeyValTemp;//键值更新为当前值.
			debounce_cnt = 0;//并复位消抖计数器.
			KeyTime=1; //新键值累计时长复位为1个时间单位
			debouncing=0;//消抖期结束
		 }
	  } 
	} 
	else //如果键值仍等于旧存值：
	{ //则复位消抖计数器（注意：只要消抖中途读到一次键值等于旧存值，消抖计数器均从0开始重新计数）.
		debounce_cnt = 0;
		KeyValTempOld=KeyValTemp;
	}
}

//***************************************************************
//  无须单击、双击、长按、连续保持等功能键的可以删除以下函数
//***************************************************************
/**　多功能按键状态机
  * 入口参数：实体按键编号（参数为KEYCLR用于复位状态机）
  * 返回：键值（按键事件值）＝(KeyNum+2)*10+键事件值; 其它返回0.
  */
#define THE_KEY_IS_OFF			(!(Cont & KeyOnCode))
#define THE_KEY_IS_ON				(Cont & KeyOnCode)
#define THE_KEY_PRESSED			((Trg & KeyOnCode) && (Cont & KeyOnCode))

u8 Get_Key_State(u8 KeyNum)
{
	//按键记忆状态(每字节低四位存state，高4位存repeat)
	static u8 KeyState[KeyNumMax];
	
	KeyS_Type KeyOnCode;
	u8 i,state,repeat,event=0;
	if(KeyNum==KB_CLR) //参数为KB_CLR时，则消除所有按键记忆状态
	{
		for(i=0;i<KeyNumMax;i++) KeyState[i]=0;
		return 0;
	}
	KeyOnCode=(KeyS_Type)1<<KeyNum;
	state=KeyState[KeyNum]&0x0f; //取相应的记忆状态值
	repeat=KeyState[KeyNum]>>4;
	
	if(Trg && (Trg!=KeyOnCode)) state=0; //出现其它键，则状态清0
	
	switch (state) 
	{
	case 0://状态0：键完全松开
		if(THE_KEY_PRESSED) 
			{	//初次按键触发并有效
			event = (u8)PRESS_DOWN;
			repeat = 1;
			state = 1;//初次按键有效，变成状态1
			} 
			else //无效电平，即按键为松开状态
			  event = (u8)NONE_PRESS;
	  break;

	case 1://状态1：初次按键触发并有效
		if(THE_KEY_IS_OFF) { //检测到按键松开		
			event = (u8)PRESS_UP;
			state = 2;//按键按下后松开，变成状态2
      }
		 else if(KeyTime > LONG_TICKS) {//按键未松开，且持续时间已经超过LONG_TICKS
			event = (u8)LONG_RRESS_START;
			state = 5;//即长按触发启动，变成状态5
		  }
		break;

	case 2://状态2：按键按下后已松开
		if(THE_KEY_PRESSED) { //再次检测到按下   
			event = (u8)PRESS_DOWN;
			repeat++;//重按次数累计
			if(repeat == 2) state = 3;//如果重按次数等于2,则变成状态3
			} 
		else //持续松开
      {
		   if(KeyTime > SHORT_TICKS)  
			  {//如果松开时间超过SHORT_TICKS，即一次按键结束
				 state = 0;//因按键松开时间超过SHORT_TICKS，则复位成状态0	
				 if(repeat==1) event=(u8)SINGLE_CLICK;//次数为1的情况下触发单击事件
			   else if(repeat==2) event=(u8)DOUBLE_CLICK;//重按次数为2的情况下触发双击事件
			  }
			} //隐含：如果松开时间还没有超过SHORT_TICKS，仍然维持状态2，有待后续判断		
		break;

	case 3://状态3：按下、松开、又按下（即第二次按下）				
		if(THE_KEY_IS_OFF)  //检测到按键松开
			{
				event = (u8)PRESS_UP;
			  if(KeyTime < SHORT_TICKS) state = 2; //松开时间小于SHORT_TICKS，回到状态2 
			  else state = 0;//松开时间大于SHORT_TICKS，则变成状态0
		  }//隐含：如果仍按下则停留在状态3等待松开（第二次按下没有长按之说）
		break;

	case 5://状态5：长按触发已经启动
		if(THE_KEY_IS_ON)  //如果按键仍持续按下				
	     event = (u8)LONG_PRESS_HOLD;//长按并保持按键事件成立
		else { //如果按键松开
			event = (u8)PRESS_UP;
			state = 0; //则恢复到状态0
		  }
		break;
	}
	KeyState[KeyNum]=state; //保存相应的记忆状态值
	KeyState[KeyNum]+= repeat<<4;
	if(event>=(u8)PRESS_DOWN) //设定只输出特殊功能键（修改此处可输出按下/松开等一般事件）
//	if(event) //输出所有事件		
		return KEYOUT_BASE_DEF+event;
	else return 0;
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
