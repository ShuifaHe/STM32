# STM32
STM32 CM3 Project
/****************************** 本人逐步实现的寄存器位操作模式 ***************************************/

    利用本文件，可以比较方便、高效精准地以位段方式访问寄存器，结合多BIT访问、寄存器直接访问等方式，能够最终完全抛开库函数。

     一、以外设原有名称为基础，在外设名称前面加前缀b, 与库函数区分的同时，指示这用作bitband操作。

     二、进一步强化助记，将部分具体寄存器名称弱化,比如某外设有多个控制寄存器CR1、CR2、CR3，则统一省略为CR，不必记忆是第几个CR。
     尝试对于常用的外设如RCC，进一步根据功能进行了改造，比如将所有使能时钟控制归纳成bRCC_ENABLE_XXX（其中XXX为外设名），
     便于记忆，不用再考虑某外设挂在哪条具体总线上等细节，相当于进行了一个小小的二次封装。

     三、BIT位的名称尽量采用原名称，以便对照查阅芯片数据手册。寄存器地址偏移量按一般用户手册以十六进制填充。

     四、考虑到位操作的优势在精准高效地对位进行操作，这也是编写此文件的出发点。但考虑到两位以上的组合操作也无法避免，
     并且直接访问寄存器某几位也比较麻烦, 这里采用带参的宏定义制定了统一的访问格式.

     五、对于一般没有位定义、通常整字访问的寄存器，仍可沿用传统的寄存器方式，这里也尝试定义了形如wBKP_DR1的宏指令方式对其进行访问，
     即：wRTC_ALRH的效果等同于寄存器直接访问的RTC->ALRH；
     而对于有多个同类外设时，对于外设某寄存器整体的读写，则引入类似如下的宏定义，
                        pTIM(bTIM3)->PSC        前缀“p”表示这是一个指向结构体的指针，等效于寄存器直接访问的TIM3->DR1
	但引入带参宏定义的好处是，实现了与访问多组外设单BIT位或多BIT位时形参的统一，便于移植
	
****************** 位段操作统一格式 *******************************************

单BIT位访问方式：
		bRCC_ENABLE_RTC=1或0; 	        //唯一外设
		
		bTIM_CR_DIR（bTIMx)=1或0; 	//多组外设
		
		bGPIO_BRR(bGPIOx,n)=1或0;	//多组外设多个端口
					也可读，或者用于逻辑判断，EX：if(!bRCC_ENABLE_RTC)
    
多BIT位访问方式：
		SET_RCC_PLLMUL(a);(设置PLL倍频值为a)		写	 	//唯一外设
		val=GET_RCC_PLLMUL;(获取PLL倍频设置值)		读

		SET_SPI_CR_BR(bSPIx,a);	(设置bSPIx接口的速率值为a)	写	//多组外设
		val=GET_SPI_CR_BR(bSPIx);(获取bSPIx接口的速率值)	       读

寄存器整体访问方式：一般仍沿用传统的寄存器直接访问方式，如BKP->DR1			
************************************************************************

（简单示例体会）

拿前不久我使用的一个从STOP模式中快速恢复时钟的函数为例，
库函数版本如下：

/**
  * 本函数用于从STOP模式唤醒后重新配置系统时钟：使能HSE，PLL并选择PLL作为系统时钟源
  *         
  */
  
static void SYSCLKConfig_STOP(void)


{  
  /* 从STOP模式唤醒后重新配置系统时钟 */
  /* 使能 HSE */
		
  RCC_HSEConfig(RCC_HSE_ON);
  
 
 /* 等待HSE时钟就绪 */
 
 while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET)
  {}
  
 
 /* 使能PLL */
 
 RCC_PLLCmd(ENABLE);
  
 
 /* 等待PLL就绪 */
 
 while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
  {}
 
 
 /* 选择PLL作为系统时钟源 */
 RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);


/* 等待时钟源配置就绪 */

while (RCC_GetSYSCLKSource() != 0x08)
  {}

}


//*****************************************************************

//引入位段操作宏定义后，可以改写成：

//如果再定义#define YES 1 或（和） #define ENABLE  1  的话，全函数如下：

static void SYSCLKConfig_STOP(void)

{  
  bRCC_CR_HSEON=YES;         
  while (! bRCC_CR_HSERDY);
  
  bRCC_CR_PLLON=YES;          
  while(! bRCC_CR_PLLRDY);     
  bRCC_CFGR_SW_PLL=YES;          
  while (! bRCC_CFGR_SWS_PLL);     
}


//怎么样？ 非常简洁！！！

//不仅程序简洁，方便书写，而且可读性也非常强。 


//看起来就像自然描述语言一样，但却非常底层，

//与库函数版本相比，没有调用任何一个子程序，

//同时也远远超越了寄存器版本的高效。
