# STM32
STM32 CM3 Project
BITBAND宏定义：

本人逐步实现的寄存器位操作模式（简单示例体会）

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

//引入位段操作宏定义后，可以改写成：
//如果再定义#define YES 1 或（和） #define ENABLE  1  的话，全函数如下：

static void SYSCLKConfig_STOP(void)
{  
  bRCC_CR_HSEON=YES;         
  while (! bRCC_CR_HSERDY);    bRCC_CR_PLLON=YES;          
  while(! bRCC_CR_PLLRDY);     
  bRCC_CFGR_SW_PLL=YES;          
  while (! bRCC_CFGR_SWS_PLL);     
}

//怎么样？ 非常简洁！！！
//不仅程序简洁，方便书写，而且可读性也非常强。 
//看起来就像自然描述语言一样，但却非常底层，远远超越了寄存器版本的高效。
