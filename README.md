# STM32
STM32 CM3 Project
BITBAND宏定义：

本人逐步实现的寄存器位操作模式（简单示例体会）

用位操作风格改写原子的寄存器版本SYS.C中的进入待机模式函数：
原子代码为：
//进入待机模式          
void Sys_Standby(void)
{
        SCB->SCR|=1<<2;//使能SLEEPDEEP位 (SYS->CTRL)           
  RCC->APB1ENR|=1<<28;     //使能电源时钟            
         PWR->CSR|=1<<8;          //设置WKUP用于唤醒
        PWR->CR|=1<<2;           //清除Wake-up 标志
        PWR->CR|=1<<1;           //PDDS置位                  
        WFI_SET();                                 //执行WFI指令                 
}

引入位段操作宏定义后，可以改写成：
void Sys_Standby(void)
{
        SCB->SCR|=1<<2;//使能SLEEPDEEP位 (SYS->CTRL)           
  bRCC_ENABLE_PWR=1;        //使能电源时钟            
         bPWR_CSR_EWUP=1;          //设置WKUP用于唤醒
        bPWR_CR_CWUF=1;           //清除Wake-up 标志
        bPWR_CR_PDDS=1;           //PDDS置位                  
        WFI_SET();                                 //执行WFI指令                 
} 
注意第一句SCB->SCR|=1<<2;没有改写成位操作，因为有几个系统寄存器并不在位绑定区。 


再拿前不久我使用的一个从STOP模式中快速恢复时钟的函数为例，
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
