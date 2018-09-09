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
