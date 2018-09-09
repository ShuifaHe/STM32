#ifndef __MYBITBAND_H
#define __MYBITBAND_H	  
#include <stm32f10x.h>  
/****************************** 本人逐步实现的寄存器位操作模式 ***************************************/
// 以外设原有名称为基础，在外设名称前面加前缀b, 与库函数区分的同时，指示这用作bitband操作。
// 进一步强化助记，将部分具体寄存器名称弱化,具体寄存器名称在位定义前有注释行。
// BIT位的名称尽量采用原名称，以便对照.  寄存器地址偏移量按一般用户手册以十六进制填充。
// 考虑到位操作的优势在精准高效地对位进行操作，故对两位以上的组合予以省略。
// CopyRight By Warship
// Modified date: 20180716

//多BIT位掩码宏定义
#define 	MASKb1			((uint16_t)0x0001)
#define 	MASKb2			((uint16_t)0x0003)
#define 	MASKb3			((uint16_t)0x0007)
#define 	MASKb4			((uint16_t)0x000F)
#define 	MASKb5			((uint16_t)0x001F)
#define 	MASKb6			((uint16_t)0x003F)
#define 	MASKb7			((uint16_t)0x007F)
#define 	MASKb8			((uint16_t)0x00FF)


/******************************************************************************/
/*                                                                            */
/*                    PWR寄存器--电源控制寄存器                               */
/*                                                                            */
/******************************************************************************/
//typedef struct
//{
//  __IO uint32_t CR;
//  __IO uint32_t CSR;
//} PWR_TypeDef;

//PWR_CR--电源控制寄存器
#define  bPWR_CR_LPDS          BIT_ADDR(PWR_BASE, 0)     //深睡眠下的低功耗（PDDS=0时，与PDDS位协同操作）定义：0（在待机模式下电压调压器开启），1（在待机模式下电压调压器处于低功耗模式）
#define  bPWR_CR_PDDS          BIT_ADDR(PWR_BASE, 1)     //掉电深睡眠（与LPDS位协同操作）定义：0（当CPU进入深睡眠时进入停机模式，调压器状态由LPDS位控制），1（CPU进入深睡眠时进入待机模式）
#define  bPWR_CR_CWUF          BIT_ADDR(PWR_BASE, 2)     //清除唤醒位（始终输出为0）定义：0（无效），1（2个系统时钟周期后清除WUF唤醒位（写）
#define  bPWR_CR_CSBF          BIT_ADDR(PWR_BASE, 3)     //清除待机位（始终输出为0）定义：0（无效），1（清除SBF待机位（写）
#define  bPWR_CR_PVDE          BIT_ADDR(PWR_BASE, 4)     //电源电压检测器（PVD）使能。定义：0（禁止PVD），1（开启PVD）
#define  bPWR_CR_DBP           BIT_ADDR(PWR_BASE, 8)     //取消后备区域写保护。复位值为0。定义：0为禁止写入，1为允许写入。注：如果rtc时钟是HSE/128，必须保持为1

//#define  PWR_CR_PLS  以下3BIT定义PVD电压阀值
#define  bPWR_CR_PLS_0         BIT_ADDR(PWR_BASE, 5)     //定义:  000（2.2v），001（2.3v），010（2.4v）
#define  bPWR_CR_PLS_1         BIT_ADDR(PWR_BASE, 6)     //011（2.5v），100（2.6v），101（2.7v）
#define  bPWR_CR_PLS_2         BIT_ADDR(PWR_BASE, 7)     //110（2.8v），111（2.9v）



//PWR_CSR--电源控制状态寄存器
#define  bPWR_CSR_WUF           BIT_ADDR(PWR_BASE+4, 0)     //唤醒标志（该位由硬件设置，并只能由POR/PDR（上电/掉电复位）或设置电源控制寄存器（PWR_CR）的CWUF位清除）
                                                       //定义：0（没有唤醒事件），1（在WKUP引脚上发生唤醒事件或出现RTC闹钟事件） 
                                                       //注：当WKUP引脚已经是高电平时，在（通过设置EWUP位）使能WKUP引脚时，会检测到一个额外事件
#define  bPWR_CSR_SBF           BIT_ADDR(PWR_BASE+4, 1)     //待机标志位（该位由硬件设置，并只能由POR/PDR（上电/掉电复位）或设置电源控制寄存器（PWR_CR）的CSBUF位清除）定义：0（不在待机）1（已待机）
#define  bPWR_CSR_PVDO          BIT_ADDR(PWR_BASE+4, 2)     //PVDO-PVD输出（当PVD被PVDE位使能后该位才有效）定义：0（VDD/VDDA高于PLS[2-0]选定的PVD阀值），1（VDD/VDDA低于PLS[2-0]选定的PVD阀值）
                                                       //注：在待机模式下PVD被停止，因此，待机模式后或复位后，直到设置PVDE位之前，该位为0
#define  bPWR_CSR_EWUP          BIT_ADDR(PWR_BASE+4, 8)     //EWUP使能WKUP引脚。定义：0（WKUP为通用IO），1（用于待机唤醒模式，WKUP引脚被强置为输入下拉的配置（WKUP引脚上的上升沿将系统从待机模式唤醒）


/******************************************************************************/
/*                                                                            */
/*                    EXIT寄存器--外部中断/事件控制器(n=0-18)                  */
/*                                                                            */
/******************************************************************************/
//typedef struct
//{
//  __IO uint32_t IMR;
//  __IO uint32_t EMR;
//  __IO uint32_t RTSR;
//  __IO uint32_t FTSR;
//  __IO uint32_t SWIER;
//  __IO uint32_t PR;
//} EXTI_TypeDef;

#define  bEXTI_INT_MASK(n)									BIT_ADDR(EXTI_BASE, n)    //中断屏蔽:0屏蔽,1不屏蔽
#define  bEXTI_EVT_MASK(n)									BIT_ADDR(EXTI_BASE+4, n)  //事件屏蔽:0屏蔽,1不屏蔽
#define  bEXTI_TRIG_RISE(n)									BIT_ADDR(EXTI_BASE+8, n)  //上升沿触发:0禁用,1使能,可与下降沿触发共存
#define  bEXTI_TRIG_FALL(n)									BIT_ADDR(EXTI_BASE+12, n) //下降沿触发:0禁用,1使能,可与上升沿触发共存
#define  bEXTI_SFT_RQST(n)								  BIT_ADDR(EXTI_BASE+16, n) //软中断请求,写1产生中断挂起,向下面的PR相关位写1则清本位
#define  bEXTI_INT_PENDING(n)								BIT_ADDR(EXTI_BASE+20, n) //中断挂起,硬件置1,写1清0

/******************************************************************************/
/*                                                                            */
/*                         RCC寄存器--复位和时钟控制                           */
/*                                                                            */
/******************************************************************************/
//typedef struct
//{
//  __IO uint32_t CR;
//  __IO uint32_t CFGR;
//  __IO uint32_t CIR;
//  __IO uint32_t APB2RSTR;
//  __IO uint32_t APB1RSTR;
//  __IO uint32_t AHBENR;
//  __IO uint32_t APB2ENR;
//  __IO uint32_t APB1ENR;
//  __IO uint32_t BDCR;
//  __IO uint32_t CSR;

//#ifdef STM32F10X_CL  
//  __IO uint32_t AHBRSTR;
//  __IO uint32_t CFGR2;
//#endif /* STM32F10X_CL */ 

//#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || defined (STM32F10X_HD_VL)   
//  uint32_t RESERVED0;
//  __IO uint32_t CFGR2;
//#endif /* STM32F10X_LD_VL || STM32F10X_MD_VL || STM32F10X_HD_VL */ 
//} RCC_TypeDef;

//RCC_CR--时钟控制寄存器
#define  bRCC_CLK_HSION        BIT_ADDR(RCC_BASE, 0) //LSI时钟: 0禁用,1开启
#define  bRCC_CLK_HSIRDY        BIT_ADDR(RCC_BASE, 1) //LSI时钟状态由硬件控制(只读):0不可用,1就绪
#define  bRCC_CLK_HSEON        BIT_ADDR(RCC_BASE, 16) //HSE时钟: 0禁用,1开启
#define  bRCC_CLK_HSERDY        BIT_ADDR(RCC_BASE, 17) //HSE时钟状态由硬件控制(只读):0不可用,1就绪
#define  bRCC_CLK_HSEBYP       BIT_ADDR(RCC_BASE, 18) //外部时钟旁路(调试用)-- 0不旁路  1旁路
#define  bRCC_CLK_CSSON     BIT_ADDR(RCC_BASE, 19) //系统时钟安全系统使能  0时钟检测禁用  1外部时钟就绪后启动检测
#define  bRCC_CLK_PLLON       BIT_ADDR(RCC_BASE, 24) //PLL倍频: 0禁用,1开启
#define  bRCC_CLK_PLLRDY      BIT_ADDR(RCC_BASE, 25) //PLL倍频状态由硬件控制(只读):0不可用,1就绪
//本寄存器还有5BIT的HSITRIM  内部高速时钟调整
//           8BIT的HSICAL  内部高速时钟校准  用于补偿因温度等变化对内部RC振荡器时钟频率的影响.

//RCC_CFGR--时钟配置寄存器
#define  bRCC_CONFIG_SW0        BIT_ADDR(RCC_BASE+0x04, 0) //系统时钟选择2BIT-- 00:HSI  01:HSE
#define  bRCC_CONFIG_SW1        BIT_ADDR(RCC_BASE+0x04, 1) //                   10:PLL  11: 无效 
#define  bRCC_CONFIG_SW_PLL   	BIT_ADDR(RCC_BASE+0x04, 1)  //SYSCLK时钟选择位1，置位时为选择PLL作为系统时钟源

#define  bRCC_CONFIG_SWS0        BIT_ADDR(RCC_BASE+0x04, 2) //系统时钟源指示,只读: 定义同上
#define  bRCC_CONFIG_SWS1        BIT_ADDR(RCC_BASE+0x04, 3) //

#define  bRCC_CONFIG_SWS_PLL   		BIT_ADDR(RCC_BASE+0x04, 3)  //SYSCLK时钟指示位1，为1时指示PLL已经为系统时钟源
#define  bRCC_CONFIG_PLLSRC       BIT_ADDR(RCC_BASE+0x04, 16) //PLL钟源选择, 0: HSI/2  1:HSE（PREDIV1的输出）
#define  bRCC_CONFIG_PLLXTPRE     BIT_ADDR(RCC_BASE+0x04, 17) //输出至PLL的HSE是否分频 0:不分频  1:二分频
#define  bRCC_CONFIG_USBPRE       BIT_ADDR(RCC_BASE+0x04, 22) //USB预分频控制 0: PLL/1.5   1: PLL
#define  bRCC_CONFIG_ADCPRE0      BIT_ADDR(RCC_BASE+0x04, 14) //ADC(对PLCK2)预分频2BIT控制: 00: 2分频  01: 4分频
#define  bRCC_CONFIG_ADCPRE1      BIT_ADDR(RCC_BASE+0x04, 15) //                           10: 6分频  11: 8分频 


#define  SET_RCC_SW(a)   (MEM_ADDR(RCC_BASE+0x04)=(MEM_ADDR(RCC_BASE+0x04)&(~RCC_CFGR_SW))|((a&MASKb2)<<0))
//#define  RCC_CFGR_SW                         ((uint32_t)0x00000003)        /*!< SW[1:0] bits (System clock Switch) */
#define  SW_HSI                     0       /*!< HSI selected as system clock */
#define  SW_HSE                     1        /*!< HSE selected as system clock */
#define  SW_PLL                     2       /*!< PLL selected as system clock */


//4BIT的HPRE控制位: 控制系统时钟SYSCLK分频至AHB
///*!< HPRE BIT7:4 */
#define  SET_RCC_HPRE(a)   (MEM_ADDR(RCC_BASE+0x04)=(MEM_ADDR(RCC_BASE+0x04)&(~RCC_CFGR_HPRE))|((a&MASKb4)<<4))
     
//#define  RCC_CFGR_HPRE                       ((uint32_t)0x000000F0)        /*!< HPRE[3:0] bits (AHB prescaler) */
#define  HPRE_DIV1                  0        /*!< SYSCLK not divided */
#define  HPRE_DIV2                  1        /*!< SYSCLK divided by 2 */
#define  HPRE_DIV4                  3        /*!< SYSCLK divided by 4 */
#define  HPRE_DIV8                  4        /*!< SYSCLK divided by 8 */
#define  HPRE_DIV16                 5        /*!< SYSCLK divided by 16 */
#define  HPRE_DIV64                 6        /*!< SYSCLK divided by 64 */
#define  HPRE_DIV128                7        /*!< SYSCLK divided by 128 */
#define  HPRE_DIV256                8        /*!< SYSCLK divided by 256 */
#define  HPRE_DIV512                9        /*!< SYSCLK divided by 512 */

///*!< PPRE1 BIT10:8 共3BIT的PPRE1控制位: 控制至APB1的预分频*/           
#define  SET_RCC_HPRE1(a)   (MEM_ADDR(RCC_BASE+0x04)=(MEM_ADDR(RCC_BASE+0x04)&(~RCC_CFGR_PPRE1))|((a&MASKb3)<<8))
//#define  RCC_CFGR_PPRE1                      ((uint32_t)0x00000700)        /*!< PRE1[2:0] bits (APB1 prescaler) */
#define  PPRE1_DIV1                 0        /*!< HCLK not divided */
#define  PPRE1_DIV2                 4        /*!< HCLK divided by 2 */
#define  PPRE1_DIV4                 5        /*!< HCLK divided by 4 */
#define  PPRE1_DIV8                 6        /*!< HCLK divided by 8 */
#define  PPRE1_DIV16                7        /*!< HCLK divided by 16 */

///*!< PPRE2 BIT13:11 共3BIT的PPRE2控制位: 控制至APB2的预分频*/
#define  SET_RCC_HPRE2(a)   (MEM_ADDR(RCC_BASE+0x04)=(MEM_ADDR(RCC_BASE+0x04)&(~RCC_CFGR_PPRE2))|((a&MASKb3)<<11))
//#define  RCC_CFGR_PPRE2                      ((uint32_t)0x00003800)        /*!< PRE2[2:0] bits (APB2 prescaler) */
#define  PPRE2_DIV1                 0        /*!< HCLK not divided */
#define  PPRE2_DIV2                 4        /*!< HCLK divided by 2 */
#define  PPRE2_DIV4                 5        /*!< HCLK divided by 4 */
#define  PPRE2_DIV8                 6        /*!< HCLK divided by 8 */
#define  PPRE2_DIV16                7        /*!< HCLK divided by 16 */

///*!< ADCPPRE BIT15:14 */
#define  SET_RCC_ADCPRE(a)   (MEM_ADDR(RCC_BASE+0x04)=(MEM_ADDR(RCC_BASE+0x04)&(~RCC_CFGR_ADCPRE))|((a&MASKb2)<<14))
//#define  RCC_CFGR_ADCPRE                     ((uint32_t)0x0000C000)        /*!< ADCPRE[1:0] bits (ADC prescaler) */
#define  ADCPRE_DIV2                0        /*!< PCLK2 divided by 2 */
#define  ADCPRE_DIV4                1        /*!< PCLK2 divided by 4 */
#define  ADCPRE_DIV6                2        /*!< PCLK2 divided by 6 */
#define  ADCPRE_DIV8                3        /*!< PCLK2 divided by 8 */

//4BIT(BIT18:21)的PLLMUL:选择PLL的倍频数,2-16
#define  SET_RCC_PLLMUL(a)   (MEM_ADDR(RCC_BASE+0x04)=(MEM_ADDR(RCC_BASE+0x04)&(~RCC_CFGR_PLLMULL))|((a&MASKb4)<<18))
#define  PLLMULL2                  0        /*!< PLL input clock*2 */
#define  PLLMULL3                  1        /*!< PLL input clock*3 */
#define  PLLMULL4                  2       /*!< PLL input clock*4 */
#define  PLLMULL5                  3        /*!< PLL input clock*5 */
#define  PLLMULL6                  4        /*!< PLL input clock*6 */
#define  PLLMULL7                  5        /*!< PLL input clock*7 */
#define  PLLMULL8                  6        /*!< PLL input clock*8 */
#define  PLLMULL9                  7        /*!< PLL input clock*9 */
#define  PLLMULL10                 8        /*!< PLL input clock10 */
#define  PLLMULL11                 9        /*!< PLL input clock*11 */
#define  PLLMULL12                 10        /*!< PLL input clock*12 */
#define  PLLMULL13                 11        /*!< PLL input clock*13 */
#define  PLLMULL14                 12        /*!< PLL input clock*14 */
#define  PLLMULL15                 13        /*!< PLL input clock*15 */
#define  PLLMULL16                 14        /*!< PLL input clock*16 */

///*!< MCO configuration 注意不同的芯片有差异  3BIT的MCO控制位: 对MCU输出时钟的选择进行控制  */     
#define  SET_RCC_MCO(a)   (MEM_ADDR(RCC_BASE+0x04)=(MEM_ADDR(RCC_BASE+0x04)&(~RCC_CFGR_MCO))|((a&MASKb4)<<24))
 #define  MCO_NOCLOCK               ((uint32_t)0x00)        /*!< No clock */
 #define  MCO_SYSCLK                ((uint32_t)0x04)        /*!< System clock selected as MCO source */
 #define  MCO_HSI                   ((uint32_t)0x05)        /*!< HSI clock selected as MCO source */
 #define  MCO_HSE                   ((uint32_t)0x06)        /*!< HSE clock selected as MCO source  */
 #define  MCO_PLL                   ((uint32_t)0x07)        /*!< PLL clock divided by 2 selected as MCO source */

//RCC_CIR--时钟中断寄存器
#define  bRCC_INT_LSIRDYFLG      BIT_ADDR(RCC_BASE+0x08, 0) //LSI稳定且对应IE置位时由硬件置位,只读0无效,1可用
#define  bRCC_INT_LSERDYFLG      BIT_ADDR(RCC_BASE+0x08, 1) //LSE稳定且对应IE置位时由硬件置位,只读0无效,1可用
#define  bRCC_INT_HSIRDYFLG      BIT_ADDR(RCC_BASE+0x08, 2) //HSI稳定且对应IE置位时由硬件置位,只读0无效,1可用
#define  bRCC_INT_HSERDYFLG      BIT_ADDR(RCC_BASE+0x08, 3) //HSE稳定且对应IE置位时由硬件置位,只读0无效,1可用
#define  bRCC_INT_PLLRDYFLG      BIT_ADDR(RCC_BASE+0x08, 4) //PLL锁定且对应IE置位时由硬件置位,只读0无效,1可用
#define  bRCC_INT_CSSF           BIT_ADDR(RCC_BASE+0x08, 7) //外部振荡器失效时由硬件置位,只读0:无效,1中断可用
#define  bRCC_INT_LSIRDYIE       BIT_ADDR(RCC_BASE+0x08, 8) //LSI可用   中断使能, 0禁用  1使能
#define  bRCC_INT_LSERDYIE       BIT_ADDR(RCC_BASE+0x08, 9) //下同 
#define  bRCC_INT_HSIRDYIE       BIT_ADDR(RCC_BASE+0x08, 10)
#define  bRCC_INT_HSERDYIE       BIT_ADDR(RCC_BASE+0x08, 11)
#define  bRCC_INT_PLLRDYIE       BIT_ADDR(RCC_BASE+0x08, 12)
#define  bRCC_INT_LSIRDYCLR      BIT_ADDR(RCC_BASE+0x08, 16) //写1以清零相应的LSIRDYF,下同
#define  bRCC_INT_LSERDYCLR      BIT_ADDR(RCC_BASE+0x08, 17)
#define  bRCC_INT_HSIRDYCLR      BIT_ADDR(RCC_BASE+0x08, 18)
#define  bRCC_INT_HSERDYCLR      BIT_ADDR(RCC_BASE+0x08, 19)
#define  bRCC_INT_PLLRDYCLR      BIT_ADDR(RCC_BASE+0x08, 20)
#define  bRCC_INT_CSSCLR         BIT_ADDR(RCC_BASE+0x08, 23)

//RCC_APB2RSTR寄存器
#define  bRCC_RESET_AFIO                 	BIT_ADDR(RCC_BASE+0x0C, 0)   //0无效,1复位,下同
#define  bRCC_RESET_GPIOA                 BIT_ADDR(RCC_BASE+0x0C, 2)
#define  bRCC_RESET_GPIOB                 BIT_ADDR(RCC_BASE+0x0C, 3)
#define  bRCC_RESET_GPIOC                 BIT_ADDR(RCC_BASE+0x0C, 4)
#define  bRCC_RESET_GPIOD                 BIT_ADDR(RCC_BASE+0x0C, 5)
#define  bRCC_RESET_GPIOE                 BIT_ADDR(RCC_BASE+0x0C, 6)
#define  bRCC_RESET_ADC1                 	BIT_ADDR(RCC_BASE+0x0C, 9)
#define  bRCC_RESET_ADC2                 	BIT_ADDR(RCC_BASE+0x0C, 10)
#define  bRCC_RESET_TIM1                 	BIT_ADDR(RCC_BASE+0x0C, 11)
#define  bRCC_RESET_SPI1                 	BIT_ADDR(RCC_BASE+0x0C, 12)
#define  bRCC_RESET_USART1               	BIT_ADDR(RCC_BASE+0x0C, 14)

//RCC_APB1RSTR寄存器
#define  bRCC_RESET_TIM2                 BIT_ADDR(RCC_BASE+0x10, 0)   //0无效,1复位,下同
#define  bRCC_RESET_TIM3                 BIT_ADDR(RCC_BASE+0x10, 1)
#define  bRCC_RESET_TIM4                 BIT_ADDR(RCC_BASE+0x10, 2)
#define  bRCC_RESET_WWDG                 BIT_ADDR(RCC_BASE+0x10, 11)
#define  bRCC_RESET_SPI2                 BIT_ADDR(RCC_BASE+0x10, 14)
#define  bRCC_RESET_USART2               BIT_ADDR(RCC_BASE+0x10, 17)
#define  bRCC_RESET_USART3               BIT_ADDR(RCC_BASE+0x10, 18)
#define  bRCC_RESET_I2C1                 BIT_ADDR(RCC_BASE+0x10, 21)
#define  bRCC_RESET_I2C2                 BIT_ADDR(RCC_BASE+0x10, 22)
#define  bRCC_RESET_USB                  BIT_ADDR(RCC_BASE+0x10, 23)
#define  bRCC_RESET_CAN               	 BIT_ADDR(RCC_BASE+0x10, 25)
#define  bRCC_RESET_BKP                  BIT_ADDR(RCC_BASE+0x10, 27)
#define  bRCC_RESET_PWR                  BIT_ADDR(RCC_BASE+0x10, 28)

//RCC_AHBEN寄存器
#define  bRCC_ENABLE_DMA                   BIT_ADDR(RCC_BASE+0x14, 0)  //0关闭时钟,1开启时钟,下同
#define  bRCC_ENABLE_SRAM                  BIT_ADDR(RCC_BASE+0x14, 2)
#define  bRCC_ENABLE_FLITF                 BIT_ADDR(RCC_BASE+0x14, 4)
#define  bRCC_ENABLE_CRC                 	 BIT_ADDR(RCC_BASE+0x14, 6)
#define  bRCC_ENABLE_FSMC                	 BIT_ADDR(RCC_BASE+0x14, 8)  //?
#define  bRCC_ENABLE_SDIO                	 BIT_ADDR(RCC_BASE+0x14, 10)  //?

//RCC_APB2ENR寄存器
#define  bRCC_ENABLE_AFIO                 BIT_ADDR(RCC_BASE+0x18, 0)   //0关闭时钟,1开启时钟,下同
#define  bRCC_ENABLE_GPIOA                 BIT_ADDR(RCC_BASE+0x18, 2)
#define  bRCC_ENABLE_GPIOB                BIT_ADDR(RCC_BASE+0x18, 3)
#define  bRCC_ENABLE_GPIOC                BIT_ADDR(RCC_BASE+0x18, 4)
#define  bRCC_ENABLE_GPIOD                 BIT_ADDR(RCC_BASE+0x18, 5)
#define  bRCC_ENABLE_GPIOE                 BIT_ADDR(RCC_BASE+0x18, 6)
#define  bRCC_ENABLE_GPIOF                 BIT_ADDR(RCC_BASE+0x18, 7)
#define  bRCC_ENABLE_GPIOG                 BIT_ADDR(RCC_BASE+0x18, 8)
#define  bRCC_ENABLE_ADC1                 BIT_ADDR(RCC_BASE+0x18, 9)
#define  bRCC_ENABLE_ADC2                 BIT_ADDR(RCC_BASE+0x18, 10)
#define  bRCC_ENABLE_TIM1                 BIT_ADDR(RCC_BASE+0x18, 11)
#define  bRCC_ENABLE_SPI1                 BIT_ADDR(RCC_BASE+0x18, 12)
#define  bRCC_ENABLE_USART1               BIT_ADDR(RCC_BASE+0x18, 14)

//RCC_APB1ENR寄存器
#define  bRCC_ENABLE_TIM2                 BIT_ADDR(RCC_BASE+0x1C, 0)   //0关闭时钟,1开启时钟,下同
#define  bRCC_ENABLE_TIM3                 BIT_ADDR(RCC_BASE+0x1C, 1)
#define  bRCC_ENABLE_TIM4                 BIT_ADDR(RCC_BASE+0x1C, 2)
#define  bRCC_ENABLE_WWDG                 BIT_ADDR(RCC_BASE+0x1C, 11)
#define  bRCC_ENABLE_SPI2                 BIT_ADDR(RCC_BASE+0x1C, 14)
#define  bRCC_ENABLE_USART2               BIT_ADDR(RCC_BASE+0x1C, 17)
#define  bRCC_ENABLE_USART3               BIT_ADDR(RCC_BASE+0x1C, 18)
#define  bRCC_ENABLE_I2C1                 BIT_ADDR(RCC_BASE+0x1C, 21)
#define  bRCC_ENABLE_I2C2                 BIT_ADDR(RCC_BASE+0x1C, 22)
#define  bRCC_ENABLE_USB                  BIT_ADDR(RCC_BASE+0x1C, 23)
#define  bRCC_ENABLE_CAN               	  BIT_ADDR(RCC_BASE+0x1C, 25)
#define  bRCC_ENABLE_BKP                  BIT_ADDR(RCC_BASE+0x1C, 27)
#define  bRCC_ENABLE_PWR                  BIT_ADDR(RCC_BASE+0x1C, 28)

//RCC_BDCR寄存器--备份区域控制
#define  bRCC_CLK_LSEON                  BIT_ADDR(RCC_BASE+0x20, 0)  //LSE时钟: 0禁用,1开启
#define  bRCC_CLK_LSERDY                 BIT_ADDR(RCC_BASE+0x20, 1) //LSE时钟状态由硬件控制(只读):0不可用,1就绪
#define  bRCC_CLK_RTCSEL0                BIT_ADDR(RCC_BASE+0x20, 24) //RTC时钟源选择两位共同控制: 00:无时钟 01:LSE
#define  bRCC_CLK_RTCSEL1                BIT_ADDR(RCC_BASE+0x20, 26) //                      10:LSI   11: HSE/128 
#define  bRCC_ENABLE_RTC                 BIT_ADDR(RCC_BASE+0x20, 27)  //0禁用RTC; 1:使能RTC
#define  bRCC_RESET_BKUPDOMAIN           BIT_ADDR(RCC_BASE+0x20, 28)   //备份区域软复位  写1复位 0复位未被激活

//RCC_CSR寄存器--状态与控制
#define  bRCC_CLK_LSION                   BIT_ADDR(RCC_BASE+0x24, 0)   //LSI时钟: 0禁用,1开启
#define  bRCC_CLK_LSIRDY                  BIT_ADDR(RCC_BASE+0x24, 1)   //LSI时钟状态由硬件控制(只读):0不可用,1就绪
#define  bRCC_FLG_RMVF                    BIT_ADDR(RCC_BASE+0x24, 24) //清除复位标志 写0复位未激活的复位标志,写1则清复位标志
#define  bRCC_FLG_PINRSTF                 BIT_ADDR(RCC_BASE+0x24, 26)  //引脚复位标志  硬件置1软件写RMVF位清0
#define  bRCC_FLG_PORRSTF                 BIT_ADDR(RCC_BASE+0x24, 27)  //端口复位标志  硬件置1软件写RMVF位清0
#define  bRCC_FLG_SFTRSTF                 BIT_ADDR(RCC_BASE+0x24, 28)   //软件复位标志  硬件置1软件写RMVF位清0
#define  bRCC_FLG_IWDGRSTF                BIT_ADDR(RCC_BASE+0x24, 29)  //独立看门狗复位标志  硬件置1软件写RMVF位清0
#define  bRCC_FLG_WWDGRSTF                BIT_ADDR(RCC_BASE+0x24, 30)//窗口看门狗复位标志  硬件置1软件写RMVF位清0
#define  bRCC_FLG_LPWRRSTF                BIT_ADDR(RCC_BASE+0x24, 31)//低功耗管理复位标志  硬件置1软件写RMVF位清0 

/******************************************************************************/
/*                                                                            */
/*                         AFIO寄存器                                         */
/*                                                                            */
/******************************************************************************/
//typedef struct
//{
//  __IO uint32_t EVCR;
//  __IO uint32_t MAPR;
//  __IO uint32_t EXTICR[4];
//  uint32_t RESERVED0;
//  __IO uint32_t MAPR2;  
//} AFIO_TypeDef;
//AFIO_EVCR--事件控制寄存器
#define  bAFIO_EVCR_EVOE        BIT_ADDR(AFIO_BASE, 7) //事件输出使能. 为1时,事件由指定的端口输出(端口由BIT6:4,引脚由BIT3:0决定)
#define  bAFIO_MAPR_SPI1_REMAP        BIT_ADDR(AFIO_BASE+4, 0) //0正常 置1时SPI1原PA4/PA5/PA6/PA7分别映射到PA15/PB3/PB4/PB5
#define  bAFIO_MAPR_I2C1_REMAP        BIT_ADDR(AFIO_BASE+4, 1) //0正常 置1时I2C1原PB6/PB7分别映射到PB8/PB9
#define  bAFIO_MAPR_USART1_REMAP        BIT_ADDR(AFIO_BASE+4, 2) //0正常 置1时USART1原PA9/PA10分别映射到PB6/PB7
#define  bAFIO_MAPR_USART2_REMAP        BIT_ADDR(AFIO_BASE+4, 3) //0正常 置1时USART2原PA0/1/2/3/4分别映射到PD3/4/5/6/7
#define  bAFIO_MAPR_USART3_REMAPb0      BIT_ADDR(AFIO_BASE+4, 4) //两BIT控制: 00正常 01部分重映射 10未用 11满重映射
#define  bAFIO_MAPR_USART3_REMAPb1      BIT_ADDR(AFIO_BASE+4, 5) //
#define  bAFIO_MAPR_TIM1_REMAPb0      BIT_ADDR(AFIO_BASE+4, 6) //两BIT控制: 00正常 01部分重映射 10未用 11满重映射
#define  bAFIO_MAPR_TIM1_REMAPb1      BIT_ADDR(AFIO_BASE+4, 7) //
#define  bAFIO_MAPR_TIM2_REMAPb0      BIT_ADDR(AFIO_BASE+4, 8) //两BIT控制: 00正常 01部分重映射 10部分重映射 11满重映射
#define  bAFIO_MAPR_TIM2_REMAPb1      BIT_ADDR(AFIO_BASE+4, 9) //
#define  bAFIO_MAPR_TIM3_REMAPb0      BIT_ADDR(AFIO_BASE+4, 10) //两BIT控制: 00正常 01未用 10部分重映射 11完全重映射
#define  bAFIO_MAPR_TIM3_REMAPb1      BIT_ADDR(AFIO_BASE+4, 11) //
#define  bAFIO_MAPR_TIM4_REMAP      BIT_ADDR(AFIO_BASE+4, 12) //只控制100引脚封装中TIM4的通道1-4的映射. 0正常,1由原来的PB6/7/8/9映射到PD12/13/14/15
#define  bAFIO_MAPR_CAN_REMAPb0      BIT_ADDR(AFIO_BASE+4, 13) //两BIT控制: 
#define  bAFIO_MAPR_CAN_REMAPb1      BIT_ADDR(AFIO_BASE+4, 14) //
#define  bAFIO_MAPR_PD01_REMAP      BIT_ADDR(AFIO_BASE+4, 15) //0正常, 1 PD0/1端口分别映像到OSC_IN/OSC_OUT引脚
#define  bAFIO_MAPR_SWJ_CFGb0      BIT_ADDR(AFIO_BASE+4, 24) //3BIT控制串行JTAG配置(只写):000（完全SWJ，复位状态），001（完全SWJ，但没有NJTRST），
#define  bAFIO_MAPR_SWJ_CFGb1      BIT_ADDR(AFIO_BASE+4, 25) //010（关闭JATG启动SW），100（关闭JATG，关闭SW）其它值无定义
#define  bAFIO_MAPR_SWJ_CFGb2      BIT_ADDR(AFIO_BASE+4, 26) //
//有些新器件,可能对BIT16/17/18/19/20有映射定义(分别对应TIM5/ADC1/ADC2等引脚功能),具体需要用时详见手册并添加宏
//AFIO的另一个重要功能是:共用4个外部中断配置寄存器EXTICR1-4, 控制16条中断线的端口组(分别用4BIT决定PA或PB/....../PG)的选择,不宜使用位操作功能,从略.

/******************************************************************************/
/*                                                                            */
/*                         GPIOx寄存器                                        */
/*                                                                            */
/******************************************************************************/
//#define GPIOA_BASE            (APB2PERIPH_BASE + 0x0800)
//#define GPIOB_BASE            (APB2PERIPH_BASE + 0x0C00)
//#define GPIOC_BASE            (APB2PERIPH_BASE + 0x1000)
//#define GPIOD_BASE            (APB2PERIPH_BASE + 0x1400)
//#define GPIOE_BASE            (APB2PERIPH_BASE + 0x1800)
//#define GPIOF_BASE            (APB2PERIPH_BASE + 0x1C00)
//#define GPIOG_BASE            (APB2PERIPH_BASE + 0x2000)
//typedef struct
//{
//  __IO uint32_t CRL; //0x00
//  __IO uint32_t CRH; //0x04
//  __IO uint32_t IDR; //0x08
//  __IO uint32_t ODR; //0x0C
//  __IO uint32_t BSRR;//0x10
//  __IO uint32_t BRR; //0x14
//  __IO uint32_t LCKR;//0x18
//} GPIO_TypeDef;
//注意不同的GPIOx是划分在不同的基址上的, 不同的基址以GPIOA_BASE/GPIOB_BASE等等来区分.
//GPIO的位操作,可以用大家广泛使用的PAout(n)......PCin(n)的方式
//对GPIO的端口配置: 每线采用4BIT,不太宜使用位操作功能,从略.
//GPIO的端口配置锁定功能GPIOx_LCKR寄存器,适合采用位操作方式,但一般此功能不常用.
#define  bGPIOA_BSRR(n)   BIT_ADDR(GPIOA_BASE+0x10, n)
#define  bGPIOB_BSRR(n)   BIT_ADDR(GPIOB_BASE+0x10, n)
#define  bGPIOC_BSRR(n)   BIT_ADDR(GPIOC_BASE+0x10, n)
#define  bGPIOD_BSRR(n)   BIT_ADDR(GPIOD_BASE+0x10, n)
#define  bGPIOE_BSRR(n)   BIT_ADDR(GPIOE_BASE+0x10, n)
#define  bGPIOF_BSRR(n)   BIT_ADDR(GPIOF_BASE+0x10, n)
#define  bGPIOG_BSRR(n)   BIT_ADDR(GPIOG_BASE+0x10, n)
#define  bGPIOA_BRR(n)   BIT_ADDR(GPIOA_BASE+0x14, n)
#define  bGPIOB_BRR(n)   BIT_ADDR(GPIOB_BASE+0x14, n)
#define  bGPIOC_BRR(n)   BIT_ADDR(GPIOC_BASE+0x14, n)
#define  bGPIOD_BRR(n)   BIT_ADDR(GPIOD_BASE+0x14, n)
#define  bGPIOE_BRR(n)   BIT_ADDR(GPIOE_BASE+0x14, n)
#define  bGPIOF_BRR(n)   BIT_ADDR(GPIOF_BASE+0x14, n)
#define  bGPIOG_BRR(n)   BIT_ADDR(GPIOG_BASE+0x14, n)
#define  bGPIOA_LOCK(n)   BIT_ADDR(GPIOA_BASE+0x18, n)
#define  bGPIOB_LOCK(n)   BIT_ADDR(GPIOB_BASE+0x18, n)
#define  bGPIOC_LOCK(n)   BIT_ADDR(GPIOC_BASE+0x18, n)
#define  bGPIOD_LOCK(n)   BIT_ADDR(GPIOD_BASE+0x18, n)
#define  bGPIOE_LOCK(n)   BIT_ADDR(GPIOE_BASE+0x18, n)
#define  bGPIOF_LOCK(n)   BIT_ADDR(GPIOF_BASE+0x18, n)
#define  bGPIOG_LOCK(n)   BIT_ADDR(GPIOG_BASE+0x18, n)
#define  bGPIOA_LOCKKEY   BIT_ADDR(GPIOA_BASE+0x18, 16)
#define  bGPIOB_LOCKKEY   BIT_ADDR(GPIOB_BASE+0x18, 16)
#define  bGPIOC_LOCKKEY   BIT_ADDR(GPIOC_BASE+0x18, 16)
#define  bGPIOD_LOCKKEY   BIT_ADDR(GPIOD_BASE+0x18, 16)
#define  bGPIOE_LOCKKEY   BIT_ADDR(GPIOE_BASE+0x18, 16)
#define  bGPIOF_LOCKKEY   BIT_ADDR(GPIOF_BASE+0x18, 16)
#define  bGPIOG_LOCKKEY   BIT_ADDR(GPIOG_BASE+0x18, 16)

#define  bGPIOx_BSRR(m,n)  BIT_ADDR(GPIOA_BASE+0x400*m+0x10, n) //置位指定端口的指定位
#define  bGPIOx_BRR(m,n)   BIT_ADDR(GPIOA_BASE+0x400*m+0x14, n)  //复位指定端口的指定位
#define  bGPIOx_ODR(m,n)   BIT_ADDR(GPIOA_BASE+0x400*m+0x0C, n)  //指定端口的指定位
#define  bGPIOx_IDR(m,n)   BIT_ADDR(GPIOA_BASE+0x400*m+0x08, n)  //指定端口的指定位



/******************************************************************************/
/*                                                                            */
/*                         BKP寄存器                                          */
/*                                                                            */
/******************************************************************************/
//前10个寄存器完全可为用户自由使用,地址为:BKP_BASE+0x04至BKP_BASE+0x28,每个寄存器占用4字节空间,即一个字,但只有低16位有效.
//可直接用下列宏指令对其进行访问， BKP_DR1等同于BKP->DR1
#define  BKP_DR1  			MEM_ADDR(BKP_BASE+0x04)
#define  BKP_DR2  			MEM_ADDR(BKP_BASE+0x08)
#define  BKP_DR3  			MEM_ADDR(BKP_BASE+0x0C)
#define  BKP_DR4  			MEM_ADDR(BKP_BASE+0x10)
#define  BKP_DR5  			MEM_ADDR(BKP_BASE+0x14)
#define  BKP_DR6  			MEM_ADDR(BKP_BASE+0x18)
#define  BKP_DR7  			MEM_ADDR(BKP_BASE+0x1C)
#define  BKP_DR8  			MEM_ADDR(BKP_BASE+0x20)
#define  BKP_DR9  			MEM_ADDR(BKP_BASE+0x24)
#define  BKP_DR10  			MEM_ADDR(BKP_BASE+0x28)
//这些寄存器不会被系统复位，电源复位，待机唤醒所复位
//注意对后备寄存器的写操作必须使能PWR及BKP的时钟，即bRCC_ENABLE_PWR=1;bRCC_ENABLE_BKP=1; 
//并且取消后备寄存器写保护：bPWR_CR_DBP=1;  		
//对后备寄存器的读则无须以上三点，随时可读。		

//BKP_RTCCR（RTC时钟校准寄存器）
//BIT6:0 CAL校准值。表示在每2的20次方个时钟脉冲内将有多少个脉冲被跳过。这可用来对RTC进行校准，以1000000/（2的20次方比例减慢时钟）可用被减慢0-121ppm
#define  bBKP_RTCCR_CCO        BIT_ADDR(BKP_BASE+0x2C, 7) //CCO校准时钟输出。定义：0（无影响），1（此位置1可在侵入检引脚输出经64分频后的RTC时钟。
                                                         //  当CCO位置1时，必须关闭侵入检测）注：vdd断电，该位清除
#define  bBKP_RTCCR_ASOE       BIT_ADDR(BKP_BASE+0x2C, 8) //允许输出闹钟或秒脉冲（根据ASOS位的置位，该位允许RTC闹钟或秒脉冲输出到TAMPER引脚。
                                                         // 脉冲宽度为1个RTC时钟周期。置位时不能开启TAMPER功能）
#define  bBKP_RTCCR_ASOS       BIT_ADDR(BKP_BASE+0x2C, 9) //闹钟或秒输出（当设置ASOE位，ASOS位可用于选择在TAMPER引脚上输出的是RTC秒脉冲还是闹钟脉冲信号）
                                                         //定义：0（输出RTC闹钟脉冲），1（输出秒脉冲）注：后备区复位清除
//BKP_CR备份控制寄存器
#define  bBKP_CR_TPE        BIT_ADDR(BKP_BASE+0x30, 0) //TPAL侵入检测TAMPER引脚有效电平。0检测TAMPER脚高电平清除备份数据 1检测TAMPER脚低电平清除备份数据
#define  bBKP_CR_TPAL       BIT_ADDR(BKP_BASE+0x30, 1) //TPE启动入侵检测TAMPER引脚。定义：0（TAMPER脚为普通IO），1（开启检测）
//BKP_CR备份控制/状态寄存器
#define  bBKP_CSR_CTE        BIT_ADDR(BKP_BASE+0x34, 0) //CTE清除侵入检测事件（只能写入，读出值为0）定义：0（无效）1（清除TEF侵入检测事件标志（并复位侵入检测器）
#define  bBKP_CSR_CTI        BIT_ADDR(BKP_BASE+0x34, 1) //CTI清除侵入检测中断（只能写入，读出值为0）定义：0（无效）1（清除侵入检测中断和TIF侵入检测中断标志）
#define  bBKP_CSR_TPIE       BIT_ADDR(BKP_BASE+0x34, 2) //允许侵入TAMPER引脚中断。定义0（禁止侵入检测中断），1（允许（BKP_CR寄存器TPE位也必须置1）
#define  bBKP_CSR_TEF        BIT_ADDR(BKP_BASE+0x34, 8) //TEF侵入事件标志由硬件置位。通过向CTE位写1可清除此标志位）定义：0（无侵入事件），1（有侵入事件）
#define  bBKP_CSR_TIF        BIT_ADDR(BKP_BASE+0x34, 9) //TIF侵入中断标志（当检测有侵入事件且TPIE为1时，此为硬件置1，通过向CTI位写1来清除标志位（同时也清除中断）。
                                                       //如果TPIE被清除，此位也会被清除。

/******************************************************************************/
/*                                                                            */
/*                         RTC寄存器                                          */
/*                                                                            */
/******************************************************************************/
//RTC控制寄存器高位RTC_CRH
#define  bRTC_CRH_SECIE        BIT_ADDR(RTC_BASE+0x00, 0) //SECIE允许秒中断位，定义：0（屏蔽中断），1（允许中断）
#define  bRTC_CRH_ALRIE        BIT_ADDR(RTC_BASE+0x00, 1)  //ALRIE允许闹钟中断位，定义：0（屏蔽中断），1（允许中断）
#define  bRTC_CRH_OWIE         BIT_ADDR(RTC_BASE+0x00, 2) //OWIE允许溢出中断位，定义：0（屏蔽中断），1（允许中断）
//RTC控制寄存器低位RTC_CRL
#define  bRTC_CRL_SECF        BIT_ADDR(RTC_BASE+0x04, 0)  //秒标志,硬件置’1’同时RTC计数器加1。因此,此标志为分辨率可编程的RTC计数器提供一个周期性的信号(通常为1秒)。
#define  bRTC_CRL_ALRF        BIT_ADDR(RTC_BASE+0x04, 1) //硬件置1,只能由软件清’0’。对此位写’1’是无效的，定义：0（无闹钟），1（有闹钟）
#define  bRTC_CRL_OWF         BIT_ADDR(RTC_BASE+0x04, 2) //OWF溢出标志，由硬件置’1’。如果RTC_CRH寄存器中OWIE=1，则产生中断。此位只能由软件清’0’。
#define  bRTC_CRL_RSF        BIT_ADDR(RTC_BASE+0x04, 3)  //RSF寄存器同步标志，每当RTC_CNT寄存器和RTC_DIV寄存器由软件更新或清’0’时，此位由硬件置’1’。在APB1复位后，//或APB1时钟停止后，此位必须由软件清’0’。要进行任何的读操作之前，用户程序必须等待这位被硬件置’1’，以确保RTC_CNT、RTC_ALR或RTC_PRL已经被同步。
#define  bRTC_CRL_CNF        BIT_ADDR(RTC_BASE+0x04, 4) //CNF配置标志，此位必须由软件置’1’以进入配置模式，从而允许向RTC_CNT、RTC_ALR或RTC_PRL寄存器写入数据。只有当//此位在被置’1’并重新由软件清’0’后，才会执行写操作，定义：0（退出配置模式(开始更新RTC寄存器)，1（进入配置模式）
#define  bRTC_CRL_RTOFF      BIT_ADDR(RTC_BASE+0x04, 5) //RTC操作关闭（只读位）RTC模块利用这位来指示对其寄存器进行的最后一次操作的状态，指示操作是否完成。若此位
//为’0’，则表示无法对任何的RTC寄存器进行写操作。定义：0（上一次对RTC寄存器的写操作仍在进行），1（上一次对RTC寄存器的写操作已经完成）
//另外,RTC寄存器还有预分频装载寄存器/预分频除法寄存器/32位的RTC计数器/32位的闹钟计数器, 这些都不宜进行位操作,直接按字整体读写.

/******************************************************************************/
/*                                                                            */
/*         USART寄存器                                   											*/
/*                                                                            */
/******************************************************************************/
//#define USART1_BASE           (APB2PERIPH_BASE + 0x3800)  //注意此地址与下面的地址不连续
//#define USART2_BASE           (APB1PERIPH_BASE + 0x4400)
//#define USART3_BASE           (APB1PERIPH_BASE + 0x4800)
//#define UART4_BASE            (APB1PERIPH_BASE + 0x4C00)
//#define UART5_BASE            (APB1PERIPH_BASE + 0x5000)

//typedef struct
//{
//  __IO uint16_t SR;  //0x00
//  uint16_t  RESERVED0;
//  __IO uint16_t DR;  //0x04
//  uint16_t  RESERVED1;
//  __IO uint16_t BRR; //0x08
//  uint16_t  RESERVED2;
//  __IO uint16_t CR1; //0x0C
//  uint16_t  RESERVED3;
//  __IO uint16_t CR2; //0x10
//  uint16_t  RESERVED4;
//  __IO uint16_t CR3; //0x14
//  uint16_t  RESERVED5;
//  __IO uint16_t GTPR; //0x18
//  uint16_t  RESERVED6;
//} USART_TypeDef;

//USART状态寄存器SR
#define  bUSART1_SR_PE 			BIT_ADDR(USART1_BASE+0x00, 0) //校验错误（Parity Error）在接收模式下，如果出现奇偶校验错误，硬件对该位置位。
#define  bUSART1_SR_FE 			BIT_ADDR(USART1_BASE+0x00, 1) //帧错误（Framing Error）当检测到同步错位，过多的噪声或者检测到断开符，该位被硬件置位。
#define  bUSART1_SR_NE 			BIT_ADDR(USART1_BASE+0x00, 2) //噪声错误（Noise Error）在接收到的帧检测到噪音时，由硬件对该位置位。
#define  bUSART1_SR_ORE 		BIT_ADDR(USART1_BASE+0x00, 3) //过载错误（Overrun Error）当RXNE仍然是’1’的时候，当前被接收在移位寄存器中的数据，需要传送至RDR寄存器时，硬件将该位置位。如果USART_CR1中的RXNEIE为’1’的话，则产生中断。
#define  bUSART1_SR_IDLE		BIT_ADDR(USART1_BASE+0x00, 4) //IDLE：监测到总线空闲；当检测到总线空闲时，该位被硬件置位。如果USART_CR1中的IDLEIE为’1’，则产生中断。
#define  bUSART1_SR_RXNE		BIT_ADDR(USART1_BASE+0x00, 5) //接收据寄存器非空（Receive Not Empty），当该位被置位的时候，就是提示已经有数据被接收到了，并且可以读出来了。通过读USART_DR可以将该位清零，也可以向该位写0，直接清除。
#define  bUSART1_SR_TC 			BIT_ADDR(USART1_BASE+0x00, 6) //发送完成（Transmit Complete），当该位被置位的时候，表示USART_DR内的数据已经被发送完成了。如果设置了这个位的中断，则会产生中断。该位也有两种清零方式：1）读USART_SR，写USART_DR。2）直接向该位写0。
#define  bUSART1_SR_TXE 		BIT_ADDR(USART1_BASE+0x00, 7) //发送数据寄存器空（Transmit Empty）当TDR寄存器中的数据被硬件转移到移位寄存器的时候，该位被硬件置位。如果USART_CR1寄存器中的TXEIE为1，则产生中断。对USART_DR的写操作，将该位清零。
#define  bUSART1_SR_LBD 		BIT_ADDR(USART1_BASE+0x00, 8) //LIN断开检测（LIN Break Detect）当探测到LIN断开时，该位由硬件置’1’，由软件清’0’(向该位写0)。如果USART_CR3中的LBDIE = 1，则产生中断。
#define  bUSART1_SR_CTS 		BIT_ADDR(USART1_BASE+0x00, 9)  //如果设置了CTSE位，当nCTS输入变化状态时，该位被硬件置高。由软件将其清零。如果USART_CR3中的CTSIE为’1’，则产生中断。
//USART状态寄存器CR1
#define  bUSART1_CR1_SBK     BIT_ADDR(USART1_BASE+0x0C, 0)  //发送断开帧（Send Break）位；使用该位来发送断开字符。该位可以由软件设置或清除。操作过程应该是软件设置位它，然后在断开帧的停止位时，由硬件将该位复位。
#define  bUSART1_CR1_RWU     BIT_ADDR(USART1_BASE+0x0C, 1)  //接收唤醒（Receiver Wakeup）位；置0，正常模式；置1，静默模式。
#define  bUSART1_CR1_RE      BIT_ADDR(USART1_BASE+0x0C, 2)  //接收使能（Receive Enable）位，用法同 TE。
#define  bUSART1_CR1_TE      BIT_ADDR(USART1_BASE+0x0C, 3)  //发送使能（Transmit Enable）位，设置为1，将开启串口的发送功能。
#define  bUSART1_CR1_IDLEIE  BIT_ADDR(USART1_BASE+0x0C, 4)  //IDLE中断使能（IDLE Interrupt Enable）位，置0，禁止中断；置1，当USART_SR中的IDLE为’1’时，产生USART中断。
#define  bUSART1_CR1_RXNEIE  BIT_ADDR(USART1_BASE+0x0C, 5)  //接收缓冲区非空中断使能（Receive Non-Empty Interrupt Enable）位，设置该位为 1，当 USART_SR中的 ORE 或者 RXNE 位为 1 时，将产生串口中断。
#define  bUSART1_CR1_TCIE    BIT_ADDR(USART1_BASE+0x0C, 6)  //发送完成中断使能（Transmit Complete Interrupt Enable）位，设置该位为 1，当 USART_SR 中的 TC位为 1 时，将产生串口中断。
#define  bUSART1_CR1_TXEIE   BIT_ADDR(USART1_BASE+0x0C, 7)  //发送缓冲区空中断使能（Transmit Interrupt Enable）位，设置该位为 1，当 USART_SR 中的 TXE 位为1 时，将产生串口中断。
#define  bUSART1_CR1_PEIE    BIT_ADDR(USART1_BASE+0x0C, 8)  //PE中断使能（Parity Error Interrupt Enable），置0，禁止中断；置1，当USART_SR中的PE为’1’时，产生USART中断。
#define  bUSART1_CR1_PS      BIT_ADDR(USART1_BASE+0x0C, 9)   //校验位选择（Parity Select）位，设置为0则为偶校验，否则为奇校验。
#define  bUSART1_CR1_PCE     BIT_ADDR(USART1_BASE+0x0C, 10)  //校验控制使能（Parity Control Enable）位，置0，则禁止校验，否则使能校验。
#define  bUSART1_CR1_WAKE    BIT_ADDR(USART1_BASE+0x0C, 11) //唤醒；置0，被空闲总线唤醒；置1，被地址标记唤醒。
#define  bUSART1_CR1_M       BIT_ADDR(USART1_BASE+0x0C, 12) //字长选择位，当该位为0的时候设置串口为8个字长外加停止位，停止位的个数是根据USART_CR2的[13:12]位设置来决定的，默认为0。
#define  bUSART1_CR1_UE      BIT_ADDR(USART1_BASE+0x0C, 13) //串口使能（Usart Enable）位，通过该位置1，以使能串口。当该位被清零，分频器和输出停止工作，以减少功耗。
#define  bUSART1_CR1_OVER8   BIT_ADDR(USART1_BASE+0x0C, 14)           /*!< USART Oversmapling 8-bits */
//USART状态寄存器CR2
#define  SET_USART1_CR2_ADD(a)   (MEM_ADDR(USART1_BASE+0x10)=(MEM_ADDR(USART1_BASE+0x10)&(~USART_CR2_ADD))|((a&MASKb5)<<0))
// #define  bUSART1_CR2_ADD     BIT_ADDR(USART1_BASE+0x10, 0)  //本设备的USART节点地址(BIT 0:4共5位)。这是在多处理器通信下的静默模式中使用的，使用地址标记来唤醒某个USART设备。
#define  bUSART1_CR2_LBDL    BIT_ADDR(USART1_BASE+0x10, 5)  //LIN断开符检测长度（LIN Break Detection Length）位；该位用来选择是11位还是10位的断开符检测。
#define  bUSART1_CR2_LBDIE   BIT_ADDR(USART1_BASE+0x10, 6)  //LIN断开符检测中断使能（LIN Break Detection Interrupt Enable）位；置0，禁止中断；置1，只要USART_SR寄存器中的LBD为’1’就产生中断。
#define  bUSART1_CR2_LBCL    BIT_ADDR(USART1_BASE+0x10, 7)  //最后一位时钟脉冲（Last Bit Clock Pulse）位；在同步模式下，使用该位来控制是否在CK引脚上输出最后发送的那个数据字节(MSB)对应的时钟脉冲。
#define  bUSART1_CR2_CPHA    BIT_ADDR(USART1_BASE+0x10, 9)  //时钟相位（Clock Phase）位；在同步模式下，可以用该位选择SLCK引脚上时钟输出的相位。
#define  bUSART1_CR2_CPOL    BIT_ADDR(USART1_BASE+0x10, 10) //时钟极性（Clock Polarity）位；在同步模式下，可以用该位选择SLCK引脚上时钟输出的极性。
#define  bUSART1_CR2_CLKEN   BIT_ADDR(USART1_BASE+0x10, 11) //时钟使能（Clock Enable）位；该位用来使能CK引脚。
#define  SET_USART1_CR2_STOP(a)   (MEM_ADDR(USART1_BASE+0x10)=(MEM_ADDR(USART1_BASE+0x10)&(~USART_CR2_STOP))|((a&MASKb2)<<12))
// #define  bUSART1_CR2_STOP    BIT_ADDR(USART1_BASE+0x10, 12) //停止位（STOP）位；这2位用来设置停止位的位数；00：1个停止位；01：0.5个停止位；10：2个停止位；11：1.5个停止位；          /*!< STOP[1:0] bits (STOP bits) */
#define  bUSART1_CR2_STOP_0  BIT_ADDR(USART1_BASE+0x10, 12) //           
#define  bUSART1_CR2_STOP_1  BIT_ADDR(USART1_BASE+0x10, 13) //
#define  bUSART1_CR2_LINEN   BIT_ADDR(USART1_BASE+0x10, 14) //LIN模式使能（LIN Enable）位；在LIN模式下，可以用USART_CR1寄存器中的SBK位发送LIN同步断开符(低13位)，以及检测LIN同步断开符。

//USART状态寄存器CR3
#define  bUSART1_CR3_EIE     BIT_ADDR(USART1_BASE+0x14, 0)  //错误中断使能（Error Interrupt Enable）位；
#define  bUSART1_CR3_IREN    BIT_ADDR(USART1_BASE+0x14, 1)  //红外模式使能（IrDA Enable）位；
#define  bUSART1_CR3_IRLP    BIT_ADDR(USART1_BASE+0x14, 2)  //红外低功耗（IrDA Low-Power）位；该位用来选择普通模式还是低功耗红外模式；
#define  bUSART1_CR3_HDSEL   BIT_ADDR(USART1_BASE+0x14, 3)  //半双工选择（Half-duplex Selection）位；选择单线半双工模式；
#define  bUSART1_CR3_NACK    BIT_ADDR(USART1_BASE+0x14, 4)  //智能卡NACK（Smartcard NACK）位；置0，不发送NACK；置1，发送NACK；
#define  bUSART1_CR3_SCEN    BIT_ADDR(USART1_BASE+0x14, 5)  //智能卡使能（Smartcard Enable）位；该位用来使能智能卡模式
#define  bUSART1_CR3_DMAR    BIT_ADDR(USART1_BASE+0x14, 6)  //DMA接收（DMA Receiver）位； 
#define  bUSART1_CR3_DMAT    BIT_ADDR(USART1_BASE+0x14, 7)  //DMA发送（DMA Transmitter）位； 
#define  bUSART1_CR3_RTSE    BIT_ADDR(USART1_BASE+0x14, 8)  //RTS使能（RTS Enable）位； 
#define  bUSART1_CR3_CTSE    BIT_ADDR(USART1_BASE+0x14, 9)  //CTS使能（CTS Enable）位；
#define  bUSART1_CR3_CTSIE   BIT_ADDR(USART1_BASE+0x14, 10) //CTS中断使能（CTS Interrupt Enable）位；
#define  bUSART1_CR3_ONEBIT  BIT_ADDR(USART1_BASE+0x14, 11)  

//USART状态寄存器GTPR
#define  bUSART1_GTPR_PSC     BIT_ADDR(USART1_BASE+0x18, 0) //预分频器值（Prescaler）位；在红外或智能卡模式，才需要这个功能。
#define  bUSART1_GTPR_GT      BIT_ADDR(USART1_BASE+0x18, 8) //保护时间值（Guard Time）位；该位域规定了以波特时钟为单位的保护时间。在智能卡模式下，需要这个功能。当保护时间过去后，才会设置发送完成标志。

/******************************************************************************/
/*                                                                            */
/*                        SPI接口寄存器                         */
/*                                                                            */
/******************************************************************************/
//#define SPI1_BASE             (APB2PERIPH_BASE + 0x3000)
//#define SPI2_BASE             (APB1PERIPH_BASE + 0x3800)
//#define SPI3_BASE             (APB1PERIPH_BASE + 0x3C00)

//其中，DR_DR、CRCPR_CRCPOLY、RXCRCR_RXCRC、TXCRCR_TXCRC为16位寄存器，不宜作位处理

//typedef struct
//{
//  __IO uint16_t CR1;  //0x00
//  uint16_t  RESERVED0;
//  __IO uint16_t CR2;  //0x04
//  uint16_t  RESERVED1;
//  __IO uint16_t SR;   //0x08
//  uint16_t  RESERVED2;
//  __IO uint16_t DR;  //0x0C
//  uint16_t  RESERVED3;
//  __IO uint16_t CRCPR; //0x10
//  uint16_t  RESERVED4;
//  __IO uint16_t RXCRCR; //0x14 
//  uint16_t  RESERVED5;
//  __IO uint16_t TXCRCR; //0x18
//  uint16_t  RESERVED6; 
//  __IO uint16_t I2SCFGR; //0x1C
//  uint16_t  RESERVED7; 
//  __IO uint16_t I2SPR; //0x20
//  uint16_t  RESERVED8;  
//} SPI_TypeDef;

/*******************  Bit definition for SPI_CR1 register  ********************/
#define  bSPI1_CR1_CPHA     BIT_ADDR(SPI1_BASE+0x00, 0)  /*!< Clock Phase */
#define  bSPI1_CR1_CPOL     BIT_ADDR(SPI1_BASE+0x00, 1)            /*!< Clock Polarity */
#define  bSPI1_CR1_MSTR     BIT_ADDR(SPI1_BASE+0x00, 2)            /*!< Master Selection */

// SPI总线速度设置
//SpeedSet:
#define SPI_SPEED_2   	0    //SPI_SPEED_2   2分频   (SPI 36M@sys 72M) 
#define SPI_SPEED_4   	1    //SPI_SPEED_2   4分频   (SPI 18M@sys 72M)
#define SPI_SPEED_8   	2    //SPI_SPEED_8   8分频   (SPI 9M@sys 72M)
#define SPI_SPEED_16  	3    //SPI_SPEED_16  16分频  (SPI 4.5M@sys 72M)
#define SPI_SPEED_32  	4    //SPI_SPEED_32  32分频  (SPI 2.25M@sys 72M)
#define SPI_SPEED_64  	5    //SPI_SPEED_64  64分频   (SPI 1.125M@sys 72M)
#define SPI_SPEED_128  	6    //SPI_SPEED_128 128分频 (SPI 562.5K@sys 72M)
#define SPI_SPEED_256 	7    //SPI_SPEED_256 256分频 (SPI 281.25K@sys 72M)

#define  SET_SPI1_CR1_BR(a)   (MEM_ADDR(SPI1_BASE)=(MEM_ADDR(SPI1_BASE)&(~SPI_CR1_BR))|((a&MASKb3)<<3))
//   #define  bSPI1_CR1_BR       BIT_ADDR(SPI1_BASE+0x00, 3)            /*!< BR[2:0] bits (Baud Rate Control) */

#define  bSPI1_CR1_SPE      BIT_ADDR(SPI1_BASE+0x00, 6)            /*!< SPI Enable */
#define  bSPI1_CR1_LSBFIRST BIT_ADDR(SPI1_BASE+0x00, 7)            /*!< Frame Format */
#define  bSPI1_CR1_SSI      BIT_ADDR(SPI1_BASE+0x00, 8)            /*!< Internal slave select */
#define  bSPI1_CR1_SSM      BIT_ADDR(SPI1_BASE+0x00, 9)            /*!< Software slave management */
#define  bSPI1_CR1_RXONLY   BIT_ADDR(SPI1_BASE+0x00, 10)            /*!< Receive only */
#define  bSPI1_CR1_DFF      BIT_ADDR(SPI1_BASE+0x00, 11)            /*!< Data Frame Format */
#define  bSPI1_CR1_CRCNEXT  BIT_ADDR(SPI1_BASE+0x00, 12)            /*!< Transmit CRC next */
#define  bSPI1_CR1_CRCEN    BIT_ADDR(SPI1_BASE+0x00, 13)            /*!< Hardware CRC calculation enable */
#define  bSPI1_CR1_BIDIOE   BIT_ADDR(SPI1_BASE+0x00, 14)            /*!< Output enable in bidirectional mode */
#define  bSPI1_CR1_BIDIMODE BIT_ADDR(SPI1_BASE+0x00, 15)            /*!< Bidirectional data mode enable */

/*******************  Bit definition for SPI_CR2 register  ********************/
#define  bSPI1_CR2_RXDMAEN  BIT_ADDR(SPI1_BASE+0x04, 0)               /*!< Rx Buffer DMA Enable */
#define  bSPI1_CR2_TXDMAEN  BIT_ADDR(SPI1_BASE+0x04, 1)               /*!< Tx Buffer DMA Enable */
#define  bSPI1_CR2_SSOE     BIT_ADDR(SPI1_BASE+0x04, 2)               /*!< SS Output Enable */
#define  bSPI1_CR2_ERRIE    BIT_ADDR(SPI1_BASE+0x04, 4)               /*!< Error Interrupt Enable */
#define  bSPI1_CR2_RXNEIE   BIT_ADDR(SPI1_BASE+0x04, 6)               /*!< RX buffer Not Empty Interrupt Enable */
#define  bSPI1_CR2_TXEIE    BIT_ADDR(SPI1_BASE+0x04, 7)               /*!< Tx buffer Empty Interrupt Enable */

/********************  Bit definition for SPI_SR register  ********************/
#define  bSPI1_SR_RXNE     BIT_ADDR(SPI1_BASE+0x08, 0)              /*!< Receive buffer Not Empty */
#define  bSPI1_SR_TXE      BIT_ADDR(SPI1_BASE+0x08, 1)               /*!< Transmit buffer Empty */
#define  bSPI1_SR_CHSIDE   BIT_ADDR(SPI1_BASE+0x08, 2)               /*!< Channel side */
#define  bSPI1_SR_UDR      BIT_ADDR(SPI1_BASE+0x08, 3)               /*!< Underrun flag */
#define  bSPI1_SR_CRCERR   BIT_ADDR(SPI1_BASE+0x08, 4)               /*!< CRC Error flag */
#define  bSPI1_SR_MODF     BIT_ADDR(SPI1_BASE+0x08, 5)               /*!< Mode fault */
#define  bSPI1_SR_OVR      BIT_ADDR(SPI1_BASE+0x08, 6)               /*!< Overrun flag */
#define  bSPI1_SR_BSY      BIT_ADDR(SPI1_BASE+0x08, 7)               /*!< Busy flag */

/******************  Bit definition for SPI_I2SCFGR register  *****************/
#define  bSPI1_I2SCFGR_CHLEN     BIT_ADDR(SPI1_BASE+0x1C, 0)            /*!< Channel length (number of bits per audio channel) */

#define  bSPI1_I2SCFGR_DATLEN    BIT_ADDR(SPI1_BASE+0x1C, 1)             /*!< DATLEN[1:0] bits (Data length to be transferred) */
#define  bSPI1_I2SCFGR_DATLEN_0  BIT_ADDR(SPI1_BASE+0x1C, 1)            /*!< Bit 0 */
#define  bSPI1_I2SCFGR_DATLEN_1  BIT_ADDR(SPI1_BASE+0x1C, 2)            /*!< Bit 1 */

#define  bSPI1_I2SCFGR_CKPOL     BIT_ADDR(SPI1_BASE+0x1C, 3)            /*!< steady state clock polarity */

#define  bSPI1_I2SCFGR_I2SSTD    BIT_ADDR(SPI1_BASE+0x1C, 4)            /*!< I2SSTD[1:0] bits (I2S standard selection) */
#define  bSPI1_I2SCFGR_I2SSTD_0  BIT_ADDR(SPI1_BASE+0x1C, 4)            /*!< Bit 0 */
#define  bSPI1_I2SCFGR_I2SSTD_1  BIT_ADDR(SPI1_BASE+0x1C, 5)            /*!< Bit 1 */

#define  bSPI1_I2SCFGR_PCMSYNC   BIT_ADDR(SPI1_BASE+0x1C, 7)            /*!< PCM frame synchronization */

#define  bSPI1_I2SCFGR_I2SCFG    BIT_ADDR(SPI1_BASE+0x1C, 8)            /*!< I2SCFG[1:0] bits (I2S configuration mode) */
#define  bSPI1_I2SCFGR_I2SCFG_0  BIT_ADDR(SPI1_BASE+0x1C, 8)            /*!< Bit 0 */
#define  bSPI1_I2SCFGR_I2SCFG_1  BIT_ADDR(SPI1_BASE+0x1C, 9)            /*!< Bit 1 */

#define  bSPI1_I2SCFGR_I2SE      BIT_ADDR(SPI1_BASE+0x1C, 10)            /*!< I2S Enable */
#define  bSPI1_I2SCFGR_I2SMOD    BIT_ADDR(SPI1_BASE+0x1C, 11)            /*!< I2S mode selection */

/******************  Bit definition for SPI_I2SPR register  *******************/
#define  SET_SPI1_I2SPR_I2SDIV(a)   (MEM_ADDR(SPI1_BASE+0x20)=(MEM_ADDR(SPI1_BASE+0x20)&(~SPI_I2SPR_I2SDIV))|((a&MASKb8)<<0))
// #define  bSPI1_I2SPR_I2SDIV      BIT_ADDR(SPI1_BASE+0x20, 0)  //长度8BIT I2S 线性预分频

#define  bSPI1_I2SPR_ODD         BIT_ADDR(SPI1_BASE+0x20, 8)            /*!< Odd factor for the prescaler */
#define  bSPI1_I2SPR_MCKOE       BIT_ADDR(SPI1_BASE+0x20, 9)            /*!< Master Clock Output Enable */


/*******************  Bit definition for SPI_CR1 register  ********************/
#define  bSPI2_CR1_CPHA     BIT_ADDR(SPI2_BASE+0x00, 0)  /*!< Clock Phase */
#define  bSPI2_CR1_CPOL     BIT_ADDR(SPI2_BASE+0x00, 1)            /*!< Clock Polarity */
#define  bSPI2_CR1_MSTR     BIT_ADDR(SPI2_BASE+0x00, 2)            /*!< Master Selection */

#define  SET_SPI2_CR1_BR(a)   (MEM_ADDR(SPI2_BASE)=(MEM_ADDR(SPI2_BASE)&(~SPI_CR1_BR))|((a&MASKb3)<<3))
//  #define  bSPI2_CR1_BR       BIT_ADDR(SPI2_BASE+0x00, 3)            /*!< BR[2:0] bits (Baud Rate Control) */

#define  bSPI2_CR1_SPE      BIT_ADDR(SPI2_BASE+0x00, 6)            /*!< SPI Enable */
#define  bSPI2_CR1_LSBFIRST BIT_ADDR(SPI2_BASE+0x00, 7)            /*!< Frame Format */
#define  bSPI2_CR1_SSI      BIT_ADDR(SPI2_BASE+0x00, 8)            /*!< Internal slave select */
#define  bSPI2_CR1_SSM      BIT_ADDR(SPI2_BASE+0x00, 9)            /*!< Software slave management */
#define  bSPI2_CR1_RXONLY   BIT_ADDR(SPI2_BASE+0x00, 10)            /*!< Receive only */
#define  bSPI2_CR1_DFF      BIT_ADDR(SPI2_BASE+0x00, 11)            /*!< Data Frame Format */
#define  bSPI2_CR1_CRCNEXT  BIT_ADDR(SPI2_BASE+0x00, 12)            /*!< Transmit CRC next */
#define  bSPI2_CR1_CRCEN    BIT_ADDR(SPI2_BASE+0x00, 13)            /*!< Hardware CRC calculation enable */
#define  bSPI2_CR1_BIDIOE   BIT_ADDR(SPI2_BASE+0x00, 14)            /*!< Output enable in bidirectional mode */
#define  bSPI2_CR1_BIDIMODE BIT_ADDR(SPI2_BASE+0x00, 15)            /*!< Bidirectional data mode enable */

/*******************  Bit definition for SPI_CR2 register  ********************/
#define  bSPI2_CR2_RXDMAEN  BIT_ADDR(SPI2_BASE+0x04, 0)               /*!< Rx Buffer DMA Enable */
#define  bSPI2_CR2_TXDMAEN  BIT_ADDR(SPI2_BASE+0x04, 1)               /*!< Tx Buffer DMA Enable */
#define  bSPI2_CR2_SSOE     BIT_ADDR(SPI2_BASE+0x04, 2)               /*!< SS Output Enable */
#define  bSPI2_CR2_ERRIE    BIT_ADDR(SPI2_BASE+0x04, 4)               /*!< Error Interrupt Enable */
#define  bSPI2_CR2_RXNEIE   BIT_ADDR(SPI2_BASE+0x04, 6)               /*!< RX buffer Not Empty Interrupt Enable */
#define  bSPI2_CR2_TXEIE    BIT_ADDR(SPI2_BASE+0x04, 7)               /*!< Tx buffer Empty Interrupt Enable */

/********************  Bit definition for SPI_SR register  ********************/
#define  bSPI2_SR_RXNE     BIT_ADDR(SPI2_BASE+0x08, 0)              /*!< Receive buffer Not Empty */
#define  bSPI2_SR_TXE      BIT_ADDR(SPI2_BASE+0x08, 1)               /*!< Transmit buffer Empty */
#define  bSPI2_SR_CHSIDE   BIT_ADDR(SPI2_BASE+0x08, 2)               /*!< Channel side */
#define  bSPI2_SR_UDR      BIT_ADDR(SPI2_BASE+0x08, 3)               /*!< Underrun flag */
#define  bSPI2_SR_CRCERR   BIT_ADDR(SPI2_BASE+0x08, 4)               /*!< CRC Error flag */
#define  bSPI2_SR_MODF     BIT_ADDR(SPI2_BASE+0x08, 5)               /*!< Mode fault */
#define  bSPI2_SR_OVR      BIT_ADDR(SPI2_BASE+0x08, 6)               /*!< Overrun flag */
#define  bSPI2_SR_BSY      BIT_ADDR(SPI2_BASE+0x08, 7)               /*!< Busy flag */

/******************  Bit definition for SPI_I2SCFGR register  *****************/
#define  bSPI2_I2SCFGR_CHLEN     BIT_ADDR(SPI2_BASE+0x1C, 0)            /*!< Channel length (number of bits per audio channel) */


/******************************************************************************/
/*                                                                            */
/*                        TIM定时器接口寄存器                                  */
/*                                                                            */
/******************************************************************************/
//typedef struct
//{
//  __IO uint16_t CR1;    //0x00
//  uint16_t  RESERVED0;
//  __IO uint16_t CR2;		//0x04
//  uint16_t  RESERVED1;
//  __IO uint16_t SMCR;		//0x08
//  uint16_t  RESERVED2;
//  __IO uint16_t DIER;		//0x0C
//  uint16_t  RESERVED3;
//  __IO uint16_t SR;			//0x10
//  uint16_t  RESERVED4;
//  __IO uint16_t EGR;		//0x14
//  uint16_t  RESERVED5;
//  __IO uint16_t CCMR1;	//0x18
//  uint16_t  RESERVED6;
//  __IO uint16_t CCMR2;	//0x1C
//  uint16_t  RESERVED7;
//  __IO uint16_t CCER;		//0x20
//  uint16_t  RESERVED8;
//  __IO uint16_t CNT;		//0x24
//  uint16_t  RESERVED9;
//  __IO uint16_t PSC;		//0x28
//  uint16_t  RESERVED10;
//  __IO uint16_t ARR;		//0x2C
//  uint16_t  RESERVED11;
//  __IO uint16_t RCR;		//0x30
//  uint16_t  RESERVED12;
//  __IO uint16_t CCR1;		//0x34
//  uint16_t  RESERVED13;
//  __IO uint16_t CCR2;		//0x38
//  uint16_t  RESERVED14;
//  __IO uint16_t CCR3;		//0x3C
//  uint16_t  RESERVED15;
//  __IO uint16_t CCR4;		//0x40
//  uint16_t  RESERVED16;
//  __IO uint16_t BDTR;		//0x44
//  uint16_t  RESERVED17;
//  __IO uint16_t DCR;		//0x48
//  uint16_t  RESERVED18;
//  __IO uint16_t DMAR;		//0x4C
//  uint16_t  RESERVED19;
//}TIM_TypeDef;

#define bTimer1             (APB2PERIPH_BASE + 0x2C00)
#define bTimer2             (APB1PERIPH_BASE + 0x0000)
#define bTimer3             (APB1PERIPH_BASE + 10)
#define bTimer4             (APB1PERIPH_BASE + 11)
#define bTimer5             (APB1PERIPH_BASE + 0x0C00)
#define bTimer6             (APB1PERIPH_BASE + 12)
#define bTimer7             (APB1PERIPH_BASE + 0x1400)
#define bTimer8             (APB2PERIPH_BASE + 0x3400)


//这是一种新的尝试方式， 减少宏定义的工作量，使用形如bCR1_CEN(Timer1) 来调用，Timer1为上述基址宏
// #define  bCR1_CEN(TIMx_BASE)                   BIT_ADDR(TIMx_BASE+0x00,0)            /*!< Counter enable */
/*******************  TIM_CR1 控制寄存器  ********************/
#define   CR1_CEN(TIMx_BASE)                         BIT_ADDR(TIMx_BASE +0x00,0)            /*!< Counter enable */
#define   CR1_UDIS(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x00,1)            /*!< Update disable */
#define   CR1_URS(TIMx_BASE)                         BIT_ADDR(TIMx_BASE +0x00,2)            /*!< Update request source */
#define   CR1_OPM(TIMx_BASE)                         BIT_ADDR(TIMx_BASE +0x00,3)            /*!< One pulse mode */
#define   CR1_DIR(TIMx_BASE)                         BIT_ADDR(TIMx_BASE +0x00,4)            /*!< 方向，定义：0（计数器向上计数），1（计数器向下计数），注：当计数器配置为中央对齐模式或编码器模式时，该位为只读 */

#define 	SET_CR1_CMS(TIMx_BASE,a)   (MEM_ADDR(TIMx_BASE+0x00)=(MEM_ADDR(TIMx_BASE+0x00)&(~TIM_CR1_CMS))|((a&0x03)<<5))
//  #define   CR1_CMS(TIMx_BASE)                         BIT_ADDR(TIMx_BASE +0x00,5)        /*!< CMS[1:0] 中央对齐模式选择，定义：00：边沿对齐模式。计数器依据方向位(DIR)向上或向下计数。 */
#define   CR1_CMS_0(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x00,5)            /*!< 01、10、11（中央对齐模式1、2、3。计数器交替地向上和向下计数。详略*/
#define   CR1_CMS_1(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x00,6)            /*!<  */

#define   CR1_ARPE(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x00,7)            /*!< Auto-reload preload enable */

#define 	SET_CR1_CKD(TIMx_BASE,a)   (MEM_ADDR(TIMx_BASE+0x00)=(MEM_ADDR(TIMx_BASE+0x00)&(~TIM_CR1_CKD))|((a&0x03)<<8))
//  #define   CR1_CKD(TIMx_BASE)                         BIT_ADDR(TIMx_BASE +0x00,8)            /*!< CKD[1:0] CKD[1:0]时钟分频因子，定义在定时器时钟(CK_INT)频率与数字滤波器(ETR，TIx)使用的采样频率之间的分频比例。 */
#define   CR1_CKD_0(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x00,8)            /*!< 定义：00（tDTS = tCK_INT），01（tDTS = 2 x tCK_INT），10（tDTS = 4 x tCK_INT）11：保留 */
#define   CR1_CKD_1(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x00,9)            /*!< Bit 1 */

/*******************  TIM_CR2 控制寄存器  ********************/
#define   CR2_CCPC(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x04,0)            /*!< Capture/Compare Preloaded Control */
#define   CR2_CCUS(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x04,2)            /*!< Capture/Compare Control Update Selection */
#define   CR2_CCDS(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x04,3)            /*!< Capture/Compare DMA Selection */

#define 	SET_CR2_MMS(TIMx_BASE,a)   (MEM_ADDR(TIMx_BASE+0x04)=(MEM_ADDR(TIMx_BASE+0x04)&(~TIM_CR2_MMS))|((a&MASKb3)<<4))
//  #define   CR2_MMS(TIMx_BASE)                         BIT_ADDR(TIMx_BASE +0x04,4)            /*!< MMS[2:0] bits (Master Mode Selection) */

#define   CR2_TI1S(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x04,7)            /*!< TI1 Selection */
#define   CR2_OIS1(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x04,8)            /*!< Output Idle state 1 (OC1 output) */
#define   CR2_OIS1N(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x04,9)            /*!< Output Idle state 1 (OC1N output) */
#define   CR2_OIS2(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x04,10)            /*!< Output Idle state 2 (OC2 output) */
#define   CR2_OIS2N(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x04,11)            /*!< Output Idle state 2 (OC2N output) */
#define   CR2_OIS3(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x04,12)            /*!< Output Idle state 3 (OC3 output) */
#define   CR2_OIS3N(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x04,13)            /*!< Output Idle state 3 (OC3N output) */
#define   CR2_OIS4(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x04,14)            /*!< Output Idle state 4 (OC4 output) */

/*******************  Bit definition for TIM_SMCR register  *******************/
#define 	SET_SMCR_SMS(TIMx_BASE,a)   (MEM_ADDR(TIMx_BASE+0x08)=(MEM_ADDR(TIMx_BASE+0x08)&(~TIM_SMCR_SMS))|((a&MASKb3)<<0))
//  #define   SMCR_SMS(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x08,0)            /*!< SMS[2:0] bits (Slave mode selection) */

#define 	SET_SMCR_TS(TIMx_BASE,a)   (MEM_ADDR(TIMx_BASE+0x08)=(MEM_ADDR(TIMx_BASE+0x08)&(~TIM_SMCR_TS))|((a&MASKb3)<<4))
//  #define   SMCR_TS(TIMx_BASE)                         BIT_ADDR(TIMx_BASE +0x08,4)            /*!< TS[2:0] bits (Trigger selection) */

#define   SMCR_MSM(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x08,7)            /*!< Master/slave mode */

#define 	SET_SMCR_ETF(TIMx_BASE,a)   (MEM_ADDR(TIMx_BASE+0x08)=(MEM_ADDR(TIMx_BASE+0x08)&(~TIM_SMCR_ETF))|((a&MASKb4)<<8))
//  #define   SMCR_ETF(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x08,8)            /*!< ETF[3:0] bits (External trigger filter) */

#define 	SET_SMCR_ETPS(TIMx_BASE,a)   (MEM_ADDR(TIMx_BASE+0x08)=(MEM_ADDR(TIMx_BASE+0x08)&(~TIM_SMCR_ETPS))|((a&MASKb2)<<12))
//  #define   SMCR_ETPS(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x08,12)            /*!< ETPS[1:0] bits (External trigger prescaler) */

#define   SMCR_ECE(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x08,14)            /*!< External clock enable */
#define   SMCR_ETP(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x08,15)            /*!< External trigger polarity */

/*******************  Bit definition for TIM_DIER register  *******************/
#define   DIER_UIE(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x0C,0)            /*!< Update interrupt enable */
#define   DIER_CC1IE(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x0C,1)            /*!< Capture/Compare 1 interrupt enable */
#define   DIER_CC2IE(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x0C,2)            /*!< Capture/Compare 2 interrupt enable */
#define   DIER_CC3IE(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x0C,3)            /*!< Capture/Compare 3 interrupt enable */
#define   DIER_CC4IE(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x0C,4)            /*!< Capture/Compare 4 interrupt enable */
#define   DIER_COMIE(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x0C,5)            /*!< COM interrupt enable */
#define   DIER_TIE(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x0C,6)            /*!< Trigger interrupt enable */
#define   DIER_BIE(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x0C,7)            /*!< Break interrupt enable */
#define   DIER_UDE(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x0C,8)            /*!< Update DMA request enable */
#define   DIER_CC1DE(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x0C,9)            /*!< Capture/Compare 1 DMA request enable */
#define   DIER_CC2DE(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x0C,10)            /*!< Capture/Compare 2 DMA request enable */
#define   DIER_CC3DE(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x0C,11)            /*!< Capture/Compare 3 DMA request enable */
#define   DIER_CC4DE(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x0C,12)            /*!< Capture/Compare 4 DMA request enable */
#define   DIER_COMDE(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x0C,13)            /*!< COM DMA request enable */
#define   DIER_TDE(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x0C,14)            /*!< Trigger DMA request enable */

/********************  TIM_SR 状态寄存器  ********************/
#define   SR_UIF(TIMx_BASE)                          BIT_ADDR(TIMx_BASE +0x10,0)            /*!< 更新中断标记（硬件置1，软件清0）定义：0（无更新事件）1（更新中断等待响应。当寄存器被更新时该位由硬件置’1’） */
#define   SR_CC1IF(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x10,1)            /*!< Capture/Compare 1 interrupt Flag */
#define   SR_CC2IF(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x10,2)            /*!< Capture/Compare 2 interrupt Flag */
#define   SR_CC3IF(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x10,3)            /*!< Capture/Compare 3 interrupt Flag */
#define   SR_CC4IF(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x10,4)            /*!< Capture/Compare 4 interrupt Flag */
#define   SR_COMIF(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x10,5)            /*!< COM interrupt Flag */
#define   SR_TIF(TIMx_BASE)                          BIT_ADDR(TIMx_BASE +0x10,6)            /*!< Trigger interrupt Flag */
#define   SR_BIF(TIMx_BASE)                          BIT_ADDR(TIMx_BASE +0x10,7)            /*!< Break interrupt Flag */
#define   SR_CC1OF(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x10,9)            /*!< Capture/Compare 1 Overcapture Flag */
#define   SR_CC2OF(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x10,10)            /*!< Capture/Compare 2 Overcapture Flag */
#define   SR_CC3OF(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x10,11)            /*!< Capture/Compare 3 Overcapture Flag */
#define   SR_CC4OF(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x10,12)            /*!< Capture/Compare 4 Overcapture Flag */

/*******************  Bit definition for TIM_EGR register  ********************/
#define   EGR_UG(TIMx_BASE)                          BIT_ADDR(TIMx_BASE +0x14,0)               /*!< Update Generation */
#define   EGR_CC1G(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x14,1)               /*!< Capture/Compare 1 Generation */
#define   EGR_CC2G(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x14,2)               /*!< Capture/Compare 2 Generation */
#define   EGR_CC3G(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x14,3)               /*!< Capture/Compare 3 Generation */
#define   EGR_CC4G(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x14,4)               /*!< Capture/Compare 4 Generation */
#define   EGR_COMG(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x14,5)               /*!< Capture/Compare Control Update Generation */
#define   EGR_TG(TIMx_BASE)                          BIT_ADDR(TIMx_BASE +0x14,6)               /*!< Trigger Generation */
#define   EGR_BG(TIMx_BASE)                          BIT_ADDR(TIMx_BASE +0x14,7)               /*!< Break Generation */

/******************  Bit definition for TIM_CCMR1 register  *******************/
#define   CCMR1_CC1S(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x18,0)            /*!< CC1S[1:0] bits (Capture/Compare 1 Selection) */
#define   CCMR1_CC1S_0(TIMx_BASE)                    BIT_ADDR(TIMx_BASE +0x18,0)            /*!< Bit 0 */
#define   CCMR1_CC1S_1(TIMx_BASE)                    BIT_ADDR(TIMx_BASE +0x18,1)            /*!< Bit 1 */

#define   CCMR1_OC1FE(TIMx_BASE)                     BIT_ADDR(TIMx_BASE +0x18,2)            /*!< Output Compare 1 Fast enable */
#define   CCMR1_OC1PE(TIMx_BASE)                     BIT_ADDR(TIMx_BASE +0x18,3)            /*!< Output Compare 1 Preload enable */

#define 	SET_CCMR1_OC1M(TIMx_BASE,a)   (MEM_ADDR(TIMx_BASE+0x18)=(MEM_ADDR(TIMx_BASE+0x18)&(~TIM_CCMR1_OC1M))|((a&0x07)<<4))
//  #define   CCMR1_OC1M(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x18,4)            /*!< OC1M[2:0] bits (Output Compare 1 Mode) */

#define   CCMR1_OC1CE(TIMx_BASE)                     BIT_ADDR(TIMx_BASE +0x18,7)            /*!< Output Compare 1Clear Enable */

#define   CCMR1_CC2S(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x18,8)            /*!< CC2S[1:0] bits (Capture/Compare 2 Selection) */
#define   CCMR1_CC2S_0(TIMx_BASE)                    BIT_ADDR(TIMx_BASE +0x18,8)            /*!< Bit 0 */
#define   CCMR1_CC2S_1(TIMx_BASE)                    BIT_ADDR(TIMx_BASE +0x18,9)            /*!< Bit 1 */

#define   CCMR1_OC2FE(TIMx_BASE)                     BIT_ADDR(TIMx_BASE +0x18,10)            /*!< Output Compare 2 Fast enable */
#define   CCMR1_OC2PE(TIMx_BASE)                     BIT_ADDR(TIMx_BASE +0x18,11)            /*!< Output Compare 2 Preload enable */

#define 	SET_CCMR1_OC2M(TIMx_BASE,a)   (MEM_ADDR(TIMx_BASE+0x18)=(MEM_ADDR(TIMx_BASE+0x18)&(~TIM_CCMR1_OC2M))|((a&0x07)<<12))
//  #define   CCMR1_OC2M(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x18,12)            /*!< OC2M[2:0] bits (Output Compare 2 Mode) */

#define   CCMR1_OC2CE(TIMx_BASE)                     BIT_ADDR(TIMx_BASE +0x18,15)            /*!< Output Compare 2 Clear Enable */

/*----------------------------------------------------------------------------*/

#define   CCMR1_IC1PSC(TIMx_BASE)                    BIT_ADDR(TIMx_BASE +0x18,2)            /*!< IC1PSC[1:0] bits (Input Capture 1 Prescaler) */
#define   CCMR1_IC1PSC_0(TIMx_BASE)                  BIT_ADDR(TIMx_BASE +0x18,2)            /*!< Bit 0 */
#define   CCMR1_IC1PSC_1(TIMx_BASE)                  BIT_ADDR(TIMx_BASE +0x18,3)            /*!< Bit 1 */

#define 	SET_CCMR1_IC1F(TIMx_BASE,a)   (MEM_ADDR(TIMx_BASE+0x18)=(MEM_ADDR(TIMx_BASE+0x18)&(~TIM_CCMR1_IC1F))|((a&0x0F)<<4))
//  #define   CCMR1_IC1F(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x18,4)            /*!< IC1F[3:0] bits (Input Capture 1 Filter) */

#define   CCMR1_IC2PSC(TIMx_BASE)                    BIT_ADDR(TIMx_BASE +0x18,10)            /*!< IC2PSC[1:0] bits (Input Capture 2 Prescaler) */
#define   CCMR1_IC2PSC_0(TIMx_BASE)                  BIT_ADDR(TIMx_BASE +0x18,10)            /*!< Bit 0 */
#define   CCMR1_IC2PSC_1(TIMx_BASE)                  BIT_ADDR(TIMx_BASE +0x18,11)            /*!< Bit 1 */

#define 	SET_CCMR1_IC2F(TIMx_BASE,a)   (MEM_ADDR(TIMx_BASE+0x18)=(MEM_ADDR(TIMx_BASE+0x18)&(~TIM_CCMR1_IC2F))|((a&0x0F)<<12))
//  #define   CCMR1_IC2F(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x18,12)            /*!< IC2F[3:0] bits (Input Capture 2 Filter) */

/******************  Bit definition for TIM_CCMR2 register  *******************/
#define   CCMR2_CC3S(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x1C,0)            /*!< CC3S[1:0] bits (Capture/Compare 3 Selection) */
#define   CCMR2_CC3S_0(TIMx_BASE)                    BIT_ADDR(TIMx_BASE +0x1C,0)            /*!< Bit 0 */
#define   CCMR2_CC3S_1(TIMx_BASE)                    BIT_ADDR(TIMx_BASE +0x1C,1)            /*!< Bit 1 */

#define   CCMR2_OC3FE(TIMx_BASE)                     BIT_ADDR(TIMx_BASE +0x1C,2)            /*!< Output Compare 3 Fast enable */
#define   CCMR2_OC3PE(TIMx_BASE)                     BIT_ADDR(TIMx_BASE +0x1C,3)            /*!< Output Compare 3 Preload enable */

#define 	SET_CCMR2_OC3M(TIMx_BASE,a)   (MEM_ADDR(TIMx_BASE+0x1C)=(MEM_ADDR(TIMx_BASE+0x1C)&(~TIM_CCMR2_OC3M))|((a&0x07)<<4))
//  #define   CCMR2_OC3M(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x1C,4)            /*!< OC3M[2:0] bits (Output Compare 3 Mode) */


#define   CCMR2_OC3CE(TIMx_BASE)                     BIT_ADDR(TIMx_BASE +0x1C,7)            /*!< Output Compare 3 Clear Enable */

#define   CCMR2_CC4S(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x1C,8)            /*!< CC4S[1:0] bits (Capture/Compare 4 Selection) */
#define   CCMR2_CC4S_0(TIMx_BASE)                    BIT_ADDR(TIMx_BASE +0x1C,8)            /*!< Bit 0 */
#define   CCMR2_CC4S_1(TIMx_BASE)                    BIT_ADDR(TIMx_BASE +0x1C,9)            /*!< Bit 1 */

#define   CCMR2_OC4FE(TIMx_BASE)                     BIT_ADDR(TIMx_BASE +0x1C,10)            /*!< Output Compare 4 Fast enable */
#define   CCMR2_OC4PE(TIMx_BASE)                     BIT_ADDR(TIMx_BASE +0x1C,11)            /*!< Output Compare 4 Preload enable */

#define 	SET_CCMR2_OC4M(TIMx_BASE,a)   (MEM_ADDR(TIMx_BASE+0x1C)=(MEM_ADDR(TIMx_BASE+0x1C)&(~TIM_CCMR2_OC4M))|((a&0x07)<<12))
//  #define   CCMR2_OC4M(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x1C,12)            /*!< OC4M[2:0] bits (Output Compare 4 Mode) */


#define   CCMR2_OC4CE(TIMx_BASE)                     BIT_ADDR(TIMx_BASE +0x1C,15)            /*!< Output Compare 4 Clear Enable */

/*----------------------------------------------------------------------------*/

#define   CCMR2_IC3PSC(TIMx_BASE)                    BIT_ADDR(TIMx_BASE +0x1C,2)            /*!< IC3PSC[1:0] bits (Input Capture 3 Prescaler) */
#define   CCMR2_IC3PSC_0(TIMx_BASE)                  BIT_ADDR(TIMx_BASE +0x1C,2)            /*!< Bit 0 */
#define   CCMR2_IC3PSC_1(TIMx_BASE)                  BIT_ADDR(TIMx_BASE +0x1C,3)            /*!< Bit 1 */

#define 	SET_CCMR2_IC3F(TIMx_BASE,a)   (MEM_ADDR(TIMx_BASE+0x1C)=(MEM_ADDR(TIMx_BASE+0x1C)&(~TIM_CCMR2_IC3F))|((a&0x0f)<<4))
//  #define   CCMR2_IC3F(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x1C,4)            /*!< IC3F[3:0] bits (Input Capture 3 Filter) */

#define   CCMR2_IC4PSC(TIMx_BASE)                    BIT_ADDR(TIMx_BASE +0x1C,10)            /*!< IC4PSC[1:0] bits (Input Capture 4 Prescaler) */
#define   CCMR2_IC4PSC_0(TIMx_BASE)                  BIT_ADDR(TIMx_BASE +0x1C,10)            /*!< Bit 0 */
#define   CCMR2_IC4PSC_1(TIMx_BASE)                  BIT_ADDR(TIMx_BASE +0x1C,11)            /*!< Bit 1 */

#define 	SET_CCMR2_IC4F(TIMx_BASE,a)   (MEM_ADDR(TIMx_BASE+0x1C)=(MEM_ADDR(TIMx_BASE+0x1C)&(~TIM_CCMR2_IC4F))|((a&0x0f)<<12))
//#define   CCMR2_IC4F(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x1C,12)            /*!< IC4F[3:0] bits (Input Capture 4 Filter) */


/*******************  Bit definition for TIM_CCER register  *******************/
#define   CCER_CC1E(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x20,0)            /*!< Capture/Compare 1 output enable */
#define   CCER_CC1P(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x20,1)            /*!< Capture/Compare 1 output Polarity */
#define   CCER_CC1NE(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x20,2)            /*!< Capture/Compare 1 Complementary output enable */
#define   CCER_CC1NP(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x20,3)            /*!< Capture/Compare 1 Complementary output Polarity */
#define   CCER_CC2E(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x20,4)            /*!< Capture/Compare 2 output enable */
#define   CCER_CC2P(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x20,5)            /*!< Capture/Compare 2 output Polarity */
#define   CCER_CC2NE(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x20,6)            /*!< Capture/Compare 2 Complementary output enable */
#define   CCER_CC2NP(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x20,7)            /*!< Capture/Compare 2 Complementary output Polarity */
#define   CCER_CC3E(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x20,8)            /*!< Capture/Compare 3 output enable */
#define   CCER_CC3P(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x20,9)            /*!< Capture/Compare 3 output Polarity */
#define   CCER_CC3NE(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x20,10)            /*!< Capture/Compare 3 Complementary output enable */
#define   CCER_CC3NP(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x20,11)            /*!< Capture/Compare 3 Complementary output Polarity */
#define   CCER_CC4E(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x20,12)            /*!< Capture/Compare 4 output enable */
#define   CCER_CC4P(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x20,13)            /*!< Capture/Compare 4 output Polarity */
#define   CCER_CC4NP(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x20,15)            /*!< Capture/Compare 4 Complementary output Polarity */

/*******************  Bit definition for TIM_CNT register  ********************/
#define   CNT_CNT(TIMx_BASE)           (MEM_ADDR(TIMx_BASE+0x24))    /*!< Counter Value */             

/*******************  Bit definition for TIM_PSC register  ********************/
#define   PSC_PSC(TIMx_BASE)           (MEM_ADDR(TIMx_BASE+0x28))            /*!< Prescaler Value */

/*******************  Bit definition for TIM_ARR register  ********************/
#define   ARR_ARR(TIMx_BASE)           (MEM_ADDR(TIMx_BASE+0x2C))            /*!< actual auto-reload Value */

/*******************  Bit definition for TIM_RCR register  ********************/
#define   RCR_REP(TIMx_BASE)           (MEM_ADDR(TIMx_BASE+0x30))               /*!< Repetition Counter Value */

/*******************  Bit definition for TIM_CCR1 register  *******************/
#define   CCR1_CCR1(TIMx_BASE)         (MEM_ADDR(TIMx_BASE+0x34))           /*!< Capture/Compare 1 Value */

/*******************  Bit definition for TIM_CCR2 register  *******************/
#define   CCR2_CCR2(TIMx_BASE)         (MEM_ADDR(TIMx_BASE+0x38))            /*!< Capture/Compare 2 Value */

/*******************  Bit definition for TIM_CCR3 register  *******************/
#define   CCR3_CCR3(TIMx_BASE)         (MEM_ADDR(TIMx_BASE+0x3C))            /*!< Capture/Compare 3 Value */

/*******************  Bit definition for TIM_CCR4 register  *******************/
#define   CCR4_CCR4(TIMx_BASE)         (MEM_ADDR(TIMx_BASE+0x40))            /*!< Capture/Compare 4 Value */

/*******************  Bit definition for TIM_BDTR register  *******************/
/*!< DTG[0:7] bits (Dead-Time Generator set-up) */
#define 	SET_BDTR_DTG(TIMx_BASE,a)   (MEM_ADDR(TIMx_BASE+0x44)=(MEM_ADDR(TIMx_BASE+0x44)&(~TIM_BDTR_DTG))|((a&MASKb8)<<0))

#define   BDTR_LOCK(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x44,8)            /*!< LOCK[1:0] bits (Lock Configuration) */
#define   BDTR_LOCK_0(TIMx_BASE)                     BIT_ADDR(TIMx_BASE +0x44,8)            /*!< Bit 0 */
#define   BDTR_LOCK_1(TIMx_BASE)                     BIT_ADDR(TIMx_BASE +0x44,9)            /*!< Bit 1 */

#define   BDTR_OSSI(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x44,10)            /*!< Off-State Selection for Idle mode */
#define   BDTR_OSSR(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x44,11)            /*!< Off-State Selection for Run mode */
#define   BDTR_BKE(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x44,12)            /*!< Break enable */
#define   BDTR_BKP(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x44,13)            /*!< Break Polarity */
#define   BDTR_AOE(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x44,14)            /*!< Automatic Output enable */
#define   BDTR_MOE(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x44,15)            /*!< Main Output enable */

/*******************  Bit definition for TIM_DCR register  ********************/
/*!< DBA从第0BIT开始共5位： (DMA Base Address) */
#define 	SET_DCR_DBA(TIMx_BASE,a)   (MEM_ADDR(TIMx_BASE+0x48)=(MEM_ADDR(TIMx_BASE+0x48)&(~TIM_DCR_DBA))|((a&MASKb5)<<0))
/*!< DBL从第8BIT开始共5位： (DMA Burst Length) */
#define 	SET_DCR_DBL(TIMx_BASE,a)   (MEM_ADDR(TIMx_BASE+0x48)=(MEM_ADDR(TIMx_BASE+0x48)&(~TIM_DCR_DBL))|((a&MASKb5)<<8))


/*******************  Bit definition for TIM_DMAR register  *******************/
#define   DMAR_DMAB(TIMx_BASE)       (MEM_ADDR(TIMx_BASE+0x4C))            /*!< DMA register for burst accesses */

/*************************************************************************************************************************/
#endif

