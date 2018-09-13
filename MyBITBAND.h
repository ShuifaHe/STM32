#ifndef __MYBITBAND_H
#define __MYBITBAND_H	  
#include <stm32f10x.h>  
/****************************** 本人逐步实现的寄存器位操作模式 ***************************************/
// 以外设原有名称为基础，在外设名称前面加前缀b, 与库函数区分的同时，指示这用作bitband操作。
// 进一步强化助记，将部分具体寄存器名称弱化,具体寄存器名称在位定义前有注释行。
// BIT位的名称尽量采用原名称，以便对照.  寄存器地址偏移量按一般用户手册以十六进制填充。
// 考虑到位操作的优势在精准高效地对位进行操作，故对两位以上的组合予以省略。
// CopyRight By Warship
// Modified date: 20180912

/******************** 位段操作统一格式 *******************************************
单BIT位访问方式：
		bRCC_ENABLE_RTC=1或0; 						//唯一外设
		bTIM_CR_DIR（bTIMx)=1或0;						//多组外设
		bGPIO_BRR(bGPIOx,n)=1或0;							//多组外设多个端口

多BIT位访问方式：
		SET_RCC_PLLMUL(a);	写	 					//唯一外设
		GET_RCC_PLLMUL;			读

		SET_SPI_CR_BR(bSPIx,a);	写						//多组外设
		GET_SPI_CR_BR(bSPIx);		读		

*************************************************************************/

//为减少宏定义的工作量，对于多个相同的外设，尤其是两个以上，如定时器、串行通信等，避免大量类似的宏定义
//使用带参数的宏，形如bCR_CEN(bTIM1),其中TIM1为基址宏
//以下定义的统一以b开头的基址宏，直接引用了stm32f10x.h中定义的xxx_BASE,
//也可以不用这些定义，直接使用xxx_BASE
#define bURT1             USART1_BASE  
#define bURT2             USART2_BASE
#define bURT3             USART3_BASE
#define bURT4             UART4_BASE
#define bURT5             UART5_BASE

#define bTIM1             TIM1_BASE
#define bTIM2             TIM2_BASE
#define bTIM3             TIM3_BASE
#define bTIM4             TIM4_BASE
#define bTIM5             TIM5_BASE
#define bTIM6             TIM6_BASE
#define bTIM7             TIM7_BASE
#define bTIM8             TIM8_BASE
#define bTIM9            	TIM9_BASE
#define bTIM10            TIM10_BASE
#define bTIM11            TIM11_BASE
#define bTIM12            TIM12_BASE
#define bTIM13            TIM13_BASE
#define bTIM14            TIM14_BASE
#define bTIM15            TIM15_BASE
#define bTIM16            TIM16_BASE
#define bTIM17            TIM17_BASE

#define bSPI1							SPI1_BASE
#define bSPI2							SPI2_BASE
#define bSPI3							SPI3_BASE

#define bI2C1							I2C1_BASE
#define bI2C2							I2C2_BASE

#define bCAN1							CAN1_BASE
#define bCAN2							CAN2_BASE

#define bDMA1							DMA1_BASE
#define bDMA2							DMA2_BASE

#define bADC1							ADC1_BASE
#define bADC2							ADC2_BASE
#define bADC3							ADC3_BASE


//多BIT位掩码宏定义
#define 	MASKb1			((uint32_t)0x0001)
#define 	MASKb2			((uint32_t)0x0003)
#define 	MASKb3			((uint32_t)0x0007)
#define 	MASKb4			((uint32_t)0x000F)
#define 	MASKb5			((uint32_t)0x001F)
#define 	MASKb6			((uint32_t)0x003F)
#define 	MASKb7			((uint32_t)0x007F)
#define 	MASKb8			((uint32_t)0x00FF)
#define 	MASKb9			((uint32_t)0x01FF)
#define 	MASKb10			((uint32_t)0x03FF)
#define 	MASKb11			((uint32_t)0x07FF)
#define 	MASKb12			((uint32_t)0x0FFF)
#define 	MASKb13			((uint32_t)0x1FFF)
#define 	MASKb14			((uint32_t)0x3FFF)
#define 	MASKb15			((uint32_t)0x7FFF)
#define 	MASKb16			((uint32_t)0xFFFF)
#define 	MASKb17			((uint32_t)0x1FFFF)
#define 	MASKb18			((uint32_t)0x3FFFF)
#define 	MASKb19			((uint32_t)0x7FFFF)
#define 	MASKb20			((uint32_t)0x0FFFFF)
#define 	MASKb21			((uint32_t)0x1FFFFF)
#define 	MASKb22			((uint32_t)0x3FFFFF)
#define 	MASKb23			((uint32_t)0x7FFFFF)
#define 	MASKb24			((uint32_t)0x0FFFFFF)
#define 	MASKb25			((uint32_t)0x1FFFFFF)
#define 	MASKb26			((uint32_t)0x3FFFFFF)
#define 	MASKb27			((uint32_t)0x7FFFFFF)
#define 	MASKb28			((uint32_t)0x0FFFFFFF)
#define 	MASKb29			((uint32_t)0x1FFFFFFF)
#define 	MASKb30			((uint32_t)0x3FFFFFFF)
#define 	MASKb31			((uint32_t)0x7FFFFFFF)
#define 	MASKb32			((uint32_t)0x0FFFFFFFF)

/******************************************************************************/
/*                                                                            */
/*                    PWR寄存器--电源控制寄存器                               */
/*                                                                            */
/******************************************************************************/
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
#define  SET_PWR_CR_PLS(a)   (MEM_ADDR(PWR_BASE)=(MEM_ADDR(PWR_BASE)&(~PWR_CR_PLS))|((a&MASKb3)<<5))
//PVD电压阀值,示例：设置2.5V电压阀值时使用SET_PWR_CR_PLS(PVD_2V5);
#define  PVD_2V2	0x000
#define  PVD_2V3	0x001
#define  PVD_2V4	0x010
#define  PVD_2V5	0x011
#define  PVD_2V6	0x100
#define  PVD_2V7	0x101
#define  PVD_2V8	0x110
#define  PVD_2V9	0x111


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

//RCC_CR--时钟控制寄存器
#define  bRCC_CLK_HSION        BIT_ADDR(RCC_BASE, 0) //LSI时钟: 0禁用,1开启
#define  bRCC_CLK_HSIRDY        BIT_ADDR(RCC_BASE, 1) //LSI时钟状态由硬件控制(只读):0不可用,1就绪
#define  bRCC_CLK_HSEON        BIT_ADDR(RCC_BASE, 16) //HSE时钟: 0禁用,1开启
#define  bRCC_CLK_HSERDY        BIT_ADDR(RCC_BASE, 17) //HSE时钟状态由硬件控制(只读):0不可用,1就绪
#define  bRCC_CLK_HSEBYP       BIT_ADDR(RCC_BASE, 18) //外部时钟旁路(调试用)-- 0不旁路  1旁路
#define  bRCC_CLK_CSSON     	BIT_ADDR(RCC_BASE, 19) //系统时钟安全系统使能  0时钟检测禁用  1外部时钟就绪后启动检测
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

//AFIO_EVCR--事件控制寄存器
#define  bAFIO_EVCR_EVOE        		BIT_ADDR(AFIO_BASE, 7) //事件输出使能. 为1时,事件由指定的端口输出(端口由BIT6:4,引脚由BIT3:0决定)
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
//USART状态寄存器SR
#define  bURT_SR_PE(USARTx_BASE) 			BIT_ADDR(USARTx_BASE+0x00, 0) //校验错误（Parity Error）在接收模式下，如果出现奇偶校验错误，硬件对该位置位。
#define  bURT_SR_FE(USARTx_BASE) 			BIT_ADDR(USARTx_BASE+0x00, 1) //帧错误（Framing Error）当检测到同步错位，过多的噪声或者检测到断开符，该位被硬件置位。
#define  bURT_SR_NE(USARTx_BASE) 			BIT_ADDR(USARTx_BASE+0x00, 2) //噪声错误（Noise Error）在接收到的帧检测到噪音时，由硬件对该位置位。
#define  bURT_SR_ORE(USARTx_BASE) 		BIT_ADDR(USARTx_BASE+0x00, 3) //过载错误（Overrun Error）当RXNE仍然是’1’的时候，当前被接收在移位寄存器中的数据，需要传送至RDR寄存器时，硬件将该位置位。如果USART_CR1中的RXNEIE为’1’的话，则产生中断。
#define  bURT_SR_IDLE(USARTx_BASE)		BIT_ADDR(USARTx_BASE+0x00, 4) //IDLE：监测到总线空闲；当检测到总线空闲时，该位被硬件置位。如果USART_CR1中的IDLEIE为’1’，则产生中断。
#define  bURT_SR_RXNE(USARTx_BASE)		BIT_ADDR(USARTx_BASE+0x00, 5) //接收据寄存器非空（Receive Not Empty），当该位被置位的时候，就是提示已经有数据被接收到了，并且可以读出来了。通过读USART_DR可以将该位清零，也可以向该位写0，直接清除。
#define  bURT_SR_TC(USARTx_BASE) 			BIT_ADDR(USARTx_BASE+0x00, 6) //发送完成（Transmit Complete），当该位被置位的时候，表示USART_DR内的数据已经被发送完成了。如果设置了这个位的中断，则会产生中断。该位也有两种清零方式：1）读USART_SR，写USART_DR。2）直接向该位写0。
#define  bURT_SR_TXE(USARTx_BASE) 		BIT_ADDR(USARTx_BASE+0x00, 7) //发送数据寄存器空（Transmit Empty）当TDR寄存器中的数据被硬件转移到移位寄存器的时候，该位被硬件置位。如果USART_CR1寄存器中的TXEIE为1，则产生中断。对USART_DR的写操作，将该位清零。
#define  bURT_SR_LBD(USARTx_BASE) 		BIT_ADDR(USARTx_BASE+0x00, 8) //LIN断开检测（LIN Break Detect）当探测到LIN断开时，该位由硬件置’1’，由软件清’0’(向该位写0)。如果USART_CR3中的LBDIE = 1，则产生中断。
#define  bURT_SR_CTS(USARTx_BASE) 		BIT_ADDR(USARTx_BASE+0x00, 9)  //如果设置了CTSE位，当nCTS输入变化状态时，该位被硬件置高。由软件将其清零。如果USART_CR3中的CTSIE为’1’，则产生中断。
//USART状态寄存器CR1
#define  bURT_CR_SBK(USARTx_BASE)     BIT_ADDR(USARTx_BASE+0x0C, 0)  //发送断开帧（Send Break）位；使用该位来发送断开字符。该位可以由软件设置或清除。操作过程应该是软件设置位它，然后在断开帧的停止位时，由硬件将该位复位。
#define  bURT_CR_RWU(USARTx_BASE)     BIT_ADDR(USARTx_BASE+0x0C, 1)  //接收唤醒（Receiver Wakeup）位；置0，正常模式；置1，静默模式。
#define  bURT_CR_RE(USARTx_BASE)      BIT_ADDR(USARTx_BASE+0x0C, 2)  //接收使能（Receive Enable）位，用法同 TE。
#define  bURT_CR_TE(USARTx_BASE)      BIT_ADDR(USARTx_BASE+0x0C, 3)  //发送使能（Transmit Enable）位，设置为1，将开启串口的发送功能。
#define  bURT_CR_IDLEIE(USARTx_BASE)  BIT_ADDR(USARTx_BASE+0x0C, 4)  //IDLE中断使能（IDLE Interrupt Enable）位，置0，禁止中断；置1，当USART_SR中的IDLE为’1’时，产生USART中断。
#define  bURT_CR_RXNEIE(USARTx_BASE)  BIT_ADDR(USARTx_BASE+0x0C, 5)  //接收缓冲区非空中断使能（Receive Non-Empty Interrupt Enable）位，设置该位为 1，当 USART_SR中的 ORE 或者 RXNE 位为 1 时，将产生串口中断。
#define  bURT_CR_TCIE(USARTx_BASE)    BIT_ADDR(USARTx_BASE+0x0C, 6)  //发送完成中断使能（Transmit Complete Interrupt Enable）位，设置该位为 1，当 USART_SR 中的 TC位为 1 时，将产生串口中断。
#define  bURT_CR_TXEIE(USARTx_BASE)   BIT_ADDR(USARTx_BASE+0x0C, 7)  //发送缓冲区空中断使能（Transmit Interrupt Enable）位，设置该位为 1，当 USART_SR 中的 TXE 位为1 时，将产生串口中断。
#define  bURT_CR_PEIE(USARTx_BASE)    BIT_ADDR(USARTx_BASE+0x0C, 8)  //PE中断使能（Parity Error Interrupt Enable），置0，禁止中断；置1，当USART_SR中的PE为’1’时，产生USART中断。
#define  bURT_CR_PS(USARTx_BASE)      BIT_ADDR(USARTx_BASE+0x0C, 9)   //校验位选择（Parity Select）位，设置为0则为偶校验，否则为奇校验。
#define  bURT_CR_PCE(USARTx_BASE)     BIT_ADDR(USARTx_BASE+0x0C, 10)  //校验控制使能（Parity Control Enable）位，置0，则禁止校验，否则使能校验。
#define  bURT_CR_WAKE(USARTx_BASE)    BIT_ADDR(USARTx_BASE+0x0C, 11) //唤醒；置0，被空闲总线唤醒；置1，被地址标记唤醒。
#define  bURT_CR_M(USARTx_BASE)       BIT_ADDR(USARTx_BASE+0x0C, 12) //字长选择位，当该位为0的时候设置串口为8个字长外加停止位，停止位的个数是根据USART_CR2的[13:12]位设置来决定的，默认为0。
#define  bURT_CR_UE(USARTx_BASE)      BIT_ADDR(USARTx_BASE+0x0C, 13) //串口使能（Usart Enable）位，通过该位置1，以使能串口。当该位被清零，分频器和输出停止工作，以减少功耗。
#define  bURT_CR_OVER8(USARTx_BASE)   BIT_ADDR(USARTx_BASE+0x0C, 14)           /*!< USART Oversmapling 8-bits */
//USART状态寄存器CR2
#define  SET_URT_CR_ADD(USARTx_BASE,a)   (MEM_ADDR(USARTx_BASE+0x10)=(MEM_ADDR(USARTx_BASE+0x10)&(~USART_CR2_ADD))|((a&MASKb5)<<0))
// #define  bCR2_ADD     BIT_ADDR(USARTx_BASE+0x10, 0)  //本设备的USART节点地址(BIT 0:4共5位)。这是在多处理器通信下的静默模式中使用的，使用地址标记来唤醒某个USART设备。
#define  bURT_CR_LBDL(USARTx_BASE)    BIT_ADDR(USARTx_BASE+0x10, 5)  //LIN断开符检测长度（LIN Break Detection Length）位；该位用来选择是11位还是10位的断开符检测。
#define  bURT_CR_LBDIE(USARTx_BASE)   BIT_ADDR(USARTx_BASE+0x10, 6)  //LIN断开符检测中断使能（LIN Break Detection Interrupt Enable）位；置0，禁止中断；置1，只要USART_SR寄存器中的LBD为’1’就产生中断。
#define  bURT_CR_LBCL(USARTx_BASE)    BIT_ADDR(USARTx_BASE+0x10, 7)  //最后一位时钟脉冲（Last Bit Clock Pulse）位；在同步模式下，使用该位来控制是否在CK引脚上输出最后发送的那个数据字节(MSB)对应的时钟脉冲。
#define  bURT_CR_CPHA(USARTx_BASE)    BIT_ADDR(USARTx_BASE+0x10, 9)  //时钟相位（Clock Phase）位；在同步模式下，可以用该位选择SLCK引脚上时钟输出的相位。
#define  bURT_CR_CPOL(USARTx_BASE)    BIT_ADDR(USARTx_BASE+0x10, 10) //时钟极性（Clock Polarity）位；在同步模式下，可以用该位选择SLCK引脚上时钟输出的极性。
#define  bURT_CR_CLKEN(USARTx_BASE)   BIT_ADDR(USARTx_BASE+0x10, 11) //时钟使能（Clock Enable）位；该位用来使能CK引脚。
#define  SET_URT_CR_STOP(USARTx_BASE,a)   (MEM_ADDR(USARTx_BASE+0x10)=(MEM_ADDR(USARTx_BASE+0x10)&(~USART_CR2_STOP))|((a&MASKb2)<<12))
// #define  bCR2_STOP(USARTx_BASE)    BIT_ADDR(USARTx_BASE+0x10, 12) //停止位（STOP）位；这2位用来设置停止位的位数；00：1个停止位；01：0.5个停止位；10：2个停止位；11：1.5个停止位；          /*!< STOP[1:0] bits (STOP bits) */
#define  bURT_CR_STOP_0(USARTx_BASE)  BIT_ADDR(USARTx_BASE+0x10, 12) //           
#define  bURT_CR_STOP_1(USARTx_BASE)  BIT_ADDR(USARTx_BASE+0x10, 13) //
#define  bURT_CR_LINEN(USARTx_BASE)   BIT_ADDR(USARTx_BASE+0x10, 14) //LIN模式使能（LIN Enable）位；在LIN模式下，可以用USART_CR1寄存器中的SBK位发送LIN同步断开符(低13位)，以及检测LIN同步断开符。

//USART状态寄存器CR3
#define  bURT_CR_EIE(USARTx_BASE)     BIT_ADDR(USARTx_BASE+0x14, 0)  //错误中断使能（Error Interrupt Enable）位；
#define  bURT_CR_IREN(USARTx_BASE)    BIT_ADDR(USARTx_BASE+0x14, 1)  //红外模式使能（IrDA Enable）位；
#define  bURT_CR_IRLP(USARTx_BASE)    BIT_ADDR(USARTx_BASE+0x14, 2)  //红外低功耗（IrDA Low-Power）位；该位用来选择普通模式还是低功耗红外模式；
#define  bURT_CR_HDSEL(USARTx_BASE)   BIT_ADDR(USARTx_BASE+0x14, 3)  //半双工选择（Half-duplex Selection）位；选择单线半双工模式；
#define  bURT_CR_NACK(USARTx_BASE)    BIT_ADDR(USARTx_BASE+0x14, 4)  //智能卡NACK（Smartcard NACK）位；置0，不发送NACK；置1，发送NACK；
#define  bURT_CR_SCEN(USARTx_BASE)    BIT_ADDR(USARTx_BASE+0x14, 5)  //智能卡使能（Smartcard Enable）位；该位用来使能智能卡模式
#define  bURT_CR_DMAR(USARTx_BASE)    BIT_ADDR(USARTx_BASE+0x14, 6)  //DMA接收（DMA Receiver）位； 
#define  bURT_CR_DMAT(USARTx_BASE)    BIT_ADDR(USARTx_BASE+0x14, 7)  //DMA发送（DMA Transmitter）位； 
#define  bURT_CR_RTSE(USARTx_BASE)    BIT_ADDR(USARTx_BASE+0x14, 8)  //RTS使能（RTS Enable）位； 
#define  bURT_CR_CTSE(USARTx_BASE)    BIT_ADDR(USARTx_BASE+0x14, 9)  //CTS使能（CTS Enable）位；
#define  bURT_CR_CTSIE(USARTx_BASE)   BIT_ADDR(USARTx_BASE+0x14, 10) //CTS中断使能（CTS Interrupt Enable）位；
#define  bURT_CR_ONEBIT(USARTx_BASE)  BIT_ADDR(USARTx_BASE+0x14, 11)  

//USART状态寄存器GTPR
#define  bURT_GTPR_PSC(USARTx_BASE)     BIT_ADDR(USARTx_BASE+0x18, 0) //预分频器值（Prescaler）位；在红外或智能卡模式，才需要这个功能。
#define  bURT_GTPR_GT(USARTx_BASE)      BIT_ADDR(USARTx_BASE+0x18, 8) //保护时间值（Guard Time）位；该位域规定了以波特时钟为单位的保护时间。在智能卡模式下，需要这个功能。当保护时间过去后，才会设置发送完成标志。

/******************************************************************************/
/*                                                                            */
/*                        SPI接口寄存器                         */
/*                                                                            */
/******************************************************************************/

/*******************    SPI_CR1 寄存器  ********************/
#define  bSPI_CR_CPHA(SPIx_BASE)     BIT_ADDR(SPIx_BASE+0x00, 0)  /*!< Clock Phase */
#define  bSPI_CR_CPOL(SPIx_BASE)     BIT_ADDR(SPIx_BASE+0x00, 1)            /*!< Clock Polarity */
#define  bSPI_CR_MSTR(SPIx_BASE)     BIT_ADDR(SPIx_BASE+0x00, 2)            /*!< Master Selection */

#define  SET_SPI_CR_BR(SPIx_BASE,a)   (MEM_ADDR(SPIx_BASE)=(MEM_ADDR(SPIx_BASE)&(~SPI_CR1_BR))|((a&MASKb3)<<3))
//   #define  bSPI1_CR1_BR       BIT_ADDR(SPI1_BASE+0x00, 3)            /*!< BR[2:0] bits (Baud Rate Control) */
// SPI总线速度设置
#define SPI_SPEED_2   	0    //SPI_SPEED_2   2分频   (SPI 36M@sys 72M) 
#define SPI_SPEED_4   	1    //SPI_SPEED_2   4分频   (SPI 18M@sys 72M)
#define SPI_SPEED_8   	2    //SPI_SPEED_8   8分频   (SPI 9M@sys 72M)
#define SPI_SPEED_16  	3    //SPI_SPEED_16  16分频  (SPI 4.5M@sys 72M)
#define SPI_SPEED_32  	4    //SPI_SPEED_32  32分频  (SPI 2.25M@sys 72M)
#define SPI_SPEED_64  	5    //SPI_SPEED_64  64分频   (SPI 1.125M@sys 72M)
#define SPI_SPEED_128  	6    //SPI_SPEED_128 128分频 (SPI 562.5K@sys 72M)
#define SPI_SPEED_256 	7    //SPI_SPEED_256 256分频 (SPI 281.25K@sys 72M)

#define  bSPI_CR_SPE(SPIx_BASE)      BIT_ADDR(SPIx_BASE+0x00, 6)            /*!< SPI Enable */
#define  bSPI_CR_LSBFIRST(SPIx_BASE) BIT_ADDR(SPIx_BASE+0x00, 7)            /*!< Frame Format */
#define  bSPI_CR_SSI(SPIx_BASE)      BIT_ADDR(SPIx_BASE+0x00, 8)            /*!< Internal slave select */
#define  bSPI_CR_SSM(SPIx_BASE)      BIT_ADDR(SPIx_BASE+0x00, 9)            /*!< Software slave management */
#define  bSPI_CR_RXONLY(SPIx_BASE)   BIT_ADDR(SPIx_BASE+0x00, 10)            /*!< Receive only */
#define  bSPI_CR_DFF(SPIx_BASE)      BIT_ADDR(SPIx_BASE+0x00, 11)            /*!< Data Frame Format */
#define  bSPI_CR_CRCNEXT(SPIx_BASE)  BIT_ADDR(SPIx_BASE+0x00, 12)            /*!< Transmit CRC next */
#define  bSPI_CR_CRCEN(SPIx_BASE)    BIT_ADDR(SPIx_BASE+0x00, 13)            /*!< Hardware CRC calculation enable */
#define  bSPI_CR_BIDIOE(SPIx_BASE)   BIT_ADDR(SPIx_BASE+0x00, 14)            /*!< Output enable in bidirectional mode */
#define  bSPI_CR_BIDIMODE(SPIx_BASE) BIT_ADDR(SPIx_BASE+0x00, 15)            /*!< Bidirectional data mode enable */

/*******************    SPI_CR2 寄存器  ********************/
#define  bSPI_CR_RXDMAEN(SPIx_BASE)  BIT_ADDR(SPIx_BASE+0x04, 0)               /*!< Rx Buffer DMA Enable */
#define  bSPI_CR_TXDMAEN(SPIx_BASE)  BIT_ADDR(SPIx_BASE+0x04, 1)               /*!< Tx Buffer DMA Enable */
#define  bSPI_CR_SSOE(SPIx_BASE)     BIT_ADDR(SPIx_BASE+0x04, 2)               /*!< SS Output Enable */
#define  bSPI_CR_ERRIE(SPIx_BASE)    BIT_ADDR(SPIx_BASE+0x04, 4)               /*!< Error Interrupt Enable */
#define  bSPI_CR_RXNEIE(SPIx_BASE)   BIT_ADDR(SPIx_BASE+0x04, 6)               /*!< RX buffer Not Empty Interrupt Enable */
#define  bSPI_CR_TXEIE(SPIx_BASE)    BIT_ADDR(SPIx_BASE+0x04, 7)               /*!< Tx buffer Empty Interrupt Enable */

/********************    SPI_SR 寄存器  ********************/
#define  bSPI_SR_RXNE(SPIx_BASE)     BIT_ADDR(SPIx_BASE+0x08, 0)              /*!< Receive buffer Not Empty */
#define  bSPI_SR_TXE(SPIx_BASE)      BIT_ADDR(SPIx_BASE+0x08, 1)               /*!< Transmit buffer Empty */
#define  bSPI_SR_CHSIDE(SPIx_BASE)   BIT_ADDR(SPIx_BASE+0x08, 2)               /*!< Channel side */
#define  bSPI_SR_UDR(SPIx_BASE)      BIT_ADDR(SPIx_BASE+0x08, 3)               /*!< Underrun flag */
#define  bSPI_SR_CRCERR(SPIx_BASE)   BIT_ADDR(SPIx_BASE+0x08, 4)               /*!< CRC Error flag */
#define  bSPI_SR_MODF(SPIx_BASE)     BIT_ADDR(SPIx_BASE+0x08, 5)               /*!< Mode fault */
#define  bSPI_SR_OVR(SPIx_BASE)      BIT_ADDR(SPIx_BASE+0x08, 6)               /*!< Overrun flag */
#define  bSPI_SR_BSY(SPIx_BASE)      BIT_ADDR(SPIx_BASE+0x08, 7)               /*!< Busy flag */

/******************    SPI_I2SCFGR 寄存器  *****************/
#define  bSPI_I2SCFGR_CHLEN(SPIx_BASE)     BIT_ADDR(SPIx_BASE+0x1C, 0)            /*!< Channel length (number of bits per audio channel) */
#define  bSPI_I2SCFGR_DATLEN(SPIx_BASE)    BIT_ADDR(SPIx_BASE+0x1C, 1)             /*!< DATLEN[1:0] bits (Data length to be transferred) */
#define  bSPI_I2SCFGR_DATLEN_0(SPIx_BASE)  BIT_ADDR(SPIx_BASE+0x1C, 1)            /*!< Bit 0 */
#define  bSPI_I2SCFGR_DATLEN_1(SPIx_BASE)  BIT_ADDR(SPIx_BASE+0x1C, 2)            /*!< Bit 1 */

#define  bSPI_I2SCFGR_CKPOL(SPIx_BASE)     BIT_ADDR(SPIx_BASE+0x1C, 3)            /*!< steady state clock polarity */

#define  bSPI_I2SCFGR_I2SSTD(SPIx_BASE)    BIT_ADDR(SPIx_BASE+0x1C, 4)            /*!< I2SSTD[1:0] bits (I2S standard selection) */
#define  bSPI_I2SCFGR_I2SSTD_0(SPIx_BASE)  BIT_ADDR(SPIx_BASE+0x1C, 4)            /*!< Bit 0 */
#define  bSPI_I2SCFGR_I2SSTD_1(SPIx_BASE)  BIT_ADDR(SPIx_BASE+0x1C, 5)            /*!< Bit 1 */

#define  bSPI_I2SCFGR_PCMSYNC(SPIx_BASE)   BIT_ADDR(SPIx_BASE+0x1C, 7)            /*!< PCM frame synchronization */

#define  bSPI_I2SCFGR_I2SCFG(SPIx_BASE)    BIT_ADDR(SPIx_BASE+0x1C, 8)            /*!< I2SCFG[1:0] bits (I2S configuration mode) */
#define  bSPI_I2SCFGR_I2SCFG_0(SPIx_BASE)  BIT_ADDR(SPIx_BASE+0x1C, 8)            /*!< Bit 0 */
#define  bSPI_I2SCFGR_I2SCFG_1(SPIx_BASE)  BIT_ADDR(SPIx_BASE+0x1C, 9)            /*!< Bit 1 */

#define  bSPI_I2SCFGR_I2SE(SPIx_BASE)      BIT_ADDR(SPIx_BASE+0x1C, 10)            /*!< I2S Enable */
#define  bSPI_I2SCFGR_I2SMOD(SPIx_BASE)    BIT_ADDR(SPIx_BASE+0x1C, 11)            /*!< I2S mode selection */

/******************    SPI_I2SPR 寄存器  *******************/
#define  SET_SPI_I2SPR_I2SDIV(SPIx_BASE,a)   (MEM_ADDR(SPIx_BASE+0x20)=(MEM_ADDR(SPIx_BASE+0x20)&(~SPI_I2SPR_I2SDIV))|((a&MASKb8)<<0))
// #define  bSPI1_I2SPR_I2SDIV      BIT_ADDR(SPI1_BASE+0x20, 0)  //长度8BIT I2S 线性预分频

#define  bSPI_I2SPR_ODD(SPIx_BASE)         BIT_ADDR(SPIx_BASE+0x20, 8)            /*!< Odd factor for the prescaler */
#define  bSPI_I2SPR_MCKOE(SPIx_BASE)       BIT_ADDR(SPIx_BASE+0x20, 9)            /*!< Master Clock Output Enable */


/******************************************************************************/
/*                                                                            */
/*                        TIM定时器接口寄存器                                  */
/*                                                                            */
/******************************************************************************/

/*******************  TIM_CR1 控制寄存器  ********************/
#define   bTIM_CR_CEN(TIMx_BASE)                         BIT_ADDR(TIMx_BASE,0)            /*!< Counter enable */
#define   bTIM_CR_UDIS(TIMx_BASE)                        BIT_ADDR(TIMx_BASE,1)            /*!< Update disable */
#define   bTIM_CR_URS(TIMx_BASE)                         BIT_ADDR(TIMx_BASE,2)            /*!< Update request source */
#define   bTIM_CR_OPM(TIMx_BASE)                         BIT_ADDR(TIMx_BASE,3)            /*!< One pulse mode */
#define   bTIM_CR_DIR(TIMx_BASE)                         BIT_ADDR(TIMx_BASE,4)            /*!< 方向，定义：0（计数器向上计数），1（计数器向下计数），注：当计数器配置为中央对齐模式或编码器模式时，该位为只读 */

#define 	SET_TIM_CR_CMS(TIMx_BASE,a)   (MEM_ADDR(TIMx_BASE+0x00)=(MEM_ADDR(TIMx_BASE+0x00)&(~TIM_CR1_CMS))|((a&0x03)<<5))
//  #define   CR1_CMS(TIMx_BASE)                         BIT_ADDR(TIMx_BASE +0x00,5)        /*!< CMS[1:0] 中央对齐模式选择，定义：00：边沿对齐模式。计数器依据方向位(DIR)向上或向下计数。 */
#define   bTIM_CR_CMS_0(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x00,5)            /*!< 01、10、11（中央对齐模式1、2、3。计数器交替地向上和向下计数。详略*/
#define   bTIM_CR_CMS_1(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x00,6)            /*!<  */

#define   bTIM_CR_ARPE(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x00,7)            /*!< Auto-reload preload enable */

#define 	SET_TIM_CR_CKD(TIMx_BASE,a)   (MEM_ADDR(TIMx_BASE+0x00)=(MEM_ADDR(TIMx_BASE+0x00)&(~TIM_CR1_CKD))|((a&0x03)<<8))
//  #define   CR1_CKD(TIMx_BASE)                         BIT_ADDR(TIMx_BASE +0x00,8)            /*!< CKD[1:0] CKD[1:0]时钟分频因子，定义在定时器时钟(CK_INT)频率与数字滤波器(ETR，TIx)使用的采样频率之间的分频比例。 */
#define   bTIM_CR_CKD_0(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x00,8)            /*!< 定义：00（tDTS = tCK_INT），01（tDTS = 2 x tCK_INT），10（tDTS = 4 x tCK_INT）11：保留 */
#define   bTIM_CR_CKD_1(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x00,9)            /*!< Bit 1 */

/*******************  TIM_CR2 控制寄存器  ********************/
#define   bTIM_CR_CCPC(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x04,0)            /*!< Capture/Compare Preloaded Control */
#define   bTIM_CR_CCUS(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x04,2)            /*!< Capture/Compare Control Update Selection */
#define   bTIM_CR_CCDS(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x04,3)            /*!< Capture/Compare DMA Selection */

#define 	SET_TIM_CR_MMS(TIMx_BASE,a)   (MEM_ADDR(TIMx_BASE+0x04)=(MEM_ADDR(TIMx_BASE+0x04)&(~TIM_CR2_MMS))|((a&MASKb3)<<4))
//  #define   CR2_MMS(TIMx_BASE)                         BIT_ADDR(TIMx_BASE +0x04,4)            /*!< MMS[2:0] bits (Master Mode Selection) */

#define   bTIM_CR_TI1S(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x04,7)            /*!< TI1 Selection */
#define   bTIM_CR_OIS1(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x04,8)            /*!< Output Idle state 1 (OC1 output) */
#define   bTIM_CR_OIS1N(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x04,9)            /*!< Output Idle state 1 (OC1N output) */
#define   bTIM_CR_OIS2(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x04,10)            /*!< Output Idle state 2 (OC2 output) */
#define   bTIM_CR_OIS2N(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x04,11)            /*!< Output Idle state 2 (OC2N output) */
#define   bTIM_CR_OIS3(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x04,12)            /*!< Output Idle state 3 (OC3 output) */
#define   bTIM_CR_OIS3N(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x04,13)            /*!< Output Idle state 3 (OC3N output) */
#define   bTIM_CR_OIS4(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x04,14)            /*!< Output Idle state 4 (OC4 output) */

/*******************  TIM_SMCR 寄存器  *******************/
#define 	SET_TIM_SMCR_SMS(TIMx_BASE,a)   (MEM_ADDR(TIMx_BASE+0x08)=(MEM_ADDR(TIMx_BASE+0x08)&(~TIM_SMCR_SMS))|((a&MASKb3)<<0))
//  #define   SMCR_SMS(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x08,0)            /*!< SMS[2:0] bits (Slave mode selection) */

#define 	SET_TIM_SMCR_TS(TIMx_BASE,a)   (MEM_ADDR(TIMx_BASE+0x08)=(MEM_ADDR(TIMx_BASE+0x08)&(~TIM_SMCR_TS))|((a&MASKb3)<<4))
//  #define   SMCR_TS(TIMx_BASE)                         BIT_ADDR(TIMx_BASE +0x08,4)            /*!< TS[2:0] bits (Trigger selection) */

#define   bTIM_SMCR_MSM(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x08,7)            /*!< Master/slave mode */

#define 	SET_TIM_SMCR_ETF(TIMx_BASE,a)   (MEM_ADDR(TIMx_BASE+0x08)=(MEM_ADDR(TIMx_BASE+0x08)&(~TIM_SMCR_ETF))|((a&MASKb4)<<8))
//  #define   SMCR_ETF(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x08,8)            /*!< ETF[3:0] bits (External trigger filter) */

#define 	SET_TIM_SMCR_ETPS(TIMx_BASE,a)   (MEM_ADDR(TIMx_BASE+0x08)=(MEM_ADDR(TIMx_BASE+0x08)&(~TIM_SMCR_ETPS))|((a&MASKb2)<<12))
//  #define   SMCR_ETPS(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x08,12)            /*!< ETPS[1:0] bits (External trigger prescaler) */

#define   bTIM_SMCR_ECE(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x08,14)            /*!< External clock enable */
#define   bTIM_SMCR_ETP(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x08,15)            /*!< External trigger polarity */

/*******************  TIM_DIER 寄存器  *******************/
#define   bTIM_DIER_UIE(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x0C,0)            /*!< Update interrupt enable */
#define   bTIM_DIER_CC1IE(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x0C,1)            /*!< Capture/Compare 1 interrupt enable */
#define   bTIM_DIER_CC2IE(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x0C,2)            /*!< Capture/Compare 2 interrupt enable */
#define   bTIM_DIER_CC3IE(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x0C,3)            /*!< Capture/Compare 3 interrupt enable */
#define   bTIM_DIER_CC4IE(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x0C,4)            /*!< Capture/Compare 4 interrupt enable */
#define   bTIM_DIER_COMIE(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x0C,5)            /*!< COM interrupt enable */
#define   bTIM_DIER_TIE(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x0C,6)            /*!< Trigger interrupt enable */
#define   bTIM_DIER_BIE(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x0C,7)            /*!< Break interrupt enable */
#define   bTIM_DIER_UDE(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x0C,8)            /*!< Update DMA request enable */
#define   bTIM_DIER_CC1DE(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x0C,9)            /*!< Capture/Compare 1 DMA request enable */
#define   bTIM_DIER_CC2DE(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x0C,10)            /*!< Capture/Compare 2 DMA request enable */
#define   bTIM_DIER_CC3DE(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x0C,11)            /*!< Capture/Compare 3 DMA request enable */
#define   bTIM_DIER_CC4DE(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x0C,12)            /*!< Capture/Compare 4 DMA request enable */
#define   bTIM_DIER_COMDE(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x0C,13)            /*!< COM DMA request enable */
#define   bTIM_DIER_TDE(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x0C,14)            /*!< Trigger DMA request enable */

/********************  TIM_SR 状态寄存器  ********************/
#define   bTIM_SR_UIF(TIMx_BASE)                          BIT_ADDR(TIMx_BASE +0x10,0)            /*!< 更新中断标记（硬件置1，软件清0）定义：0（无更新事件）1（更新中断等待响应。当寄存器被更新时该位由硬件置’1’） */
#define   bTIM_SR_CC1IF(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x10,1)            /*!< Capture/Compare 1 interrupt Flag */
#define   bTIM_SR_CC2IF(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x10,2)            /*!< Capture/Compare 2 interrupt Flag */
#define   bTIM_SR_CC3IF(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x10,3)            /*!< Capture/Compare 3 interrupt Flag */
#define   bTIM_SR_CC4IF(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x10,4)            /*!< Capture/Compare 4 interrupt Flag */
#define   bTIM_SR_COMIF(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x10,5)            /*!< COM interrupt Flag */
#define   bTIM_SR_TIF(TIMx_BASE)                          BIT_ADDR(TIMx_BASE +0x10,6)            /*!< Trigger interrupt Flag */
#define   bTIM_SR_BIF(TIMx_BASE)                          BIT_ADDR(TIMx_BASE +0x10,7)            /*!< Break interrupt Flag */
#define   bTIM_SR_CC1OF(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x10,9)            /*!< Capture/Compare 1 Overcapture Flag */
#define   bTIM_SR_CC2OF(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x10,10)            /*!< Capture/Compare 2 Overcapture Flag */
#define   bTIM_SR_CC3OF(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x10,11)            /*!< Capture/Compare 3 Overcapture Flag */
#define   bTIM_SR_CC4OF(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x10,12)            /*!< Capture/Compare 4 Overcapture Flag */

/*******************  TIM_EGR 寄存器  ********************/
#define   bTIM_EGR_UG(TIMx_BASE)                          BIT_ADDR(TIMx_BASE +0x14,0)               /*!< Update Generation */
#define   bTIM_EGR_CC1G(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x14,1)               /*!< Capture/Compare 1 Generation */
#define   bTIM_EGR_CC2G(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x14,2)               /*!< Capture/Compare 2 Generation */
#define   bTIM_EGR_CC3G(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x14,3)               /*!< Capture/Compare 3 Generation */
#define   bTIM_EGR_CC4G(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x14,4)               /*!< Capture/Compare 4 Generation */
#define   bTIM_EGR_COMG(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x14,5)               /*!< Capture/Compare Control Update Generation */
#define   bTIM_EGR_TG(TIMx_BASE)                          BIT_ADDR(TIMx_BASE +0x14,6)               /*!< Trigger Generation */
#define   bTIM_EGR_BG(TIMx_BASE)                          BIT_ADDR(TIMx_BASE +0x14,7)               /*!< Break Generation */

/******************  TIM_CCMR1 寄存器  *******************/
#define   bTIM_CCMR_CC1S(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x18,0)            /*!< CC1S[1:0] bits (Capture/Compare 1 Selection) */
#define   bTIM_CCMR_CC1S_0(TIMx_BASE)                    BIT_ADDR(TIMx_BASE +0x18,0)            /*!< Bit 0 */
#define   bTIM_CCMR_CC1S_1(TIMx_BASE)                    BIT_ADDR(TIMx_BASE +0x18,1)            /*!< Bit 1 */

#define   bTIM_CCMR_OC1FE(TIMx_BASE)                     BIT_ADDR(TIMx_BASE +0x18,2)            /*!< Output Compare 1 Fast enable */
#define   bTIM_CCMR_OC1PE(TIMx_BASE)                     BIT_ADDR(TIMx_BASE +0x18,3)            /*!< Output Compare 1 Preload enable */

#define 	SET_TIM_CCMR1_OC1M(TIMx_BASE,a)   (MEM_ADDR(TIMx_BASE+0x18)=(MEM_ADDR(TIMx_BASE+0x18)&(~TIM_CCMR1_OC1M))|((a&0x07)<<4))
//  #define   CCMR1_OC1M(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x18,4)            /*!< OC1M[2:0] bits (Output Compare 1 Mode) */

#define   bTIM_CCMR_OC1CE(TIMx_BASE)                     BIT_ADDR(TIMx_BASE +0x18,7)            /*!< Output Compare 1Clear Enable */

#define   bTIM_CCMR_CC2S(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x18,8)            /*!< CC2S[1:0] bits (Capture/Compare 2 Selection) */
#define   bTIM_CCMR_CC2S_0(TIMx_BASE)                    BIT_ADDR(TIMx_BASE +0x18,8)            /*!< Bit 0 */
#define   bTIM_CCMR_CC2S_1(TIMx_BASE)                    BIT_ADDR(TIMx_BASE +0x18,9)            /*!< Bit 1 */

#define   bTIM_CCMR_OC2FE(TIMx_BASE)                     BIT_ADDR(TIMx_BASE +0x18,10)            /*!< Output Compare 2 Fast enable */
#define   bTIM_CCMR_OC2PE(TIMx_BASE)                     BIT_ADDR(TIMx_BASE +0x18,11)            /*!< Output Compare 2 Preload enable */

#define 	SET_TIM_CCMR_OC2M(TIMx_BASE,a)   (MEM_ADDR(TIMx_BASE+0x18)=(MEM_ADDR(TIMx_BASE+0x18)&(~TIM_CCMR1_OC2M))|((a&0x07)<<12))
//  #define   CCMR1_OC2M(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x18,12)            /*!< OC2M[2:0] bits (Output Compare 2 Mode) */

#define   bTIM_CCMR_OC2CE(TIMx_BASE)                     BIT_ADDR(TIMx_BASE +0x18,15)            /*!< Output Compare 2 Clear Enable */

/*----------------------------------------------------------------------------*/

#define   bTIM_CCMR_IC1PSC(TIMx_BASE)                    BIT_ADDR(TIMx_BASE +0x18,2)            /*!< IC1PSC[1:0] bits (Input Capture 1 Prescaler) */
#define   bTIM_CCMR_IC1PSC_0(TIMx_BASE)                  BIT_ADDR(TIMx_BASE +0x18,2)            /*!< Bit 0 */
#define   bTIM_CCMR_IC1PSC_1(TIMx_BASE)                  BIT_ADDR(TIMx_BASE +0x18,3)            /*!< Bit 1 */

#define 	SET_TIM_CCMR_IC1F(TIMx_BASE,a)   (MEM_ADDR(TIMx_BASE+0x18)=(MEM_ADDR(TIMx_BASE+0x18)&(~TIM_CCMR1_IC1F))|((a&0x0F)<<4))
//  #define   CCMR1_IC1F(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x18,4)            /*!< IC1F[3:0] bits (Input Capture 1 Filter) */

#define   bTIM_CCMR_IC2PSC(TIMx_BASE)                    BIT_ADDR(TIMx_BASE +0x18,10)            /*!< IC2PSC[1:0] bits (Input Capture 2 Prescaler) */
#define   bTIM_CCMR_IC2PSC_0(TIMx_BASE)                  BIT_ADDR(TIMx_BASE +0x18,10)            /*!< Bit 0 */
#define   bTIM_CCMR_IC2PSC_1(TIMx_BASE)                  BIT_ADDR(TIMx_BASE +0x18,11)            /*!< Bit 1 */

#define 	SET_TIM_CCMR_IC2F(TIMx_BASE,a)   (MEM_ADDR(TIMx_BASE+0x18)=(MEM_ADDR(TIMx_BASE+0x18)&(~TIM_CCMR1_IC2F))|((a&0x0F)<<12))
//  #define   CCMR1_IC2F(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x18,12)            /*!< IC2F[3:0] bits (Input Capture 2 Filter) */

/******************    TIM_CCMR2 寄存器  *******************/
#define   bTIM_CCMR_CC3S(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x1C,0)            /*!< CC3S[1:0] bits (Capture/Compare 3 Selection) */
#define   bTIM_CCMR_CC3S_0(TIMx_BASE)                    BIT_ADDR(TIMx_BASE +0x1C,0)            /*!< Bit 0 */
#define   bTIM_CCMR_CC3S_1(TIMx_BASE)                    BIT_ADDR(TIMx_BASE +0x1C,1)            /*!< Bit 1 */

#define   bTIM_CCMR_OC3FE(TIMx_BASE)                     BIT_ADDR(TIMx_BASE +0x1C,2)            /*!< Output Compare 3 Fast enable */
#define   bTIM_CCMR_OC3PE(TIMx_BASE)                     BIT_ADDR(TIMx_BASE +0x1C,3)            /*!< Output Compare 3 Preload enable */

#define 	SET_CCMR_OC3M(TIMx_BASE,a)   (MEM_ADDR(TIMx_BASE+0x1C)=(MEM_ADDR(TIMx_BASE+0x1C)&(~TIM_CCMR2_OC3M))|((a&0x07)<<4))
//  #define   CCMR2_OC3M(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x1C,4)            /*!< OC3M[2:0] bits (Output Compare 3 Mode) */


#define   bTIM_CCMR_OC3CE(TIMx_BASE)                     BIT_ADDR(TIMx_BASE +0x1C,7)            /*!< Output Compare 3 Clear Enable */

#define   bTIM_CCMR_CC4S(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x1C,8)            /*!< CC4S[1:0] bits (Capture/Compare 4 Selection) */
#define   bTIM_CCMR_CC4S_0(TIMx_BASE)                    BIT_ADDR(TIMx_BASE +0x1C,8)            /*!< Bit 0 */
#define   bTIM_CCMR_CC4S_1(TIMx_BASE)                    BIT_ADDR(TIMx_BASE +0x1C,9)            /*!< Bit 1 */

#define   bTIM_CCMR_OC4FE(TIMx_BASE)                     BIT_ADDR(TIMx_BASE +0x1C,10)            /*!< Output Compare 4 Fast enable */
#define   bTIM_CCMR_OC4PE(TIMx_BASE)                     BIT_ADDR(TIMx_BASE +0x1C,11)            /*!< Output Compare 4 Preload enable */

#define 	SET_TIM_CCMR_OC4M(TIMx_BASE,a)   (MEM_ADDR(TIMx_BASE+0x1C)=(MEM_ADDR(TIMx_BASE+0x1C)&(~TIM_CCMR2_OC4M))|((a&0x07)<<12))
//  #define   CCMR2_OC4M(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x1C,12)            /*!< OC4M[2:0] bits (Output Compare 4 Mode) */


#define   bTIM_CCMR_OC4CE(TIMx_BASE)                     BIT_ADDR(TIMx_BASE +0x1C,15)            /*!< Output Compare 4 Clear Enable */

/*----------------------------------------------------------------------------*/

#define   bTIM_CCMR_IC3PSC(TIMx_BASE)                    BIT_ADDR(TIMx_BASE +0x1C,2)            /*!< IC3PSC[1:0] bits (Input Capture 3 Prescaler) */
#define   bTIM_CCMR_IC3PSC_0(TIMx_BASE)                  BIT_ADDR(TIMx_BASE +0x1C,2)            /*!< Bit 0 */
#define   bTIM_CCMR_IC3PSC_1(TIMx_BASE)                  BIT_ADDR(TIMx_BASE +0x1C,3)            /*!< Bit 1 */

#define 	SET_TIM_CCMR_IC3F(TIMx_BASE,a)   (MEM_ADDR(TIMx_BASE+0x1C)=(MEM_ADDR(TIMx_BASE+0x1C)&(~TIM_CCMR2_IC3F))|((a&0x0f)<<4))
//  #define   CCMR2_IC3F(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x1C,4)            /*!< IC3F[3:0] bits (Input Capture 3 Filter) */

#define   bTIM_CCMR_IC4PSC(TIMx_BASE)                    BIT_ADDR(TIMx_BASE +0x1C,10)            /*!< IC4PSC[1:0] bits (Input Capture 4 Prescaler) */
#define   bTIM_CCMR_IC4PSC_0(TIMx_BASE)                  BIT_ADDR(TIMx_BASE +0x1C,10)            /*!< Bit 0 */
#define   bTIM_CCMR_IC4PSC_1(TIMx_BASE)                  BIT_ADDR(TIMx_BASE +0x1C,11)            /*!< Bit 1 */

#define 	SET_TIM_CCMR_IC4F(TIMx_BASE,a)   (MEM_ADDR(TIMx_BASE+0x1C)=(MEM_ADDR(TIMx_BASE+0x1C)&(~TIM_CCMR2_IC4F))|((a&0x0f)<<12))
//#define   CCMR2_IC4F(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x1C,12)            /*!< IC4F[3:0] bits (Input Capture 4 Filter) */


/*******************    TIM_CCER 寄存器  *******************/
#define   bTIM_CCER_CC1E(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x20,0)            /*!< Capture/Compare 1 output enable */
#define   bTIM_CCER_CC1P(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x20,1)            /*!< Capture/Compare 1 output Polarity */
#define   bTIM_CCER_CC1NE(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x20,2)            /*!< Capture/Compare 1 Complementary output enable */
#define   bTIM_CCER_CC1NP(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x20,3)            /*!< Capture/Compare 1 Complementary output Polarity */
#define   bTIM_CCER_CC2E(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x20,4)            /*!< Capture/Compare 2 output enable */
#define   bTIM_CCER_CC2P(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x20,5)            /*!< Capture/Compare 2 output Polarity */
#define   bTIM_CCER_CC2NE(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x20,6)            /*!< Capture/Compare 2 Complementary output enable */
#define   bTIM_CCER_CC2NP(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x20,7)            /*!< Capture/Compare 2 Complementary output Polarity */
#define   bTIM_CCER_CC3E(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x20,8)            /*!< Capture/Compare 3 output enable */
#define   bTIM_CCER_CC3P(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x20,9)            /*!< Capture/Compare 3 output Polarity */
#define   bTIM_CCER_CC3NE(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x20,10)            /*!< Capture/Compare 3 Complementary output enable */
#define   bTIM_CCER_CC3NP(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x20,11)            /*!< Capture/Compare 3 Complementary output Polarity */
#define   bTIM_CCER_CC4E(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x20,12)            /*!< Capture/Compare 4 output enable */
#define   bTIM_CCER_CC4P(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x20,13)            /*!< Capture/Compare 4 output Polarity */
#define   bTIM_CCER_CC4NP(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x20,15)            /*!< Capture/Compare 4 Complementary output Polarity */

/*******************    TIM_CNT 寄存器  ********************/
#define   TIM_CNT(TIMx_BASE)           (MEM_ADDR(TIMx_BASE+0x24))    /*!< Counter Value */             

/*******************    TIM_PSC 寄存器  ********************/
#define   TIM_PSC(TIMx_BASE)           (MEM_ADDR(TIMx_BASE+0x28))            /*!< Prescaler Value */

/*******************    TIM_ARR 寄存器  ********************/
#define   TIM_ARR(TIMx_BASE)           (MEM_ADDR(TIMx_BASE+0x2C))            /*!< actual auto-reload Value */

/*******************    TIM_RCR 寄存器  ********************/
#define   TIM_RCR(TIMx_BASE)           (MEM_ADDR(TIMx_BASE+0x30))               /*!< Repetition Counter Value */

/*******************    TIM_CCR1 寄存器  *******************/
#define   TIM_CCR1(TIMx_BASE)         (MEM_ADDR(TIMx_BASE+0x34))           /*!< Capture/Compare 1 Value */

/*******************    TIM_CCR2 寄存器  *******************/
#define   TIM_CCR2(TIMx_BASE)         (MEM_ADDR(TIMx_BASE+0x38))            /*!< Capture/Compare 2 Value */

/*******************    TIM_CCR3 寄存器  *******************/
#define   TIM_CCR3(TIMx_BASE)         (MEM_ADDR(TIMx_BASE+0x3C))            /*!< Capture/Compare 3 Value */

/*******************    TIM_CCR4 寄存器  *******************/
#define   TIM_CCR4(TIMx_BASE)         (MEM_ADDR(TIMx_BASE+0x40))            /*!< Capture/Compare 4 Value */

/*******************    TIM_BDTR 寄存器  *******************/
/*!< DTG[0:7] bits (Dead-Time Generator set-up) */
#define 	SET_TIM_BDTR_DTG(TIMx_BASE,a)   (MEM_ADDR(TIMx_BASE+0x44)=(MEM_ADDR(TIMx_BASE+0x44)&(~TIM_BDTR_DTG))|(a&MASKb8))

#define   bTIM_BDTR_LOCK(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x44,8)            /*!< LOCK[1:0] bits (Lock Configuration) */
#define   bTIM_BDTR_LOCK_0(TIMx_BASE)                     BIT_ADDR(TIMx_BASE +0x44,8)            /*!< Bit 0 */
#define   bTIM_BDTR_LOCK_1(TIMx_BASE)                     BIT_ADDR(TIMx_BASE +0x44,9)            /*!< Bit 1 */

#define   bTIM_BDTR_OSSI(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x44,10)            /*!< Off-State Selection for Idle mode */
#define   bTIM_BDTR_OSSR(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x44,11)            /*!< Off-State Selection for Run mode */
#define   bTIM_BDTR_BKE(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x44,12)            /*!< Break enable */
#define   bTIM_BDTR_BKP(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x44,13)            /*!< Break Polarity */
#define   bTIM_BDTR_AOE(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x44,14)            /*!< Automatic Output enable */
#define   bTIM_BDTR_MOE(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x44,15)            /*!< Main Output enable */

/*******************    TIM_DCR 寄存器  ********************/
/*!< DBA从第0BIT开始共5位： (DMA Base Address) */
#define 	SET_TIM_DCR_DBA(TIMx_BASE,a)   (MEM_ADDR(TIMx_BASE+0x48)=(MEM_ADDR(TIMx_BASE+0x48)&(~TIM_DCR_DBA))|((a&MASKb5)<<0))
/*!< DBL从第8BIT开始共5位： (DMA Burst Length) */
#define 	SET_TIM_DCR_DBL(TIMx_BASE,a)   (MEM_ADDR(TIMx_BASE+0x48)=(MEM_ADDR(TIMx_BASE+0x48)&(~TIM_DCR_DBL))|((a&MASKb5)<<8))


/*******************    TIM_DMAR 寄存器  *******************/
#define   bTIM_DMAR_DMAB(TIMx_BASE)       (MEM_ADDR(TIMx_BASE+0x4C))            /*!< DMA 寄存器 for burst accesses */

/******************************************************************************/
/*                                                                            */
/*                           独立看门狗寄存器  		                             */
/*                                                                            */
/******************************************************************************/

/*******************    IWDG_KR 寄存器  ********************/
       
#define  IWDG_KEY					(MEM_ADDR(IWDG_BASE))/*!< Key value (只写, 读出为0000h) */
/*******************    IWDG_PR 寄存器  ********************/
/*!< 从第0BIT开始共3位：PR[2:0] (预分频器) */
#define 	SET_IWDG_PR(a)   	(MEM_ADDR(IWDG_BASE+0x04)=(MEM_ADDR(IWDG_BASE+0x04)&(~IWDG_PR_PR))|(a&MASKb3))

/*******************    IWDG_RLR 寄存器  *******************/
        
#define  IWDG_RL					(MEM_ADDR(IWDG_BASE+0x08)) /*!< 重装载值 */
/*******************    IWDG_SR 寄存器  ********************/
#define  bIWDG_SR_PVU             BIT_ADDR(IWDG_BASE +0x0C,0)              /*!< 看门狗预分频值更新 */
#define  bIWDG_SR_RVU							BIT_ADDR(IWDG_BASE +0x0C,1)							 /*!< 看门狗计数器重装载更新 */
/******************************************************************************/
/*                                                                            */
/*                            窗口看门狗寄存器                    			        */
/*                                                                            */
/******************************************************************************/

/*******************    WWDG_CR 寄存器  ********************/
/*!< T从第0BIT开始共7位：  T[6:0] bits (7-Bit counter (MSB to LSB)) */
#define 	SET_WWDG_CR_T(a)   (MEM_ADDR(WWDG_BASE)=(MEM_ADDR(WWDG_BASE)&(~WWDG_CR_T))|(a&MASKb7))

#define  bWWDG_CR_WDGA                        BIT_ADDR(WWDG_BASE,7)               /*!< Activation bit */

/*******************    WWDG_CFR 寄存器  *******************/
/*!< W从第0BIT开始共7位：  W[6:0] bits (7-bit 窗口值) */
#define  SET_WWDG_CFR_W(a)   (MEM_ADDR(WWDG_BASE+0x04)=(MEM_ADDR(WWDG_BASE+0x04)&(~WWDG_CFR_W))|(a&MASKb7))

#define  bWWDG_CFR_WDGTB                      BIT_ADDR(WWDG_BASE +0x04,7)            /*!< WDGTB[1:0] bits (Timer Base) */
#define  bWWDG_CFR_WDGTB0                     BIT_ADDR(WWDG_BASE +0x04,7)            /*!< Bit 0 */
#define  bWWDG_CFR_WDGTB1                     BIT_ADDR(WWDG_BASE +0x04,8)            /*!< Bit 1 */

#define  bWWDG_CFR_EWI                        BIT_ADDR(WWDG_BASE +0x04,9)            /*!< Early Wakeup Interrupt */

/*******************    WWDG_SR 寄存器  ********************/
      
#define  bWWDG_SR_EWIF												BIT_ADDR(WWDG_BASE +0x08,0) /*!< Early Wakeup Interrupt Flag */

/******************************************************************************/
/*                                                                            */
/*                         bxCAN总线控制器                                    */
/*                                                                            */
/******************************************************************************/

/*!< CAN 控制及状态寄存器族 */
/*******************   MCR 主控制寄存器  ********************/
#define  bCAN_MCR_INRQ(CANx_BASE)  		BIT_ADDR(CANx_BASE,0) 	 /*!< Initialization Request */
#define  bCAN_MCR_SLEEP(CANx_BASE) 		BIT_ADDR(CANx_BASE,1)            /*!< Sleep Mode Request */
#define  bCAN_MCR_TXFP(CANx_BASE)     BIT_ADDR(CANx_BASE,2)            /*!< Transmit FIFO Priority */
#define  bCAN_MCR_RFLM(CANx_BASE)     BIT_ADDR(CANx_BASE,3)            /*!< Receive FIFO Locked Mode */
#define  bCAN_MCR_NART(CANx_BASE)     BIT_ADDR(CANx_BASE,4)            /*!< No Automatic Retransmission */
#define  bCAN_MCR_AWUM(CANx_BASE)     BIT_ADDR(CANx_BASE,5)            /*!< Automatic Wakeup Mode */
#define  bCAN_MCR_ABOM(CANx_BASE)     BIT_ADDR(CANx_BASE,6)            /*!< Automatic Bus-Off Management */
#define  bCAN_MCR_TTCM(CANx_BASE)     BIT_ADDR(CANx_BASE,7)            /*!< Time Triggered Communication Mode */
#define  bCAN_MCR_RESET(CANx_BASE)    BIT_ADDR(CANx_BASE,15)            /*!< CAN software master reset */

/*******************   MSR 主状态寄存器  ********************/
#define  bCAN_MSR_INAK(CANx_BASE)     BIT_ADDR(CANx_BASE+0x004,0)            /*!< Initialization Acknowledge */
#define  bCAN_MSR_SLAK(CANx_BASE)     BIT_ADDR(CANx_BASE+0x004,1)            /*!< Sleep Acknowledge */
#define  bCAN_MSR_ERRI(CANx_BASE)     BIT_ADDR(CANx_BASE+0x004,2)            /*!< Error Interrupt */
#define  bCAN_MSR_WKUI(CANx_BASE)     BIT_ADDR(CANx_BASE+0x004,3)            /*!< Wakeup Interrupt */
#define  bCAN_MSR_SLAKI(CANx_BASE)    BIT_ADDR(CANx_BASE+0x004,4)            /*!< Sleep Acknowledge Interrupt */
#define  bCAN_MSR_TXM(CANx_BASE)      BIT_ADDR(CANx_BASE+0x004,8)            /*!< Transmit Mode */
#define  bCAN_MSR_RXM(CANx_BASE)      BIT_ADDR(CANx_BASE+0x004,9)            /*!< Receive Mode */
#define  bCAN_MSR_SAMP(CANx_BASE)     BIT_ADDR(CANx_BASE+0x004,10)            /*!< Last Sample Point */
#define  bCAN_MSR_RX(CANx_BASE)       BIT_ADDR(CANx_BASE+0x004,11)            /*!< CAN Rx Signal */

/*******************   TSR 发送状态寄存器  ********************/
#define  bCAN_TSR_RQCP0(CANx_BASE)    BIT_ADDR(CANx_BASE+0x008,0)        /*!< Request Completed Mailbox0 */
#define  bCAN_TSR_TXOK0(CANx_BASE)    BIT_ADDR(CANx_BASE+0x008,1)        /*!< Transmission OK of Mailbox0 */
#define  bCAN_TSR_ALST0(CANx_BASE)    BIT_ADDR(CANx_BASE+0x008,2)        /*!< Arbitration Lost for Mailbox0 */
#define  bCAN_TSR_TERR0(CANx_BASE)    BIT_ADDR(CANx_BASE+0x008,3)        /*!< Transmission Error of Mailbox0 */
#define  bCAN_TSR_ABRQ0(CANx_BASE)    BIT_ADDR(CANx_BASE+0x008,7)        /*!< Abort Request for Mailbox0 */
#define  bCAN_TSR_RQCP1(CANx_BASE)    BIT_ADDR(CANx_BASE+0x008,8)        /*!< Request Completed Mailbox1 */
#define  bCAN_TSR_TXOK1(CANx_BASE)    BIT_ADDR(CANx_BASE+0x008,9)        /*!< Transmission OK of Mailbox1 */
#define  bCAN_TSR_ALST1(CANx_BASE)    BIT_ADDR(CANx_BASE+0x008,10)        /*!< Arbitration Lost for Mailbox1 */
#define  bCAN_TSR_TERR1(CANx_BASE)    BIT_ADDR(CANx_BASE+0x008,11)        /*!< Transmission Error of Mailbox1 */
#define  bCAN_TSR_ABRQ1(CANx_BASE)    BIT_ADDR(CANx_BASE+0x008,15)        /*!< Abort Request for Mailbox 1 */
#define  bCAN_TSR_RQCP2(CANx_BASE)    BIT_ADDR(CANx_BASE+0x008,16)        /*!< Request Completed Mailbox2 */
#define  bCAN_TSR_TXOK2(CANx_BASE)    BIT_ADDR(CANx_BASE+0x008,17)        /*!< Transmission OK of Mailbox 2 */
#define  bCAN_TSR_ALST2(CANx_BASE)    BIT_ADDR(CANx_BASE+0x008,18)        /*!< Arbitration Lost for mailbox 2 */
#define  bCAN_TSR_TERR2(CANx_BASE)    BIT_ADDR(CANx_BASE+0x008,19)        /*!< Transmission Error of Mailbox 2 */
#define  bCAN_TSR_ABRQ2(CANx_BASE)    BIT_ADDR(CANx_BASE+0x008,23)        /*!< Abort Request for Mailbox 2 */
#define  bCAN_TSR_CODE(CANx_BASE)     BIT_ADDR(CANx_BASE+0x008,24)        /*!< Mailbox Code */

#define  bCAN_TSR_TME(CANx_BASE)      BIT_ADDR(CANx_BASE+0x008,26)        /*!< TME[2:0] bits */
#define  bCAN_TSR_TME0(CANx_BASE)     BIT_ADDR(CANx_BASE+0x008,26)        /*!< Transmit Mailbox 0 Empty */
#define  bCAN_TSR_TME1(CANx_BASE)     BIT_ADDR(CANx_BASE+0x008,27)        /*!< Transmit Mailbox 1 Empty */
#define  bCAN_TSR_TME2(CANx_BASE)     BIT_ADDR(CANx_BASE+0x008,28)        /*!< Transmit Mailbox 2 Empty */

#define  bCAN_TSR_LOW(CANx_BASE)      BIT_ADDR(CANx_BASE+0x008,29)        /*!< LOW[2:0] bits */
#define  bCAN_TSR_LOW0(CANx_BASE)     BIT_ADDR(CANx_BASE+0x008,29)        /*!< Lowest Priority Flag for Mailbox 0 */
#define  bCAN_TSR_LOW1(CANx_BASE)     BIT_ADDR(CANx_BASE+0x008,30)        /*!< Lowest Priority Flag for Mailbox 1 */
#define  bCAN_TSR_LOW2(CANx_BASE)     BIT_ADDR(CANx_BASE+0x008,31)        /*!< Lowest Priority Flag for Mailbox 2 */

/*******************   RF0R 接收FIFO 0寄存器  *******************/
//#define  CAN_RF0R_FMP0(CANx_BASE)         ((uint8_t)0x03)               /*!< FIFO 0 报文数目（只读）*/
#define  GET_CAN_RF0R_FMP0(CANx_BASE)   (MEM_ADDR(CANx_BASE+0x00C)&CAN_RF0R_FMP0)
#define  bCAN_RF0R_FULL0(CANx_BASE)   BIT_ADDR(CANx_BASE+0x00C,3)               /*!< FIFO 0 Full */
#define  bCAN_RF0R_FOVR0(CANx_BASE)   BIT_ADDR(CANx_BASE+0x00C,4)               /*!< FIFO 0 Overrun */
#define  bCAN_RF0R_RFOM0(CANx_BASE)   BIT_ADDR(CANx_BASE+0x00C,5)               /*!< Release FIFO 0 Output Mailbox */

/*******************   RF1R 接收FIFO 1寄存器  *******************/
//#define  CAN_RF1R_FMP1(CANx_BASE)         ((uint8_t)0x03)               /*!< FIFO 1 报文数目（只读）*/
#define  GET_CAN_RF1R_FMP1(CANx_BASE)   (MEM_ADDR(CANx_BASE+0x010)&CAN_RF1R_FMP1)
#define  bCAN_RF1R_FULL1(CANx_BASE)   BIT_ADDR(CANx_BASE+0x010,3)               /*!< FIFO 1 Full */
#define  bCAN_RF1R_FOVR1(CANx_BASE)   BIT_ADDR(CANx_BASE+0x010,4)               /*!< FIFO 1 Overrun */
#define  bCAN_RF1R_RFOM1(CANx_BASE)   BIT_ADDR(CANx_BASE+0x010,5)               /*!< Release FIFO 1 Output Mailbox */

/********************   IER 中断使能寄存器  *******************/
#define  bCAN_IER_TMEIE(CANx_BASE)    BIT_ADDR(CANx_BASE+0x014,0)        /*!< Transmit Mailbox Empty Interrupt Enable */
#define  bCAN_IER_FMPIE0(CANx_BASE)   BIT_ADDR(CANx_BASE+0x014,1)        /*!< FIFO Message Pending Interrupt Enable */
#define  bCAN_IER_FFIE0(CANx_BASE)    BIT_ADDR(CANx_BASE+0x014,2)        /*!< FIFO Full Interrupt Enable */
#define  bCAN_IER_FOVIE0(CANx_BASE)   BIT_ADDR(CANx_BASE+0x014,3)        /*!< FIFO Overrun Interrupt Enable */
#define  bCAN_IER_FMPIE1(CANx_BASE)   BIT_ADDR(CANx_BASE+0x014,4)        /*!< FIFO Message Pending Interrupt Enable */
#define  bCAN_IER_FFIE1(CANx_BASE)    BIT_ADDR(CANx_BASE+0x014,5)        /*!< FIFO Full Interrupt Enable */
#define  bCAN_IER_FOVIE1(CANx_BASE)   BIT_ADDR(CANx_BASE+0x014,6)        /*!< FIFO Overrun Interrupt Enable */
#define  bCAN_IER_EWGIE(CANx_BASE)    BIT_ADDR(CANx_BASE+0x014,8)        /*!< Error Warning Interrupt Enable */
#define  bCAN_IER_EPVIE(CANx_BASE)    BIT_ADDR(CANx_BASE+0x014,9)        /*!< Error Passive Interrupt Enable */
#define  bCAN_IER_BOFIE(CANx_BASE)    BIT_ADDR(CANx_BASE+0x014,10)        /*!< Bus-Off Interrupt Enable */
#define  bCAN_IER_LECIE(CANx_BASE)    BIT_ADDR(CANx_BASE+0x014,11)        /*!< Last Error Code Interrupt Enable */
#define  bCAN_IER_ERRIE(CANx_BASE)    BIT_ADDR(CANx_BASE+0x014,15)        /*!< Error Interrupt Enable */
#define  bCAN_IER_WKUIE(CANx_BASE)    BIT_ADDR(CANx_BASE+0x014,16)        /*!< Wakeup Interrupt Enable */
#define  bCAN_IER_SLKIE(CANx_BASE)    BIT_ADDR(CANx_BASE+0x014,17)        /*!< Sleep Interrupt Enable */

/********************   ESR 错误状态寄存器  *******************/
#define  bCAN_ESR_EWGF(CANx_BASE)     BIT_ADDR(CANx_BASE+0x018,0)        /*!< Error Warning Flag */
#define  bCAN_ESR_EPVF(CANx_BASE)     BIT_ADDR(CANx_BASE+0x018,1)        /*!< Error Passive Flag */
#define  bCAN_ESR_BOFF(CANx_BASE)     BIT_ADDR(CANx_BASE+0x018,2)        /*!< Bus-Off Flag */
//#define  CAN_ESR_LEC	 ((uint32_t)0x00000070)        /*!< LEC[2:0] bits (Last Error Code) */
#define  SET_CAN_ESR_LEC(CANx_BASE,a)   (MEM_ADDR(CANx_BASE+0x018)=(MEM_ADDR(CANx_BASE+0x018)&(~CAN_ESR_LEC))|((a&MASKb3)<<4))
#define  GET_CAN_ESR_LEC(CANx_BASE)   ((MEM_ADDR(CANx_BASE+0x018)&CAN_ESR_LEC)>>4)
#define  bCAN_ESR_LEC_0(CANx_BASE)    BIT_ADDR(CANx_BASE+0x018,4)        /*!< Bit 0 */
#define  bCAN_ESR_LEC_1(CANx_BASE)    BIT_ADDR(CANx_BASE+0x018,5)        /*!< Bit 1 */
#define  bCAN_ESR_LEC_2(CANx_BASE)    BIT_ADDR(CANx_BASE+0x018,6)        /*!< Bit 2 */
//#define  CAN_ESR_TEC（只读） ((uint32_t)0x00FF0000)        /*!< Least significant byte of the 9-bit Transmit Error Counter */
#define  GET_CAN_ESR_TEC(CANx_BASE)   ((MEM_ADDR(CANx_BASE+0x018)&CAN_ESR_TEC)>>16)
//#define  CAN_ESR_REC(CANx_BASE)（只读） ((uint32_t)0xFF000000)        /*!< Receive Error Counter */
#define  GET_CAN_ESR_REC(CANx_BASE)   ((MEM_ADDR(CANx_BASE+0x018)&CAN_ESR_REC)>>24)

/*******************   BTR 位时序寄存器  ********************/
//#define  CAN_BTR_BRP                          ((uint32_t)0x000003FF)        /*!< Baud Rate Prescaler */
#define  SET_CAN_BTR_BRP(CANx_BASE,a)   (MEM_ADDR(CANx_BASE+0x01C)=(MEM_ADDR(CANx_BASE+0x01C)&(~CAN_BTR_BRP))|((a&MASKb10)<<0))
//#define  CAN_BTR_TS1                        ((uint32_t)0x000F0000)        /*!< Time Segment 1 */
#define  SET_CAN_BTR_TS1(CANx_BASE,a)   (MEM_ADDR(CANx_BASE+0x01C)=(MEM_ADDR(CANx_BASE+0x01C)&(~CAN_BTR_TS1))|((a&MASKb4)<<16))
//#define  CAN_BTR_TS2                         ((uint32_t)0x00700000)        /*!< Time Segment 2 */
#define  SET_CAN_BTR_TS2(CANx_BASE,a)   (MEM_ADDR(CANx_BASE+0x01C)=(MEM_ADDR(CANx_BASE+0x01C)&(~CAN_BTR_TS2))|((a&MASKb3)<<20))
//#define  CAN_BTR_SJW                         ((uint32_t)0x03000000)        /*!< Resynchronization Jump Width */
#define  SET_CAN_BTR_SJW(CANx_BASE,a)   (MEM_ADDR(CANx_BASE+0x01C)=(MEM_ADDR(CANx_BASE+0x01C)&(~CAN_BTR_SJW))|((a&MASKb2)<<24))
#define  bCAN_BTR_LBKM                  BIT_ADDR(CANx_BASE+0x01C,30)        /*!< Loop Back Mode (Debug) */
#define  bCAN_BTR_SILM                  BIT_ADDR(CANx_BASE+0x01C,31)        /*!< Silent Mode */

/*!< 邮箱寄存器族 */
/******************   TI0R 发送邮箱标识符寄存器  ********************/
#define  bCAN_TI0R_TXRQ(CANx_BASE)      BIT_ADDR(CANx_BASE+0x180,0)        /*!< Transmit Mailbox Request */
#define  bCAN_TI0R_RTR(CANx_BASE)       BIT_ADDR(CANx_BASE+0x180,1)        /*!< Remote Transmission Request */
#define  bCAN_TI0R_IDE(CANx_BASE)       BIT_ADDR(CANx_BASE+0x180,2)        /*!< Identifier Extension */
//#define  CAN_TI0R_EXID                       ((uint32_t)0x001FFFF8)        /*!< Extended Identifier */
#define  SET_CAN_TI0R_EXID(CANx_BASE,a)   (MEM_ADDR(CANx_BASE+0x180)=(MEM_ADDR(CANx_BASE+0x180)&(~CAN_TI0R_EXID))|((a&MASKb18)<<3))
//#define  CAN_TI0R_STID                       ((uint32_t)0xFFE00000)        /*!< Standard Identifier or Extended Identifier */
#define  SET_CAN_TI0R_STID(CANx_BASE,a)   (MEM_ADDR(CANx_BASE+0x180)=(MEM_ADDR(CANx_BASE+0x180)&(~CAN_TI0R_STID))|((a&MASKb11)<<21))

/******************   TDT0R 发送邮箱长度控制及时戳寄存器  *******************/
//#define  CAN_TDT0R_DLC                        ((uint32_t)0x0000000F)        /*!< Data Length Code */
#define  SET_CAN_TDT0R_DLC(CANx_BASE,a)   (MEM_ADDR(CANx_BASE+0x184)=(MEM_ADDR(CANx_BASE+0x184)&(~CAN_TDT0R_DLC))|((a&MASKb4)<<0))
#define  bCAN_TDT0R_TGT(CANx_BASE)      BIT_ADDR(CANx_BASE+0x184,8)        /*!< Transmit Global Time */
//#define  CAN_TDT0R_TIME                      ((uint32_t)0xFFFF0000)        /*!< Message Time Stamp */
#define  SET_CAN_TDT0R_TIME(CANx_BASE,a)   (MEM_ADDR(CANx_BASE+0x184)=(MEM_ADDR(CANx_BASE+0x184)&(~CAN_TDT0R_TIME))|((a&MASKb8)<<16))

/******************   TDL0R 发送邮箱数据低字节寄存器  *******************/
#define  CAN_TDL0R(CANx_BASE)           (MEM_ADDR(CANx_BASE+0x188))/*!< Data byte 0-3 */

/******************   TDH0R 发送邮箱数据高字节寄存器  *******************/
#define  CAN_TDH0R(CANx_BASE)           (MEM_ADDR(CANx_BASE+0x18C))/*!< Data byte 4-7 */

/*******************   TI1R 发送邮箱标识符寄存器  *******************/
#define  bCAN_TI1R_TXRQ(CANx_BASE)     BIT_ADDR(CANx_BASE+0x190,0)        /*!< Transmit Mailbox Request */
#define  bCAN_TI1R_RTR(CANx_BASE)      BIT_ADDR(CANx_BASE+0x190,1)        /*!< Remote Transmission Request */
#define  bCAN_TI1R_IDE(CANx_BASE)      BIT_ADDR(CANx_BASE+0x190,2)        /*!< Identifier Extension */
//#define  CAN_TI1R_EXID                       ((uint32_t)0x001FFFF8)        /*!< Extended Identifier */
#define  SET_CAN_TI1R_EXID(CANx_BASE,a)   (MEM_ADDR(CANx_BASE+0x190)=(MEM_ADDR(CANx_BASE+0x190)&(~CAN_TI1R_EXID))|((a&MASKb18)<<3))
//#define  CAN_TI1R_STID                       ((uint32_t)0xFFE00000)        /*!< Standard Identifier or Extended Identifier */
#define  SET_CAN_TI1R_STID(CANx_BASE,a)   (MEM_ADDR(CANx_BASE+0x190)=(MEM_ADDR(CANx_BASE+0x190)&(~CAN_TI1R_STID))|((a&MASKb11)<<21))

/*******************   TDT1R 发送邮箱长度控制及时戳寄存器  ******************/
//#define  CAN_TDT1R_DLC                        ((uint32_t)0x0000000F)        /*!< Data Length Code */
#define  SET_CAN_TDT1R_DLC(CANx_BASE,a)   (MEM_ADDR(CANx_BASE+0x194)=(MEM_ADDR(CANx_BASE+0x194)&(~CAN_TDT1R_DLC))|((a&MASKb4)<<0))
#define  bCAN_TDT1R_TGT                BIT_ADDR(CANx_BASE+0x194,8)        /*!< Transmit Global Time */
//#define  CAN_ TDT1R_TIME                      ((uint32_t)0xFFFF0000)        /*!< Message Time Stamp */
#define  SET_CAN_TDT1R_TIME(CANx_BASE,a)   (MEM_ADDR(CANx_BASE+0x194)=(MEM_ADDR(CANx_BASE+0x194)&(~CAN_TDT1R_TIME))|((a&MASKb8)<<16))

/*******************   TDL1R 发送邮箱数据低字节寄存器  ******************/
#define  CAN_TDL1R(CANx_BASE)           (MEM_ADDR(CANx_BASE+0x198))/*!< Data byte 0-3 */
/*******************   TDH1R 发送邮箱数据高字节寄存器  ******************/
#define  CAN_TDH1R(CANx_BASE)           (MEM_ADDR(CANx_BASE+0x19C))/*!< Data byte 4-7 */

/*******************   TI2R 发送邮箱标识符寄存器  *******************/
#define  bCAN_TI2R_TXRQ(CANx_BASE)    BIT_ADDR(CANx_BASE+0x1A0,0)        /*!< Transmit Mailbox Request */
#define  bCAN_TI2R_RTR(CANx_BASE)     BIT_ADDR(CANx_BASE+0x1A0,1)        /*!< Remote Transmission Request */
#define  bCAN_TI2R_IDE(CANx_BASE)     BIT_ADDR(CANx_BASE+0x1A0,2)        /*!< Identifier Extension */
//#define  CAN_ TI2R_EXID                       ((uint32_t)0x001FFFF8)        /*!< Extended identifier */
#define  SET_CAN_TI2R_EXID(CANx_BASE,a)   (MEM_ADDR(CANx_BASE+0x1A0)=(MEM_ADDR(CANx_BASE+0x1A0)&(~CAN_TI2R_EXID))|((a&MASKb18)<<3))
//#define  CAN_ TI2R_STID                       ((uint32_t)0xFFE00000)        /*!< Standard Identifier or Extended Identifier */
#define  SET_CAN_TI2R_STID(CANx_BASE,a)   (MEM_ADDR(CANx_BASE+0x1A0)=(MEM_ADDR(CANx_BASE+0x1A0)&(~CAN_TI2R_STID))|((a&MASKb11)<<21))

/*******************   TDT2R 发送邮箱长度控制及时戳寄存器  ******************/  
//#define  CAN_ TDT2R_DLC                        ((uint32_t)0x0000000F)        /*!< Data Length Code */
#define  SET_CAN_TDT2R_DLC(CANx_BASE,a)   (MEM_ADDR(CANx_BASE+0x1A4)=(MEM_ADDR(CANx_BASE+0x1A4)&(~CAN_TDT2R_DLC))|((a&MASKb4)<<0))
#define  bCAN_TDT2R_TGT               BIT_ADDR(CANx_BASE+0x1A4,8)        /*!< Transmit Global Time */
//#define  CAN_ TDT2R_TIME                      ((uint32_t)0xFFFF0000)        /*!< Message Time Stamp */
#define  SET_CAN_TDT2R_TIME(CANx_BASE,a)   (MEM_ADDR(CANx_BASE+0x1A4)=(MEM_ADDR(CANx_BASE+0x1A4)&(~CAN_TDT2R_TIME))|((a&MASKb8)<<16))

/*******************   TDL2R 发送邮箱数据低字节寄存器  ******************/
#define  CAN_TDL2R(CANx_BASE)           (MEM_ADDR(CANx_BASE+0x1A8))/*!< Data byte 0-3 */
/*******************   TDH2R 发送邮箱数据高字节寄存器  ******************/
#define  CAN_TDH2R(CANx_BASE)           (MEM_ADDR(CANx_BASE+0x1AC))/*!< Data byte 4-7 */

/*******************   RI0R 接收FIFO邮箱标识符寄存器  *******************/
#define  bCAN_RI0R_RTR                BIT_ADDR(CANx_BASE+0x1B0,1)        /*!< Remote Transmission Request */
#define  bCAN_RI0R_IDE                BIT_ADDR(CANx_BASE+0x1B0,2)        /*!< Identifier Extension */
//#define  CAN_RI0R_EXID                       ((uint32_t)0x001FFFF8)        /*!< Extended Identifier */
#define  GET_CAN_RI0R_EXID(CANx_BASE)   ((MEM_ADDR(CANx_BASE+0x1B0)&CAN_RI0R_EXID)>>3)
//#define  CAN_RI0R_STID                       ((uint32_t)0xFFE00000)        /*!< Standard Identifier or Extended Identifier */
#define  GET_CAN_RI0R_STID(CANx_BASE)   ((MEM_ADDR(CANx_BASE+0x1B0)&CAN_RI0R_STID)>>21)

/*******************   RDT0R 接收FIFO邮箱长度控制及时戳寄存器  ******************/
//#define  CAN_RDT0R_DLC                        ((uint32_t)0x0000000F)        /*!< Data Length Code */
#define  GET_CAN_RDT0R_DLC(CANx_BASE)   (MEM_ADDR(CANx_BASE+0x1B4)&CAN_RDT0R_DLC)
//#define  CAN_RDT0R_FMI                        ((uint32_t)0x0000FF00)        /*!< Filter Match Index */
#define  GET_CAN_RDT0R_FMI(CANx_BASE)   ((MEM_ADDR(CANx_BASE+0x1B4)&CAN_RDT0R_FMI)>>8)
//#define  CAN_RDT0R_TIME                      ((uint32_t)0xFFFF0000)        /*!< Message Time Stamp */
#define  GET_CAN_RDT0R_TIME(CANx_BASE)   ((MEM_ADDR(CANx_BASE+0x1B4)&CAN_RDT0R_TIME)>>16)

/*******************   RDL0R 接收FIFO邮箱数据低字节寄存器  ******************/
#define  CAN_RDL0R(CANx_BASE)           (MEM_ADDR(CANx_BASE+0x1B8))/*!< Data byte 0-3 （只读）*/

/*******************   RDH0R 接收FIFO邮箱数据高字节寄存器  ******************/
#define  CAN_RDH0R(CANx_BASE)           (MEM_ADDR(CANx_BASE+0x0x1BC))/*!< Data byte 4-7 （只读）*/

/*******************   RI1R 接收FIFO邮箱标识符寄存器  *******************/
#define  bCAN_RI1R_RTR(CANx_BASE)    BIT_ADDR(CANx_BASE+0x1C0,1)        /*!< Remote Transmission Request */
#define  bCAN_RI1R_IDE(CANx_BASE)    BIT_ADDR(CANx_BASE+0x1C0,2)        /*!< Identifier Extension */
//#define  CAN_RI1R_EXID                       ((uint32_t)0x001FFFF8)        /*!< Extended identifier */
#define  GET_CAN_RI1R_EXID(CANx_BASE,a)   ((MEM_ADDR(CANx_BASE+0x1C0)&CAN_RI1R_EXID)>>3)
//#define  CAN_RI1R_STID                       ((uint32_t)0xFFE00000)        /*!< Standard Identifier or Extended Identifier */
#define  GET_CAN_RI1R_STID(CANx_BASE)   ((MEM_ADDR(CANx_BASE+0x1C0)&CAN_RI1R_STID)>>21)

/*******************   RDT1R 接收FIFO邮箱长度控制及时戳寄存器  ******************/
//#define  CAN_RDT1R_DLC                        ((uint32_t)0x0000000F)        /*!< Data Length Code */
#define  GET_CAN_RDT1R_DLC(CANx_BASE)   (MEM_ADDR(CANx_BASE+0x1C4)&CAN_RDT1R_DLC)
//#define  CAN_RDT1R_FMI                        ((uint32_t)0x0000FF00)        /*!< Filter Match Index */
#define  GET_CAN_RDT1R_FMI(CANx_BASE)   ((MEM_ADDR(CANx_BASE+0x1C4)&CAN_RDT1R_FMI)>>8)
//#define  CAN_RDT1R_TIME                      ((uint32_t)0xFFFF0000)        /*!< Message Time Stamp */
#define  GET_CAN_RDT1R_TIME(CANx_BASE)   ((MEM_ADDR(CANx_BASE+0x1C4)&CAN_RDT1R_TIME)>>16)

/*******************   RDL1R 接收FIFO邮箱数据低字节寄存器  ******************/
#define  CAN_RDL1R(CANx_BASE)           (MEM_ADDR(CANx_BASE+0x1C8))/*!< Data byte 0-3 （只读）*/

/*******************   RDH1R 接收FIFO邮箱数据高字节寄存器  ******************/
#define  CAN_RDH1R(CANx_BASE)           (MEM_ADDR(CANx_BASE+0x1CC))/*!< Data byte 4-7（只读） */

/*!< CAN过滤器族 */
/*******************   FMR 过滤器主寄存器  ********************/
#define  bCAN_FMR_FINIT(CANx_BASE)    BIT_ADDR(CANx_BASE+0x200,0)             /*!< Filter Init Mode */

/*******************   FM1R 过滤器模式寄存器  *******************/
//#define  CAN_FM1R_FBM                        ((uint16_t)0x3FFF)            /*!< Filter Mode */
#define  bCAN_FM1R_FBM(CANx_BASE,n)   BIT_ADDR(CANx_BASE+0x204,n)


/*******************   FS1R 过滤器比例寄存器  *******************/
//#define  CAN_FS1R_FSC                        ((uint16_t)0x3FFF)            /*!< Filter Scale Configuration */
#define  bCAN_FS1R_FSC(CANx_BASE,n)    BIT_ADDR(CANx_BASE+0x20C,n)

/******************   FFA1R 过滤器FIFO分配寄存器  *******************/
//#define  CAN_FFA1R_FFA                       ((uint16_t)0x3FFF)            /*!< Filter FIFO Assignment */
#define  bCAN_FFA1R_FFA(CANx_BASE,n)   BIT_ADDR(CANx_BASE+0x214,n)


/*******************   FA1R 过滤器激活寄存器  *******************/
//#define  CAN_ FA1R_FACT                       ((uint16_t)0x3FFF)            /*!< Filter Active */
#define  bCAN_FA1R_FACT(CANx_BASE,n)   BIT_ADDR(CANx_BASE+0x21C,n)


/*******************   F0R1 过滤器寄存器组0  *******************/
#define  bCAN_F0R1_FB(CANx_BASE,n)     BIT_ADDR(CANx_BASE+0x240,n)/*!< Filter bit 0-31 */
#define  bCAN_F0R2_FB(CANx_BASE,n)     BIT_ADDR(CANx_BASE+0x244,n)/*!< Filter bit 0-31 */

/*******************   F1R1 过滤器寄存器组1  *******************/
#define  bCAN_F1R1_FB(CANx_BASE,n)     BIT_ADDR(CANx_BASE+0x248,n)/*!< Filter bit 0-31 */
#define  bCAN_F1R2_FB(CANx_BASE,n)     BIT_ADDR(CANx_BASE+0x24C,n)/*!< Filter bit 0-31 */

/*******************   F2R1 过滤器寄存器组2  *******************/
#define  bCAN_F2R1_FB(CANx_BASE,n)     BIT_ADDR(CANx_BASE+0x250,n)/*!< Filter bit 0-31 */
#define  bCAN_F2R2_FB(CANx_BASE,n)     BIT_ADDR(CANx_BASE+0x254,n)/*!< Filter bit 0-31 */

/*******************   F3R1 过滤器寄存器组3  *******************/
#define  bCAN_F3R1_FB(CANx_BASE,n)     BIT_ADDR(CANx_BASE+0x258,n)/*!< Filter bit 0-31 */
#define  bCAN_F3R2_FB(CANx_BASE,n)     BIT_ADDR(CANx_BASE+0x25C,n)/*!< Filter bit 0-31 */

/*******************   F4R1 过滤器寄存器组4  *******************/
#define  bCAN_F4R1_FB(CANx_BASE,n)     BIT_ADDR(CANx_BASE+0x260,n)/*!< Filter bit 0-31 */
#define  bCAN_F4R2_FB(CANx_BASE,n)     BIT_ADDR(CANx_BASE+0x264,n)/*!< Filter bit 0-31 */

/*******************   F5R1 过滤器寄存器组5  *******************/
#define  bCAN_F5R1_FB(CANx_BASE,n)     BIT_ADDR(CANx_BASE+0x268,n)/*!< Filter bit 0-31 */
#define  bCAN_F5R2_FB(CANx_BASE,n)     BIT_ADDR(CANx_BASE+0x26C,n)/*!< Filter bit 0-31 */

/*******************   F6R1 过滤器寄存器组6  *******************/
#define  bCAN_F6R1_FB(CANx_BASE,n)     BIT_ADDR(CANx_BASE+0x270,n)/*!< Filter bit 0-31 */
#define  bCAN_F6R2_FB(CANx_BASE,n)     BIT_ADDR(CANx_BASE+0x274,n)/*!< Filter bit 0-31 */

/*******************   F7R1 过滤器寄存器组7  *******************/
#define  bCAN_F7R1_FB(CANx_BASE,n)     BIT_ADDR(CANx_BASE+0x278,n)/*!< Filter bit 0-31 */
#define  bCAN_F7R2_FB(CANx_BASE,n)     BIT_ADDR(CANx_BASE+0x27C,n)/*!< Filter bit 0-31 */

/*******************   F8R1 过滤器寄存器组8  *******************/
#define  bCAN_F8R1_FB(CANx_BASE,n)     BIT_ADDR(CANx_BASE+0x280,n)/*!< Filter bit 0-31 */
#define  bCAN_F8R2_FB(CANx_BASE,n)     BIT_ADDR(CANx_BASE+0x284,n)/*!< Filter bit 0-31 */

/*******************   F9R1 过滤器寄存器组9  *******************/
#define  bCAN_F9R1_FB(CANx_BASE,n)     BIT_ADDR(CANx_BASE+0x288,n)/*!< Filter bit 0-31 */
#define  bCAN_F9R2_FB(CANx_BASE,n)     BIT_ADDR(CANx_BASE+0x28C,n)/*!< Filter bit 0-31 */

/*******************   F10R1 过滤器寄存器组10  ******************/
#define  bCAN_F10R1_FB(CANx_BASE,n)    BIT_ADDR(CANx_BASE+0x290,n)/*!< Filter bit 0-31 */
#define  bCAN_F10R2_FB(CANx_BASE,n)    BIT_ADDR(CANx_BASE+0x294,n)/*!< Filter bit 0-31 */

/*******************   F11R1 过滤器寄存器组11  ******************/
#define  bCAN_F11R1_FB(CANx_BASE,n)    BIT_ADDR(CANx_BASE+0x298,n)/*!< Filter bit 0-31 */
#define  bCAN_F11R2_FB(CANx_BASE,n)    BIT_ADDR(CANx_BASE+0x29C,n)/*!< Filter bit 0-31 */

/*******************   F12R1 过滤器寄存器组12  ******************/
#define  bCAN_F12R1_FB(CANx_BASE,n)    BIT_ADDR(CANx_BASE+0x2A0,n)/*!< Filter bit 0-31 */
#define  bCAN_F12R2_FB(CANx_BASE,n)    BIT_ADDR(CANx_BASE+0x2A4,n)/*!< Filter bit 0-31 */

/*******************   F13R1 过滤器寄存器组13  ******************/
#define  bCAN_F13R1_FB(CANx_BASE,n)    BIT_ADDR(CANx_BASE+0x2A8,n)/*!< Filter bit 0-31 */
#define  bCAN_F13R2_FB(CANx_BASE,n)    BIT_ADDR(CANx_BASE+0x2AC,n)/*!< Filter bit 0-31 */

/******************************************************************************/
/*                                                                            */
/*                      I2C接口                                               */
/*                                                                            */
/******************************************************************************/

/*******************  I2C_CR1 控制寄存器  ********************/
#define  bI2C_CR_PE(I2Cx_BASE)            BIT_ADDR(I2Cx_BASE +0x00,0)            /*!< Peripheral Enable */
#define  bI2C_CR_SMBUS(I2Cx_BASE )         BIT_ADDR(I2Cx_BASE +0x00,1)            /*!< SMBus Mode */
#define  bI2C_CR_SMBTYPE(I2Cx_BASE )       BIT_ADDR(I2Cx_BASE +0x00,3)            /*!< SMBus Type */
#define  bI2C_CR_ENARP(I2Cx_BASE )         BIT_ADDR(I2Cx_BASE +0x00,4)            /*!< ARP Enable */
#define  bI2C_CR_ENPEC(I2Cx_BASE )         BIT_ADDR(I2Cx_BASE +0x00,5)            /*!< PEC Enable */
#define  bI2C_CR_ENGC(I2Cx_BASE )          BIT_ADDR(I2Cx_BASE +0x00,6)            /*!< General Call Enable */
#define  bI2C_CR_NOSTRETCH(I2Cx_BASE )     BIT_ADDR(I2Cx_BASE +0x00,7)            /*!< Clock Stretching Disable (Slave mode) */
#define  bI2C_CR_START(I2Cx_BASE )         BIT_ADDR(I2Cx_BASE +0x00,8)            /*!< Start Generation */
#define  bI2C_CR_STOP(I2Cx_BASE )          BIT_ADDR(I2Cx_BASE +0x00,9)            /*!< Stop Generation */
#define  bI2C_CR_ACK(I2Cx_BASE )           BIT_ADDR(I2Cx_BASE +0x00,10)            /*!< Acknowledge Enable */
#define  bI2C_CR_POS(I2Cx_BASE )           BIT_ADDR(I2Cx_BASE +0x00,11)            /*!< Acknowledge/PEC Position (for data reception) */
#define  bI2C_CR_PEC(I2Cx_BASE )           BIT_ADDR(I2Cx_BASE +0x00,12)            /*!< Packet Error Checking */
#define  bI2C_CR_ALERT(I2Cx_BASE )         BIT_ADDR(I2Cx_BASE +0x00,13)            /*!< SMBus Alert */
#define  bI2C_CR_SWRST(I2Cx_BASE )         BIT_ADDR(I2Cx_BASE +0x00,15)            /*!< Software Reset */

/*******************  I2C_CR2 控制寄存器  ********************/
//#define  I2C_CR_FREQ(I2Cx_BASE )   ((uint16_t)0x003F)            /*!< FREQ[5:0] bits (Peripheral Clock Frequency) */
#define  SET_I2C_CR_FREQ(CANx_BASE,a)   (MEM_ADDR(I2Cx_BASE +0x04)=(MEM_ADDR(I2Cx_BASE +0x04)&(~I2C_CR_FREQ))|(a&MASKb6))

#define  bI2C_CR_ITERREN(I2Cx_BASE )       BIT_ADDR(I2Cx_BASE +0x04,8)            /*!< Error Interrupt Enable */
#define  bI2C_CR_ITEVTEN(I2Cx_BASE )       BIT_ADDR(I2Cx_BASE +0x04,9)            /*!< Event Interrupt Enable */
#define  bI2C_CR_ITBUFEN(I2Cx_BASE )       BIT_ADDR(I2Cx_BASE +0x04,10)            /*!< Buffer Interrupt Enable */
#define  bI2C_CR_DMAEN(I2Cx_BASE )         BIT_ADDR(I2Cx_BASE +0x04,11)            /*!< DMA Requests Enable */
#define  bI2C_CR_LAST(I2Cx_BASE )          BIT_ADDR(I2Cx_BASE +0x04,12)            /*!< DMA Last Transfer */

/*******************  I2C_OAR1 自身地址寄存器  *******************/
//#define  I2C_OAR1_ADD1_7                     ((uint16_t)0x00FE)            /*!< Interface Address */
#define  SET_I2C_OAR_ADD1_7(I2Cx_BASE ,a)   (MEM_ADDR(I2Cx_BASE +0x08)=(MEM_ADDR(I2Cx_BASE +0x08)&(~I2C_OAR1_ADD1_7))|((a&MASKb7)<<1))

//#define  I2C_OAR_ADD8_9                     ((uint16_t)0x0300)            /*!< Interface Address */
#define  SET_I2C_OAR_ADD8_9(I2Cx_BASE ,a)   (MEM_ADDR(I2Cx_BASE +0x08)=(MEM_ADDR(I2Cx_BASE +0x08)&(~I2C_OAR1_ADD8_9))|((a&MASKb2)<<8))

#define  bI2C_OAR_ADD0                       BIT_ADDR(I2Cx_BASE +0x08,0)            /*!< Bit 0 */
#define  bI2C_OAR_ADDMODE(I2Cx_BASE )        BIT_ADDR(I2Cx_BASE +0x08,15)            /*!< Addressing Mode (Slave mode) */

/*******************  I2C_OAR2 自身地址寄存器  *******************/
#define  bI2C_OAR_ENDUAL(I2Cx_BASE )         BIT_ADDR(I2Cx_BASE +0x0C,0)               /*!< Dual addressing mode enable */
//#define  I2C_OAR2_ADD2                       ((uint8_t)0xFE)               /*!< Interface address */
#define  SET_I2C_OAR_ADD2(I2Cx_BASE ,a)   (MEM_ADDR(I2Cx_BASE +0x0C)=(MEM_ADDR(I2Cx_BASE +0x0C)&(~I2C_OAR2_ADD2))|((a&MASKb7)<<1))

/********************  I2C_DR 数据寄存器  ********************/
//#define  I2C_DR_DR                           ((uint8_t)0xFF)               /*!< 8-bit Data Register */
#define  I2C_DR(I2Cx_BASE)   				(MEM_ADDR(I2Cx_BASE +0x10)&I2C_DR_DR)
/*******************  I2C_SR1 状态寄存器  ********************/
#define  bI2C_SR_SB(I2Cx_BASE )                          BIT_ADDR(I2Cx_BASE +0x14,0)            /*!< Start Bit (Master mode) */
#define  bI2C_SR_ADDR(I2Cx_BASE )                        BIT_ADDR(I2Cx_BASE +0x14,1)            /*!< Address sent (master mode)/matched (slave mode) */
#define  bI2C_SR_BTF(I2Cx_BASE )                         BIT_ADDR(I2Cx_BASE +0x14,2)            /*!< Byte Transfer Finished */
#define  bI2C_SR_ADD10(I2Cx_BASE )                       BIT_ADDR(I2Cx_BASE +0x14,3)            /*!< 10-bit header sent (Master mode) */
#define  bI2C_SR_STOPF(I2Cx_BASE )                       BIT_ADDR(I2Cx_BASE +0x14,4)            /*!< Stop detection (Slave mode) */
#define  bI2C_SR_RXNE(I2Cx_BASE )                        BIT_ADDR(I2Cx_BASE +0x14,6)            /*!< Data Register not Empty (receivers) */
#define  bI2C_SR_TXE(I2Cx_BASE )                         BIT_ADDR(I2Cx_BASE +0x14,7)            /*!< Data Register Empty (transmitters) */
#define  bI2C_SR_BERR(I2Cx_BASE )                        BIT_ADDR(I2Cx_BASE +0x14,8)            /*!< Bus Error */
#define  bI2C_SR_ARLO(I2Cx_BASE )                        BIT_ADDR(I2Cx_BASE +0x14,9)            /*!< Arbitration Lost (master mode) */
#define  bI2C_SR_AF(I2Cx_BASE )                          BIT_ADDR(I2Cx_BASE +0x14,10)            /*!< Acknowledge Failure */
#define  bI2C_SR_OVR(I2Cx_BASE )                         BIT_ADDR(I2Cx_BASE +0x14,11)            /*!< Overrun/Underrun */
#define  bI2C_SR_PECERR(I2Cx_BASE )                      BIT_ADDR(I2Cx_BASE +0x14,12)            /*!< PEC Error in reception */
#define  bI2C_SR_TIMEOUT(I2Cx_BASE )                     BIT_ADDR(I2Cx_BASE +0x14,14)            /*!< Timeout or Tlow Error */
#define  bI2C_SR_SMBALERT(I2Cx_BASE )                    BIT_ADDR(I2Cx_BASE +0x14,15)            /*!< SMBus Alert */

/*******************  I2C_SR2 状态寄存器（只读）  ********************/
#define  bI2C_SR_MSL(I2Cx_BASE )                         BIT_ADDR(I2Cx_BASE +0x18,0)            /*!< Master/Slave */
#define  bI2C_SR_BUSY(I2Cx_BASE )                        BIT_ADDR(I2Cx_BASE +0x18,1)            /*!< Bus Busy */
#define  bI2C_SR_TRA(I2Cx_BASE )                         BIT_ADDR(I2Cx_BASE +0x18,2)            /*!< Transmitter/Receiver */
#define  bI2C_SR_GENCALL(I2Cx_BASE )                     BIT_ADDR(I2Cx_BASE +0x18,4)            /*!< General Call Address (Slave mode) */
#define  bI2C_SR_SMBDEFAULT(I2Cx_BASE )                  BIT_ADDR(I2Cx_BASE +0x18,5)            /*!< SMBus Device Default Address (Slave mode) */
#define  bI2C_SR_SMBHOST(I2Cx_BASE )                     BIT_ADDR(I2Cx_BASE +0x18,6)            /*!< SMBus Host Header (Slave mode) */
#define  bI2C_SR_DUALF(I2Cx_BASE )                       BIT_ADDR(I2Cx_BASE +0x18,7)            /*!< Dual Flag (Slave mode) */
//#define  I2C_SR_PEC          ((uint16_t)0xFF00)            /*!< 包错误校验寄存器（只读） */
#define  GET_I2C_SR_PEC(I2Cx_BASE )  				((MEM_ADDR(I2Cx_BASE +0x18)&I2C_SR_PEC)>>8)
/*******************  I2C_CCR 时钟控制寄存器  ********************/
//#define  I2C_CCR_CCR                         ((uint16_t)0x0FFF)            /*!< Clock Control Register in Fast/Standard mode (Master mode) */
#define  SET_I2C_CCR_CCR(I2Cx_BASE ,a)   (MEM_ADDR(I2Cx_BASE +0x1C)=(MEM_ADDR(I2Cx_BASE +0x1C)&(~I2C_CCR_CCR))|(a&MASKb12))
#define  bI2C_CCR_DUTY(I2Cx_BASE )                        BIT_ADDR(I2Cx_BASE +0x1C,14)            /*!< Fast Mode Duty Cycle */
#define  bI2C_CCR_FS(I2Cx_BASE )                          BIT_ADDR(I2Cx_BASE +0x00,15)            /*!< I2C Master Mode Selection */

/******************  I2C_TRISE寄存器  *******************/
//#define  I2C_TRISE_TRISE                     ((uint8_t)0x3F)               /*!< Maximum Rise Time in Fast/Standard mode (Master mode) */
#define  SET_I2C_TRISE_TRISE(I2Cx_BASE ,a)   (MEM_ADDR(I2Cx_BASE +0x20)=(MEM_ADDR(I2Cx_BASE +0x20)&(~I2C_TRISE_TRISE))|(a&MASKb6))

/******************************************************************************/
/*                                                                            */
/*                             DMA 控制器                                     */
/*                                                                            */
/******************************************************************************/
//每个DMA控制器外设实际上是由最大7个相互独立的DMA通道组成
//这里用宏参数n表示DMA通道号，n=1-7
/*******************   DMA_ISR 寄存器 ********************/
#define  bDMA_ISR_GIF(DMAx_BASE,n)	BIT_ADDR(DMAx_BASE+0x00,n*4)     /*!< 通道n 全局中断标志 */
#define  bDMA_ISR_TCIF(DMAx_BASE,n)	BIT_ADDR(DMAx_BASE+0x00,n*4+1)   /*!< 通道n 传输完成中断标志 */
#define  bDMA_ISR_HTIF(DMAx_BASE,n)	BIT_ADDR(DMAx_BASE+0x00,n*4+2)    /*!< 通道n 传输到半中断标志 */
#define  bDMA_ISR_TEIF(DMAx_BASE,n)	BIT_ADDR(DMAx_BASE+0x00,n*4+3)    /*!< 通道n 传输错误中断标志 */

/*******************   DMA_IFCR 寄存器 *******************/
#define  bDMA_IFCR_CGIF(DMAx_BASE,n)	BIT_ADDR(DMAx_BASE+0x04,ch*4)     /*!< 通道n 全局中断标志清除 */
#define  bDMA_IFCR_CTCIF(DMAx_BASE,n)	BIT_ADDR(DMAx_BASE+0x04,ch*4+1)   /*!< 通道n 传输完成中断标志清除 */
#define  bDMA_IFCR_CHTIF(DMAx_BASE,n)	BIT_ADDR(DMAx_BASE+0x04,ch*4+2)    /*!< 通道n 传输到半中断标志清除 */
#define  bDMA_IFCR_CTEIF(DMAx_BASE,n)	BIT_ADDR(DMAx_BASE+0x04,ch*4+3)    /*!< 通道n 传输错误中断标志清除 */

/*******************   DMA_CCRx 寄存器 *******************/
#define  bDMA_CCRx_EN(DMAx_BASE,n)      BIT_ADDR(DMAx_BASE+0x08+n*20,0)     /*!< 通道n 使能 */
#define  bDMA_CCRx_TCIE(DMAx_BASE,n)    BIT_ADDR(DMAx_BASE+0x08+n*20,1)     /*!< 通道n 传输完成中断使能 */
#define  bDMA_CCRx_HTIE(DMAx_BASE,n)    BIT_ADDR(DMAx_BASE+0x08+n*20,2)     /*!< 通道n 传输到半中断使能 */
#define  bDMA_CCRx_TEIE(DMAx_BASE,n)    BIT_ADDR(DMAx_BASE+0x08+n*20,3)     /*!< 通道n 传输错误中断使能 */
#define  bDMA_CCRx_DIR(DMAx_BASE,n)     BIT_ADDR(DMAx_BASE+0x08+n*20,4)     /*!< 通道n 数据传输方向 */
#define  bDMA_CCRx_CIRC(DMAx_BASE,n)    BIT_ADDR(DMAx_BASE+0x08+n*20,5)    /*!< 通道n 循环模式 */
#define  bDMA_CCRx_PINC(DMAx_BASE,n)    BIT_ADDR(DMAx_BASE+0x08+n*20,6)    /*!< 通道n 外设地址自增模式 */
#define  bDMA_CCRx_MINC(DMAx_BASE,n)    BIT_ADDR(DMAx_BASE+0x08+n*20,7)    /*!< 通道n 存储器地址自增模式 */

#define  SET_DMA_CCRx_PSIZE(DMAx_BASE,n,a)   (MEM_ADDR(DMAx_BASE+0x08+n*20)=(MEM_ADDR(DMAx_BASE+0x08+n*20)&(~(MASKb2<<8)))|((a&MASKb2)<<8))
/*!< PSIZE[1:0] bits (通道n 外设数据宽度00: 8位；01: 16位；10: 32位；) */

#define  SET_DMA_CCRx_MSIZE(DMAx_BASE,n,a)   (MEM_ADDR(DMAx_BASE+0x08+n*20)=(MEM_ADDR(DMAx_BASE+0x08+n*20)&(~(MASKb2<<10)))|((a&MASKb2)<<10))
/*!< MSIZE[1:0] bits (通道n 存储器数据宽度00: 8位；01: 16位；10: 32位；) */

#define  SET_DMA_CCRx_PL(DMAx_BASE,n,a)   (MEM_ADDR(DMAx_BASE+0x08+n*20)=(MEM_ADDR(DMAx_BASE+0x08+n*20)&(~(MASKb2<<12)))|((a&MASKb2)<<12))
/*!< PL[1:0] bits(通道n 优先级00低、01中、10高、11最高；) */

#define  bDMA_CCRx_MEM2MEM(DMAx_BASE,n) BIT_ADDR(DMAx_BASE+0x08+n*20,13)    /*!< 通道n 存储器到存储器模式 */

/******************   DMA_CNDTRx 寄存器 ******************/
#define  DMA_CNDTRx_NDT(DMAx_BASE,n)    (MEM_ADDR(DMAx_BASE+0x0C+n*20)      /*!< 通道n 传输数据计数 */


/******************   DMA_CPARx 寄存器 *******************/
#define  DMA_CPARx_PA(DMAx_BASE,n)    (MEM_ADDR(DMAx_BASE+0x10+n*20)        /*!< 通道n 外设地址寄存器32位 */


/******************   DMA_CMARx 寄存器 *******************/
#define  DMA_CMARx_MA(DMAx_BASE,n)    (MEM_ADDR(DMAx_BASE+0x14+n*20)        /*!< 通道n 存储器地址寄存器32位 */


/******************************************************************************/
/*                                                                            */
/*                        ADC模数转换器                                       */
/*                                                                            */
/******************************************************************************/

/********************   ADC_SR 寄存器 ********************/
#define  bADC_SR_AWD(ADCx_BASE)      BIT_ADDR(ADCx_BASE+0x00,0)               /*!< 模拟狗标志位 */
#define  bADC_SR_EOC(ADCx_BASE)      BIT_ADDR(ADCx_BASE+0x00,1)               /*!< 转换结束（EOC）*/
#define  bADC_SR_JEOC(ADCx_BASE)     BIT_ADDR(ADCx_BASE+0x00,2)               /*!< 注入通道转换结束 */
#define  bADC_SR_JSTRT(ADCx_BASE)    BIT_ADDR(ADCx_BASE+0x00,3)               /*!< 注入通道启动标志 */
#define  bADC_SR_STRT(ADCx_BASE)     BIT_ADDR(ADCx_BASE+0x00,4)               /*!< 规则通道启动标志 */

/*******************   ADC_CR1 寄存器 ********************/
//#define  ADC_CR1_AWDCH                       ((uint32_t)0x0000001F)        /*!< AWDCH[4:0] bits (模拟狗监测通道选择) */
#define  SET_ADC_CR_AWDCH(ADCx_BASE ,a)   (MEM_ADDR(ADCx_BASE +0x04)=(MEM_ADDR(ADCx_BASE +0x04)&(~ADC_CR1_AWDCH))|((a&MASKb5)<<0))

#define  bADC_CR_EOCIE(ADCx_BASE)     BIT_ADDR(ADCx_BASE+0x04,5)        /*!< 转换结束（EOC）中断使能 */
#define  bADC_CR_AWDIE(ADCx_BASE)     BIT_ADDR(ADCx_BASE+0x04,6)        /*!< 模拟狗中断使能 */
#define  bADC_CR_JEOCIE(ADCx_BASE)    BIT_ADDR(ADCx_BASE+0x04,7)        /*!< 注入通道中断使能 */
#define  bADC_CR_SCAN(ADCx_BASE)      BIT_ADDR(ADCx_BASE+0x04,8)        /*!< 扫描模式 */
#define  bADC_CR_AWDSGL(ADCx_BASE)    BIT_ADDR(ADCx_BASE+0x04,9)        /*!< Enable the watchdog on a single channel in scan mode */
#define  bADC_CR_JAUTO(ADCx_BASE)     BIT_ADDR(ADCx_BASE+0x04,10)        /*!< Automatic injected group conversion */
#define  bADC_CR_DISCEN(ADCx_BASE)    BIT_ADDR(ADCx_BASE+0x04,11)        /*!< Discontinuous mode on regular channels */
#define  bADC_CR_JDISCEN(ADCx_BASE)   BIT_ADDR(ADCx_BASE+0x04,12)        /*!< Discontinuous mode on injected channels */

//#define  ADC_CR1_DISCNUM                     ((uint32_t)0x0000E000)        /*!< DISCNUM[2:0] bits (间断模式通道计数) */
#define  SET_ADC_CR_DISCNUM(ADCx_BASE ,a)   (MEM_ADDR(ADCx_BASE +0x04)=(MEM_ADDR(ADCx_BASE +0x04)&(~ADC_CR1_DISCNUM))|((a&MASKb3)<<13))

//#define  ADC_CR1_DUALMOD                     ((uint32_t)0x000F0000)        /*!< DUALMOD[3:0] bits (双ADC模式选择) */
#define  SET_ADC_CR_DUALMOD(ADCx_BASE ,a)   (MEM_ADDR(ADCx_BASE +0x04)=(MEM_ADDR(ADCx_BASE +0x04)&(~ADC_CR1_DUALMOD))|((a&MASKb4)<<16))

#define  bADC_CR_JAWDEN(ADCx_BASE)     BIT_ADDR(ADCx_BASE+0x04,22)        /*!< Analog watchdog enable on injected channels */
#define  bADC_CR_AWDEN(ADCx_BASE)      BIT_ADDR(ADCx_BASE+0x04,23)        /*!< Analog watchdog enable on regular channels */

  
/*******************   ADC_CR2 寄存器 ********************/
#define  bADC_CR_ADON(ADCx_BASE)     BIT_ADDR(ADCx_BASE+0x08,0)        /*!< A/D Converter ON / OFF */
#define  bADC_CR_CONT(ADCx_BASE)     BIT_ADDR(ADCx_BASE+0x08,1)        /*!< Continuous Conversion */
#define  bADC_CR_CAL(ADCx_BASE)     BIT_ADDR(ADCx_BASE+0x08,2)        /*!< A/D Calibration */
#define  bADC_CR_RSTCAL(ADCx_BASE)     BIT_ADDR(ADCx_BASE+0x08,3)        /*!< Reset Calibration */
#define  bADC_CR_DMA(ADCx_BASE)     BIT_ADDR(ADCx_BASE+0x08,8)        /*!< Direct Memory access mode */
#define  bADC_CR_ALIGN(ADCx_BASE)     BIT_ADDR(ADCx_BASE+0x08,11)        /*!< Data Alignment */

//#define  ADC_CR2_JEXTSEL                     ((uint32_t)0x00007000)        /*!< JEXTSEL[2:0] bits (External event select for injected group) */
#define  SET_ADC_CR_JEXTSEL(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x08)=(MEM_ADDR(ADCx_BASE +0x08)&(~ADC_CR2_JEXTSEL))|((a&MASKb3)<<12))

#define  bADC_CR_JEXTTRIG(ADCx_BASE)     BIT_ADDR(ADCx_BASE+0x08,15)        /*!< External Trigger Conversion mode for injected channels */

#define  ADC_CR2_EXTSEL                      ((uint32_t)0x000E0000)        /*!< EXTSEL[2:0] bits (External Event Select for regular group) */
#define  SET_ADC_CR_EXTSEL(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x08)=(MEM_ADDR(ADCx_BASE +0x08)&(~ADC_CR2_EXTSEL))|((a&MASKb3)<<17))

#define  bADC_CR_EXTTRIG(ADCx_BASE)     BIT_ADDR(ADCx_BASE+0x08,20)        /*!< External Trigger Conversion mode for regular channels */
#define  bADC_CR_JSWSTART(ADCx_BASE)    BIT_ADDR(ADCx_BASE+0x08,21)        /*!< Start Conversion of injected channels */
#define  bADC_CR_SWSTART(ADCx_BASE)     BIT_ADDR(ADCx_BASE+0x08,22)        /*!< Start Conversion of regular channels */
#define  bADC_CR_TSVREFE(ADCx_BASE)     BIT_ADDR(ADCx_BASE+0x08,23)        /*!< Temperature Sensor and VREFINT Enable */

/******************   ADC_SMPR1/2 寄存器 *******************/
/* 3bits (18个通道均可独立设置采样时间间隔) */
#define  SET_ADC_SMP0(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x10)=(MEM_ADDR(ADCx_BASE +0x10)&(~ADC_SMPR2_SMP0))|((a&MASKb3)<<0))
#define  SET_ADC_SMP1(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x10)=(MEM_ADDR(ADCx_BASE +0x10)&(~ADC_SMPR2_SMP1))|((a&MASKb3)<<3))
#define  SET_ADC_SMP2(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x10)=(MEM_ADDR(ADCx_BASE +0x10)&(~ADC_SMPR2_SMP2))|((a&MASKb3)<<6))
#define  SET_ADC_SMP3(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x10)=(MEM_ADDR(ADCx_BASE +0x10)&(~ADC_SMPR2_SMP3))|((a&MASKb3)<<9))
#define  SET_ADC_SMP4(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x10)=(MEM_ADDR(ADCx_BASE +0x10)&(~ADC_SMPR2_SMP4))|((a&MASKb3)<<12))
#define  SET_ADC_SMP5(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x10)=(MEM_ADDR(ADCx_BASE +0x10)&(~ADC_SMPR2_SMP5))|((a&MASKb3)<<15))
#define  SET_ADC_SMP6(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x10)=(MEM_ADDR(ADCx_BASE +0x10)&(~ADC_SMPR2_SMP6))|((a&MASKb3)<<18))
#define  SET_ADC_SMP7(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x10)=(MEM_ADDR(ADCx_BASE +0x10)&(~ADC_SMPR2_SMP7))|((a&MASKb3)<<21))
#define  SET_ADC_SMP8(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x10)=(MEM_ADDR(ADCx_BASE +0x10)&(~ADC_SMPR2_SMP8))|((a&MASKb3)<<24))
#define  SET_ADC_SMP9(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x10)=(MEM_ADDR(ADCx_BASE +0x10)&(~ADC_SMPR2_SMP9))|((a&MASKb3)<<27))
#define  SET_ADC_SMP10(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x0C)=(MEM_ADDR(ADCx_BASE +0x0C)&(~ADC_SMPR1_SMP10))|((a&MASKb3)<<0))
#define  SET_ADC_SMP11(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x0C)=(MEM_ADDR(ADCx_BASE +0x0C)&(~ADC_SMPR1_SMP11))|((a&MASKb3)<<3))
#define  SET_ADC_SMP12(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x0C)=(MEM_ADDR(ADCx_BASE +0x0C)&(~ADC_SMPR1_SMP12))|((a&MASKb3)<<6))
#define  SET_ADC_SMP13(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x0C)=(MEM_ADDR(ADCx_BASE +0x0C)&(~ADC_SMPR1_SMP13))|((a&MASKb3)<<9))
#define  SET_ADC_SMP14(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x0C)=(MEM_ADDR(ADCx_BASE +0x0C)&(~ADC_SMPR1_SMP14))|((a&MASKb3)<<12))
#define  SET_ADC_SMP15(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x0C)=(MEM_ADDR(ADCx_BASE +0x0C)&(~ADC_SMPR1_SMP15))|((a&MASKb3)<<15))
#define  SET_ADC_SMP16(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x0C)=(MEM_ADDR(ADCx_BASE +0x0C)&(~ADC_SMPR1_SMP16))|((a&MASKb3)<<18))
#define  SET_ADC_SMP17(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x0C)=(MEM_ADDR(ADCx_BASE +0x0C)&(~ADC_SMPR1_SMP17))|((a&MASKb3)<<21))

/******************   ADC_JOFR1 寄存器 *******************/
#define  ADC_JOFFSET1(ADCx_BASE)    (MEM_ADDR(ADCx_BASE +0x14))   /*!< 注入通道1 数据偏移量 */

/******************   ADC_JOFR2 寄存器 *******************/
#define  ADC_JOFFSET2(ADCx_BASE)    (MEM_ADDR(ADCx_BASE +0x18))   /*!< 注入通道2 数据偏移量 */

/******************   ADC_JOFR3 寄存器 *******************/
#define  ADC_JOFFSET3(ADCx_BASE)    (MEM_ADDR(ADCx_BASE +0x1C))    /*!< 注入通道3 数据偏移量 */

/******************   ADC_JOFR4 寄存器 *******************/
#define  ADC_JOFFSET4(ADCx_BASE)    (MEM_ADDR(ADCx_BASE +0x20))    /*!< 注入通道4 数据偏移量 */

/*******************   ADC_HTR 寄存器 ********************/
#define  ADC_HT(ADCx_BASE)    (MEM_ADDR(ADCx_BASE +0x24))          /*!< 模拟看门狗阀值高限 */

/*******************   ADC_LTR 寄存器 ********************/
#define  ADC_LT(ADCx_BASE)    (MEM_ADDR(ADCx_BASE +0x28))          /*!< 模拟看门狗阀值低限 */

/*******************   ADC_SQR1/2/3 寄存器 *******************/
/*定义规则组序列，队列元素最大16个，每元素用5BIT表示，代表相应的通道号0－17 */
#define  SET_ADC_SQ1(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x34)=(MEM_ADDR(ADCx_BASE +0x34)&(~ADC_SQR3_SQ1))|((a&MASKb5)<<0))
#define  SET_ADC_SQ2(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x34)=(MEM_ADDR(ADCx_BASE +0x34)&(~ADC_SQR3_SQ2))|((a&MASKb5)<<5))
#define  SET_ADC_SQ3(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x34)=(MEM_ADDR(ADCx_BASE +0x34)&(~ADC_SQR3_SQ3))|((a&MASKb5)<<10))
#define  SET_ADC_SQ4(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x34)=(MEM_ADDR(ADCx_BASE +0x34)&(~ADC_SQR3_SQ4))|((a&MASKb5)<<15))
#define  SET_ADC_SQ5(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x34)=(MEM_ADDR(ADCx_BASE +0x34)&(~ADC_SQR3_SQ5))|((a&MASKb5)<<20))
#define  SET_ADC_SQ6(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x34)=(MEM_ADDR(ADCx_BASE +0x34)&(~ADC_SQR3_SQ6))|((a&MASKb5)<<25))
#define  SET_ADC_SQ7(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x30)=(MEM_ADDR(ADCx_BASE +0x30)&(~ADC_SQR2_SQ7))|((a&MASKb5)<<0))
#define  SET_ADC_SQ8(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x30)=(MEM_ADDR(ADCx_BASE +0x30)&(~ADC_SQR2_SQ8))|((a&MASKb5)<<5))
#define  SET_ADC_SQ9(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x30)=(MEM_ADDR(ADCx_BASE +0x30)&(~ADC_SQR2_SQ9))|((a&MASKb5)<<10))
#define  SET_ADC_SQ10(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x30)=(MEM_ADDR(ADCx_BASE +0x30)&(~ADC_SQR2_SQ10))|((a&MASKb5)<<15))
#define  SET_ADC_SQ11(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x30)=(MEM_ADDR(ADCx_BASE +0x30)&(~ADC_SQR2_SQ11))|((a&MASKb5)<<20))
#define  SET_ADC_SQ12(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x30)=(MEM_ADDR(ADCx_BASE +0x30)&(~ADC_SQR2_SQ12))|((a&MASKb5)<<25))
#define  SET_ADC_SQ13(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x2C)=(MEM_ADDR(ADCx_BASE +0x2C)&(~ADC_SQR1_SQ13))|((a&MASKb5)<<0))
#define  SET_ADC_SQ14(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x2C)=(MEM_ADDR(ADCx_BASE +0x2C)&(~ADC_SQR1_SQ14))|((a&MASKb5)<<5))
#define  SET_ADC_SQ15(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x2C)=(MEM_ADDR(ADCx_BASE +0x2C)&(~ADC_SQR1_SQ15))|((a&MASKb5)<<10))
#define  SET_ADC_SQ16(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x2C)=(MEM_ADDR(ADCx_BASE +0x2C)&(~ADC_SQR1_SQ16))|((a&MASKb5)<<15))
/*!< 4BIT，定义规则组序列的长度，0-15分别表示1-16 */
#define  SET_ADC_SQR_L(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x2C)=(MEM_ADDR(ADCx_BASE +0x2C)&(~ADC_SQR1_L))|((a&MASKb4)<<20))


/*******************   ADC_JSQR 寄存器 *******************/
/*!< 定义注入组序列，队列元素最大4个，每元素用5BIT表示，代表相应的通道号0－17 */
#define  SET_ADC_JSQ1(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x38)=(MEM_ADDR(ADCx_BASE +0x38)&(~ADC_JSQR_JSQ1))|((a&MASKb5)<<0))
#define  SET_ADC_JSQ2(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x38)=(MEM_ADDR(ADCx_BASE +0x38)&(~ADC_JSQR_JSQ2))|((a&MASKb5)<<5))
#define  SET_ADC_JSQ3(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x38)=(MEM_ADDR(ADCx_BASE +0x38)&(~ADC_JSQR_JSQ3))|((a&MASKb5)<<10))
#define  SET_ADC_JSQ4(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x38)=(MEM_ADDR(ADCx_BASE +0x38)&(~ADC_JSQR_JSQ4))|((a&MASKb5)<<15))

//#define  ADC_JSQR_JL                         ((uint32_t)0x00300000)        /*!< JL[1:0] bits (注入组序列长度，0-3分别表示1-4) */
#define  SET_ADC_JSQR_JL(ADCx_BASE,a)   (MEM_ADDR(ADCx_BASE +0x38)=(MEM_ADDR(ADCx_BASE +0x38)&(~ADC_JSQR_JL))|((a&MASKb2)<<20))

/*******************   ADC_JDR1 寄存器 *******************/
#define  ADC_JDR1(ADCx_BASE)    (MEM_ADDR(ADCx_BASE +0x3C))			/*!< 注入通道1 数据寄存器 */

/*******************   ADC_JDR2 寄存器 *******************/
#define  ADC_JDR2(ADCx_BASE)    (MEM_ADDR(ADCx_BASE +0x40))     /*!< 注入通道2 数据寄存器 */

/*******************   ADC_JDR3 寄存器 *******************/
#define  ADC_JDR3(ADCx_BASE)    (MEM_ADDR(ADCx_BASE +0x44))     /*!< 注入通道3 数据寄存器 */

/*******************   ADC_JDR4 寄存器 *******************/
#define  ADC_JDR4(ADCx_BASE)    (MEM_ADDR(ADCx_BASE +0x48))     /*!< 注入通道4 数据寄存器 */

/********************   ADC_DR 寄存器 ********************/
#define  ADC_DR(ADCx_BASE)    		(MEM_ADDR(ADCx_BASE +0x4C))        /*!< 规则通道 数据寄存器（32位，其中低12位为ADC转换值） */
#define  ADC_DR_ADC2(ADCx_BASE) 	((MEM_ADDR(ADCx_BASE +0x4C))>>16)  /*!< 双ADC时ADC2 数据（实为上述寄存器的高位） */
/*************************************************************************************************************************/
#endif

