#ifndef __MYBITBAND_H
#define __MYBITBAND_H	  
//#include <stm32f10x.h>  
/****************************** 本人逐步实现的寄存器位操作模式 ***************************************/
// 利用本文件，可以比较方便、高效精准地以位段方式访问寄存器，结合多BIT访问等方式，能够最终完全抛开库函数。
// 一、以外设原有名称为基础，在外设名称前面加前缀b, 与库函数区分的同时，指示这用作bitband操作。
// 二、进一步强化助记，将部分具体寄存器名称弱化,比如某外设有多个控制寄存器CR1、CR2、CR3，则统一省略为CR，不必记忆是第几个CR。
// 尝试对于常用的外设如RCC，进一步根据功能进行了改造，比如将所有使能时钟控制归纳成bRCC_ENABLE_XXX（其中XXX为外设名），
// 便于记忆，不用再考虑某外设挂在哪条具体总线上等细节，相当于进行了一个小的二次封装。
// 三、BIT位的名称尽量采用原名称，以便对照查阅芯片数据手册。寄存器地址偏移量按一般用户手册以十六进制填充。
// 四、考虑到位操作的优势在精准高效地对位进行操作，这也是编写此文件的出发点。但考虑到两位以上的组合操作也无法避免，
// 并且直接访问寄存器某几位也比较麻烦, 这里采用带参的宏定义制定了统一的访问格式.
// 五、对于一般没有位定义、通常整字访问的寄存器，这里也定义了形如wBKP_DR1的宏指令方式对其进行访问， 即：wBKP_DR1的效果等同于寄存器直接访问的BKP->DR1
// CopyRight By Warship
// Created date: 	20180916
// Modified date: 20190512

/************************** 位段操作统一格式 *******************************************
单BIT位访问方式：
		bRCC_ENABLE_RTC=1或0; 														//唯一外设
		bTIM_CR_DIR（bTIMx)=1或0;													//多组外设
		bGPIO_BRR(bGPIOx,n)=1或0;													//多组外设多个端口
		也可读，或者用于逻辑判断，EX：if(!bRCC_ENABLE_RTC)
    
多BIT位访问方式：
		SET_RCC_PLLMUL(a);(设置PLL倍频值为a)			写	 							//唯一外设
		val=GET_RCC_PLLMUL;(获取PLL倍频设置值)		读

		SET_SPI_CR_BR(bSPIx,a);	(设置bSPIx接口的速率值为a)	写			//多组外设
		val=GET_SPI_CR_BR(bSPIx);(获取bSPIx接口的速率值)		读

寄存器整体访问方式：
		wBKP_DR1			（读或写）		等效于寄存器直接访问的BKP->DR1
															部分寄存器没有设置宏定义，请沿用上述寄存器直接访问方式

另外，本文件末附有当不使用位段操作而用逻辑运算实现相关操作的基本方法，供参考。
不过这样就失去了位段操作的效率，同时代码的可读性也会变差
**************************************************************************************/

//为减少宏定义的工作量，对于多个相同的外设，尤其是两个以上，如定时器、串行通信等，避免大量类似的宏定义
//使用带参数的宏，形如bCR_CEN(bTIM1),其中TIM1为基址宏
//以下统一定义的以b开头的基址宏，直接引用了stm32f10x.h中定义的xxx_BASE,
//也可以不用这些定义，直接使用xxx_BASE
//注意：可位段操作的外设地址区间为4000 0000－400F FFFF,基本上涵盖了绝大部分的外设。

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
#define 	MASKb1			((uint32_t)0x00000001)
#define 	MASKb2			((uint32_t)0x00000003)
#define 	MASKb3			((uint32_t)0x00000007)
#define 	MASKb4			((uint32_t)0x0000000F)
#define 	MASKb5			((uint32_t)0x0000001F)
#define 	MASKb6			((uint32_t)0x0000003F)
#define 	MASKb7			((uint32_t)0x0000007F)
#define 	MASKb8			((uint32_t)0x000000FF)
#define 	MASKb9			((uint32_t)0x000001FF)
#define 	MASKb10			((uint32_t)0x000003FF)
#define 	MASKb11			((uint32_t)0x000007FF)
#define 	MASKb12			((uint32_t)0x00000FFF)
#define 	MASKb13			((uint32_t)0x00001FFF)
#define 	MASKb14			((uint32_t)0x00003FFF)
#define 	MASKb15			((uint32_t)0x00007FFF)
#define 	MASKb16			((uint32_t)0x0000FFFF)
#define 	MASKb17			((uint32_t)0x0001FFFF)
#define 	MASKb18			((uint32_t)0x0003FFFF)
#define 	MASKb19			((uint32_t)0x0007FFFF)
#define 	MASKb20			((uint32_t)0x000FFFFF)
#define 	MASKb21			((uint32_t)0x001FFFFF)
#define 	MASKb22			((uint32_t)0x003FFFFF)
#define 	MASKb23			((uint32_t)0x007FFFFF)
#define 	MASKb24			((uint32_t)0x00FFFFFF)
#define 	MASKb25			((uint32_t)0x01FFFFFF)
#define 	MASKb26			((uint32_t)0x03FFFFFF)
#define 	MASKb27			((uint32_t)0x07FFFFFF)
#define 	MASKb28			((uint32_t)0x0FFFFFFF)
#define 	MASKb29			((uint32_t)0x1FFFFFFF)
#define 	MASKb30			((uint32_t)0x3FFFFFFF)
#define 	MASKb31			((uint32_t)0x7FFFFFFF)
#define 	MASKb32			((uint32_t)0xFFFFFFFF)

//寄存器多位访问宏定义：参数（寄存器地址、起始比特位、掩码（位数）、设置值）
//本定义不限于位段使用，对于任何特定地址的存储器均可用
#define SET_REG_BITn(REG_ADDR,Sbit,MASKn,a)  	(MEM_ADDR(REG_ADDR)=(MEM_ADDR(REG_ADDR)&(~(MASKn<<Sbit)))|((a & MASKn)<<Sbit))
#define GET_REG_BITn(REG_ADDR,Sbit,MASKn)   	((MEM_ADDR(REG_ADDR)>>Sbit)& MASKn)

/******************************************************************************/
/*                                                                            */
/*                    PWR寄存器--电源控制寄存器                               */
/*                                                                            */
/******************************************************************************/

/****************** PWR_CR--电源控制寄存器 ******************/
#define  bPWR_CR_LPDS          BIT_ADDR(PWR_BASE, 0)     //深睡眠下的低功耗（PDDS=0时，与PDDS位协同操作）定义：0（在待机模式下电压调压器开启），1（在待机模式下电压调压器处于低功耗模式）
#define  bPWR_CR_PDDS          BIT_ADDR(PWR_BASE, 1)     //掉电深睡眠（与LPDS位协同操作）定义：0（当CPU进入深睡眠时进入停机模式，调压器状态由LPDS位控制），1（CPU进入深睡眠时进入待机模式）
#define  bPWR_CR_CWUF          BIT_ADDR(PWR_BASE, 2)     //清除唤醒位（始终输出为0）定义：0（无效），1（2个系统时钟周期后清除WUF唤醒位（写）
#define  bPWR_CR_CSBF          BIT_ADDR(PWR_BASE, 3)     //清除待机位（始终输出为0）定义：0（无效），1（清除SBF待机位（写）
#define  bPWR_CR_PVDE          BIT_ADDR(PWR_BASE, 4)     //电源电压检测器（PVD）使能。定义：0（禁止PVD），1（开启PVD）
#define  bPWR_CR_DBP           BIT_ADDR(PWR_BASE, 8)     //取消后备区域写保护。复位值为0。定义：0为禁止写入，1为允许写入。注：如果rtc时钟是HSE/128，必须保持为1

//#define  PWR_CR_PLS  以下3BIT定义PVD电压阀值
#define  SET_PWR_CR_PLS(a)   SET_REG_BITn(PWR_BASE,5,MASK3,a)
//PVD电压阀值,示例：设置2.5V电压阀值时使用SET_PWR_CR_PLS(PVD_2V5);
#define  PVD_2V2	0x000
#define  PVD_2V3	0x001
#define  PVD_2V4	0x010
#define  PVD_2V5	0x011
#define  PVD_2V6	0x100
#define  PVD_2V7	0x101
#define  PVD_2V8	0x110
#define  PVD_2V9	0x111



/****************** PWR_CSR--电源控制状态寄存器 ******************/
#define  bPWR_CSR_WUF           BIT_ADDR(PWR_BASE+4, 0) //唤醒标志（该位由硬件设置，并只能由POR/PDR（上电/掉电复位）或设置电源控制寄存器（PWR_CR）的CWUF位清除）
                                                       //定义：0（没有唤醒事件），1（在WKUP引脚上发生唤醒事件或出现RTC闹钟事件） 
                                                       //注：当WKUP引脚已经是高电平时，在使能WKUP引脚（通过设置EWUP位）时，会检测到一个额外事件
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


/****************** RCC_CR--时钟控制寄存器 ******************/
#define  bRCC_CLK_HSION        	BIT_ADDR(RCC_BASE, 0) //LSI时钟: 0禁用,1开启
#define  bRCC_CLK_HSIRDY       	BIT_ADDR(RCC_BASE, 1) //LSI时钟状态由硬件控制(只读):0不可用,1就绪
#define  bRCC_CLK_HSEON        	BIT_ADDR(RCC_BASE, 16) //HSE时钟: 0禁用,1开启
#define  bRCC_CLK_HSERDY       	BIT_ADDR(RCC_BASE, 17) //HSE时钟状态由硬件控制(只读):0不可用,1就绪
#define  bRCC_CLK_HSEBYP       	BIT_ADDR(RCC_BASE, 18) //外部时钟旁路(调试用)-- 0不旁路  1旁路
#define  bRCC_CLK_CSSON     		BIT_ADDR(RCC_BASE, 19) //系统时钟安全系统使能  0时钟检测禁用  1外部时钟就绪后启动检测
#define  bRCC_CLK_PLLON       	BIT_ADDR(RCC_BASE, 24) //PLL倍频: 0禁用,1开启
#define  bRCC_CLK_PLLRDY      	BIT_ADDR(RCC_BASE, 25) //PLL倍频状态由硬件控制(只读):0不可用,1就绪
//5BIT的HSITRIM  内部高速时钟调整  用于补偿因温度等变化对内部RC振荡器时钟频率的影响.
#define  SET_RCC_CLK_HSITRIM(a)   SET_REG_BITn(RCC_BASE,3,MASKb5,a)
//8BIT的HSICAL  内部高速时钟校准  
#define  SET_RCC_CLK_HSICAL(a)   SET_REG_BITn(RCC_BASE,8,MASKb8,a)


/****************** RCC_CFGR--时钟配置寄存器 ******************/
//系统时钟选择：BIT1:0
#define  SET_RCC_CONFIG_SW(a)   SET_REG_BITn(RCC_BASE+0x04,0,MASKb2,a)  //整体写
#define  bRCC_CONFIG_SW0        BIT_ADDR(RCC_BASE+0x04, 0) //系统时钟选择2BIT-- 00:HSI  01:HSE
#define  bRCC_CONFIG_SW1        BIT_ADDR(RCC_BASE+0x04, 1) //                  10:PLL  11: 无效 
#define  bRCC_CONFIG_SW_PLL   	BIT_ADDR(RCC_BASE+0x04, 1) //SYSCLK时钟选择位1，置位时为选择PLL作为系统时钟源

#define  SET_RCC_SW(a)  				SET_REG_BITn(RCC_BASE+0x04,0,MASKb2,a) 
#define  GET_RCC_SW							GET_REG_BITn(RCC_BASE+0x04,0,MASKb2)

#define  SW_HSI                     0x0       /*!< HSI 内部高速时钟被选择为系统时钟 */
#define  SW_HSE                     0x1       /*!< HSE 外部高速时钟被选择为系统时钟 */
#define  SW_PLL                     0x2       /*!< PLL 锁相环的输出被选择为系统时钟 */


//系统时钟源指示BIT3:2,只读: 定义同上
#define  GET_RCC_CONFIG_SW   GET_REG_BITn(RCC_BASE+0x04,2,MASKb2)		//整体读
//（注意：宏名采用GET_RCC_CONFIG_SW,而不是GET_RCC_CONFIG_SWS,是因为正好它是只读，为与上述的写操作SET_RCC_CONFIG_SW(a)统一，便于记忆）
#define  bRCC_CONFIG_SWS0        BIT_ADDR(RCC_BASE+0x04, 2) 
#define  bRCC_CONFIG_SWS1        BIT_ADDR(RCC_BASE+0x04, 3) //

#define  bRCC_CONFIG_SWS_PLL   		BIT_ADDR(RCC_BASE+0x04, 3)  //SYSCLK时钟指示位1，为1时指示PLL已经为系统时钟源

#define  bRCC_CONFIG_PLLSRC       BIT_ADDR(RCC_BASE+0x04, 16) //PLL钟源选择, 0: HSI/2  1:HSE（PREDIV1的输出，即下行的PLLXTPRE所决定的输出）
#define  bRCC_CONFIG_PLLXTPRE     BIT_ADDR(RCC_BASE+0x04, 17) //输出至PLL的HSE是否分频 0:不分频  1:二分频
#define  bRCC_CONFIG_USBPRE       BIT_ADDR(RCC_BASE+0x04, 22) //USB预分频控制 0: PLL/1.5   1: PLL

//4BIT的HPRE控制位: 控制系统时钟SYSCLK分频至AHB
///*!< HPRE BIT7:4 */
#define  SET_RCC_HPRE(a)   SET_REG_BITn(RCC_BASE+0x04,4,MASKb4,a)
#define  GET_RCC_HPRE		   GET_REG_BITn(RCC_BASE+0x04,4,MASKb4)
#define  HPRE_DIV1                  0        /*!< SYSCLK not divided */
#define  HPRE_DIV2                  1        /*!< SYSCLK divided by 2 */
#define  HPRE_DIV4                  3        /*!< SYSCLK divided by 4 */
#define  HPRE_DIV8                  4        /*!< SYSCLK divided by 8 */
#define  HPRE_DIV16                 5        /*!< SYSCLK divided by 16 */
#define  HPRE_DIV64                 6        /*!< SYSCLK divided by 64 */
#define  HPRE_DIV128                7        /*!< SYSCLK divided by 128 */
#define  HPRE_DIV256                8        /*!< SYSCLK divided by 256 */
#define  HPRE_DIV512                9        /*!< SYSCLK divided by 512 */

/*!< PPRE1 BIT10:8 共3BIT的PPRE1控制位: 控制至APB1的预分频*/ 
#define  SET_RCC_HPRE1(a)   	SET_REG_BITn(RCC_BASE+0x04,8,MASKb3,a)
#define  GET_RCC_HPRE1		   	GET_REG_BITn(RCC_BASE+0x04,8,MASKb3)
//取值定义如下：
#define  PPRE1_DIV1                 0        /*!< HCLK not divided */
#define  PPRE1_DIV2                 4        /*!< HCLK divided by 2 */
#define  PPRE1_DIV4                 5        /*!< HCLK divided by 4 */
#define  PPRE1_DIV8                 6        /*!< HCLK divided by 8 */
#define  PPRE1_DIV16                7        /*!< HCLK divided by 16 */

/*!< PPRE2 BIT13:11 共3BIT的PPRE2控制位: 控制至APB2的预分频*/
#define  SET_RCC_HPRE2(a)   	SET_REG_BITn(RCC_BASE+0x04,11,MASKb3,a)
#define  GET_RCC_HPRE2		   	GET_REG_BITn(RCC_BASE+0x04,11,MASKb3)
//取值定义如下：
#define  PPRE2_DIV1                 0        /*!< HCLK 不分频 */
#define  PPRE2_DIV2                 4        /*!< HCLK 2分频 */
#define  PPRE2_DIV4                 5        /*!< HCLK 4分频 */
#define  PPRE2_DIV8                 6        /*!< HCLK 8分频 */
#define  PPRE2_DIV16                7        /*!< HCLK 16分频 */

/*!< ADCPPRE BIT15:14 ：控制ADC的预分频 */
#define  SET_RCC_ADCPRE(a)   	SET_REG_BITn(RCC_BASE+0x04,14,MASKb2,a)
#define  GET_RCC_ADCPRE		   	GET_REG_BITn(RCC_BASE+0x04,14,MASKb2)
//取值定义如下：
#define  ADCPRE_DIV2                0        /*!< PCLK2 2分频 */
#define  ADCPRE_DIV4                1        /*!< PCLK2 4分频 */
#define  ADCPRE_DIV6                2        /*!< PCLK2 6分频 */
#define  ADCPRE_DIV8                3        /*!< PCLK2 8分频 */

//4BIT(BIT21:18)的PLLMUL:选择PLL的倍频数,2-16
#define  SET_RCC_PLLMUL(a)   	SET_REG_BITn(RCC_BASE+0x04,18,MASKb4,a)
#define  GET_RCC_PLLMUL		   	GET_REG_BITn(RCC_BASE+0x04,18,MASKb4)
//取值定义如下：
#define  PLLMULL2                  0        /*!< PLL 2倍频 */
#define  PLLMULL3                  1        /*!< PLL 3倍频 */
#define  PLLMULL4                  2       /*!< PLL 4倍频 */
#define  PLLMULL5                  3        /*!< PLL 5倍频 */
#define  PLLMULL6                  4        /*!< PLL 6倍频 */
#define  PLLMULL7                  5        /*!< PLL 7倍频 */
#define  PLLMULL8                  6        /*!< PLL 8倍频 */
#define  PLLMULL9                  7        /*!< PLL 9倍频 */
#define  PLLMULL10                 8        /*!< PLL 10倍频 */
#define  PLLMULL11                 9        /*!< PLL 11倍频 */
#define  PLLMULL12                 10        /*!< PLL 12倍频 */
#define  PLLMULL13                 11        /*!< PLL 13倍频 */
#define  PLLMULL14                 12        /*!< PLL 14倍频 */
#define  PLLMULL15                 13        /*!< PLL 15倍频 */
#define  PLLMULL16                 14        /*!< PLL 16倍频 */

///*!< MCO configuration 注意不同的芯片有差异  3BIT的MCO控制位: 对MCU输出时钟的选择进行控制  */     
#define  SET_RCC_MCO(a)   	SET_REG_BITn(RCC_BASE+0x04,24,MASKb3,a)
//取值定义如下：
#define  MCO_NOCLOCK               ((uint32_t)0x00)        /*!< No clock */
#define  MCO_SYSCLK                ((uint32_t)0x04)        /*!< System clock selected as MCO source */
#define  MCO_HSI                   ((uint32_t)0x05)        /*!< HSI clock selected as MCO source */
#define  MCO_HSE                   ((uint32_t)0x06)        /*!< HSE clock selected as MCO source  */
#define  MCO_PLL                   ((uint32_t)0x07)        /*!< PLL clock divided by 2 selected as MCO source */


/****************** RCC_CIR--时钟中断寄存器 ******************/
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


/****************** RCC_APB2RSTR寄存器 ******************/
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


/****************** RCC_APB1RSTR寄存器 ******************/
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


/****************** RCC_AHBEN寄存器 ******************/
#define  bRCC_ENABLE_DMA                   BIT_ADDR(RCC_BASE+0x14, 0)  //0关闭时钟,1开启时钟,下同
#define  bRCC_ENABLE_SRAM                  BIT_ADDR(RCC_BASE+0x14, 2)
#define  bRCC_ENABLE_FLITF                 BIT_ADDR(RCC_BASE+0x14, 4)
#define  bRCC_ENABLE_CRC                 	 BIT_ADDR(RCC_BASE+0x14, 6)
#define  bRCC_ENABLE_FSMC                	 BIT_ADDR(RCC_BASE+0x14, 8)  
#define  bRCC_ENABLE_SDIO                	 BIT_ADDR(RCC_BASE+0x14, 10) 


/****************** RCC_APB2ENR寄存器 ******************/
#define  bRCC_ENABLE_AFIO                 BIT_ADDR(RCC_BASE+0x18, 0)   //0关闭时钟,1开启时钟,下同
#define  bRCC_ENABLE_GPIOx(n)             BIT_ADDR(RCC_BASE+0x18, (n+2))
//注意上面的n要用纯数字序号，0、1、2分别代表GPIOA、GPIOB、GPIOC等

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


/****************** RCC_APB1ENR寄存器 ******************/
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

//
/****************** RCC_BDCR寄存器--备份区域控制 ******************/
//备份域控制寄存器中 (RCC_BDCR) 的 LSEON 、 LSEBYP 、 RTCSEL 和 RTCEN 位处于备份域。因
//此，这些位在复位后处于写保护状态，只有在电源控制寄存器 (PWR_CR) 中的 DBP 位置 ’1’ 后才
//能对这些位进行改动。
#define  bRCC_CLK_LSEON                  BIT_ADDR(RCC_BASE+0x20, 0)  		//LSE时钟: 0禁用,1开启
#define  bRCC_CLK_LSERDY                 BIT_ADDR(RCC_BASE+0x20, 1) 		//LSE时钟状态由硬件控制(只读):0不可用,1就绪
#define  bRCC_CLK_LSEBYP                 BIT_ADDR(RCC_BASE+0x20, 2) 		//LSE时钟状态由硬件控制(只读):0不可用,1就绪

#define  SET_RCC_CLK_RTCSEL(a)   	SET_REG_BITn(RCC_BASE+0x20,8,MASKb2,a)
#define  bRCC_CLK_RTCSEL0                BIT_ADDR(RCC_BASE+0x20, 8) 		//RTC时钟源选择两位共同控制: 00:无时钟 01:LSE
#define  bRCC_CLK_RTCSEL1                BIT_ADDR(RCC_BASE+0x20, 9) 		//                      10:LSI   11: HSE/128 

#define  bRCC_ENABLE_RTC                 BIT_ADDR(RCC_BASE+0x20, 15)  	//0禁用RTC; 1:使能RTC
#define  bRCC_RESET_BKUPDOMAIN           BIT_ADDR(RCC_BASE+0x20, 16)   	//备份区域软复位  写1复位 0复位未被激活


/****************** RCC_CSR寄存器--状态与控制 ******************/
#define  bRCC_CLK_LSION                   BIT_ADDR(RCC_BASE+0x24, 0)   	//LSI时钟: 0禁用,1开启
#define  bRCC_CLK_LSIRDY                  BIT_ADDR(RCC_BASE+0x24, 1)   	//LSI时钟状态由硬件控制(只读):0不可用,1就绪
#define  bRCC_FLG_RMVF                    BIT_ADDR(RCC_BASE+0x24, 24) 	//清除复位标志 写0复位未激活的复位标志,写1则清复位标志
#define  bRCC_FLG_PINRSTF                 BIT_ADDR(RCC_BASE+0x24, 26)  	//引脚复位标志  硬件置1软件写RMVF位清0
#define  bRCC_FLG_PORRSTF                 BIT_ADDR(RCC_BASE+0x24, 27)  	//端口复位标志  硬件置1软件写RMVF位清0
#define  bRCC_FLG_SFTRSTF                 BIT_ADDR(RCC_BASE+0x24, 28)  	//软件复位标志  硬件置1软件写RMVF位清0
#define  bRCC_FLG_IWDGRSTF                BIT_ADDR(RCC_BASE+0x24, 29)  	//独立看门狗复位标志  硬件置1软件写RMVF位清0
#define  bRCC_FLG_WWDGRSTF                BIT_ADDR(RCC_BASE+0x24, 30)		//窗口看门狗复位标志  硬件置1软件写RMVF位清0
#define  bRCC_FLG_LPWRRSTF                BIT_ADDR(RCC_BASE+0x24, 31)		//低功耗管理复位标志  硬件置1软件写RMVF位清0 

/******************************************************************************/
/*                                                                            */
/*                         AFIO寄存器                                         */
/*                                                                            */
/******************************************************************************/


/****************** AFIO_EVCR--事件控制寄存器 ******************/
#define  bAFIO_EVCR_EVOE        		BIT_ADDR(AFIO_BASE, 7) 			//事件输出使能. 为1时,事件由指定的端口输出(端口由BIT6:4,引脚由BIT3:0决定)
#define  SET_AFIO_EVCR_PIN(a)   	SET_REG_BITn(AFIO_BASE,0,MASKb4,a)	//事件输出指定的引脚(端口由下行的BIT6:4决定)
#define  SET_AFIO_EVCR_PORT(a)   	SET_REG_BITn(AFIO_BASE,4,MASKb3,a)	//事件输出指定的端口(引脚由上行的BIT3:0决定)

/****************** AFIO_MAPR--引脚重映射寄存器 ******************/
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
#define  bAFIO_MAPR_TIM4_REMAP      				BIT_ADDR(AFIO_BASE+4, 12) //只控制100引脚封装中TIM4的通道1-4的映射. 0正常,1由原来的PB6/7/8/9映射到PD12/13/14/15
#define  bAFIO_MAPR_CAN_REMAPb0      				BIT_ADDR(AFIO_BASE+4, 13) //两BIT控制: 
#define  bAFIO_MAPR_CAN_REMAPb1      				BIT_ADDR(AFIO_BASE+4, 14) //
#define  bAFIO_MAPR_PD01_REMAP      				BIT_ADDR(AFIO_BASE+4, 15) //0正常, 1 PD0/1端口分别映像到OSC_IN/OSC_OUT引脚
//以下是部分器件新增的映射定义
#define  bAFIO_MAPR_TIM5CH4_IREMAP      		BIT_ADDR(AFIO_BASE+4, 16) /*!< TIM5 Channel4 Internal Remap */
#define  bAFIO_MAPR_ADC1_ETRGINJ_REMAP      BIT_ADDR(AFIO_BASE+4, 17) /*!< ADC 1 External Trigger Injected Conversion remapping */
#define  bAFIO_MAPR_ADC1_ETRGREG_REMAP      BIT_ADDR(AFIO_BASE+4, 18) /*!< ADC 1 External Trigger Regular Conversion remapping */
#define  bAFIO_MAPR_ADC2_ETRGINJ_REMAP      BIT_ADDR(AFIO_BASE+4, 19) /*!< ADC 2 External Trigger Injected Conversion remapping */
#define  bAFIO_MAPR_ADC2_ETRGREG_REMAP      BIT_ADDR(AFIO_BASE+4, 20) /*!< ADC 2 External Trigger Regular Conversion remapping */
#define  bAFIO_MAPR_ETH_REMAP      					BIT_ADDR(AFIO_BASE+4, 21) /*!< SPI3_REMAP bit (Ethernet MAC I/O remapping) */
#define  bAFIO_MAPR_CAN2_REMAP      				BIT_ADDR(AFIO_BASE+4, 22) /*!< CAN2_REMAP bit (CAN2 I/O remapping) */
#define  bAFIO_MAPR_MII_RMII_SEL      			BIT_ADDR(AFIO_BASE+4, 23) /*!< MII_RMII_SEL bit (Ethernet MII or RMII selection) */
#define  bAFIO_MAPR_SPI3_REMAP      				BIT_ADDR(AFIO_BASE+4, 28)  /*!< SPI3_REMAP bit (SPI3 remapping) */
#define  bAFIO_MAPR_TIM2ITR1_IREMAP      		BIT_ADDR(AFIO_BASE+4, 29) /*!< TIM2ITR1_IREMAP bit (TIM2 internal trigger 1 remapping) */
#define  bAFIO_MAPR_PTP_PPS_REMAP      			BIT_ADDR(AFIO_BASE+4, 30) /*!< PTP_PPS_REMAP bit (Ethernet PTP PPS remapping) */

#define  SET_AFIO_MAPR_SWJ_CFG(a)   				SET_REG_BITn(AFIO_BASE+4,24,MASKb3,a)
//3BIT控制串行JTAG配置(只写):000（完全SWJ，复位状态），001（完全SWJ，但没有NJTRST），
//010（关闭JATG启动SW），100（关闭JATG，关闭SW）其它值无定义
//

/****************** AFIO_EXTICR1-4--外部中断配置寄存器 ******************/
//4个外部中断配置寄存器EXTICR1-4, 控制16条中断线的端口组(分别用4BIT决定PA或PB/....../PG)的选择.
#define  SET_AFIO_EXTICR(EXTI_n,a)   	SET_REG_BITn(AFIO_BASE+0x08+(EXTI_n&0x0c),(EXTI_n&0x03)<<2,MASKb4,a)
//注意参数a为端口号，直接用数字序号0、1、2、3等分别表示GPIOA、GPIOB、GPIOC、GPIOD等
//例如：设EXTI2为PD2，则使用SET_AFIO_EXTICR(2,GPIO_D);
//***********以下为演算对照验证草稿，可删除
//EXTIn		(EXTI_n&0x0c)	(EXTIn&0x03)<<2
//0				0							0
//1				0							4
//2				0							8
//3				0							C
//4				4							0
//5				4							4
//6				4							8
//7				4							C
//8				8							0
//9				8							4
//10			8							8
//11			8							C
//12			C							0
//13			C							4	
//14			C							8
//15			C							C

/******************************************************************************/
/*                                                                            */
/*                         GPIOx寄存器                                        */
/*                                                                            */
/******************************************************************************/
//GPIO的有关输出输入的位操作,可以用大家广泛使用的PAout(n)......PCin(n)的方式；

//对GPIO的端口配置: 每线采用4BIT,可试用如下宏定义：
#define  SET_GPIO_CR(GPIOx_BASE,PINx,mode)   	SET_REG_BITn(GPIOx_BASE+(PINx>>3)*4,(PINx & 0x07)<<2,MASKb4,mode)
//其中的PINx使用端口的引脚序号，如PC7，则参数值为7;
//其中的mode取值如下：
//	0					模拟输入 AIN
//	4					浮空输入 IN_FLOATING
//	8					上拉/下拉输入(即IPU和IPD的设置值一样，但最后须按要求预置端口为1或0)

//	3	（2/1）	通用推挽输出50MHz（2/10MHz）Out_PP	（不同的频率请注意括号的对应关系，下同）
//	7	（6/5）	通用开漏输出50MHz（2/10MHz）Out_OD
//	B （A/9）	复用推挽输出50MHz（2/10MHz）AF_PP
//	F	（E/D）	复用开漏输出50MHz（2/10MHz）AF_OD

//EX: 设置PC7为50M通用推挽输出：SET_GPIO_CR(GPIOC,7,3);

//GPIO的端口配置锁定功能GPIOx_LCKR寄存器,适合采用位操作方式,但一般此功能不常用.
#define  bGPIOA_BSRR(n)   BIT_ADDR(GPIOA_BASE+0x10, n)
#define  bGPIOB_BSRR(n)   BIT_ADDR(GPIOB_BASE+0x10, n)
#define  bGPIOC_BSRR(n)   BIT_ADDR(GPIOC_BASE+0x10, n)
#define  bGPIOD_BSRR(n)   BIT_ADDR(GPIOD_BASE+0x10, n)
#define  bGPIOE_BSRR(n)   BIT_ADDR(GPIOE_BASE+0x10, n)
#define  bGPIOF_BSRR(n)   BIT_ADDR(GPIOF_BASE+0x10, n)
#define  bGPIOG_BSRR(n)   BIT_ADDR(GPIOG_BASE+0x10, n)
#define  bGPIOA_BRR(n)   	BIT_ADDR(GPIOA_BASE+0x14, n)
#define  bGPIOB_BRR(n)   	BIT_ADDR(GPIOB_BASE+0x14, n)
#define  bGPIOC_BRR(n)   	BIT_ADDR(GPIOC_BASE+0x14, n)
#define  bGPIOD_BRR(n)   	BIT_ADDR(GPIOD_BASE+0x14, n)
#define  bGPIOE_BRR(n)   	BIT_ADDR(GPIOE_BASE+0x14, n)
#define  bGPIOF_BRR(n)   	BIT_ADDR(GPIOF_BASE+0x14, n)
#define  bGPIOG_BRR(n)   	BIT_ADDR(GPIOG_BASE+0x14, n)
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

#define  bGPIOx_IDR(m,n)   BIT_ADDR(GPIOA_BASE+0x400*m+0x08, n)  //指定端口的指定位
#define  bGPIOx_ODR(m,n)   BIT_ADDR(GPIOA_BASE+0x400*m+0x0C, n)  //指定端口的指定位
#define  bGPIOx_BSRR(m,n)  BIT_ADDR(GPIOA_BASE+0x400*m+0x10, n) //置位指定端口的指定位
#define  bGPIOx_BRR(m,n)   BIT_ADDR(GPIOA_BASE+0x400*m+0x14, n)  //复位指定端口的指定位

//EX: 读取PC7的输入电平可使用：bGPIOx_IDR(GPIO_C,7)或bGPIOx_IDR(2,7)
//以上的m、n要用纯数字序号，对于m用0、1、2等分别代表GPIOA、GPIOB、GPIOC等
//对于n用0、1、2等分别代表Pin0、Pin1、Pin2等
/******************************************************************************/
/*                                                                            */
/*                         BKP寄存器                                          */
/*                                                                            */
/******************************************************************************/
//前10个寄存器完全可为用户自由使用,地址为:BKP_BASE+0x04至BKP_BASE+0x28,每个寄存器占用4字节空间,即一个字,但只有低16位有效.
//可直接用下列宏指令对其进行访问， wBKP_DR1等同于BKP->DR1
#define  wBKP_DR1  			MEM_ADDR(BKP_BASE+0x04)
#define  wBKP_DR2  			MEM_ADDR(BKP_BASE+0x08)
#define  wBKP_DR3  			MEM_ADDR(BKP_BASE+0x0C)
#define  wBKP_DR4  			MEM_ADDR(BKP_BASE+0x10)
#define  wBKP_DR5  			MEM_ADDR(BKP_BASE+0x14)
#define  wBKP_DR6  			MEM_ADDR(BKP_BASE+0x18)
#define  wBKP_DR7  			MEM_ADDR(BKP_BASE+0x1C)
#define  wBKP_DR8  			MEM_ADDR(BKP_BASE+0x20)
#define  wBKP_DR9  			MEM_ADDR(BKP_BASE+0x24)
#define  wBKP_DR10  		MEM_ADDR(BKP_BASE+0x28)
//这些寄存器不会被系统复位，电源复位，待机唤醒所复位
//注意对后备寄存器的写操作必须使能PWR及BKP的时钟，即bRCC_ENABLE_PWR=1;bRCC_ENABLE_BKP=1; 
//对后备寄存器的读则无须以上三点，随时可读。		


/******************BKP_RTCCR（RTC时钟校准寄存器） ******************/
//BIT6:0 CAL校准值。表示在每2的20次方个时钟脉冲内将有多少个脉冲被跳过。这可用来对RTC进行校准，以1000000/（2的20次方比例减慢时钟）可用被减慢0-121ppm
#define  bBKP_RTCCR_CCO        BIT_ADDR(BKP_BASE+0x2C, 7) //CCO校准时钟输出。定义：0（无影响），1（此位置1可在侵入检引脚输出经64分频后的RTC时钟。
                                                         //  当CCO位置1时，必须关闭侵入检测）注：vdd断电，该位清除
#define  bBKP_RTCCR_ASOE       BIT_ADDR(BKP_BASE+0x2C, 8) //允许输出闹钟或秒脉冲（根据ASOS位的置位，该位允许RTC闹钟或秒脉冲输出到TAMPER引脚。
                                                         // 脉冲宽度为1个RTC时钟周期。置位时不能开启TAMPER功能）
#define  bBKP_RTCCR_ASOS       BIT_ADDR(BKP_BASE+0x2C, 9) //闹钟或秒输出（当设置ASOE位，ASOS位可用于选择在TAMPER引脚上输出的是RTC秒脉冲还是闹钟脉冲信号）
                                                         //定义：0（输出RTC闹钟脉冲），1（输出秒脉冲）注：后备区复位清除

/****************** BKP_CR备份控制寄存器 ******************/
#define  bBKP_CR_TPE        BIT_ADDR(BKP_BASE+0x30, 0) //TPAL侵入检测TAMPER引脚有效电平。0检测TAMPER脚高电平清除备份数据 1检测TAMPER脚低电平清除备份数据
#define  bBKP_CR_TPAL       BIT_ADDR(BKP_BASE+0x30, 1) //TPE启动入侵检测TAMPER引脚。定义：0（TAMPER脚为普通IO），1（开启检测）

/****************** BKP_CR备份控制/状态寄存器 ******************/
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

/****************** RTC_CRH 控制寄存器高位 ******************/
#define  bRTC_CR_SECIE        BIT_ADDR(RTC_BASE+0x00, 0) //SECIE允许秒中断位，定义：0（屏蔽中断），1（允许中断）
#define  bRTC_CR_ALRIE        BIT_ADDR(RTC_BASE+0x00, 1)  //ALRIE允许闹钟中断位，定义：0（屏蔽中断），1（允许中断）
#define  bRTC_CR_OWIE         BIT_ADDR(RTC_BASE+0x00, 2) //OWIE允许溢出中断位，定义：0（屏蔽中断），1（允许中断）


/****************** RTC_CRL 控制寄存器低位 ******************/
#define  bRTC_CR_SECF        BIT_ADDR(RTC_BASE+0x04, 0)  
//0位：SECF秒标志,当32位可编程预分频器溢出时,此位由硬件置’1’同时RTC计数器加1。因此,此标志为分辨率可编程的RTC计数器提供一个周期性的信号(通常为1秒)。															
//     如果RTC_CRH寄存器中SECIE=1，则产生中断。此位只能由软件清除。对此位写’1’是无效的 ,定义：0（秒标志条件不成立），1（秒标志条件成立）	
#define  bRTC_CR_ALRF        BIT_ADDR(RTC_BASE+0x04, 1) 
//1位：ALRF闹钟标志，当32位可编程计数器达到RTC_ALR寄存器所设置的预定值，此位由硬件置’1’。如果RTC_CRH寄存器中ALRIE=1，则产生中断。															
//     此位只能由软件清’0’。对此位写’1’是无效的，定义：0（无闹钟），1（有闹钟）															
#define  bRTC_CR_OWF         BIT_ADDR(RTC_BASE+0x04, 2) 
//2位：OWF溢出标志，当32位可编程计数器溢出时，此位由硬件置’1’。如果RTC_CRH寄存器中OWIE=1，则产生中断。此位只能由软件清’0’。															
//     对此位写’1’是无效的，定义：0（无溢出），1（32位可编程计数器溢出）		
#define  bRTC_CR_RSF        	BIT_ADDR(RTC_BASE+0x04, 3)  
//3位：RSF寄存器同步标志，每当RTC_CNT寄存器和RTC_DIV寄存器由软件更新或清’0’时，此位由硬件置’1’。在APB1复位后，或APB1时钟停止后，此位必须由															
//     软件清’0’。要进行任何的读操作之前，用户程序必须等待这位被硬件置’1’，以确保RTC_CNT、RTC_ALR或RTC_PRL已经被同步。															
//     定义：0（寄存器尚未被同步），1（寄存器已经被同步）
#define  bRTC_CR_CNF        	BIT_ADDR(RTC_BASE+0x04, 4) 
//4位：CNF配置标志，此位必须由软件置’1’以进入配置模式，从而允许向RTC_CNT、RTC_ALR或RTC_PRL寄存器写入数据。只有当此位在被置’1’															
//     并重新由软件清’0’后，才会执行写操作，定义：0（退出配置模式(开始更新RTC寄存器)，1（进入配置模式）	
#define  bRTC_CR_RTOFF      	BIT_ADDR(RTC_BASE+0x04, 5) //RTC操作关闭（只读位）RTC模块利用这位来指示对其寄存器进行的最后一次操作的状态，指示操作是否完成。若此位
//为’0’，则表示无法对任何的RTC寄存器进行写操作。定义：0（上一次对RTC寄存器的写操作仍在进行），1（上一次对RTC寄存器的写操作已经完成）
														
//注1：任何标志位都将保持挂起状态，直到适当的RTC_CR请求位被软件复位，表示所请求的中断已经被接受															
//注2：在复位时禁止所有中断，无挂起的中断请求，可以对RTC寄存器进行写操作。															
//注3：当APB1时钟不运行时，OWF、ALRF、SECF和RSF位不被更新															
//注4：OWF、ALRF、SECF和RSF位只能由硬件置位，由软件来清零。															
//注5：若ALRF=1且ALRIE=1，则允许产生RTC全局中断。如果在EXTI控制器中允许产生EXTI线 17中断，则允许产生RTC全局中断和RTC闹钟中断															
//注6：若ALRF=1，如果在EXTI控制器中设置了EXTI线 17的中断模式，则允许产生RTC闹钟中断；如果在EXTI控制器中设置了EXTI线 17的事件模式，															
//     则这条线上会产生一个脉冲(不会产生RTC闹钟中断)


//另外,以下的RTC寄存器都不宜进行位操作,直接按字整体读写，现暂按寄存器名访问列出如下：

/****************** RTC_PRLH(RTC预分频装载寄存器高位) ******************/
#define  wRTC_PRLH		  	RTC->PRLH          
//PRL[19:16]RTC预分频装载值高位，fTR_CLK = fRTCCLK/(PRL[19:0]+1)，注：不推荐使用0值，否则无法正确的产生RTC中断和标志位。															
//
/****************** RTC_PRLL(RTC预分频装载寄存器低位) ******************/
#define  wRTC_PRLL		  	RTC->PRLL									//RTC预分频装载值低位	
									
/****************** RTC_DIVH(RTC预分频器余数寄存器高位) ******************/
#define  wRTC_DIVH		  	RTC->DIVH		//RTC时钟分频器余数高位,只读寄存器		

/****************** RTC_DIVL(RTC预分频器余数寄存器低位) ******************/
#define  wRTC_DIVL		  	RTC->DIVL		//RTC时钟分频器余数低位,只读寄存器	
												
/****************** RTC_CNTH（RTC计数器寄存器高位） ******************/
#define  wRTC_CNTH		  	RTC->CNTH
//RTC计数器高位，可通过读RTC_CNTH寄存器来获得RTC计数器当前值的高位部分。要对此寄存器进行写操作前，必须先进入配置模式

/****************** RTC_CNTL（RTC计数器寄存器低位） ******************/
#define  wRTC_CNTL		  	RTC->CNTL
//RTC计数器低位，可通过读RTC_CNTH寄存器来获得RTC计数器当前值的低位部分。要对此寄存器进行写操作前，必须先进入配置模式															

/****************** RTC_ALRH（RTC闹钟寄存器高位） ******************/
#define  wRTC_ALRH		  	RTC->ALRH
//RTC闹钟值高位，此寄存器用来保存由软件写入的闹钟时间的高位部分。要对此寄存器进行写操作，必须先进入配置模式

/****************** RTC_ALRL（RTC闹钟寄存器低位） ******************/
#define  wRTC_ALRL		  	RTC->ALRL
//RTC闹钟值低位，此寄存器用来保存由软件写入的闹钟时间的低位部分。要对此寄存器进行写操作，必须先进入配置模式															


/******************************************************************************/
/*                                                                            */
/*         USART寄存器                                   											*/
/*                                                                            */
/******************************************************************************/

/****************** USART_SR 状态寄存器 ******************/
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


/****************** USART_CR1 控制寄存器  ******************/
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

/****************** USART_CR2 控制寄存器  ******************/
#define  SET_URT_CR_ADD(USARTx_BASE,a)   	SET_REG_BITn(USARTx_BASE+0x10,0,MASKb5,a)
					//设置本设备的USART节点地址(BIT 0:4共5位)。这是在多处理器通信下的静默模式中使用的，使用地址标记来唤醒某个USART设备。

#define  bURT_CR_LBDL(USARTx_BASE)    BIT_ADDR(USARTx_BASE+0x10, 5)  //LIN断开符检测长度（LIN Break Detection Length）位；该位用来选择是11位还是10位的断开符检测。
#define  bURT_CR_LBDIE(USARTx_BASE)   BIT_ADDR(USARTx_BASE+0x10, 6)  //LIN断开符检测中断使能（LIN Break Detection Interrupt Enable）位；置0，禁止中断；置1，只要USART_SR寄存器中的LBD为’1’就产生中断。
#define  bURT_CR_LBCL(USARTx_BASE)    BIT_ADDR(USARTx_BASE+0x10, 7)  //最后一位时钟脉冲（Last Bit Clock Pulse）位；在同步模式下，使用该位来控制是否在CK引脚上输出最后发送的那个数据字节(MSB)对应的时钟脉冲。
#define  bURT_CR_CPHA(USARTx_BASE)    BIT_ADDR(USARTx_BASE+0x10, 9)  //时钟相位（Clock Phase）位；在同步模式下，可以用该位选择SLCK引脚上时钟输出的相位。
#define  bURT_CR_CPOL(USARTx_BASE)    BIT_ADDR(USARTx_BASE+0x10, 10) //时钟极性（Clock Polarity）位；在同步模式下，可以用该位选择SLCK引脚上时钟输出的极性。
#define  bURT_CR_CLKEN(USARTx_BASE)   BIT_ADDR(USARTx_BASE+0x10, 11) //时钟使能（Clock Enable）位；该位用来使能CK引脚。
#define  SET_URT_CR_STOP(USARTx_BASE,a)   	SET_REG_BITn(USARTx_BASE+0x10,12,MASKb2,a)
						//停止位（STOP）位；这2位用来设置停止位的位数；00：1个停止位；01：0.5个停止位；10：2个停止位；11：1.5个停止位；
#define  bURT_CR_STOP_0(USARTx_BASE)  BIT_ADDR(USARTx_BASE+0x10, 12) //           
#define  bURT_CR_STOP_1(USARTx_BASE)  BIT_ADDR(USARTx_BASE+0x10, 13) //
#define  bURT_CR_LINEN(USARTx_BASE)   BIT_ADDR(USARTx_BASE+0x10, 14) //LIN模式使能（LIN Enable）位；在LIN模式下，可以用USART_CR1寄存器中的SBK位发送LIN同步断开符(低13位)，以及检测LIN同步断开符。

/****************** USART_CR3 控制寄存器  ******************/
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

#define  SET_SPI_CR_BR(SPIx_BASE,a)   	SET_REG_BITn(SPIx_BASE,3,MASKb3,a)
//取值定义如下： SPI总线速度设置
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

//15位：BIDIMODE双向数据模式使能，定义：0（双线双向模式），1（单线双向模式）															
//14位：BIDIOE双向模式下的输出使能，和BIDIMODE位一起决定在“单线双向”模式下数据的输出方向。定义：0（输出禁止(只收模式)），1（输出使能(只发模式)）															
//      这个“单线”数据线在主设备端为MOSI引脚，在从设备端为MISO引脚。															
//13位：CRCEN硬件CRC校验使能，定义：0（禁止CRC计算），1（启动CRC计算）注：只有在禁止SPI时(SPE=0)，才能写该位，否则出错。该位只能在全双工模式下使用。															
//12位：CRCNEXT下一个发送CRC，定义：0（下一个发送的值来自发送缓冲区），1（下一个发送的值来自发送CRC寄存器），															
//      注：在SPI_DR寄存器写入最后一个数据后应马上设置该位。															
//11位：DFF数据帧格式，定义：0（使用8位数据帧格式进行发送/接收），1（使用16位数据帧），注：只有当SPI禁止(SPE=0)时，才能写该位，否则出错。															
//10位：RXONLY只接收，该位和BIDIMODE位一起决定在“双线双向”模式下的传输方向。在多个从设备的配置中，在未被访问的从设备上该位被置1，															
//      使得只有被访问的从设备有输出，从而不会造成数据线上数据冲突。定义：0（全双工(发送和接收)），1（禁止输出(只接收模式)）															
//9位：SSM软件从设备管理，当SSM被置位时，NSS引脚上的电平由SSI位的值决定。定义：0（禁止软件从设备管理），1（启用软件从设备管理）															
//8位：SSI内部从设备选择，该位只在SSM位为’1’时有意义。它决定了NSS上的电平，在NSS引脚上的I/O操作无效。															
//7位：LSBFIRST帧格式，定义：0（先发送MSB），1（先发送LSB），注：当通信在进行时不能改变该位的值。															
//6位：SPE-SPI使能，定义：0（禁止SPI设备），1（开启SPI设备）															
//5-3位：BR[2:0]波特率控制，定义：000（fPCLK/2）001（fPCLK/4）010（fPCLK/8）011（fPCLK/16）100（/32）101（/64）110（/128）111（/256）注：通信时不能改															
//2位：MSTR主设备选择，定义：0（配置为从设备），1（配置为主设备），注：当通信正在进行的时候，不能修改该位。															
//1位：CPOL时钟极性，定义：0（空闲状态时，SCK保持低电平），1（空闲状态时，SCK保持高电平），注：当通信正在进行的时候，不能修改该位。															
//0位：CPHA时钟相位，定义：0（数据采样从第一个时钟边沿开始），1（数据采样从第二个时钟边沿开始）注：当通信正在进行的时候，不能修改该位。															


/*******************    SPI_CR2 寄存器  ********************/
#define  bSPI_CR_RXDMAEN(SPIx_BASE)  BIT_ADDR(SPIx_BASE+0x04, 0)               /*!< Rx Buffer DMA Enable */
#define  bSPI_CR_TXDMAEN(SPIx_BASE)  BIT_ADDR(SPIx_BASE+0x04, 1)               /*!< Tx Buffer DMA Enable */
#define  bSPI_CR_SSOE(SPIx_BASE)     BIT_ADDR(SPIx_BASE+0x04, 2)               /*!< SS Output Enable */
#define  bSPI_CR_ERRIE(SPIx_BASE)    BIT_ADDR(SPIx_BASE+0x04, 4)               /*!< Error Interrupt Enable */
#define  bSPI_CR_RXNEIE(SPIx_BASE)   BIT_ADDR(SPIx_BASE+0x04, 6)               /*!< RX buffer Not Empty Interrupt Enable */
#define  bSPI_CR_TXEIE(SPIx_BASE)    BIT_ADDR(SPIx_BASE+0x04, 7)               /*!< Tx buffer Empty Interrupt Enable */
//7位：TXEIE发送缓冲区空中断使能，定义：0（禁止TXE中断），1（允许TXE中断，当TXE标志置位为’1’时产生中断请求）															
//6位：RXNEIE接收缓冲区非空中断使能，定义：0（禁止RXNE中断），1（允许RXNE中断，当RXNE标志置位时产生中断请求）															
//5位：ERRIR错误中断使能，当错误(CRCERR、OVR、MODF)产生时，该位控制是否产生中断，定义：0（禁止错误中断），1（允许错误中断）															
//2位：SSOE-SS输出使能,定义：0（禁止在主模式下SS输出，该设备可以工作在多主设备模式）,1（设备开启时，开启主模式下SS输出，该设备不能工作在多主设备模式															
//1位：TXDMAEN发送缓冲区DMA使能，当该位被设置时，TXE标志一旦被置位就发出DMA请求，定义：0（禁止发送缓冲区DMA），1（启动发送缓冲区DMA）															
//0位：RXDMAEN接收缓冲区DMA使能，当该位被设置时，RXNE标志一旦被置位就发出DMA请求，定义：0（禁止接收缓冲区DMA），1（启动接收缓冲区DMA）															


/********************    SPI_SR 寄存器  ********************/
#define  bSPI_SR_RXNE(SPIx_BASE)     BIT_ADDR(SPIx_BASE+0x08, 0)              /*!< Receive buffer Not Empty */
#define  bSPI_SR_TXE(SPIx_BASE)      BIT_ADDR(SPIx_BASE+0x08, 1)               /*!< Transmit buffer Empty */
#define  bSPI_SR_CHSIDE(SPIx_BASE)   BIT_ADDR(SPIx_BASE+0x08, 2)               /*!< Channel side */
#define  bSPI_SR_UDR(SPIx_BASE)      BIT_ADDR(SPIx_BASE+0x08, 3)               /*!< Underrun flag */
#define  bSPI_SR_CRCERR(SPIx_BASE)   BIT_ADDR(SPIx_BASE+0x08, 4)               /*!< CRC Error flag */
#define  bSPI_SR_MODF(SPIx_BASE)     BIT_ADDR(SPIx_BASE+0x08, 5)               /*!< Mode fault */
#define  bSPI_SR_OVR(SPIx_BASE)      BIT_ADDR(SPIx_BASE+0x08, 6)               /*!< Overrun flag */
#define  bSPI_SR_BSY(SPIx_BASE)      BIT_ADDR(SPIx_BASE+0x08, 7)               /*!< Busy flag */
//7位：BSY忙标志，定义：0（SPI不忙），1（SPI正忙于通信，或者发送缓冲非空）该位由硬件置位或者复位，使用这个标志时需要特别注意23.3.7节和.8节															
//6位：OVR溢出标志，定义：0（没有出现溢出错误），1（出现溢出错误），该位由硬件置位，由软件序列复位。关于软件序列的详细信息，参考23.4.7节。															
//5位：MODF模式错误，定义：0（没有出现模式错误），1（出现模式错误），该位由硬件置位，由软件序列复位。关于软件序列的详细信息，参考23.3.10节															
//4位：CRCERR-CRC错误标志，定义：0（收到的CRC值和SPI_RXCRCR寄存器中的值匹配），1（收到的CRC值和SPI_RXCRCR寄存器中的值不匹配）硬件置位，软件写0而复位															
//3位：UDR下溢标志位，定义：0（未发生下溢），1（发生下溢）该标志位由硬件置’1’，由一个软件序列清’0’，注：在SPI模式下不使用。															
//2位：CHSIDE声道，定义：0（需要传输或者接收左声道），1（需要传输或者接收右声道）注：在SPI模式下不使用。在PCM模式下无意义。															
//1位：TXE发送缓冲为空，定义：0（发送缓冲非空），1（发送缓冲为空）															
//0位：RXNE接收缓冲非空，定义：0（接收缓冲为空），1（接收缓冲非空）															


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
//11位：I2SMOD-I2S模式选择，定义：0（选择SPI模式），1（选择I2S模式）															
//10位：I2SE-I2S使能，定义：0（关闭I2S），1（I2S使能），注：在SPI模式下不使用。															
//9-8位：I2SCFG-I2S模式设置，定义：00(从设备发送)01(从设备接收)10(主设备发送)11(主设备接受)，注：该位只有在关闭了I2S时才能设置。在SPI模式下不使用。															
//7位：PCMSYNC-PCM帧同步，定义：0（短帧同步），1（长帧同步），注: 该位只在I2SSTD = 11 (使用PCM标准)时有意义。在SPI模式下不使用。															
//5-4位：I2SSTD-I2S标准选择，定义：00:(I2S飞利浦标准),01(高字节对齐标准 (左对齐),10(低字节对齐标准(右对齐),11(PCM 标准)															
//       注：为了正确操作，只有在关闭了I2S时才能设置该位。在SPI模式下不使用。															
//3位：CKPOL静止态时钟极性,定义：0(I2S时钟静止态为低电平),1(I2S时钟静止态为高电平),注:为了正确操作，该位只有在关闭了I2S时才能设置,在SPI模式下不使用。															
//2-1位：DATLEN待传输数据长度，定义：00(16位数据长度)01(24位数据长度)10(32位数据长度)11(不允许)注：该位只有在关闭了I2S时才能设置。在SPI模式下不使用。															
//0位：CHLEN声道长度 (每个音频声道的数据位数)，定义：0（16位宽），1（32位宽），只有在DATLEN=00时该位的写操作才有意义，否则声道长度都由硬件固定为32位															
//     注：为了正确操作，该位只有在关闭了I2S时才能设置。在SPI模式下不使用。															


/******************    SPI_I2SPR 寄存器  *******************/
#define  SET_SPI_I2SPR_I2SDIV(SPIx_BASE,a)   	SET_REG_BITn(SPIx_BASE+0x20,0,MASKb8,a)
// #define  bSPI1_I2SPR_I2SDIV      BIT_ADDR(SPI1_BASE+0x20, 0)  //长度8BIT I2S 线性预分频

#define  bSPI_I2SPR_ODD(SPIx_BASE)         BIT_ADDR(SPIx_BASE+0x20, 8)            /*!< Odd factor for the prescaler */
#define  bSPI_I2SPR_MCKOE(SPIx_BASE)       BIT_ADDR(SPIx_BASE+0x20, 9)            /*!< Master Clock Output Enable */
//9位：MCKOE主设备时钟输出使能，定义：0（关闭主设备时钟输出），1（主设备时钟输出使能）															
//    为了正确操作，该位只有在关闭了I2S时才能设置。仅在I2S主设备模式下使用该位。在SPI模式下不使用。															
//8位：ODD奇系数预分频，定义：0（实际分频系数=I2SDIV *2），1（实际分频系数=(I2SDIV * 2)+1）															
//    注：为了正确操作，该位只有在关闭了I2S时才能设置。仅在I2S主设备模式下使用该位。在SPI模式下不使用。															
//7-0位：I2SDIV-I2S线性预分频，禁止设置I2SDIV [7:0] = 0或者I2SDIV [7:0] = 1。参见23.4.3节															
//       注：为了正确操作，该位只有在关闭了I2S时才能设置。仅在I2S主设备模式下使用该位。在SPI模式下不使用。															



/******************************************************************************/
/*                                                                            */
/*                        TIM定时器接口寄存器                                  */
/*                                                                            */
/******************************************************************************/

/*******************  TIM_CR1 控制寄存器  ********************/
#define   bTIM_CR_CEN(TIMx_BASE)                         BIT_ADDR(TIMx_BASE,0)            /*!< 使能计数器,定义：0（禁止计数器），1（使能计数器） */
#define   bTIM_CR_UDIS(TIMx_BASE)                        BIT_ADDR(TIMx_BASE,1)            /*!< 禁止更新，软件通过该位允许/禁止UEV事件的产生 */
#define   bTIM_CR_URS(TIMx_BASE)                         BIT_ADDR(TIMx_BASE,2)            /*!< 更新请求源,软件通过该位选择UEV事件的源 */
#define   bTIM_CR_OPM(TIMx_BASE)                         BIT_ADDR(TIMx_BASE,3)            /*!< 单脉冲模式，定义：0（在发生更新事件时，计数器不停止），1（在发生下一次更新事件(清除CEN位)时，计数器停止） */
#define   bTIM_CR_DIR(TIMx_BASE)                         BIT_ADDR(TIMx_BASE,4)            /*!< 计数方向，定义：0（向上），1（向下），
																																													注：当计数器配置为中央对齐模式或编码器模式时，该位为只读 */

#define  SET_TIM_CR_CMS(TIMx_BASE,a)   	SET_REG_BITn(TIMx_BASE,5,MASKb2,a)
/*!< CMS[1:0] 中央对齐模式选择，定义：00：边沿对齐模式。计数器依据方向位(DIR)向上或向下计数。 */
/*!< 01、10、11（中央对齐模式1、2、3。计数器交替地向上和向下计数。详略*/

#define   bTIM_CR_ARPE(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x00,7)  /*!< 自动重装载预装载允许位，定义：0（TIMx_ARR寄存器没有缓冲），1（TIMx_ARR寄存器被装入缓冲器） */

#define  SET_TIM_CR_CKD(TIMx_BASE,a)   	SET_REG_BITn(TIMx_BASE,8,MASKb2,a)
/*!< CKD[1:0] CKD[1:0]时钟分频因子，定义在定时器时钟(CK_INT)频率与数字滤波器(ETR，TIx)使用的采样频率之间的分频比例。 */
/*!< 定义：00（tDTS = tCK_INT），01（tDTS = 2 x tCK_INT），10（tDTS = 4 x tCK_INT）11：保留 */

/*******************  TIM_CR2 控制寄存器  ********************/
#define   bTIM_CR_CCPC(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x04,0)            /*!< 捕获/比较预装载控制位，定义：0(CCxE，CCxNE和OCxM位不是预装载的),1(CCxE,CCxNE和OCxM位是预装载的;设置该位后,它们只在设置了COM位后被更新) */
#define   bTIM_CR_CCUS(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x04,2)            /*!< 捕获/比较控制更新选择 */
#define   bTIM_CR_CCDS(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x04,3)            /*!< 捕获/比较的DMA选择，定义：0（当发生CCx事件时，送出CCx的DMA请求），1（当发生更新事件时，送出CCx的DMA请求） */

#define  SET_TIM_CR_MMS(TIMx_BASE,a)   	SET_REG_BITn(TIMx_BASE+0x04,4,MASKb3,a)
/*!< 6-4位：MMS[2:0] bits (Master Mode Selection) */
//		MMS主模式选择，这3位用于选择在主模式下送到从定时器的同步信息(TRGO)定义：															
//       000：复位 – TIMx_EGR寄存器的UG位被用于作为触发输出(TRGO)。如果是触发输入产生的复位(从模式控制器处于复位模式)，															
//                    则TRGO上的信号相对实际的复位会有一个延迟															
//       001：使能 – 计数器使能信号CNT_EN被用于作为触发输出(TRGO)。有时需要在同一时间启动多个定时器或控制在一段时间内使能从定时器。															
//                    计数器使能信号是通过CEN控制位和门控模式下的触发输入信号的逻辑或产生。															
//            当计数器使能信号受控于触发输入时，TRGO上会有一个延迟，除非选择了主/从模式(见TIMx_SMCR寄存器中MSM位的描述)。															
//       010：更新 – 更新事件被选为触发输入(TRGO)。例如，一个主定时器的时钟可以被用作一个从定时器的预分频器															
//       011：比较脉冲 – 在发生一次捕获或一次比较成功时，当要设置CC1IF标志时(即使它已经为高)，触发输出送出一个正脉冲(TRGO)															
//       100：比较 – OC1REF信号被用于作为触发输出(TRGO)  101：比较 – OC2REF信号被用于作为触发输出(TRGO)															
//       110：比较 – OC3REF信号被用于作为触发输出(TRGO)  111：比较 – OC4REF信号被用于作为触发输出(TRGO)															


#define   bTIM_CR_TI1S(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x04,7)            /*!< TI1 Selection */
#define   bTIM_CR_OIS1(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x04,8)            /*!< Output Idle state 1 (OC1 output) */
#define   bTIM_CR_OIS1N(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x04,9)            /*!< Output Idle state 1 (OC1N output) */
#define   bTIM_CR_OIS2(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x04,10)            /*!< Output Idle state 2 (OC2 output) */
#define   bTIM_CR_OIS2N(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x04,11)            /*!< Output Idle state 2 (OC2N output) */
#define   bTIM_CR_OIS3(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x04,12)            /*!< Output Idle state 3 (OC3 output) */
#define   bTIM_CR_OIS3N(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x04,13)            /*!< Output Idle state 3 (OC3N output) */
#define   bTIM_CR_OIS4(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x04,14)            /*!< Output Idle state 4 (OC4 output) */
//13,11,9位：OIS（1-4）N输出空闲状态（OC1N），定义：0（当MOE=0时，如果实现了OC1N，则死区后OC1=0),1(：当MOE=0时，如果实现了OC1N，则死区后OC1=1)															
//           注：已经设置了LOCK(TIMx_BKR寄存器)级别1、2或3后，该位不能被修改															
//14,12,10,8位：OIS1输出空闲状态1（OC1输出），定义：0（当MOE=0时，如果实现了OC1N，则死区后OC1=0），1（当MOE=0时，如果实现了OC1N，则死区后OC1=1）															
//              注：已经设置了LOCK(TIMx_BKR寄存器)级别1、2或3后，该位不能被修改															
//7位：TI1S-TI1选择，定义：0（TIMx_CH1引脚连到TI1输入），1（TIMx_CH1、TIMx_CH2和TIMx_CH3引脚经异或后连到TI1输入）															


/*******************  TIM_SMCR 寄存器  *******************/
#define  SET_TIM_SMCR_SMS(TIMx_BASE,a)   	SET_REG_BITn(TIMx_BASE+0x08,0,MASKb3,a)/*!< SMS[2:0] bits (Slave mode selection) */

#define  SET_TIM_SMCR_TS(TIMx_BASE,a)   	SET_REG_BITn(TIMx_BASE+0x08,4,MASKb3,a) /*!< TS[2:0] bits (Trigger selection) */

#define   bTIM_SMCR_MSM(TIMx_BASE)        BIT_ADDR(TIMx_BASE+0x08,7)            /*!< Master/slave mode */

#define  SET_TIM_SMCR_ETF(TIMx_BASE,a)   	SET_REG_BITn(TIMx_BASE+0x08,8,MASKb4,a)	/*!< ETF[3:0] bits (External trigger filter) */

#define  SET_TIM_SMCR_ETPS(TIMx_BASE,a)   SET_REG_BITn(TIMx_BASE+0x08,12,MASKb2,a)	  /*!< ETPS[1:0] bits (External trigger prescaler) */

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

#define  SET_TIM_CCMR_OC1M(TIMx_BASE,a)   	SET_REG_BITn(TIMx_BASE+0x18,4,MASKb3,a)  /*!< OC1M[2:0] bits (Output Compare 1 Mode) */

#define   bTIM_CCMR_OC1CE(TIMx_BASE)                     BIT_ADDR(TIMx_BASE +0x18,7)            /*!< Output Compare 1Clear Enable */

#define   bTIM_CCMR_CC2S(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x18,8)            /*!< CC2S[1:0] bits (Capture/Compare 2 Selection) */
#define   bTIM_CCMR_CC2S_0(TIMx_BASE)                    BIT_ADDR(TIMx_BASE +0x18,8)            /*!< Bit 0 */
#define   bTIM_CCMR_CC2S_1(TIMx_BASE)                    BIT_ADDR(TIMx_BASE +0x18,9)            /*!< Bit 1 */

#define   bTIM_CCMR_OC2FE(TIMx_BASE)                     BIT_ADDR(TIMx_BASE +0x18,10)            /*!< Output Compare 2 Fast enable */
#define   bTIM_CCMR_OC2PE(TIMx_BASE)                     BIT_ADDR(TIMx_BASE +0x18,11)            /*!< Output Compare 2 Preload enable */

#define  	SET_TIM_CCMR_OC2M(TIMx_BASE,a)   	SET_REG_BITn(TIMx_BASE+0x18,12,MASKb3,a)  /*!< OC2M[2:0] bits (Output Compare 2 Mode) */

#define   bTIM_CCMR_OC2CE(TIMx_BASE)                     BIT_ADDR(TIMx_BASE +0x18,15)            /*!< Output Compare 2 Clear Enable */

/*----------------------------------------------------------------------------*/

#define   bTIM_CCMR_IC1PSC(TIMx_BASE)                    BIT_ADDR(TIMx_BASE +0x18,2)            /*!< IC1PSC[1:0] bits (Input Capture 1 Prescaler) */
#define   bTIM_CCMR_IC1PSC_0(TIMx_BASE)                  BIT_ADDR(TIMx_BASE +0x18,2)            /*!< Bit 0 */
#define   bTIM_CCMR_IC1PSC_1(TIMx_BASE)                  BIT_ADDR(TIMx_BASE +0x18,3)            /*!< Bit 1 */

#define  	SET_TIM_CCMR_IC1F(TIMx_BASE,a)   	SET_REG_BITn(TIMx_BASE+0x18,4,MASKb4,a)  /*!< IC1F[3:0] bits (Input Capture 1 Filter) */

#define   bTIM_CCMR_IC2PSC(TIMx_BASE)                    BIT_ADDR(TIMx_BASE +0x18,10)            /*!< IC2PSC[1:0] bits (Input Capture 2 Prescaler) */
#define   bTIM_CCMR_IC2PSC_0(TIMx_BASE)                  BIT_ADDR(TIMx_BASE +0x18,10)            /*!< Bit 0 */
#define   bTIM_CCMR_IC2PSC_1(TIMx_BASE)                  BIT_ADDR(TIMx_BASE +0x18,11)            /*!< Bit 1 */

#define  	SET_TIM_CCMR_IC2F(TIMx_BASE,a)   	SET_REG_BITn(TIMx_BASE+0x18,12,MASKb4,a)   /*!< IC2F[3:0] bits (Input Capture 2 Filter) */

/******************    TIM_CCMR2 寄存器  *******************/
#define   bTIM_CCMR_CC3S(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x1C,0)            /*!< CC3S[1:0] bits (Capture/Compare 3 Selection) */
#define   bTIM_CCMR_CC3S_0(TIMx_BASE)                    BIT_ADDR(TIMx_BASE +0x1C,0)            /*!< Bit 0 */
#define   bTIM_CCMR_CC3S_1(TIMx_BASE)                    BIT_ADDR(TIMx_BASE +0x1C,1)            /*!< Bit 1 */

#define   bTIM_CCMR_OC3FE(TIMx_BASE)                     BIT_ADDR(TIMx_BASE +0x1C,2)            /*!< Output Compare 3 Fast enable */
#define   bTIM_CCMR_OC3PE(TIMx_BASE)                     BIT_ADDR(TIMx_BASE +0x1C,3)            /*!< Output Compare 3 Preload enable */

#define  	SET_TIM_CCMR_OC3M(TIMx_BASE,a)   	SET_REG_BITn(TIMx_BASE+0x1C,4,MASKb3,a)   /*!< OC3M[2:0] bits (Output Compare 3 Mode) */


#define   bTIM_CCMR_OC3CE(TIMx_BASE)                     BIT_ADDR(TIMx_BASE +0x1C,7)            /*!< Output Compare 3 Clear Enable */

#define   bTIM_CCMR_CC4S(TIMx_BASE)                      BIT_ADDR(TIMx_BASE +0x1C,8)            /*!< CC4S[1:0] bits (Capture/Compare 4 Selection) */
#define   bTIM_CCMR_CC4S_0(TIMx_BASE)                    BIT_ADDR(TIMx_BASE +0x1C,8)            /*!< Bit 0 */
#define   bTIM_CCMR_CC4S_1(TIMx_BASE)                    BIT_ADDR(TIMx_BASE +0x1C,9)            /*!< Bit 1 */

#define   bTIM_CCMR_OC4FE(TIMx_BASE)                     BIT_ADDR(TIMx_BASE +0x1C,10)            /*!< Output Compare 4 Fast enable */
#define   bTIM_CCMR_OC4PE(TIMx_BASE)                     BIT_ADDR(TIMx_BASE +0x1C,11)            /*!< Output Compare 4 Preload enable */

#define  	SET_TIM_CCMR_OC4M(TIMx_BASE,a)   	SET_REG_BITn(TIMx_BASE+0x1C,12,MASKb3,a)   /*!< OC4M[2:0] bits (Output Compare 4 Mode) */


#define   bTIM_CCMR_OC4CE(TIMx_BASE)                     BIT_ADDR(TIMx_BASE +0x1C,15)            /*!< Output Compare 4 Clear Enable */

/*----------------------------------------------------------------------------*/

#define   bTIM_CCMR_IC3PSC(TIMx_BASE)                    BIT_ADDR(TIMx_BASE +0x1C,2)            /*!< IC3PSC[1:0] bits (Input Capture 3 Prescaler) */
#define   bTIM_CCMR_IC3PSC_0(TIMx_BASE)                  BIT_ADDR(TIMx_BASE +0x1C,2)            /*!< Bit 0 */
#define   bTIM_CCMR_IC3PSC_1(TIMx_BASE)                  BIT_ADDR(TIMx_BASE +0x1C,3)            /*!< Bit 1 */

#define  	SET_TIM_CCMR_IC3F(TIMx_BASE,a)   	SET_REG_BITn(TIMx_BASE+0x1C,4,MASKb4,a)    /*!< IC3F[3:0] bits (Input Capture 3 Filter) */

#define   bTIM_CCMR_IC4PSC(TIMx_BASE)                    BIT_ADDR(TIMx_BASE +0x1C,10)            /*!< IC4PSC[1:0] bits (Input Capture 4 Prescaler) */
#define   bTIM_CCMR_IC4PSC_0(TIMx_BASE)                  BIT_ADDR(TIMx_BASE +0x1C,10)            /*!< Bit 0 */
#define   bTIM_CCMR_IC4PSC_1(TIMx_BASE)                  BIT_ADDR(TIMx_BASE +0x1C,11)            /*!< Bit 1 */

#define  	SET_TIM_CCMR_IC4F(TIMx_BASE,a)   	SET_REG_BITn(TIMx_BASE+0x1C,12,MASKb4,a)    /*!< IC4F[3:0] bits (Input Capture 4 Filter) */

//输出比较和输入捕获功能不同，在寄存器中的设置也不同。															
//输出比较模式															
//15位：OC2CE：输出比较2清0使能															
//14-12位：OC2M[2:0]：输出比较2模式															
//11位：OC2PE：输出比较2预装载使能															
//10位：OC2FE：输出比较2快速使能															
//9-8位：CC2S[1:0]：捕获/比较2选择,该位定义通道的方向(输入/输出),及输入脚的选择,定义：00(CC2通道被配置为输出) 01(CC2通道被配置为输入,IC2映射在TI2上)															
//       10(CC2通道被配置为输入，IC2映射在TI1上) 11(CC2通道被配置为输入，IC2映射在TRC上。此模式仅工作在内部触发器输入被选中时。															
//       (由TIMx_SMCR寄存器的TS位选择),注：CC2S仅在通道关闭时(TIMx_CCER寄存器的CC2E=0)才是可写的。															
//9-8位：CC4S[1:0]：捕获/比较4选择;该位定义通道的方向(输入/输出);及输入脚的选择,定义:00(CC4通道被配置为输出)01(CC4通道被配置为输入,IC4映射在TI4上)															
//       10(CC4通道被配置为输入，IC4映射在TI3上) 11(CC4通道被配置为输入，IC4映射在TRC上。此模式仅工作在内部触发器输入被选中时。															
//       (由TIMx_SMCR寄存器的TS位选择),注：CC4S仅在通道关闭时(TIMx_CCER寄存器的CC4E=0)才是可写的。															
//7位：OC1CE：输出比较1清’0’使能，定义：0（OC1REF 不受ETRF输入的影响），1（一旦检测到ETRF输入高电平，清除OC1REF=0）															
//6-4位：OC1M[2:0]输出比较1模式，该3位定义了输出参考信号OC1REF的动作，而OC1REF决定了OC1、OC1N的值。OC1REF是高电平有效，而OC1、OC1N															
//       的有效电平取决于CC1P、CC1NP位，定义：000（冻结。输出比较寄存器TIMx_CCR1与计数器TIMx_CNT间的比较对OC1REF不起作用)															
//       001(匹配时设置通道1为有效电平。当计数器TIMx_CNT的值与捕获/比较寄存器1 (TIMx_CCR1)相同时，强制OC1REF为高),010(强制OC1REF为低）															
//       011（翻转。当TIMx_CCR1=TIMx_CNT时，翻转OC1REF的电平）100（强制为无效电平。强制OC1REF为低）101（强制OC1REF为高）															
//       110：PWM模式1－ 在向上计数时，一旦TIMx_CNT<TIMx_CCR1时通道1为有效电平，否则为无效电平；在向下计数时，一旦TIMx_CNT>TIMx_CCR1时通道1															
//            为无效电平(OC1REF=0)，否则为有效电平(OC1REF=1)。															
//       111：PWM模式2－ 在向上计数时，一旦TIMx_CNT<TIMx_CCR1时通道1为无效电平，否则为有效电平；在向下计数时，一旦TIMx_CNT>TIMx_CCR1时通道1															
//            为有效电平，否则为无效电平。															
//       注1：一旦LOCK级别设为3(TIMx_BDTR寄存器中的LOCK位)并且CC1S=00(该通道配置成输出)则该位不能被修改。															
//       注2：在PWM模式1或PWM模式2中，只有当比较结果改变了或在输出比较模式中从冻结模式切换到PWM模式时，OC1REF电平才改变。															
//3位：OC1PE输出比较1预装载使能，定义：0（禁止TIMx_CCR1寄存器的预装载功能，可随时写入TIMx_CCR1寄存器，并且新写入的数值立即起作用）															
//     1（开启TIMx_CCR1寄存器的预装载功能，读写操作仅对预装载寄存器操作，TIMx_CCR1的预装载值在更新事件到来时被加载至当前寄存器中）															
//     注1：一旦LOCK级别设为3(TIMx_BDTR寄存器中的LOCK位)并且CC1S=00(该通道配置成输出)则该位不能被修改。															
//     注2：仅在单脉冲模式下(TIMx_CR1寄存器的OPM=1)，可以在未确认预装载寄存器情况下使用PWM模式，否则其动作不确定															
//2位：OC1FE输出比较1 快速使能，该位用于加快CC输出对触发输入事件的响应，定义：															
//     0（根据计数器与CCR1的值，CC1正常操作，即使触发器是打开的。当触发器的输入有一个有效沿时，激活CC1输出的最小延时为5个时钟周期）															
//     1（输入到触发器的有效沿的作用就象发生了一次比较匹配。因此，OC被设置为比较电平而与比较结果无关。采样触发器的有效沿和CC1输出间的延时被缩短为															
//       3个时钟周期）OCFE只在通道被配置成PWM1或PWM2模式时起作用。															
//1-0位：CC1S[1:0]捕获/比较1 选择,这2位定义通道的方向(输入/输出),及输入脚的选择,定义:00(CC1通道被配置为输出),01(CC1通道被配置为输入,IC1映射在TI1上)															
//   10(CC1通道被配置为输入,IC1映射在TI2上),11(CC1通道被配置为输入,IC1映射在TRC上.此模式仅工作在内部触发器输入被选中时(由TIMx_SMCR寄存器的TS位选择)															
//     注：CC1S仅在通道关闭时(TIMx_CCER寄存器的CC1E=0)才是可写的。															
//1-0位：CC3S[1:0]捕获/比较3 选择,这2位定义通道的方向(输入/输出),及输入脚的选择,定义:00(CC3通道被配置为输出),01(CC3通道被配置为输入,IC3映射在TI3上)															
//   10(CC3通道被配置为输入,IC3映射在TI4上),11(CC3通道被配置为输入,IC3映射在TRC上.此模式仅工作在内部触发器输入被选中时(由TIMx_SMCR寄存器的TS位选择)															
//     注：CC3S仅在通道关闭时(TIMx_CCER寄存器的CC3E=0)才是可写的。															
//输入捕获模式															
//15-12位：输入捕获2滤波器															
//11-10位：CC2S[1:0]输入/捕获2预分频器															
//9-8位：CC2S[1:0]捕获/比较2选择，这2位定义通道的方向(输入/输出)，及输入脚的选择，定义：00（CC2通道被配置为输出）01（CC2通道被配置为输入，															
//       IC2映射在TI2上）,10（CC2通道被配置为输入，IC2映射在TI1上）,11(CC2通道被配置为输入，IC2映射在TRC上),此模式仅工作在内部触发器输入被选中时															
//       (由TIMx_SMCR寄存器的TS位选择),注：CC2S仅在通道关闭时(TIMx_CCER寄存器的CC2E=0)才是可写的。															
//9-8位：CC4S[1:0]捕获/比较4选择，这2位定义通道的方向(输入/输出)，及输入脚的选择，定义：00（CC4通道被配置为输出）01（CC4通道被配置为输入，															
//       IC4映射在TI4上）,10（CC4通道被配置为输入，IC4映射在TI3上）,11(CC4通道被配置为输入，IC4映射在TRC上),此模式仅工作在内部触发器输入被选中时															
//       (由TIMx_SMCR寄存器的TS位选择),注：CC2S仅在通道关闭时(TIMx_CCER寄存器的CC4E=0)才是可写的。															
//7-4位：IC1F[3:0]:输入捕获1滤波器,这几位定义了TI1输入的采样频率及数字滤波器长度,数字滤波器由一个事件计数器组成,记录到N个事件后会产生一个输出的跳变															
//    定义：0000(无滤波器，以fDTS采样）0010（采样频率fSAMPLING=fCK_INT，N=4）0011（采样频率fSAMPLING=fCK_INT，N=8）0100（fSAMPLING=fDTS/2，N=6）															
//          0101（fSAMPLING=fDTS/2，N=8），0110（fSAMPLING=fDTS/4，N=6），0111（fSAMPLING=fDTS/4，N=8），1000（fSAMPLING=fDTS/8，N=6）															
//          1001（fSAMPLING=fDTS/8，N=8），1010（fSAMPLING=fDTS/16，N=5），1011（fSAMPLING=fDTS/16，N=6），1100（fSAMPLING=fDTS/16，N=8）															
//          1101（fSAMPLING=fDTS/32，N=5），0110（fSAMPLING=fDTS/4，N=6），1110（fSAMPLING=fDTS/32，N=6），0111（fSAMPLING=fDTS/4，N=8）															
//          1111（fSAMPLING=fDTS/32，N=8）															
//3-2位：IC1PSC[1:0]输入/捕获1预分频器，这2位定义了CC1输入(IC1)的预分频系数，一旦CC1E=0(TIMx_CCER寄存器中)，则预分频器复位。															
//       00（无预分频器，捕获输入口上检测到的每一个边沿都触发一次捕获),01(每2个事件触发一次捕获),10(每4个事件触发一次捕获),11(每8个事件触发一次捕获)															
//1-0位：CC1S[1:0]捕获/比较1选择,这2位定义通道的方向(输入/输出),及输入脚的选择,定义:00(CC1通道被配置为输出)，01(CC1通道被配置为输入，IC1映射在TI1上       															
//       10（CC1通道被配置为输入，IC1映射在TI2上）,11：CC1通道被配置为输入，IC1映射在TRC上。此模式仅工作在内部触发器输入被选中时															
//       (由TIMx_SMCR寄存器的TS位选择)。注：CC1S仅在通道关闭时(TIMx_CCER寄存器的CC1E=0)才是可写的。															
//1-0位：CC3S[1:0]捕获/比较3选择,这2位定义通道的方向(输入/输出),及输入脚的选择,定义:00(CC3通道被配置为输出)，01(CC3通道被配置为输入，IC3映射在TI3上       															
//       10（CC3通道被配置为输入，IC3映射在TI4上）,11：CC3通道被配置为输入，IC3映射在TRC上。此模式仅工作在内部触发器输入被选中时															
//       (由TIMx_SMCR寄存器的TS位选择)。注：CC3S仅在通道关闭时(TIMx_CCER寄存器的CC3E=0)才是可写的。	


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
//以下说明以通道1为例：
//CC1NP：输入/捕获1互补输出极性，定义：0（OC1N高电平有效），1（OC1N低电平有效）															
//           注：一旦LOCK级别(TIMx_BDTR寄存器中的LOCK位)设为3或2且CC1S=00(通道配置为输出)则该位不能被修改															
//CC1NE：输入/捕获1互补输出使能，定义：0（关闭－ OC1N禁止输出，因此OC1N的电平依赖于MOE、OSSI、OSSR、OIS1、OIS1N和CC1E位的值）															
//         1（开启－ OC1N信号输出到对应的输出引脚，其输出电平依赖于MOE、OSSI、OSSR、OIS1、OIS1N和CC1E位的值。															
//CC1P：输入/捕获1输出极性，定义：CC1通道配置为输出-0（OC1高电平有效），1（OC1低电平有效）															
//             CC1通道配置为输入：该位选择是IC1还是IC1的反相信号作为触发或捕获信号，0：不反相：捕获发生在IC1的上升沿；当用作外部触发器时，IC1不反相。															
//             1（相反，捕获发生在IC1的下降沿）注：一旦LOCK级别(TIMx_BDTR寄存器中的LOCK位)设为3或2，则该位不能被修改															
//CC1E：输入/捕获1输出使能，定义：CC1通道配置为输出-															
//                       0（关闭－ OC1禁止输出，因此OC1的输出电平依赖于MOE、OSSI、OSSR、OIS1、OIS1N和CC1NE位的值），1（开启）															
//                       CC1通道配置为输入，该位决定了计数器的值是否能捕获入TIMx_CCR1寄存器。0（捕获禁止），1（捕获使能）															


/*******************    TIM_CNT 寄存器  ********************/
#define   wTIM_CNT(TIMx_BASE)           (MEM_ADDR(TIMx_BASE+0x24))    /*!0-15位：CNT[15:0]计数器的值	 */             

/*******************    TIM_PSC 寄存器  ********************/
#define   wTIM_PSC(TIMx_BASE)           (MEM_ADDR(TIMx_BASE+0x28))            /*!< Prescaler Value */
//0-15位：PSC[15:0]预分频器的值，计数器的时钟频率(CK_CNT)等于fCK_PSC/( PSC[15:0]+1)。															
//        PSC包含了每次当更新事件产生时，装入当前预分频器寄存器的值；更新事件包括计数器被TIM_EGR的UG位清’0’或被工作在复位模式的从控制器清’0’															


/*******************    TIM_ARR 寄存器  ********************/
#define   wTIM_ARR(TIMx_BASE)           (MEM_ADDR(TIMx_BASE+0x2C))            /*!< actual auto-reload Value */
//15-0位：ARR[15:0]自动重装载的值，ARR包含了将要传送至实际的自动重装载寄存器的数值，当自动重装载的值为空时，计数器不工作															


/*******************    TIM_RCR 寄存器  ********************/
#define   wTIM_RCR(TIMx_BASE)           (MEM_ADDR(TIMx_BASE+0x30))               /*!< Repetition Counter Value */
//7-0位：开启了预装载功能后，这些位允许用户设置比较寄存器的更新速率(即周期性地从预装载寄存器传输到当前寄存器)；如果允许产生更新中断，则会同时影响产生															
//       更新中断的速率。每次向下计数器REP_CNT达到0，会产生一个更新事件并且计数器REP_CNT重新从REP值开始计数。由于REP_CNT只有在周期更新事件U_RC															
//       发生时才重载REP值，因此对TIMx_RCR寄存器写入的新值只在下次周期更新事件发生时才起作用。															
//       这意味着在PWM模式中，(REP+1)对应着：－ 在边沿对齐模式下，PWM周期的数目；－ 在中心对称模式下，PWM半周期的数目；															


/*******************    TIM_CCR1 寄存器  *******************/
#define   wTIM_CCR1(TIMx_BASE)         (MEM_ADDR(TIMx_BASE+0x34))           /*!< Capture/Compare 1 Value */

/*******************    TIM_CCR2 寄存器  *******************/
#define   wTIM_CCR2(TIMx_BASE)         (MEM_ADDR(TIMx_BASE+0x38))            /*!< Capture/Compare 2 Value */

/*******************    TIM_CCR3 寄存器  *******************/
#define   wTIM_CCR3(TIMx_BASE)         (MEM_ADDR(TIMx_BASE+0x3C))            /*!< Capture/Compare 3 Value */

/*******************    TIM_CCR4 寄存器  *******************/
#define   wTIM_CCR4(TIMx_BASE)         (MEM_ADDR(TIMx_BASE+0x40))            /*!< Capture/Compare 4 Value */

//以上4个CCR寄存器分别为4个通道的捕获/比较值，下面以CCR1为例说明一下：
//15-0位：CCR1[15:0]: 捕获/比较通道1的值，若CC1通道配置为输出，CCR1包含了装入当前捕获/比较1寄存器的值(预装载值)															
//        如果在TIMx_CCMR1寄存器(OC1PE位)中未选择预装载功能，写入的数值会立即传输至当前寄存器中。否则只有当更新事件发生时，															
//        此预装载值才传输至当前捕获/比较1寄存器中。当前捕获/比较寄存器参与同计数器TIMx_CNT的比较，并在OC1端口上产生输出信号。															
//        若CC1通道配置为输入，CCR1包含了由上一次输入捕获1事件(IC1)传输的计数器值。															


/*******************    TIM_BDTR 寄存器  *******************/
/*!< DTG[0:7] bits (Dead-Time Generator set-up) */
#define  	SET_TIM_BDTR_DTG(TIMx_BASE,a)   	SET_REG_BITn(TIMx_BASE+0x44,0,MASKb8,a)

#define   bTIM_BDTR_LOCK(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x44,8)            /*!< LOCK[1:0] bits (Lock Configuration) */
#define   bTIM_BDTR_LOCK_0(TIMx_BASE)                     BIT_ADDR(TIMx_BASE +0x44,8)            /*!< Bit 0 */
#define   bTIM_BDTR_LOCK_1(TIMx_BASE)                     BIT_ADDR(TIMx_BASE +0x44,9)            /*!< Bit 1 */

#define   bTIM_BDTR_OSSI(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x44,10)            /*!< Off-State Selection for Idle mode */
#define   bTIM_BDTR_OSSR(TIMx_BASE)                       BIT_ADDR(TIMx_BASE +0x44,11)            /*!< Off-State Selection for Run mode */
#define   bTIM_BDTR_BKE(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x44,12)            /*!< Break enable */
#define   bTIM_BDTR_BKP(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x44,13)            /*!< Break Polarity */
#define   bTIM_BDTR_AOE(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x44,14)            /*!< Automatic Output enable */
#define   bTIM_BDTR_MOE(TIMx_BASE)                        BIT_ADDR(TIMx_BASE +0x44,15)            /*!< Main Output enable */
//注释： 根据锁定设置，AOE、BKP、BKE、OSSI、OSSR和DTG[7:0]位均可被写保护，有必要在第一次写入TIMx_BDTR寄存器时对它们进行配置															
//15位：MOE主输出使能，一旦刹车输入有效，该位被硬件异步清’0’。根据AOE位的设置值，该位可以由软件清’0’或被自动置1。它仅对配置为输出的通道有效。															
//      定义：0（禁止OC和OCN输出或强制为空闲状态），1（如果设置了相应的使能位(TIMx_CCER寄存器的CCxE、CCxNE位)，则开启OC和OCN输出。）															
//14位：AOE自动输出使能，定义：0（MOE只能被软件置’1’），1（MOE能被软件置’1’或在下一个更新事件被自动置’1’(如果刹车输入无效)															
//      注：一旦LOCK级别(TIMx_BDTR寄存器中的LOCK位)设为’1’，则该位不能被修改															
//13位：BKP刹车输入极性，定义：0（刹车输入低电平有效），1（刹车输入高电平有效），注：任何对该位的写操作都需要一个APB时钟的延迟以后才能起作用															
//      注：一旦LOCK级别(TIMx_BDTR寄存器中的LOCK位)设为’1’，则该位不能被修改。															
//12位：BKE刹车功能使能，定义：0（禁止刹车输入(BRK及CCS时钟失效事件)，1（开启），注：任何对该位的写操作都需要一个APB时钟的延迟以后才能起作用															
//      注：一旦LOCK级别(TIMx_BDTR寄存器中的LOCK位)设为’1’，则该位不能被修改。															
//11位：OSSR运行模式下“关闭状态”选择，该位用于当MOE=1且通道为互补输出时。没有互补输出的定时器中不存在OSSR位，定义：															
//      0（当定时器不工作时，禁止OC/OCN输出(OC/OCN使能输出信号=0)）															
//      1（当定时器不工作时，一旦CCxE=1或CCxNE=1，首先开启OC/OCN并输出无效电平，然后置OC/OCN使能输出信号=1）															
//      注：一旦LOCK级别(TIMx_BDTR寄存器中的LOCK位)设为2，则该位不能被修改。															
//10位：OSSI空闲模式下“关闭状态”选择，该位用于当MOE=0且通道设为输出时，注：一旦LOCK级别(TIMx_BDTR寄存器中的LOCK位)设为2，则该位不能被修改。															
//      定义：0（当定时器不工作时，禁止OC/OCN输出(OC/OCN使能输出信号=0)；															
//      1（当定时器不工作时，一旦CCxE=1或CCxNE=1，OC/OCN首先输出其空闲电平，然后OC/OCN使能输出信号=1。）															
//9-8位：LOOK[1:0]锁定设置，该位为防止软件错误而提供写保护，注：在系统复位后，只能写一次LOCK位，一旦写入TIMx_BDTR寄存器，则其内容冻结直至复位。															
//      定义：00（锁定关闭，寄存器无写保护），01（锁定级别1，不能写入TIMx_BDTR寄存器的DTG、BKE、BKP、AOE位和TIMx_CR2寄存器的OISx/OISxN位）															
//      10(锁定级别2,不能写入级别1中的各位,也不能写入CC极性位(一旦相关通道通过CCxS位设为输出,CC极性位是TIMx_CCER寄存器的CCxP/CCNxP位)OSSR/OSSI位															
//      11：锁定级别3,不能写入锁定级别2中的各位,也不能写入CC控制位(一旦相关通道通过CCxS位设为输出,CC控制位是TIMx_CCMRx寄存器的OCxM/OCxPE位)；															
//7-0位：死区发生器设置,控制死区事件.DTG[7:5]=0xx => DT=DTG[7:0] × Tdtg,Tdtg = TDTS;DTG[7:5]=10x => DT=(64+DTG[5:0]) × Tdtg,Tdtg=2×TDTS；															
//       DTG[7:5]=110 => DT=(32+DTG[4:0]) × Tdtg，Tdtg = 8 × TDTS；DTG[7:5]=111 => DT=(32+DTG[4:0])× Tdtg，Tdtg = 16 × TDTS；															
//       注：一旦LOCK级别(TIMx_BDTR寄存器中的LOCK位)设为1、2或3，则不能修改这些位。设DT表示其持续时间															


/*******************    TIM_DCR 寄存器  ********************/
/*!< DBA从第0BIT开始共5位： (DMA Base Address) */
#define  	SET_TIM_DCR_DBA(TIMx_BASE,a)   	SET_REG_BITn(TIMx_BASE+0x48,0,MASKb5,a)

/*!< DBL从第8BIT开始共5位： (DMA Burst Length) */
#define  	SET_TIM_DCR_DBL(TIMx_BASE,a)   	SET_REG_BITn(TIMx_BASE+0x48,8,MASKb5,a)

//12-8位：DBL[4:0]DMA连续传送长度，这些位定义了DMA在连续模式下的传送长度(当对TIMx_DMAR寄存器进行读或写时，定时器则进行一次连续传送)，															
//        即：定义传输的次数，传输可以是半字(双字节)或字节：00000：1次传输 00001：2次传输 ...... 00010：3次传输 ...... 10001：18次传输															
//4-0位：DBA[4:0]DMA基地址，这些位定义了DMA在连续模式下的基地址(当对TIMx_DMAR寄存器进行读或写时)，DBA定义为从TIMx_CR1寄存器所在地址开始的偏移量															
//       00000：TIMx_CR1， 00001：TIMx_CR2， 00010：TIMx_SMCR， ......															


/*******************    TIM_DMAR 寄存器  *******************/
#define   bTIM_DMAR_DMAB(TIMx_BASE)       (MEM_ADDR(TIMx_BASE+0x4C))            /*!< DMA 寄存器 for burst accesses */
//15-0位：DMAB[15:0]DMA连续传送寄存器，对TIMx_DMAR寄存器的读或写会导致对以下地址所在寄存器的存取操作，TIMx_CR1地址 + DBA + DMA索引，其中： 															
//        "TIMx_CR1地址"是控制寄存器1(TIMx_CR1)所在的地址:"DBA"是TIMx_DCR寄存器中定义的基地址:"DMA索引"是由DMA自动控制的偏移量,															
//        它取决于TIMx_DCR寄存器中定义的DBL。															


/******************************************************************************/
/*                                                                            */
/*                           独立看门狗寄存器  		                             */
/*                                                                            */
/******************************************************************************/

/*******************    IWDG_KR 寄存器  ********************/
       
#define  wIWDG_KEY					(MEM_ADDR(IWDG_BASE))/*!< Key value (只写, 读出为0000h) */
/*******************    IWDG_PR 寄存器  ********************/
/*!< 从第0BIT开始共3位：PR[2:0] (预分频器) */
#define  	SET_IWDG_PR(a)   	SET_REG_BITn(IWDG_BASE+0x04,0,MASKb3,a)

/*******************    IWDG_RLR 寄存器  *******************/
        
#define  wIWDG_RL					(MEM_ADDR(IWDG_BASE+0x08)) /*!< 重装载值 */
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
#define  SET_WWDG_CR_T(a)   	SET_REG_BITn(WWDG_BASE,0,MASKb7,a)

#define  bWWDG_CR_WDGA                        BIT_ADDR(WWDG_BASE,7)               /*!< Activation bit */

/*******************    WWDG_CFR 寄存器  *******************/
/*!< W从第0BIT开始共7位：  W[6:0] bits (7-bit 窗口值) */
#define  SET_WWDG_CFR_W(a)   	SET_REG_BITn(WWDG_BASE+0x04,0,MASKb7,a)

#define  SET_WWDG_CFR_WDGTB(a)   	SET_REG_BITn(WWDG_BASE+0x04,7,MASKb2,a)	/*!< WDGTB[1:0] bits (Timer Base) */

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
#define  SET_CAN_ESR_LEC(CANx_BASE,a)   SET_REG_BITn(CANx_BASE+0x018,4,MASKb3,a)
#define  GET_CAN_ESR_LEC(CANx_BASE)   ((MEM_ADDR(CANx_BASE+0x018)&CAN_ESR_LEC)>>4)

//#define  CAN_ESR_TEC（只读） ((uint32_t)0x00FF0000)        /*!< Least significant byte of the 9-bit Transmit Error Counter */
#define  GET_CAN_ESR_TEC(CANx_BASE)   ((MEM_ADDR(CANx_BASE+0x018)&CAN_ESR_TEC)>>16)
//#define  CAN_ESR_REC(CANx_BASE)（只读） ((uint32_t)0xFF000000)        /*!< Receive Error Counter */
#define  GET_CAN_ESR_REC(CANx_BASE)   ((MEM_ADDR(CANx_BASE+0x018)&CAN_ESR_REC)>>24)

/*******************   BTR 位时序寄存器  ********************/
//#define  CAN_BTR_BRP                          ((uint32_t)0x000003FF)        /*!< Baud Rate Prescaler */
#define  SET_CAN_BTR_BRP(CANx_BASE,a)   SET_REG_BITn(CANx_BASE+0x01C,0,MASKb10,a)
//#define  CAN_BTR_TS1                        ((uint32_t)0x000F0000)        /*!< Time Segment 1 */
#define  SET_CAN_BTR_TS1(CANx_BASE,a)   SET_REG_BITn(CANx_BASE+0x01C,16,MASKb4,a)
//#define  CAN_BTR_TS2                         ((uint32_t)0x00700000)        /*!< Time Segment 2 */
#define  SET_CAN_BTR_TS2(CANx_BASE,a)   SET_REG_BITn(CANx_BASE+0x01C,20,MASKb3,a)

//#define  CAN_BTR_SJW                         ((uint32_t)0x03000000)        /*!< Resynchronization Jump Width */
#define  SET_CAN_BTR_SJW(CANx_BASE,a)   SET_REG_BITn(CANx_BASE+0x01C,24,MASKb2,a)

#define  bCAN_BTR_LBKM                  BIT_ADDR(CANx_BASE+0x01C,30)        /*!< Loop Back Mode (Debug) */
#define  bCAN_BTR_SILM                  BIT_ADDR(CANx_BASE+0x01C,31)        /*!< Silent Mode */

/*!< 邮箱寄存器族 */
/******************   TI0R 发送邮箱标识符寄存器  ********************/
#define  bCAN_TI0R_TXRQ(CANx_BASE)      BIT_ADDR(CANx_BASE+0x180,0)        /*!< Transmit Mailbox Request */
#define  bCAN_TI0R_RTR(CANx_BASE)       BIT_ADDR(CANx_BASE+0x180,1)        /*!< Remote Transmission Request */
#define  bCAN_TI0R_IDE(CANx_BASE)       BIT_ADDR(CANx_BASE+0x180,2)        /*!< Identifier Extension */
//#define  CAN_TI0R_EXID                       ((uint32_t)0x001FFFF8)        /*!< Extended Identifier */

#define  SET_CAN_TI0R_EXID(CANx_BASE,a)   SET_REG_BITn(CANx_BASE+0x180,3,MASKb18,a)
//#define  CAN_TI0R_STID                       ((uint32_t)0xFFE00000)        /*!< Standard Identifier or Extended Identifier */
#define  SET_CAN_TI0R_STID(CANx_BASE,a)   SET_REG_BITn(CANx_BASE+0x180,21,MASKb11,a)

/******************   TDT0R 发送邮箱长度控制及时戳寄存器  *******************/
//#define  CAN_TDT0R_DLC                        ((uint32_t)0x0000000F)        /*!< Data Length Code */
#define  SET_CAN_TDT0R_DLC(CANx_BASE,a)   SET_REG_BITn(CANx_BASE+0x184,0,MASKb4,a)

#define  bCAN_TDT0R_TGT(CANx_BASE)      BIT_ADDR(CANx_BASE+0x184,8)        /*!< Transmit Global Time */
//#define  CAN_TDT0R_TIME                      ((uint32_t)0xFFFF0000)        /*!< Message Time Stamp */
#define  SET_CAN_TDT0R_TIME(CANx_BASE,a)   SET_REG_BITn(CANx_BASE+0x184,16,MASKb8,a)

/******************   TDL0R 发送邮箱数据低字节寄存器  *******************/
#define  CAN_TDL0R(CANx_BASE)           (MEM_ADDR(CANx_BASE+0x188))/*!< Data byte 0-3 */

/******************   TDH0R 发送邮箱数据高字节寄存器  *******************/
#define  CAN_TDH0R(CANx_BASE)           (MEM_ADDR(CANx_BASE+0x18C))/*!< Data byte 4-7 */

/*******************   TI1R 发送邮箱标识符寄存器  *******************/
#define  bCAN_TI1R_TXRQ(CANx_BASE)     BIT_ADDR(CANx_BASE+0x190,0)        /*!< Transmit Mailbox Request */
#define  bCAN_TI1R_RTR(CANx_BASE)      BIT_ADDR(CANx_BASE+0x190,1)        /*!< Remote Transmission Request */
#define  bCAN_TI1R_IDE(CANx_BASE)      BIT_ADDR(CANx_BASE+0x190,2)        /*!< Identifier Extension */
//#define  CAN_TI1R_EXID                       ((uint32_t)0x001FFFF8)        /*!< Extended Identifier */
#define  SET_CAN_TI1R_EXID(CANx_BASE,a)   SET_REG_BITn(CANx_BASE+0x190,3,MASKb18,a)

//#define  CAN_TI1R_STID                       ((uint32_t)0xFFE00000)        /*!< Standard Identifier or Extended Identifier */
#define  SET_CAN_TI1R_STID(CANx_BASE,a)   SET_REG_BITn(CANx_BASE+0x190,21,MASKb11,a)

/*******************   TDT1R 发送邮箱长度控制及时戳寄存器  ******************/
//#define  CAN_TDT1R_DLC                        ((uint32_t)0x0000000F)        /*!< Data Length Code */
#define  SET_CAN_TDT1R_DLC(CANx_BASE,a)   SET_REG_BITn(CANx_BASE+0x194,0,MASKb4,a)

#define  bCAN_TDT1R_TGT                BIT_ADDR(CANx_BASE+0x194,8)        /*!< Transmit Global Time */
//#define  CAN_ TDT1R_TIME                      ((uint32_t)0xFFFF0000)        /*!< Message Time Stamp */
#define  SET_CAN_TDT1R_TIME(CANx_BASE,a)  SET_REG_BITn(CANx_BASE+0x194,16,MASKb8,a) 

/*******************   TDL1R 发送邮箱数据低字节寄存器  ******************/
#define  CAN_TDL1R(CANx_BASE)           (MEM_ADDR(CANx_BASE+0x198))/*!< Data byte 0-3 */
/*******************   TDH1R 发送邮箱数据高字节寄存器  ******************/
#define  CAN_TDH1R(CANx_BASE)           (MEM_ADDR(CANx_BASE+0x19C))/*!< Data byte 4-7 */

/*******************   TI2R 发送邮箱标识符寄存器  *******************/
#define  bCAN_TI2R_TXRQ(CANx_BASE)    BIT_ADDR(CANx_BASE+0x1A0,0)        /*!< Transmit Mailbox Request */
#define  bCAN_TI2R_RTR(CANx_BASE)     BIT_ADDR(CANx_BASE+0x1A0,1)        /*!< Remote Transmission Request */
#define  bCAN_TI2R_IDE(CANx_BASE)     BIT_ADDR(CANx_BASE+0x1A0,2)        /*!< Identifier Extension */
//#define  CAN_ TI2R_EXID                       ((uint32_t)0x001FFFF8)        /*!< Extended identifier */
#define  SET_CAN_TI2R_EXID(CANx_BASE,a)   SET_REG_BITn(CANx_BASE+0x1A0,3,MASKb18,a)
//#define  CAN_ TI2R_STID                       ((uint32_t)0xFFE00000)        /*!< Standard Identifier or Extended Identifier */
#define  SET_CAN_TI2R_STID(CANx_BASE,a)   SET_REG_BITn(CANx_BASE+0x1A0,21,MASKb11,a)

/*******************   TDT2R 发送邮箱长度控制及时戳寄存器  ******************/  
//#define  CAN_ TDT2R_DLC                        ((uint32_t)0x0000000F)        /*!< Data Length Code */
#define  SET_CAN_TDT2R_DLC(CANx_BASE,a)   SET_REG_BITn(CANx_BASE+0x1A4,0,MASKb4,a)
#define  bCAN_TDT2R_TGT               BIT_ADDR(CANx_BASE+0x1A4,8)        /*!< Transmit Global Time */
//#define  CAN_ TDT2R_TIME                      ((uint32_t)0xFFFF0000)        /*!< Message Time Stamp */
#define  SET_CAN_TDT2R_TIME(CANx_BASE,a)   SET_REG_BITn(CANx_BASE+0x1A4,16,MASKb8,a)

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
/*!< FREQ[5:0] bits (Peripheral Clock Frequency) */
#define  SET_I2C_CR_FREQ(CANx_BASE,a)   SET_REG_BITn(I2Cx_BASE +0x04,0,MASKb6,a)

#define  bI2C_CR_ITERREN(I2Cx_BASE )       BIT_ADDR(I2Cx_BASE +0x04,8)            /*!< Error Interrupt Enable */
#define  bI2C_CR_ITEVTEN(I2Cx_BASE )       BIT_ADDR(I2Cx_BASE +0x04,9)            /*!< Event Interrupt Enable */
#define  bI2C_CR_ITBUFEN(I2Cx_BASE )       BIT_ADDR(I2Cx_BASE +0x04,10)            /*!< Buffer Interrupt Enable */
#define  bI2C_CR_DMAEN(I2Cx_BASE )         BIT_ADDR(I2Cx_BASE +0x04,11)            /*!< DMA Requests Enable */
#define  bI2C_CR_LAST(I2Cx_BASE )          BIT_ADDR(I2Cx_BASE +0x04,12)            /*!< DMA Last Transfer */

/*******************  I2C_OAR1 自身地址寄存器  *******************/
/*!< Interface Address */
#define  SET_I2C_OAR_ADD1_7(I2Cx_BASE ,a)   SET_REG_BITn(I2Cx_BASE +0x08,1,MASKb7,a)

/*!< Interface Address */
#define  SET_I2C_OAR_ADD8_9(I2Cx_BASE ,a)   SET_REG_BITn(I2Cx_BASE +0x08,8,MASKb2,a)

#define  bI2C_OAR_ADD0                       BIT_ADDR(I2Cx_BASE +0x08,0)            /*!< Bit 0 */
#define  bI2C_OAR_ADDMODE(I2Cx_BASE )        BIT_ADDR(I2Cx_BASE +0x08,15)            /*!< Addressing Mode (Slave mode) */

/*******************  I2C_OAR2 自身地址寄存器  *******************/
#define  bI2C_OAR_ENDUAL(I2Cx_BASE )         BIT_ADDR(I2Cx_BASE +0x0C,0)               /*!< Dual addressing mode enable */
/*!< Interface address */
#define  SET_I2C_OAR_ADD2(I2Cx_BASE ,a)   SET_REG_BITn(I2Cx_BASE +0x0C,1,MASKb7,a)

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
/*!< Clock Control Register in Fast/Standard mode (Master mode) */
#define  SET_I2C_CCR_CCR(I2Cx_BASE ,a)   SET_REG_BITn(I2Cx_BASE +0x1C,0,MASKb12,a)

#define  bI2C_CCR_DUTY(I2Cx_BASE )                        BIT_ADDR(I2Cx_BASE +0x1C,14)            /*!< Fast Mode Duty Cycle */
#define  bI2C_CCR_FS(I2Cx_BASE )                          BIT_ADDR(I2Cx_BASE +0x00,15)            /*!< I2C Master Mode Selection */

/******************  I2C_TRISE寄存器  *******************/
/*!< Maximum Rise Time in Fast/Standard mode (Master mode) */
#define  SET_I2C_TRISE_TRISE(I2Cx_BASE ,a)   SET_REG_BITn(I2Cx_BASE +0x20,0,MASKb6,a)

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

#define  SET_DMA_CCRx_PSIZE(DMAx_BASE,n,a)   SET_REG_BITn(DMAx_BASE+0x08+n*20,8,MASKb2,a)
/*!< PSIZE[1:0] bits (通道n 外设数据宽度00: 8位；01: 16位；10: 32位；) */

#define  SET_DMA_CCRx_MSIZE(DMAx_BASE,n,a)   SET_REG_BITn(DMAx_BASE+0x08+n*20,10,MASKb2,a)
/*!< MSIZE[1:0] bits (通道n 存储器数据宽度00: 8位；01: 16位；10: 32位；) */

#define  SET_DMA_CCRx_PL(DMAx_BASE,n,a)   SET_REG_BITn(DMAx_BASE+0x08+n*20,12,MASKb2,a)
/*!< PL[1:0] bits(通道n 优先级00低、01中、10高、11最高；) */

#define  bDMA_CCRx_MEM2MEM(DMAx_BASE,n) BIT_ADDR(DMAx_BASE+0x08+n*20,13)    /*!< 通道n 存储器到存储器模式 */

/******************   DMA_CNDTRx 寄存器 ******************/
#define  wDMA_CNDTRx_NDT(DMAx_BASE,n)    (MEM_ADDR(DMAx_BASE+0x0C+n*20)      /*!< 通道n 传输数据计数 */


/******************   DMA_CPARx 寄存器 *******************/
#define  wDMA_CPARx_PA(DMAx_BASE,n)    (MEM_ADDR(DMAx_BASE+0x10+n*20)        /*!< 通道n 外设地址寄存器32位 */


/******************   DMA_CMARx 寄存器 *******************/
#define  wDMA_CMARx_MA(DMAx_BASE,n)    (MEM_ADDR(DMAx_BASE+0x14+n*20)        /*!< 通道n 存储器地址寄存器32位 */


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
/*!< AWDCH[4:0] bits (模拟狗监测通道选择) */
#define  SET_ADC_CR_AWDCH(ADCx_BASE ,a)   SET_REG_BITn(ADCx_BASE+0x04,0,MASKb5,a)

#define  bADC_CR_EOCIE(ADCx_BASE)     BIT_ADDR(ADCx_BASE+0x04,5)        /*!< 转换结束（EOC）中断使能 */
#define  bADC_CR_AWDIE(ADCx_BASE)     BIT_ADDR(ADCx_BASE+0x04,6)        /*!< 模拟狗中断使能 */
#define  bADC_CR_JEOCIE(ADCx_BASE)    BIT_ADDR(ADCx_BASE+0x04,7)        /*!< 注入通道中断使能 */
#define  bADC_CR_SCAN(ADCx_BASE)      BIT_ADDR(ADCx_BASE+0x04,8)        /*!< 扫描模式 */
#define  bADC_CR_AWDSGL(ADCx_BASE)    BIT_ADDR(ADCx_BASE+0x04,9)        /*!< 在扫描模式下，使能单个通道的看门狗 */
#define  bADC_CR_JAUTO(ADCx_BASE)     BIT_ADDR(ADCx_BASE+0x04,10)        /*!< 注入群的自动转换 */
#define  bADC_CR_DISCEN(ADCx_BASE)    BIT_ADDR(ADCx_BASE+0x04,11)        /*!< 规则通道的非连续模式 */
#define  bADC_CR_JDISCEN(ADCx_BASE)   BIT_ADDR(ADCx_BASE+0x04,12)        /*!< 注入通道的非连续模式 */

 /*!< DISCNUM[2:0] bits (间断模式通道计数) */
#define  SET_ADC_CR_DISCNUM(ADCx_BASE ,a)   SET_REG_BITn(ADCx_BASE+0x04,13,MASKb3,a)

/*!< DUALMOD[3:0] bits (双ADC模式选择) */
#define  SET_ADC_CR_DUALMOD(ADCx_BASE ,a)   SET_REG_BITn(ADCx_BASE+0x04,16,MASKb4,a)

#define  bADC_CR_JAWDEN(ADCx_BASE)     BIT_ADDR(ADCx_BASE+0x04,22)        /*!< Analog watchdog enable on injected channels */
#define  bADC_CR_AWDEN(ADCx_BASE)      BIT_ADDR(ADCx_BASE+0x04,23)        /*!< Analog watchdog enable on regular channels */

  
/*******************   ADC_CR2 寄存器 ********************/
#define  bADC_CR_ADON(ADCx_BASE)     BIT_ADDR(ADCx_BASE+0x08,0)        /*!< A/D 转换器开启或关闭 */
#define  bADC_CR_CONT(ADCx_BASE)     BIT_ADDR(ADCx_BASE+0x08,1)        /*!< 连续转换 */
#define  bADC_CR_CAL(ADCx_BASE)     BIT_ADDR(ADCx_BASE+0x08,2)        /*!< A/D 校准 */
#define  bADC_CR_RSTCAL(ADCx_BASE)     BIT_ADDR(ADCx_BASE+0x08,3)        /*!< 复位校准寄存器 */
#define  bADC_CR_DMA(ADCx_BASE)     BIT_ADDR(ADCx_BASE+0x08,8)        /*!< DMA模式 */
#define  bADC_CR_ALIGN(ADCx_BASE)     BIT_ADDR(ADCx_BASE+0x08,11)        /*!< 数据对齐方式 */

//#define  ADC_CR2_JEXTSEL                     ((uint32_t)0x00007000)        /*!< JEXTSEL[2:0] bits (External event select for injected group) */
#define  SET_ADC_CR_JEXTSEL(ADCx_BASE,a)   SET_REG_BITn(ADCx_BASE+0x08,12,MASKb3,a)

#define  bADC_CR_JEXTTRIG(ADCx_BASE)     BIT_ADDR(ADCx_BASE+0x08,15)        /*!< 注入通道的外部触发转换模式 */

//#define  ADC_CR2_EXTSEL                      ((uint32_t)0x000E0000)        /*!< EXTSEL[2:0] bits (External Event Select for regular group) */
#define  SET_ADC_CR_EXTSEL(ADCx_BASE,a)    SET_REG_BITn(ADCx_BASE+0x08,17,MASKb3,a)

#define  bADC_CR_EXTTRIG(ADCx_BASE)     BIT_ADDR(ADCx_BASE+0x08,20)        /*!< 规则通道的外部触发模式 */
#define  bADC_CR_JSWSTART(ADCx_BASE)    BIT_ADDR(ADCx_BASE+0x08,21)        /*!< 启动注入通道转换 */
#define  bADC_CR_SWSTART(ADCx_BASE)     BIT_ADDR(ADCx_BASE+0x08,22)        /*!< 启动规则通道转换 */
#define  bADC_CR_TSVREFE(ADCx_BASE)     BIT_ADDR(ADCx_BASE+0x08,23)        /*!< Temperature Sensor and VREFINT Enable */

/******************   ADC_SMPR1/2 寄存器 *******************/
/* 3bits (18个通道均可独立设置采样时间间隔) */
//可使用以下统一的宏（其中ch为通道号0－17），或依通道编号的专用宏
#define  SET_ADC_SMP0_17(ADCx_BASE,ch,a)	((ch<10)? SET_REG_BITn(ADCx_BASE+0x10,ch*3,MASKb3,a) : SET_REG_BITn(ADCx_BASE+0x0C,(ch-10)*3,MASKb3,a))

//ADC_SMPR2 寄存器
#define  SET_ADC_SMP0(ADCx_BASE,a)   	SET_REG_BITn(ADCx_BASE+0x10,0,MASKb3,a)
#define  SET_ADC_SMP1(ADCx_BASE,a)   	SET_REG_BITn(ADCx_BASE+0x10,3,MASKb3,a)
#define  SET_ADC_SMP2(ADCx_BASE,a)   	SET_REG_BITn(ADCx_BASE+0x10,6,MASKb3,a)
#define  SET_ADC_SMP3(ADCx_BASE,a)   	SET_REG_BITn(ADCx_BASE+0x10,9,MASKb3,a)
#define  SET_ADC_SMP4(ADCx_BASE,a)   	SET_REG_BITn(ADCx_BASE+0x10,12,MASKb3,a)
#define  SET_ADC_SMP5(ADCx_BASE,a)   	SET_REG_BITn(ADCx_BASE+0x10,15,MASKb3,a)
#define  SET_ADC_SMP6(ADCx_BASE,a)   	SET_REG_BITn(ADCx_BASE+0x10,18,MASKb3,a)
#define  SET_ADC_SMP7(ADCx_BASE,a)   	SET_REG_BITn(ADCx_BASE+0x10,21,MASKb3,a)
#define  SET_ADC_SMP8(ADCx_BASE,a)   	SET_REG_BITn(ADCx_BASE+0x10,24,MASKb3,a)
#define  SET_ADC_SMP9(ADCx_BASE,a)   	SET_REG_BITn(ADCx_BASE+0x10,27,MASKb3,a)
//ADC_SMPR1 寄存器
#define  SET_ADC_SMP10(ADCx_BASE,a)   SET_REG_BITn(ADCx_BASE+0x0C,0,MASKb3,a)
#define  SET_ADC_SMP11(ADCx_BASE,a)   SET_REG_BITn(ADCx_BASE+0x0C,3,MASKb3,a)
#define  SET_ADC_SMP12(ADCx_BASE,a)   SET_REG_BITn(ADCx_BASE+0x0C,6,MASKb3,a)
#define  SET_ADC_SMP13(ADCx_BASE,a)   SET_REG_BITn(ADCx_BASE+0x0C,9,MASKb3,a)
#define  SET_ADC_SMP14(ADCx_BASE,a)   SET_REG_BITn(ADCx_BASE+0x0C,12,MASKb3,a)
#define  SET_ADC_SMP15(ADCx_BASE,a)   SET_REG_BITn(ADCx_BASE+0x0C,15,MASKb3,a)
#define  SET_ADC_SMP16(ADCx_BASE,a)   SET_REG_BITn(ADCx_BASE+0x0C,18,MASKb3,a)
#define  SET_ADC_SMP17(ADCx_BASE,a)   SET_REG_BITn(ADCx_BASE+0x0C,21,MASKb3,a)


/******************   ADC_JOFR1 寄存器 *******************/
#define  wADC_JOFFSET1(ADCx_BASE)    (MEM_ADDR(ADCx_BASE +0x14))   /*!< 注入通道1 数据偏移量 */

/******************   ADC_JOFR2 寄存器 *******************/
#define  wADC_JOFFSET2(ADCx_BASE)    (MEM_ADDR(ADCx_BASE +0x18))   /*!< 注入通道2 数据偏移量 */

/******************   ADC_JOFR3 寄存器 *******************/
#define  wADC_JOFFSET3(ADCx_BASE)    (MEM_ADDR(ADCx_BASE +0x1C))    /*!< 注入通道3 数据偏移量 */

/******************   ADC_JOFR4 寄存器 *******************/
#define  wADC_JOFFSET4(ADCx_BASE)    (MEM_ADDR(ADCx_BASE +0x20))    /*!< 注入通道4 数据偏移量 */

/*******************   ADC_HTR 寄存器 ********************/
#define  wADC_HT(ADCx_BASE)    (MEM_ADDR(ADCx_BASE +0x24))          /*!< 模拟看门狗阀值高限 */

/*******************   ADC_LTR 寄存器 ********************/
#define  wADC_LT(ADCx_BASE)    (MEM_ADDR(ADCx_BASE +0x28))          /*!< 模拟看门狗阀值低限 */

/*******************   ADC_SQR1/2/3 寄存器 *******************/
/*定义规则组序列，队列元素最大16个，每元素用5BIT表示，代表相应的通道号0－17 */
//ADC_SQR3 寄存器
#define  SET_ADC_SQ1(ADCx_BASE,a)   SET_REG_BITn(ADCx_BASE+0x34,0,MASKb5,a)
#define  SET_ADC_SQ2(ADCx_BASE,a)   SET_REG_BITn(ADCx_BASE+0x34,5,MASKb5,a)
#define  SET_ADC_SQ3(ADCx_BASE,a)   SET_REG_BITn(ADCx_BASE+0x34,10,MASKb5,a)
#define  SET_ADC_SQ4(ADCx_BASE,a)   SET_REG_BITn(ADCx_BASE+0x34,15,MASKb5,a)
#define  SET_ADC_SQ5(ADCx_BASE,a)   SET_REG_BITn(ADCx_BASE+0x34,20,MASKb5,a)
#define  SET_ADC_SQ6(ADCx_BASE,a)   SET_REG_BITn(ADCx_BASE+0x34,25,MASKb5,a)
//ADC_SQR2 寄存器
#define  SET_ADC_SQ7(ADCx_BASE,a)   SET_REG_BITn(ADCx_BASE+0x30,0,MASKb5,a)
#define  SET_ADC_SQ8(ADCx_BASE,a)   SET_REG_BITn(ADCx_BASE+0x30,5,MASKb5,a)
#define  SET_ADC_SQ9(ADCx_BASE,a)   SET_REG_BITn(ADCx_BASE+0x30,10,MASKb5,a)
#define  SET_ADC_SQ10(ADCx_BASE,a)   SET_REG_BITn(ADCx_BASE+0x30,15,MASKb5,a)
#define  SET_ADC_SQ11(ADCx_BASE,a)   SET_REG_BITn(ADCx_BASE+0x30,20,MASKb5,a)
#define  SET_ADC_SQ12(ADCx_BASE,a)   SET_REG_BITn(ADCx_BASE+0x30,25,MASKb5,a)
//ADC_SQR1 寄存器
#define  SET_ADC_SQ13(ADCx_BASE,a)   SET_REG_BITn(ADCx_BASE+0x2C,0,MASKb5,a)
#define  SET_ADC_SQ14(ADCx_BASE,a)   SET_REG_BITn(ADCx_BASE+0x2C,5,MASKb5,a)
#define  SET_ADC_SQ15(ADCx_BASE,a)   SET_REG_BITn(ADCx_BASE+0x2C,10,MASKb5,a)
#define  SET_ADC_SQ16(ADCx_BASE,a)   SET_REG_BITn(ADCx_BASE+0x2C,15,MASKb5,a)

/*!< 4BIT，定义规则组序列的长度，a=0-15分别表示规则组队列共有1-16个成员 */
#define  SET_ADC_SQR_L(ADCx_BASE,a)   SET_REG_BITn(ADCx_BASE+0x2C,20,MASKb4,a)


/*******************   ADC_JSQR 寄存器 *******************/
/*!< 定义注入组序列，队列元素最大4个，每元素用5BIT表示，代表相应的通道号0－17 */
#define  SET_ADC_JSQ1(ADCx_BASE,a)   SET_REG_BITn(ADCx_BASE+0x38,0,MASKb5,a)
#define  SET_ADC_JSQ2(ADCx_BASE,a)   SET_REG_BITn(ADCx_BASE+0x38,5,MASKb5,a)
#define  SET_ADC_JSQ3(ADCx_BASE,a)   SET_REG_BITn(ADCx_BASE+0x38,10,MASKb5,a)
#define  SET_ADC_JSQ4(ADCx_BASE,a)   SET_REG_BITn(ADCx_BASE+0x38,15,MASKb5,a)

/*!< JL[1:0] bits (注入组序列长度，0-3分别表示1-4) */
#define  SET_ADC_JSQR_JL(ADCx_BASE,a)   SET_REG_BITn(ADCx_BASE+0x38,20,MASKb2,a)

/*******************   ADC_JDR1 寄存器 *******************/
#define  wADC_JDR1(ADCx_BASE)    (MEM_ADDR(ADCx_BASE +0x3C))			/*!< 注入通道1 数据寄存器 */

/*******************   ADC_JDR2 寄存器 *******************/
#define  wADC_JDR2(ADCx_BASE)    (MEM_ADDR(ADCx_BASE +0x40))     /*!< 注入通道2 数据寄存器 */

/*******************   ADC_JDR3 寄存器 *******************/
#define  wADC_JDR3(ADCx_BASE)    (MEM_ADDR(ADCx_BASE +0x44))     /*!< 注入通道3 数据寄存器 */

/*******************   ADC_JDR4 寄存器 *******************/
#define  wADC_JDR4(ADCx_BASE)    (MEM_ADDR(ADCx_BASE +0x48))     /*!< 注入通道4 数据寄存器 */

/********************   ADC_DR 寄存器 ********************/
#define  wADC_DR(ADCx_BASE)    		(MEM_ADDR(ADCx_BASE +0x4C))        /*!< 规则通道 数据寄存器（32位，其中低12位为ADC转换值） */
#define  wADC_DR_ADC2(ADCx_BASE) 	((MEM_ADDR(ADCx_BASE +0x4C))>>16)  /*!< 双ADC时ADC2 数据（实为上述寄存器的高位） */
/*************************************************************************************************************************/


/******************************************************************************/
/*                                                                            */
/*                      DAC数模转换器                                         */
/*                                                                            */
/******************************************************************************/
//注：一般芯片只有一个DAC外设，为兼容多个DAC，这里仍采用DACx的形式，引用时请加参数bDAC，与ADC、USART等外设一致。
#define bDAC	DAC_BASE
/********************   DAC_CR 寄存器 ******************00**/
#define  bDAC_CR_EN1(DACx_BASE)         BIT_ADDR(DACx_BASE+0x00,0)        /*!< DAC channel1 enable */
#define  bDAC_CR_BOFF1(DACx_BASE)       BIT_ADDR(DACx_BASE+0x00,1)        /*!< DAC channel1 output buffer disable */
#define  bDAC_CR_TEN1(DACx_BASE)        BIT_ADDR(DACx_BASE+0x00,2)        /*!< DAC channel1 Trigger enable */

/*!< TSEL1[2:0] (DAC channel1 Trigger selection) */
#define  SET_DAC_CR_TSEL1(DACx_BASE,a)	SET_REG_BITn(DACx_BASE,3,MASKb3,a)
/*!< WAVE1[1:0] (DAC channel1 noise/triangle wave generation enable) */
#define  SET_DAC_CR_WAVE1(DACx_BASE,a)	SET_REG_BITn(DACx_BASE,6,MASKb2,a)

/*!< MAMP1[3:0] (DAC通道1屏蔽/幅值选择器) */
#define  SET_DAC_CR_MAMP1(DACx_BASE,a)	SET_REG_BITn(DACx_BASE,8,MASKb4,a)

#define  bDAC_CR_DMAEN1(DACx_BASE)        BIT_ADDR(DACx_BASE+0x00,12)        /*!< DAC channel1 DMA enable */
#define  bDAC_CR_EN2(DACx_BASE)        BIT_ADDR(DACx_BASE+0x00,16)        /*!< DAC channel2 enable */
#define  bDAC_CR_BOFF2(DACx_BASE)        BIT_ADDR(DACx_BASE+0x00,17)        /*!< DAC channel2 output buffer disable */
#define  bDAC_CR_TEN2(DACx_BASE)        BIT_ADDR(DACx_BASE+0x00,18)        /*!< DAC channel2 Trigger enable */
//USB_BASE

/*!< TSEL2[2:0] (DAC channel2 Trigger selection) */
#define  SET_DAC_CR_TSEL2(DACx_BASE,a)	SET_REG_BITn(DACx_BASE,19,MASKb3,a)

/*!< WAVE2[1:0] (DAC channel2 noise/triangle wave generation enable) */
#define  SET_DAC_CR_WAVE2(DACx_BASE,a)	SET_REG_BITn(DACx_BASE,22,MASKb2,a)

/*!< MAMP2[3:0] (DAC通道2屏蔽/幅值选择器) */
#define  SET_DAC_CR_MAMP2(DACx_BASE,a)	SET_REG_BITn(DACx_BASE,24,MASKb4,a)

#define  bDAC_CR_DMAEN2(DACx_BASE)        BIT_ADDR(DACx_BASE+0x00,28)        /*!< DAC channel2 DMA enabled */

/*****************   DAC_SWTRIGR 寄存器 **************04****/
#define  bDAC_SWTRIGR_SWTRIG1(DACx_BASE)        BIT_ADDR(DACx_BASE+0x04,0)               /*!< DAC channel1 software trigger */
#define  bDAC_SWTRIGR_SWTRIG2(DACx_BASE)        BIT_ADDR(DACx_BASE+0x04,1)               /*!< DAC channel2 software trigger */

/*****************   DAC_DHR12R1 寄存器 ****************08**/
//#define  DAC_DHR12R1_DACC1DHR                ((uint16_t)0x0FFF)            /*!< DAC channel1 12-bit Right aligned data */
#define  SET_DAC_DHR12R1_DACC1DHR(DACx_BASE,a)	SET_REG_BITn(DACx_BASE+0x08,0,MASKb12,a)

/*****************   DAC_DHR12L1 寄存器 ****************0c**/
//#define  DAC_DHR12L1_DACC1DHR                ((uint16_t)0xFFF0)            /*!< DAC channel1 12-bit Left aligned data */
#define  SET_DAC_DHR12L1_DACC1DHR(DACx_BASE,a)	SET_REG_BITn(DACx_BASE+0x0C,4,MASKb12,a)

/******************   DAC_DHR8R1 寄存器 ****************10**/
//#define  DAC_DHR8R1_DACC1DHR                 ((uint8_t)0xFF)               /*!< DAC channel1 8-bit Right aligned data */
#define  SET_DAC_DHR8R1_DACC1DHR(DACx_BASE,a)	SET_REG_BITn(DACx_BASE+0x10,0,MASKb8,a)

/*****************   DAC_DHR12R2 寄存器 ****************14**/
//#define  DAC_DHR12R2_DACC2DHR                ((uint16_t)0x0FFF)            /*!< DAC channel2 12-bit Right aligned data */
#define  SET_DAC_DHR12R2_DACC2DHR(DACx_BASE,a)	SET_REG_BITn(DACx_BASE+0x14,0,MASKb12,a)
/*****************   DAC_DHR12L2 寄存器 ****************18**/
//#define  DAC_DHR12L2_DACC2DHR                ((uint16_t)0xFFF0)            /*!< DAC channel2 12-bit Left aligned data */
#define  SET_DAC_DHR12L2_DACC2DHR(DACx_BASE,a)	SET_REG_BITn(DACx_BASE+0x18,4,MASKb12,a)
/******************   DAC_DHR8R2 寄存器 ****************1c**/
//#define  DAC_DHR8R2_DACC2DHR                 ((uint8_t)0xFF)               /*!< DAC channel2 8-bit Right aligned data */
#define  SET_DAC_DHR8R2_DACC2DHR(DACx_BASE,a)	SET_REG_BITn(DACx_BASE+0x1C,0,MASKb8,a)
/*****************   DAC_DHR12RD 寄存器 ****************20**/
//#define  DAC_DHR12RD_DACC1DHR                ((uint32_t)0x00000FFF)        /*!< DAC channel1 12-bit Right aligned data */
#define  SET_DAC_DHR12RD_DACC1DHR(DACx_BASE,a)	SET_REG_BITn(DACx_BASE+0x20,0,MASKb12,a)
//#define  DAC_DHR12RD_DACC2DHR                ((uint32_t)0x0FFF0000)        /*!< DAC channel2 12-bit Right aligned data */
#define  SET_DAC_DAC_DHR12RD_DACC2DHR(DACx_BASE,a)	SET_REG_BITn(DACx_BASE+0x20,16,MASKb12,a)
/*****************   DAC_DHR12LD 寄存器 ****************24**/
//#define  DAC_DHR12LD_DACC1DHR                ((uint32_t)0x0000FFF0)        /*!< DAC channel1 12-bit Left aligned data */
#define  SET_DAC_DHR12LD_DACC1DHR(DACx_BASE,a)	SET_REG_BITn(DACx_BASE+0x24,4,MASKb12,a)
//#define  DAC_DHR12LD_DACC2DHR                ((uint32_t)0xFFF00000)        /*!< DAC channel2 12-bit Left aligned data */
#define  SET_DAC_DHR12LD_DACC2DHR(DACx_BASE,a)	SET_REG_BITn(DACx_BASE+0x24,20,MASKb12,a)
/******************   DAC_DHR8RD 寄存器 ****************28**/
//#define  DAC_DHR8RD_DACC1DHR                 ((uint16_t)0x00FF)            /*!< DAC channel1 8-bit Right aligned data */
#define  SET_DAC_DHR8RD_DACC1DHR(DACx_BASE,a)	SET_REG_BITn(DACx_BASE+0x28,0,MASKb8,a)
//#define  DAC_DHR8RD_DACC2DHR                 ((uint16_t)0xFF00)            /*!< DAC channel2 8-bit Right aligned data */
#define  SET_DAC_DHR8RD_DACC2DHR(DACx_BASE,a)	SET_REG_BITn(DACx_BASE+0x28,8,MASKb8,a)

/*******************   DAC_DOR1 寄存器 *****************2c**/
//#define  DAC_DOR1_DACC1DOR                   ((uint16_t)0x0FFF)            /*!< DAC channel1 data output */
#define  wDAC_DOR1(DACx_BASE)				(MEM_ADDR(DACx_BASE +0x2c))
/*******************   DAC_DOR2 寄存器 *****************30**/
//#define  DAC_DOR2_DACC2DOR                   ((uint16_t)0x0FFF)            /*!< DAC channel2 data output */
#define  wDAC_DOR2(DACx_BASE)				(MEM_ADDR(DACx_BASE +0x30))
/********************   DAC_SR 寄存器 ******************34??**/
#define  bDAC_SR_DMAUDR1(DACx_BASE)        BIT_ADDR(DACx_BASE+0x34,13)        /*!< DAC channel1 DMA underrun flag */
#define  bDAC_SR_DMAUDR2(DACx_BASE)        BIT_ADDR(DACx_BASE+0x34,29)        /*!< DAC channel2 DMA underrun flag */

/******************************************************************************/
/*                                                                            */
/*                          SDIO接口                                          */
/*                                                                            */
/******************************************************************************/

/******************  SDIO_POWER 寄存器  ******************/
           
#define  SET_SDIO_POWER_PWRCTRL(a)	SET_REG_BITn(SDIO_BASE,0,MASKb2,a)		/*!< PWRCTRL[1:0] bits (Power supply control bits) */


/******************  SDIO_CLKCR 寄存器  ******************/
//#define  SDIO_CLKCR_CLKDIV                   ((uint16_t)0x00FF)            
#define  SET_SDIO_CLKCR_CLKDIV(a)		SET_REG_BITn(SDIO_BASE+0x04,0,MASKb8,a)		/*!< Clock divide factor */
#define  bSDIO_CLKCR_CLKEN          BIT_ADDR(SDIO_BASE+0x04,8)            /*!< Clock enable bit */
#define  bSDIO_CLKCR_PWRSAV         BIT_ADDR(SDIO_BASE+0x04,9)            /*!< Power saving configuration bit */
#define  bSDIO_CLKCR_BYPASS         BIT_ADDR(SDIO_BASE+0x04,10)            /*!< Clock divider bypass enable bit */

#define  SET_SDIO_CLKCR_WIDBUS(a)		SET_REG_BITn(SDIO_BASE+0x04,11,MASKb2,a) /*!< WIDBUS[1:0] bits (Wide bus mode enable bit) */

#define  bSDIO_CLKCR_NEGEDGE        BIT_ADDR(SDIO_BASE+0x04,13)           /*!< SDIO_CK dephasing selection bit */
#define  bSDIO_CLKCR_HWFC_EN        BIT_ADDR(SDIO_BASE+0x04,14)           /*!< HW Flow Control enable */

/*******************  SDIO_ARG 寄存器  *******************/
#define  wSDIO_ARG                   (MEM_ADDR(SDIO_BASE +0x08))            /*!< Command argument */

/*******************  SDIO_CMD 寄存器  *******************/
#define  SET_SDIO_CMD_CMDINDEX      SET_REG_BITn(SDIO_BASE+0x0C,0,MASKb6,a)            /*!< Command Index */

#define  SET_SDIO_CMD_WAITRESP      SET_REG_BITn(SDIO_BASE+0x0C,6,MASKb2,a)   /*!< WAITRESP[1:0] bits (Wait for response bits) */

#define  bSDIO_CMD_WAITINT          BIT_ADDR(SDIO_BASE+0x0C,8)            /*!< CPSM Waits for Interrupt Request */
#define  bSDIO_CMD_WAITPEND         BIT_ADDR(SDIO_BASE+0x0C,9)            /*!< CPSM Waits for ends of data transfer (CmdPend internal signal) */
#define  bSDIO_CMD_CPSMEN           BIT_ADDR(SDIO_BASE+0x0C,10)            /*!< Command path state machine (CPSM) Enable bit */
#define  bSDIO_CMD_SDIOSUSPEND      BIT_ADDR(SDIO_BASE+0x0C,11)            /*!< SD I/O suspend command */
#define  bSDIO_CMD_ENCMDCOMPL       BIT_ADDR(SDIO_BASE+0x0C,12)            /*!< Enable CMD completion */
#define  bSDIO_CMD_NIEN             BIT_ADDR(SDIO_BASE+0x0C,13)            /*!< Not Interrupt Enable */
#define  bSDIO_CMD_CEATACMD         BIT_ADDR(SDIO_BASE+0x0C,14)            /*!< CE-ATA command */

/*****************  SDIO_RESPCMD 寄存器  *****************/
#define  wSDIO_RESPCMD       (MEM_ADDR(SDIO_BASE+0x10))               /*!< 命令响应寄存器（只读）Response command index */

/******************  SDIO_RESP1 寄存器  ******************/
#define  wSDIO_RESP1              (MEM_ADDR(SDIO_BASE+0x14))        /*!< Card Status */

/******************  SDIO_RESP2 寄存器  ******************/
#define  wSDIO_RESP2              (MEM_ADDR(SDIO_BASE+0x18))        /*!< Card Status */

/******************  SDIO_RESP3 寄存器  ******************/
#define  wSDIO_RESP3              (MEM_ADDR(SDIO_BASE+0x1C))        /*!< Card Status */

/******************  SDIO_RESP4 寄存器  ******************/
#define  wSDIO_RESP4              (MEM_ADDR(SDIO_BASE+0x20))        /*!< Card Status */

/******************  SDIO_DTIMER 寄存器  *****************/
#define  wSDIO_DTIMER              (MEM_ADDR(SDIO_BASE+0x24))        /*!< Data timeout period. */

/******************  SDIO_DLEN 寄存器  *******************/
#define  wSDIO_DLEN              (MEM_ADDR(SDIO_BASE+0x28))        /*!< Data length value */

/******************  SDIO_DCTRL 寄存器  ******************/
#define  bSDIO_DCTRL_DTEN         BIT_ADDR(SDIO_BASE+0x2C,0)            /*!< Data transfer enabled bit */
#define  bSDIO_DCTRL_DTDIR        BIT_ADDR(SDIO_BASE+0x2C,1)            /*!< Data transfer direction selection */
#define  bSDIO_DCTRL_DTMODE       BIT_ADDR(SDIO_BASE+0x2C,2)            /*!< Data transfer mode selection */
#define  bSDIO_DCTRL_DMAEN        BIT_ADDR(SDIO_BASE+0x2C,3)            /*!< DMA enabled bit */

#define  SET_SDIO_DCTRL_DBLOCKSIZE    SET_REG_BITn(SDIO_BASE+0x2C,4,MASKb4,a)            /*!< DBLOCKSIZE[3:0] bits (Data block size) */

#define  bSDIO_DCTRL_RWSTART      BIT_ADDR(SDIO_BASE+0x2C,8)            /*!< Read wait start */
#define  bSDIO_DCTRL_RWSTOP       BIT_ADDR(SDIO_BASE+0x2C,9)            /*!< Read wait stop */
#define  bSDIO_DCTRL_RWMOD        BIT_ADDR(SDIO_BASE+0x2C,10)            /*!< Read wait mode */
#define  bSDIO_DCTRL_SDIOEN       BIT_ADDR(SDIO_BASE+0x2C,11)            /*!< SD I/O enable functions */

/******************  SDIO_DCOUNT 寄存器  *****************/
#define  wSDIO_DCOUNT              (MEM_ADDR(SDIO_BASE+0x30))        /*!< Data count value */

/******************  SDIO_STA 寄存器（只读）  ********************/
#define  bSDIO_STA_CCRCFAIL      BIT_ADDR(SDIO_BASE+0x34,0)        /*!< Command response received (CRC check failed) */
#define  bSDIO_STA_DCRCFAIL      BIT_ADDR(SDIO_BASE+0x34,1)        /*!< Data block sent/received (CRC check failed) */
#define  bSDIO_STA_CTIMEOUT      BIT_ADDR(SDIO_BASE+0x34,2)        /*!< Command response timeout */
#define  bSDIO_STA_DTIMEOUT      BIT_ADDR(SDIO_BASE+0x34,3)        /*!< Data timeout */
#define  bSDIO_STA_TXUNDERR      BIT_ADDR(SDIO_BASE+0x34,4)        /*!< Transmit FIFO underrun error */
#define  bSDIO_STA_RXOVERR       BIT_ADDR(SDIO_BASE+0x34,5)        /*!< Received FIFO overrun error */
#define  bSDIO_STA_CMDREND       BIT_ADDR(SDIO_BASE+0x34,6)        /*!< Command response received (CRC check passed) */
#define  bSDIO_STA_CMDSENT       BIT_ADDR(SDIO_BASE+0x34,7)        /*!< Command sent (no response required) */
#define  bSDIO_STA_DATAEND       BIT_ADDR(SDIO_BASE+0x34,8)        /*!< Data end (data counter, SDIDCOUNT, is zero) */
#define  bSDIO_STA_STBITERR      BIT_ADDR(SDIO_BASE+0x34,9)        /*!< Start bit not detected on all data signals in wide bus mode */
#define  bSDIO_STA_DBCKEND       BIT_ADDR(SDIO_BASE+0x34,10)        /*!< Data block sent/received (CRC check passed) */
#define  bSDIO_STA_CMDACT        BIT_ADDR(SDIO_BASE+0x34,11)        /*!< Command transfer in progress */
#define  bSDIO_STA_TXACT         BIT_ADDR(SDIO_BASE+0x34,12)        /*!< Data transmit in progress */
#define  bSDIO_STA_RXACT         BIT_ADDR(SDIO_BASE+0x34,13)        /*!< Data receive in progress */
#define  bSDIO_STA_TXFIFOHE      BIT_ADDR(SDIO_BASE+0x34,14)        /*!< Transmit FIFO Half Empty: at least 8 words can be written into the FIFO */
#define  bSDIO_STA_RXFIFOHF      BIT_ADDR(SDIO_BASE+0x34,15)        /*!< Receive FIFO Half Full: there are at least 8 words in the FIFO */
#define  bSDIO_STA_TXFIFOF       BIT_ADDR(SDIO_BASE+0x34,16)        /*!< Transmit FIFO full */
#define  bSDIO_STA_RXFIFOF       BIT_ADDR(SDIO_BASE+0x34,17)        /*!< Receive FIFO full */
#define  bSDIO_STA_TXFIFOE       BIT_ADDR(SDIO_BASE+0x34,18)        /*!< Transmit FIFO empty */
#define  bSDIO_STA_RXFIFOE       BIT_ADDR(SDIO_BASE+0x34,19)        /*!< Receive FIFO empty */
#define  bSDIO_STA_TXDAVL        BIT_ADDR(SDIO_BASE+0x34,20)        /*!< Data available in transmit FIFO */
#define  bSDIO_STA_RXDAVL        BIT_ADDR(SDIO_BASE+0x34,21)        /*!< Data available in receive FIFO */
#define  bSDIO_STA_SDIOIT        BIT_ADDR(SDIO_BASE+0x34,22)        /*!< SDIO interrupt received */
#define  bSDIO_STA_CEATAEND      BIT_ADDR(SDIO_BASE+0x34,23)        /*!< CE-ATA command completion signal received for CMD61 */

/*******************  SDIO_ICR 寄存器  *******************/
#define  bSDIO_ICR_CCRCFAILC     BIT_ADDR(SDIO_BASE+0x38,0)        /*!< CCRCFAIL flag clear bit */
#define  bSDIO_ICR_DCRCFAILC     BIT_ADDR(SDIO_BASE+0x38,1)        /*!< DCRCFAIL flag clear bit */
#define  bSDIO_ICR_CTIMEOUTC     BIT_ADDR(SDIO_BASE+0x38,2)        /*!< CTIMEOUT flag clear bit */
#define  bSDIO_ICR_DTIMEOUTC     BIT_ADDR(SDIO_BASE+0x38,3)        /*!< DTIMEOUT flag clear bit */
#define  bSDIO_ICR_TXUNDERRC     BIT_ADDR(SDIO_BASE+0x38,4)        /*!< TXUNDERR flag clear bit */
#define  bSDIO_ICR_RXOVERRC      BIT_ADDR(SDIO_BASE+0x38,5)        /*!< RXOVERR flag clear bit */
#define  bSDIO_ICR_CMDRENDC      BIT_ADDR(SDIO_BASE+0x38,6)        /*!< CMDREND flag clear bit */
#define  bSDIO_ICR_CMDSENTC      BIT_ADDR(SDIO_BASE+0x38,7)        /*!< CMDSENT flag clear bit */
#define  bSDIO_ICR_DATAENDC      BIT_ADDR(SDIO_BASE+0x38,8)        /*!< DATAEND flag clear bit */
#define  bSDIO_ICR_STBITERRC     BIT_ADDR(SDIO_BASE+0x38,9)        /*!< STBITERR flag clear bit */
#define  bSDIO_ICR_DBCKENDC      BIT_ADDR(SDIO_BASE+0x38,10)        /*!< DBCKEND flag clear bit */
#define  bSDIO_ICR_SDIOITC       BIT_ADDR(SDIO_BASE+0x38,22)        /*!< SDIOIT flag clear bit */
#define  bSDIO_ICR_CEATAENDC     BIT_ADDR(SDIO_BASE+0x38,23)        /*!< CEATAEND flag clear bit */

/******************  SDIO_MASK 寄存器  *******************/
#define  bSDIO_MASK_CCRCFAILIE   BIT_ADDR(SDIO_BASE+0x3C,0)        /*!< Command CRC Fail Interrupt Enable */
#define  bSDIO_MASK_DCRCFAILIE   BIT_ADDR(SDIO_BASE+0x3C,1)        /*!< Data CRC Fail Interrupt Enable */
#define  bSDIO_MASK_CTIMEOUTIE   BIT_ADDR(SDIO_BASE+0x3C,2)        /*!< Command TimeOut Interrupt Enable */
#define  bSDIO_MASK_DTIMEOUTIE   BIT_ADDR(SDIO_BASE+0x3C,3)        /*!< Data TimeOut Interrupt Enable */
#define  bSDIO_MASK_TXUNDERRIE   BIT_ADDR(SDIO_BASE+0x3C,4)        /*!< Tx FIFO UnderRun Error Interrupt Enable */
#define  bSDIO_MASK_RXOVERRIE    BIT_ADDR(SDIO_BASE+0x3C,5)        /*!< Rx FIFO OverRun Error Interrupt Enable */
#define  bSDIO_MASK_CMDRENDIE    BIT_ADDR(SDIO_BASE+0x3C,6)        /*!< Command Response Received Interrupt Enable */
#define  bSDIO_MASK_CMDSENTIE    BIT_ADDR(SDIO_BASE+0x3C,7)        /*!< Command Sent Interrupt Enable */
#define  bSDIO_MASK_DATAENDIE    BIT_ADDR(SDIO_BASE+0x3C,8)        /*!< Data End Interrupt Enable */
#define  bSDIO_MASK_STBITERRIE   BIT_ADDR(SDIO_BASE+0x3C,9)        /*!< Start Bit Error Interrupt Enable */
#define  bSDIO_MASK_DBCKENDIE    BIT_ADDR(SDIO_BASE+0x3C,10)        /*!< Data Block End Interrupt Enable */
#define  bSDIO_MASK_CMDACTIE     BIT_ADDR(SDIO_BASE+0x3C,11)        /*!< Command Acting Interrupt Enable */
#define  bSDIO_MASK_TXACTIE      BIT_ADDR(SDIO_BASE+0x3C,12)        /*!< Data Transmit Acting Interrupt Enable */
#define  bSDIO_MASK_RXACTIE      BIT_ADDR(SDIO_BASE+0x3C,13)        /*!< Data receive acting interrupt enabled */
#define  bSDIO_MASK_TXFIFOHEIE   BIT_ADDR(SDIO_BASE+0x3C,14)        /*!< Tx FIFO Half Empty interrupt Enable */
#define  bSDIO_MASK_RXFIFOHFIE   BIT_ADDR(SDIO_BASE+0x3C,15)        /*!< Rx FIFO Half Full interrupt Enable */
#define  bSDIO_MASK_TXFIFOFIE    BIT_ADDR(SDIO_BASE+0x3C,16)        /*!< Tx FIFO Full interrupt Enable */
#define  bSDIO_MASK_RXFIFOFIE    BIT_ADDR(SDIO_BASE+0x3C,17)        /*!< Rx FIFO Full interrupt Enable */
#define  bSDIO_MASK_TXFIFOEIE    BIT_ADDR(SDIO_BASE+0x3C,18)        /*!< Tx FIFO Empty interrupt Enable */
#define  bSDIO_MASK_RXFIFOEIE    BIT_ADDR(SDIO_BASE+0x3C,19)        /*!< Rx FIFO Empty interrupt Enable */
#define  bSDIO_MASK_TXDAVLIE     BIT_ADDR(SDIO_BASE+0x3C,20)        /*!< Data available in Tx FIFO interrupt Enable */
#define  bSDIO_MASK_RXDAVLIE     BIT_ADDR(SDIO_BASE+0x3C,21)        /*!< Data available in Rx FIFO interrupt Enable */
#define  bSDIO_MASK_SDIOITIE     BIT_ADDR(SDIO_BASE+0x3C,22)        /*!< SDIO Mode Interrupt Received interrupt Enable */
#define  bSDIO_MASK_CEATAENDIE   BIT_ADDR(SDIO_BASE+0x3C,23)        /*!< CE-ATA command completion signal received Interrupt Enable */

/*****************  SDIO_FIFOCNT 寄存器  *****************/
#define  wSDIO_FIFOCNT            (MEM_ADDR(SDIO_BASE+0x48))        /*!< Remaining number of words to be written to or read from the FIFO */

/******************  SDIO_FIFO 寄存器  *******************/
#define  wSDIO_FIFO               (MEM_ADDR(SDIO_BASE+0x80))       /*!< Receive and transmit FIFO data */


#endif
/*********************************************************************************************
以下附记一下当不用本文件的位段操作时，使用逻辑运算实现相关操作的基本方法

单比特的操作方法（实例中n=9）：
RCC->APB2ENR|=1<<n;    		//置相应寄存器的第n比特为1，对应的位段操作为：bRCC_ENABLE_ADC1=1;	
RCC->APB2ENR&=~(1<<n);   	//置相应寄存器的第n比特为0，对应的位段操作为：bRCC_ENABLE_ADC1=0;

多比特的操作方法（实例中n=18）：
RCC->CFGR&=~(MASKb4<<n);   	//先清除相应寄存器的从第n比特开始的连续4个比特
RCC->CFGR|=a<<n;  					//再把上述的比特位的值设置为a(注意a的值不可超出4比特的值域，即0－15)
														//以上示例为连续4个比特，不同比特数请修改MASKb4为MASKbx
					//以上两句对应的位段操作为：SET_RCC_PLLMUL(a);

***************************************************************************************/
