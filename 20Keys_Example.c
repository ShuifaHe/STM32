//这里再举一个4*4矩阵键盘另加直联4个方向键，共20个按键的范例

//设PD3、PD4、PD5、PD6为普通直联端口的四个键（上下左右方向键）

//4*4行列式矩阵键盘硬件连接：
//设PD8、PD9、PD10、PD11为行列式键盘的按键输入（为方便读取，这4个IO口尽量分配为连续引脚号）
//设PD12、PD13、PD14、PD15为行列式键盘的扫描输出（这4个IO口随意）

#define KeyNumMax					20			//硬件实体按键数量4*4+4=20个
typedef	u32 KeyS_Type;//定义状态字为32位数据类型

#define KEY_OUT_LINE_NULL	GPIOD->BRR = 0x0f<<12	/*清除所有输出*/
#define KEY_OUT_LINE1		GPIOD->BSRR = 0x01<<12	/*扫描输出第一行*/
#define KEY_OUT_LINE2		GPIOD->BSRR = 0x02<<12	/*扫描输出第二行*/
#define KEY_OUT_LINE3		GPIOD->BSRR = 0x04<<12	/*扫描输出第三行*/
#define KEY_OUT_LINE4		GPIOD->BSRR = 0x08<<12	/*扫描输出第四行*/

#define KEY_IN		(GPIOD->IDR&(GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11))/*扫描输入值*/
#define KEY_IN0		(KEY_IN>>8)	/*获取输入移位到低端（假如是PD0－3的话就更简单，不用移位了）*/


//按键硬件读端口位置
#define KB_RIGHT_IN  	PDin(3)//读取按键0 
#define KB_DOWN_IN  	PDin(4)//读取按键1 
#define KB_LEFT_IN  	PDin(5)//读取按键2 
#define KB_UP_IN 	PDin(6)//读取按键3 


//20个硬件实体按键的编号，键态字依此顺序按位组合
#define KB_RIGHT 		0
#define KB_DOWN  		1
#define KB_LEFT  		2 
#define KB_UP 			3
#define KB_NUM1 		4
#define KB_NUM2 		5
#define KB_NUM3 		6
#define KB_BACK 		7//退格键
#define KB_NUM4 		8
#define KB_NUM5			9
#define KB_NUM6 		10
#define KB_SPACE 		11//空格键
#define KB_NUM7 		12
#define KB_NUM8 		13
#define KB_NUM9 		14
#define KB_ESC	 		15
#define KB_XING 		16//*号键
#define KB_NUM0 		17
#define KB_JING 		18//#号键
#define KB_ENTER 		19//回车键
//***************以上内容可写在key.h 文件中********


//*************** key.c 文件相应内容 ********
//按键初始化函数
void KEY_Init(void) //IO初始化
{ 
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

	/* PD8,9,10,11按键输入*/
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;		// 下拉输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M时钟速度
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	/* PD12,13,14,15按键扫描输出*/
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M时钟速度
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	/* PD3,4,5,6按键输入,对应四个方向键*/
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		//上拉输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M时钟速度
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}


//硬件按键编码
//以上述20键为例（最大暂支持32键，对于键少的系统 KeyS_Type可定义为u16或u8）
KeyS_Type GetHalKeyCode(void)
{
	KeyS_Type ktmp=0;
	if(!KB_RIGHT_IN) 	ktmp|=1<<KB_RIGHT;
	if(!KB_DOWN_IN) 	ktmp|=1<<KB_DOWN;
	if(!KB_LEFT_IN) 	ktmp|=1<<KB_LEFT;
	if(!KB_UP_IN) 		ktmp|=1<<KB_UP;
		//扫描行列式键盘
	KEY_OUT_LINE_NULL;
	KEY_OUT_LINE1;
	ktmp |= KEY_IN0<<4;		//或者直接KEY_IN>>4

	KEY_OUT_LINE_NULL;
	KEY_OUT_LINE2;
	ktmp |= KEY_IN0<<8;		//或者直接KEY_IN

	KEY_OUT_LINE_NULL;
	KEY_OUT_LINE3;
	ktmp |= KEY_IN0<<12;		//或者直接KEY_IN<<4

	KEY_OUT_LINE_NULL;
	KEY_OUT_LINE4;
	ktmp |= KEY_IN0<<16;		//或者直接KEY_IN<<8

	return ktmp;
}
