#include "user_tim.h"
#include "user_uwb.h"
#include "exit_user.h"
#include <stdio.h>
#include <string.h>
#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_sleep.h"
#include "lcd.h"
#include "port.h"
#include "lcd_oled.h"
#include "trilateration.h"
#include <math.h>
#include "kalman.h"
#include "AT24C02.h"
#include "stm32_eval.h"
#include "usart3.h"

__attribute((aligned (8))) __IO USER_TIM  USERE_TIME_USECOUNT = {0,0,0,0,0,0,0,0};

const uint8_t TabHL1[12]={0x30,0x18,0x7a,0x10,0x38,0x5a,0x42,0x4a,0x52,0x00,0xff,0xa6};//数据码码表1-9&2字节用户码
const uint8_t TabHL2[12]={0xcf,0xe7,0x85,0xef,0xc7,0xa5,0xbd,0xb5,0xad,0xff,0x00,0x59};//数据码码表1-9&2字节用户码

static __IO uint32_t tim_count = 0;
static uint8_t gpio_bit = 0;

extern __IO uint16_t tim_iqrcounter;
extern u8 USART3_RX_BUF[USART3_MAX_RECV_LEN]; 				//接收缓冲,最大USART3_MAX_RECV_LEN个字节.
extern vu16 USART_RX_STA3;
static infrared_t infrared = {0,};

//遥控器接收状态
//[7]:收到了引导码标志
//[6]:得到了一个按键的所有信息
//[5]:保留
//[4]:标记上升沿是否已经被捕获
//[3:0]:溢出计时器
uint8_t 	RmtSta=0;
u16 Dval;		//下降沿时计数器的值
uint32_t RmtRec=0;	//红外接收到的数据
uint8_t  RmtCnt=0;	//按键按下的次数

static int infrared_send_byte(uint8_t byte);
/******************************************/
//函数名:
//功  能:
//输入值:
//返回值:
//日  期:
/*******************************************/
void Tim4_Init (u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef Time4_BaseInit;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);

    Time4_BaseInit.TIM_Period = arr; //设定计数器自动重装值 最大10ms溢出
    Time4_BaseInit.TIM_Prescaler = psc;	//预分频器,1M的计数频率,1us加1.
    Time4_BaseInit.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
    Time4_BaseInit.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式

    TIM_TimeBaseInit(TIM4,&Time4_BaseInit);

#if ANTHOR == 1
	TIM_OCInitTypeDef 	TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  //选择定时器模式:TIM PWM2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	//比较输出使能
	//TIM_OCInitStructure.TIM_Pulse = (arr+1)/2;	  //占空比 50%
	TIM_OCInitStructure.TIM_Pulse = (arr+1)/3;	  //占空比1:3
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	//输出极性:TIM输出比较极性高
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);

	infrared.tim_pulse = TIM_OCInitStructure.TIM_Pulse;
	infrared.tim_one_sign = 560/26*2;
	infrared.tim_zero_sign = 560/26 + 1685/26;
	infrared.tim_signstatus = sign_no;
	infrared.tim_count = 0;
	infrared.tim_reg = 0;

	/* 使能TIM4在CCR1上的预装载寄存器 */
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);


#endif

#if TAG == 1

	TIM_ICInitTypeDef  TIM_ICInitStructure;
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;  // 选择输入端 IC3映射到TI3上
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频
    TIM_ICInitStructure.TIM_ICFilter = 0x03;//IC4F=0011 配置输入滤波器 8个定时器时钟周期滤波
    TIM_ICInit(TIM4, &TIM_ICInitStructure);//初始化定时器输入捕获通道

	TIM_ITConfig( TIM4,TIM_IT_CC3,ENABLE);//允许更新中断 ,允许CC3IE捕获中断

#endif
    //中断优先级NVIC设置
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM4中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  //先占优先级0级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级3级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
    NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器

	TIM_ITConfig( TIM4,TIM_IT_Update,ENABLE);//允许更新中断 ,允许
    TIM_Cmd(TIM4,ENABLE);
}

static int infrared_send_byte(uint8_t byte){
	static uint8_t bit_count = 0;
	uint8_t bit_oc = 0;
	int res = -1;

	if(infrared.tim_signstatus == sign_no){

		infrared.tim_count = 0;
		if((infrared.tim_reg & 0x80) || (infrared.tim_reg & 0x08)){bit_count ++;infrared.tim_reg = 0;}
		infrared.tim_signstatus = (byte & (1 << (bit_count ))) ? sign_one : sign_zero;
		infrared.tim_reg = 0;
		if(bit_count > 7){bit_count = 0;res  = 0;}
	}
	return res;
}
/**
 * @按照协议发送红外码，在中断中执行
 * @param data
 * @param data_f
 */
static void infrared_send_data(uint8_t data,uint8_t data_f){
	static uint8_t byte_static = 0;
	if (infrared.tim_work != 0) {
		switch (byte_static) {
		case 0:
			if (infrared.tim_signstatus == sign_no && infrared.tim_reg == 0) {
				infrared.tim_signstatus = sign_head;

			}else if ((infrared.tim_reg & 0x04) && infrared.tim_signstatus == sign_head) {
				byte_static = 1;
				infrared.tim_reg = 0;
				infrared.tim_count = 0;
				infrared.tim_signstatus = sign_no;
			}
			break;
		case 1:
			if (0 == infrared_send_byte(TabHL1[9])) {
				byte_static = 2;
				infrared.tim_count = 0;
			}
			break;
		case 2:
			if (0 == infrared_send_byte(TabHL2[9])) {
				byte_static = 3;
				infrared.tim_count = 0;
			}
			break;
		case 3:
			if (0 == infrared_send_byte(data)) {
				byte_static = 4;
				infrared.tim_count = 0;
			}
			break;
		case 4:
			if (0 == infrared_send_byte(data_f)) {
				byte_static = 5;
				infrared.tim_count = 0;
			}
			break;
		default:
			byte_static = 0;
			infrared.tim_work = 0;
			infrared.tim_reg = 0;
			infrared.tim_count = 0;
			TIM4->CCR3 = 0;
			infrared.tim_signstatus = sign_no;
			break;
		}
	}
}
/**
 * @用户发送的数据
 * @param data	机器码
 * @param data_f机器码反码
 */
void user_infrared_data(uint8_t data,uint8_t data_f)
{
	infrared.tim_work = 0xff;
	infrared.tim_data = data;
	infrared.tim_data_f = data_f;
}
/******************************************/
//函数名:
//功  能:
//输入值:
//返回值:
//日  期:2017-07-22
/*******************************************/
void TIM4_IRQHandler (void)
{
    if (TIM_GetITStatus(TIM4,TIM_IT_Update) != RESET)
    {
#if ANTHOR == 1
    	if(tim_iqrcounter & 0x4000) tim_iqrcounter ++;
    	if((tim_iqrcounter & 0x3fff) > 178){		//相当于10ms中断
    		tim_iqrcounter |= 0x8000;
    	}

    	infrared_send_data(infrared.tim_data,infrared.tim_data_f);

    	if(infrared.tim_signstatus != sign_no){
    		infrared.tim_count ++;

    		if(infrared.tim_signstatus == sign_head){
    			if(infrared.tim_count >= (9000+4500)/26){
    				infrared.tim_reg |= 0x04;	//引导码发送完成
    			}else if(infrared.tim_count >= 9000/26){
    				TIM4->CCR3 = 0;
    			}else{
    				TIM4->CCR3 = 1895;
    			}
    		}
    		else if(infrared.tim_signstatus == sign_one){

    			if(infrared.tim_count >= infrared.tim_one_sign){
    				infrared.tim_reg |= 0x20;		//低电平填充
    			}else if(infrared.tim_count >= 560/26){
    				TIM4->CCR3 = 0;
    			}else {
    				TIM4->CCR3 = infrared.tim_pulse;
    				infrared.tim_reg |= 0x10;		//正在调制
    			}

    			if((infrared.tim_reg & 0x20) == 0x20){
    				TIM4->CCR3 = 0;
    				infrared.tim_reg &= ~0xff;
    				infrared.tim_reg |= 0x80;		//代表1信号发送完成
    				infrared.tim_signstatus = sign_no;
    			}

    		}
    		else if(infrared.tim_signstatus == sign_zero){
    			if(infrared.tim_count >= infrared.tim_zero_sign){
    				infrared.tim_reg |= 0x02;		//低电平填充
    			}else if(infrared.tim_count >= 560/26){
    				TIM4->CCR3 = 0;
    			}else {
    				TIM4->CCR3 = infrared.tim_pulse;
    				infrared.tim_reg |= 0x01;		//正在调制
    			}

    			if((infrared.tim_reg & 0x02) == 0x02){
    				TIM4->CCR3 = 0;
    				infrared.tim_reg &= ~0xff;
    				infrared.tim_reg |= 0x08;		//代表0信号发送完成
    				infrared.tim_signstatus = sign_no;
    			}
    		}
    	}
    	else{
    	}
#endif

#if TAG == 1
		if(RmtSta&0x80)								//上次有数据被接收到了
		{
			RmtSta&=~0X10;							//取消上升沿已经被捕获标记
			if((RmtSta&0X0F)==0X00)RmtSta|=1<<6;	//标记已经完成一次按键的键值信息采集
			if((RmtSta&0X0F)<14)RmtSta++;
			else
			{
				RmtSta&=~(1<<7);					//清空引导标识
				RmtSta&=0XF0;						//清空计数器
			}
		}

    	if(tim_iqrcounter & 0x4000) tim_iqrcounter ++;
    	if((tim_iqrcounter & 0x3fff) > 10){		//相当于10ms中断
    		tim_iqrcounter |= 0x8000;
    	}
#endif

		if (tim_iqrcounter & 0x8000) {
			USART3_RX_STA |= 1 << 15;
			tim_iqrcounter &= ~0xffff;
		}
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    }

#if TAG == 1
	if(TIM_GetITStatus(TIM4,TIM_IT_CC3)!=RESET)
	{
		if (RDATA)						//上升沿捕获
		{
			Dval = TIM_GetCapture3(TIM4);
			TIM_OC3PolarityConfig(TIM4, TIM_ICPolarity_Falling);//CC4P=1	设置为下降沿捕获
			TIM_SetCounter(TIM4, 0);//清空定时器值
			RmtSta |= 0X10;//标记上升沿已经被捕获
		} else //下降沿捕获
		{
			Dval = TIM_GetCapture3(TIM4);					//读取CCR4也可以清CC4IF标志位
			TIM_OC3PolarityConfig(TIM4, TIM_ICPolarity_Rising);//CC4P=0	设置为上升沿捕获
			if (RmtSta & 0X10)//完成一次高电平捕获
			{
				if (RmtSta & 0X80)							//接收到了引导码
				{
					if (Dval > 100 && Dval < 900)			//560为标准值,560us
					{
						RmtRec <<= 1;					//左移一位.
						RmtRec |= 0;//接收到0
					} else if (Dval > 1200 && Dval < 2000)	//1680为标准值,1680us
					{
						RmtRec <<= 1;					//左移一位.
						RmtRec |= 1;//接收到1
					} else if (Dval > 2200 && Dval < 2600)//得到按键键值增加的信息 2500为标准值2.5ms
					{
						RmtCnt++; 					//按键次数增加1次
						RmtSta &= 0XF0;//清空计时器
					}
				} else if (Dval > 4000 && Dval < 5500)		//4500为标准值4.5ms
				{
					RmtSta |= 1 << 7;					//标记成功接收到了引导码
					RmtCnt = 0;//清除按键次数计数器
				}
			}
			RmtSta &= ~(1 << 4);
		}
		TIM_ClearITPendingBit(TIM4,TIM_IT_CC3);
	}
#endif

}


/**
 *
 */
void TIM3_Int_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	//使能定时器3时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB  | RCC_APB2Periph_AFIO, ENABLE);  //使能GPIO外设和AFIO复用功能模块时钟
#if ANTHOR == 1 && OBJECT_THIS != OBJECT_3

    #if 0
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); //Timer3部分重映射  TIM3_CH2->PB5

    //设置该引脚为复用输出功能,输出TIM3 CH2的PWM脉冲波形	GPIOB.5
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; //TIM_CH2
  
#else
    //设置该引脚为复用输出功能,输出TIM3 CH4的PWM脉冲波形	GPIOB.1
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; //TIM_CH4
#endif
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIO

    //初始化TIM3
    TIM_TimeBaseStructure.TIM_Period = 899; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
    TIM_TimeBaseStructure.TIM_Prescaler =0; //设置用来作为TIMx时钟频率除数的预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

    //初始化TIM3 Channel2 PWM模式
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
    TIM_OCInitStructure.TIM_Pulse = 0;
#if 0
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC2
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR2上的预装载寄存器
#else
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC4
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR4上的预装载寄存器
#endif  
    TIM_Cmd(TIM3, ENABLE);  //使能TIM3

#elif ANTHOR == 1 && OBJECT_THIS == OBJECT_3
    //设置该引脚为复用输出功能,输出TIM3 CH4的PWM脉冲波形	GPIOB.1
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; //TIM_CH4
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIO

    GPIO_WriteBit(GPIOB,GPIO_Pin_1,1);

#endif
}

//定时器3中断服务程序
void TIM3_IRQHandler(void)   //TIM3中断
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源
    {
        SEGGER_RTT_printf(0, "<%s>[%d]\r\n\r\n",__func__,__LINE__);
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //清除TIMx的中断待处理位:TIM 中断源
    }
}
void TIM2_100us_Init(void)    //100us
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //时钟使能

    //定时器TIM2初始化
    TIM_TimeBaseStructure.TIM_Period = 99; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
    TIM_TimeBaseStructure.TIM_Prescaler =71; //设置用来作为TIMx时钟频率除数的预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位

    TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE ); //使能指定的TIM2中断,允许更新中断

    //中断优先级NVIC设置
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM2中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //先占优先级0级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级3级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
    NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器

    TIM_CLEAN();
    TIM_Cmd(TIM2, ENABLE);  //使能TIMx

    USERE_TIME_USECOUNT.tim_count = 0;
}
//定时器2中断服务程序
void TIM2_IRQHandler(void)   //TIM2中断
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)  //检查TIM2更新中断发生与否
    {
        tim_count ++;
#if ANTHOR == 1			//该方法已放弃
        USERE_TIME_USECOUNT.tim_count ++;

        if(USERE_TIME_USECOUNT.tim_count >= OBJECT_INFRARED){
        	gpio_bit++;
//            GPIO_WriteBit(GPIOB,GPIO_INFRARED, (gpio_bit % 2));  //从机发送红外数据，每半个周期切换电平

        	USERE_TIME_USECOUNT.tim_count = 0;
        }
        
#endif
      
#if TAG == 1
        if(USERE_TIME_USECOUNT.tim_mark & 0x80)USERE_TIME_USECOUNT.tim_count ++;    //主机计算接收红外状态，整个周期
#endif 
        TIM_100us = 1;
        if((tim_count%2) == 0)TIM_200us = 1;
        if((tim_count%10) == 0)TIM_1ms = 1;
        if((tim_count%100) == 0)TIM_10ms = 1;
        if((tim_count%200) == 0)TIM_20ms = 1;
        if((tim_count%500) == 0)TIM_50ms = 1;
        if((tim_count%1000) == 0)TIM_100ms = 1;
        if((tim_count%5000) == 0) {
            TIM_500ms = 1;
        }
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update  );  //清除TIMx更新中断标志
        TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE ); //使能指定的TIM2中断,允许更新中断
    }
}
/**
 *
 * @return
 */
uint32_t get_count(void)
{
    return tim_count;
}
/**
 *
 * @return
 */
uint8_t get_flag(void)
{
//  return USERE_TIME_USECOUNT.time_t.time_mt;
    return 0;
}
/**
 *
 */
static uint8_t pin_bit_mark = 0;
static unsigned long pin_bit_count = 0;

void write_pin_time(uint16_t ms)
{
	if(ms <= 0)return;
	pin_bit_count = portGetTickCount() + ms;
	pin_bit_mark |= (0x80 );
	GPIO_WriteBit(GPIOB,GPIO_Pin_1,0);
}

void write_pin_handle(void)
{
	if(pin_bit_mark & 0x80){
		if((signed long)(portGetTickCount() - pin_bit_count) <= 0){
		    GPIO_WriteBit(GPIOB,GPIO_Pin_1,0);
		}
		else
		{
		    GPIO_WriteBit(GPIOB,GPIO_Pin_1,1);
			pin_bit_mark = 0;
		}
	}
}
