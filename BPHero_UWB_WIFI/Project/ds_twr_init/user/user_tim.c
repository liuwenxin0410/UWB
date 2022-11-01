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

const uint8_t TabHL1[12]={0x30,0x18,0x7a,0x10,0x38,0x5a,0x42,0x4a,0x52,0x00,0xff,0xa6};//���������1-9&2�ֽ��û���
const uint8_t TabHL2[12]={0xcf,0xe7,0x85,0xef,0xc7,0xa5,0xbd,0xb5,0xad,0xff,0x00,0x59};//���������1-9&2�ֽ��û���

static __IO uint32_t tim_count = 0;
static uint8_t gpio_bit = 0;

extern __IO uint16_t tim_iqrcounter;
extern u8 USART3_RX_BUF[USART3_MAX_RECV_LEN]; 				//���ջ���,���USART3_MAX_RECV_LEN���ֽ�.
extern vu16 USART_RX_STA3;
static infrared_t infrared = {0,};

//ң��������״̬
//[7]:�յ����������־
//[6]:�õ���һ��������������Ϣ
//[5]:����
//[4]:����������Ƿ��Ѿ�������
//[3:0]:�����ʱ��
uint8_t 	RmtSta=0;
u16 Dval;		//�½���ʱ��������ֵ
uint32_t RmtRec=0;	//������յ�������
uint8_t  RmtCnt=0;	//�������µĴ���

static int infrared_send_byte(uint8_t byte);
/******************************************/
//������:
//��  ��:
//����ֵ:
//����ֵ:
//��  ��:
/*******************************************/
void Tim4_Init (u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef Time4_BaseInit;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);

    Time4_BaseInit.TIM_Period = arr; //�趨�������Զ���װֵ ���10ms���
    Time4_BaseInit.TIM_Prescaler = psc;	//Ԥ��Ƶ��,1M�ļ���Ƶ��,1us��1.
    Time4_BaseInit.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
    Time4_BaseInit.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ

    TIM_TimeBaseInit(TIM4,&Time4_BaseInit);

#if ANTHOR == 1
	TIM_OCInitTypeDef 	TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  //ѡ��ʱ��ģʽ:TIM PWM2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	//�Ƚ����ʹ��
	//TIM_OCInitStructure.TIM_Pulse = (arr+1)/2;	  //ռ�ձ� 50%
	TIM_OCInitStructure.TIM_Pulse = (arr+1)/3;	  //ռ�ձ�1:3
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	//�������:TIM����Ƚϼ��Ը�
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);

	infrared.tim_pulse = TIM_OCInitStructure.TIM_Pulse;
	infrared.tim_one_sign = 560/26*2;
	infrared.tim_zero_sign = 560/26 + 1685/26;
	infrared.tim_signstatus = sign_no;
	infrared.tim_count = 0;
	infrared.tim_reg = 0;

	/* ʹ��TIM4��CCR1�ϵ�Ԥװ�ؼĴ��� */
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);


#endif

#if TAG == 1

	TIM_ICInitTypeDef  TIM_ICInitStructure;
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;  // ѡ������� IC3ӳ�䵽TI3��
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ
    TIM_ICInitStructure.TIM_ICFilter = 0x03;//IC4F=0011 ���������˲��� 8����ʱ��ʱ�������˲�
    TIM_ICInit(TIM4, &TIM_ICInitStructure);//��ʼ����ʱ�����벶��ͨ��

	TIM_ITConfig( TIM4,TIM_IT_CC3,ENABLE);//��������ж� ,����CC3IE�����ж�

#endif
    //�ж����ȼ�NVIC����
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM4�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  //��ռ���ȼ�0��
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //�����ȼ�3��
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
    NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���

	TIM_ITConfig( TIM4,TIM_IT_Update,ENABLE);//��������ж� ,����
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
 * @����Э�鷢�ͺ����룬���ж���ִ��
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
 * @�û����͵�����
 * @param data	������
 * @param data_f�����뷴��
 */
void user_infrared_data(uint8_t data,uint8_t data_f)
{
	infrared.tim_work = 0xff;
	infrared.tim_data = data;
	infrared.tim_data_f = data_f;
}
/******************************************/
//������:
//��  ��:
//����ֵ:
//����ֵ:
//��  ��:2017-07-22
/*******************************************/
void TIM4_IRQHandler (void)
{
    if (TIM_GetITStatus(TIM4,TIM_IT_Update) != RESET)
    {
#if ANTHOR == 1
    	if(tim_iqrcounter & 0x4000) tim_iqrcounter ++;
    	if((tim_iqrcounter & 0x3fff) > 178){		//�൱��10ms�ж�
    		tim_iqrcounter |= 0x8000;
    	}

    	infrared_send_data(infrared.tim_data,infrared.tim_data_f);

    	if(infrared.tim_signstatus != sign_no){
    		infrared.tim_count ++;

    		if(infrared.tim_signstatus == sign_head){
    			if(infrared.tim_count >= (9000+4500)/26){
    				infrared.tim_reg |= 0x04;	//�����뷢�����
    			}else if(infrared.tim_count >= 9000/26){
    				TIM4->CCR3 = 0;
    			}else{
    				TIM4->CCR3 = 1895;
    			}
    		}
    		else if(infrared.tim_signstatus == sign_one){

    			if(infrared.tim_count >= infrared.tim_one_sign){
    				infrared.tim_reg |= 0x20;		//�͵�ƽ���
    			}else if(infrared.tim_count >= 560/26){
    				TIM4->CCR3 = 0;
    			}else {
    				TIM4->CCR3 = infrared.tim_pulse;
    				infrared.tim_reg |= 0x10;		//���ڵ���
    			}

    			if((infrared.tim_reg & 0x20) == 0x20){
    				TIM4->CCR3 = 0;
    				infrared.tim_reg &= ~0xff;
    				infrared.tim_reg |= 0x80;		//����1�źŷ������
    				infrared.tim_signstatus = sign_no;
    			}

    		}
    		else if(infrared.tim_signstatus == sign_zero){
    			if(infrared.tim_count >= infrared.tim_zero_sign){
    				infrared.tim_reg |= 0x02;		//�͵�ƽ���
    			}else if(infrared.tim_count >= 560/26){
    				TIM4->CCR3 = 0;
    			}else {
    				TIM4->CCR3 = infrared.tim_pulse;
    				infrared.tim_reg |= 0x01;		//���ڵ���
    			}

    			if((infrared.tim_reg & 0x02) == 0x02){
    				TIM4->CCR3 = 0;
    				infrared.tim_reg &= ~0xff;
    				infrared.tim_reg |= 0x08;		//����0�źŷ������
    				infrared.tim_signstatus = sign_no;
    			}
    		}
    	}
    	else{
    	}
#endif

#if TAG == 1
		if(RmtSta&0x80)								//�ϴ������ݱ����յ���
		{
			RmtSta&=~0X10;							//ȡ���������Ѿ���������
			if((RmtSta&0X0F)==0X00)RmtSta|=1<<6;	//����Ѿ����һ�ΰ����ļ�ֵ��Ϣ�ɼ�
			if((RmtSta&0X0F)<14)RmtSta++;
			else
			{
				RmtSta&=~(1<<7);					//���������ʶ
				RmtSta&=0XF0;						//��ռ�����
			}
		}

    	if(tim_iqrcounter & 0x4000) tim_iqrcounter ++;
    	if((tim_iqrcounter & 0x3fff) > 10){		//�൱��10ms�ж�
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
		if (RDATA)						//�����ز���
		{
			Dval = TIM_GetCapture3(TIM4);
			TIM_OC3PolarityConfig(TIM4, TIM_ICPolarity_Falling);//CC4P=1	����Ϊ�½��ز���
			TIM_SetCounter(TIM4, 0);//��ն�ʱ��ֵ
			RmtSta |= 0X10;//����������Ѿ�������
		} else //�½��ز���
		{
			Dval = TIM_GetCapture3(TIM4);					//��ȡCCR4Ҳ������CC4IF��־λ
			TIM_OC3PolarityConfig(TIM4, TIM_ICPolarity_Rising);//CC4P=0	����Ϊ�����ز���
			if (RmtSta & 0X10)//���һ�θߵ�ƽ����
			{
				if (RmtSta & 0X80)							//���յ���������
				{
					if (Dval > 100 && Dval < 900)			//560Ϊ��׼ֵ,560us
					{
						RmtRec <<= 1;					//����һλ.
						RmtRec |= 0;//���յ�0
					} else if (Dval > 1200 && Dval < 2000)	//1680Ϊ��׼ֵ,1680us
					{
						RmtRec <<= 1;					//����һλ.
						RmtRec |= 1;//���յ�1
					} else if (Dval > 2200 && Dval < 2600)//�õ�������ֵ���ӵ���Ϣ 2500Ϊ��׼ֵ2.5ms
					{
						RmtCnt++; 					//������������1��
						RmtSta &= 0XF0;//��ռ�ʱ��
					}
				} else if (Dval > 4000 && Dval < 5500)		//4500Ϊ��׼ֵ4.5ms
				{
					RmtSta |= 1 << 7;					//��ǳɹ����յ���������
					RmtCnt = 0;//�����������������
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

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	//ʹ�ܶ�ʱ��3ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB  | RCC_APB2Periph_AFIO, ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��
#if ANTHOR == 1 && OBJECT_THIS != OBJECT_3

    #if 0
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); //Timer3������ӳ��  TIM3_CH2->PB5

    //���ø�����Ϊ�����������,���TIM3 CH2��PWM���岨��	GPIOB.5
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; //TIM_CH2
  
#else
    //���ø�����Ϊ�����������,���TIM3 CH4��PWM���岨��	GPIOB.1
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; //TIM_CH4
#endif
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIO

    //��ʼ��TIM3
    TIM_TimeBaseStructure.TIM_Period = 899; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
    TIM_TimeBaseStructure.TIM_Prescaler =0; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

    //��ʼ��TIM3 Channel2 PWMģʽ
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
    TIM_OCInitStructure.TIM_Pulse = 0;
#if 0
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM3 OC2
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR2�ϵ�Ԥװ�ؼĴ���
#else
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM3 OC4
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR4�ϵ�Ԥװ�ؼĴ���
#endif  
    TIM_Cmd(TIM3, ENABLE);  //ʹ��TIM3

#elif ANTHOR == 1 && OBJECT_THIS == OBJECT_3
    //���ø�����Ϊ�����������,���TIM3 CH4��PWM���岨��	GPIOB.1
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; //TIM_CH4
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //�����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIO

    GPIO_WriteBit(GPIOB,GPIO_Pin_1,1);

#endif
}

//��ʱ��3�жϷ������
void TIM3_IRQHandler(void)   //TIM3�ж�
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ
    {
        SEGGER_RTT_printf(0, "<%s>[%d]\r\n\r\n",__func__,__LINE__);
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //���TIMx���жϴ�����λ:TIM �ж�Դ
    }
}
void TIM2_100us_Init(void)    //100us
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //ʱ��ʹ��

    //��ʱ��TIM2��ʼ��
    TIM_TimeBaseStructure.TIM_Period = 99; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
    TIM_TimeBaseStructure.TIM_Prescaler =71; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

    TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM2�ж�,��������ж�

    //�ж����ȼ�NVIC����
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM2�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //��ռ���ȼ�0��
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //�����ȼ�3��
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
    NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���

    TIM_CLEAN();
    TIM_Cmd(TIM2, ENABLE);  //ʹ��TIMx

    USERE_TIME_USECOUNT.tim_count = 0;
}
//��ʱ��2�жϷ������
void TIM2_IRQHandler(void)   //TIM2�ж�
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)  //���TIM2�����жϷ������
    {
        tim_count ++;
#if ANTHOR == 1			//�÷����ѷ���
        USERE_TIME_USECOUNT.tim_count ++;

        if(USERE_TIME_USECOUNT.tim_count >= OBJECT_INFRARED){
        	gpio_bit++;
//            GPIO_WriteBit(GPIOB,GPIO_INFRARED, (gpio_bit % 2));  //�ӻ����ͺ������ݣ�ÿ��������л���ƽ

        	USERE_TIME_USECOUNT.tim_count = 0;
        }
        
#endif
      
#if TAG == 1
        if(USERE_TIME_USECOUNT.tim_mark & 0x80)USERE_TIME_USECOUNT.tim_count ++;    //����������պ���״̬����������
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
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update  );  //���TIMx�����жϱ�־
        TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM2�ж�,��������ж�
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
