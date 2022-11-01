#include "exit_user.h"
#include "user_tim.h"
#include "user_uwb.h"
#include "menu.h"
#include <stdio.h>
#include <string.h>

static uint32_t count = 0;

extern uint8_t RmtSta;
extern uint32_t RmtRec;	//������յ�������
/**
 *
 */
void exit_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

#if TAG == 1
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	//��ʱ�Ӻ���
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);//��GPIO��ʱ�ӣ��ȴ򿪸��ò����޸ĸ��ù���
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);//Ҫ�ȿ�ʱ�ӣ�����ӳ�䣻����ʾ�ر�jtag��ʹ��swd��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_9;//�����ܽų�ʼ��
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; //PB8 dog_wake
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
#if 0
	EXTI_ClearITPendingBit(EXTI_Line8); //
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource8); //

	EXTI_InitStructure.EXTI_Line = EXTI_Line8; //
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; //
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //
	EXTI_InitStructure.EXTI_LineCmd = ENABLE; //
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x06;	//��ռ���ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;				//�����ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					//ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure);
#endif
#endif

#if ANTHOR == 1
	 GPIO_InitStructure.GPIO_Pin = GPIO_INFRARED;				 //BEEP-->PB.8 �˿�����
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 		 //�������
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //�ٶ�Ϊ50MHz
	 GPIO_Init(GPIOB, &GPIO_InitStructure);	 //���ݲ�����ʼ��GPIOB.8

	 GPIO_SetBits(GPIOB,GPIO_INFRARED);//���0���رշ��������

#endif   
  
  
}
/**
 *
 * @return
 */
uint8_t exit_get(void)
{
  uint8_t res = 0;
  if( (user_menu.infrared_mak & 0x80) && (USERE_TIME_USECOUNT.tim_mark & 0x40))
  {
	printf(0, "<%s>[%d]<<%d>>tim_count:%d\r\n",__func__,__LINE__,count,user_menu.infrared_count);
    if(user_menu.infrared_count == (OBJECT_INFRARED_1 * 2))
    {
      res = OBJECT_AMRK_1;
    }
    else if(user_menu.infrared_count == (OBJECT_INFRARED_2 * 2) )
    {
      res = OBJECT_AMRK_2;
    }
    else if(user_menu.infrared_count == (OBJECT_INFRARED_3 * 2) )
    {
      res = OBJECT_AMRK_3;
    }
    else if(user_menu.infrared_count == (OBJECT_INFRARED_4 * 2) )
    {
      res = OBJECT_AMRK_4;
    }
    else
    {
      res = 0;
    }
    user_menu.infrared_mak = 0;
    USERE_TIME_USECOUNT.tim_mark = 0;
  }
  
  return res;
}


/**
 *
 * @return
 */
uint8_t exit_infrared_get(void)
{
	uint8_t sta=0;
	uint8_t t1 = 0,t2 = 0;
	if(RmtSta&(1<<6))//�õ�һ��������������Ϣ��
	{
	    t1=RmtRec>>24;			//�õ���ַ��
	    t2=(RmtRec>>16)&0xff;	//�õ���ַ����
 	    if((t1==(u8)~t2)&&t1==REMOTE_ID)//����ң��ʶ����(ID)����ַ
	    {
	        t1=RmtRec>>8;
	        t2=RmtRec;
	        if(t1==(u8)~t2)sta=t1;//��ֵ��ȷ
		}
		if((sta==0)||((RmtSta&0X80)==0))//�������ݴ���/ң���Ѿ�û�а�����
		{
		 	RmtSta&=~(1<<6);//������յ���Ч������ʶ
			RmtCnt=0;		//�����������������
		}
	}
	//TODO
	sta = RmtRec>>8;
	RmtRec = 0;
	RmtCnt=0;
	//���ֻ�����
    return sta;
}

#if TAG == 1
/**
 *
 * @param
 * @return
 */
void EXTI9_5_IRQHandler(void) 
{
	count ++;
  if (EXTI_GetITStatus(EXTI_Line8) != RESET) 
  {
    if((USERE_TIME_USECOUNT.tim_mark & 0x80) == 0)
    {
      USERE_TIME_USECOUNT.tim_mark |= 0x80;
    }
    else if(USERE_TIME_USECOUNT.tim_mark & 0x80)
    {
      USERE_TIME_USECOUNT.tim_mark &= ~0x80;
      USERE_TIME_USECOUNT.tim_mark |= 0x40;
      user_menu.infrared_count = USERE_TIME_USECOUNT.tim_count;
      USERE_TIME_USECOUNT.tim_count = 0;
      user_menu.infrared_mak |= 0x80;
    }
    EXTI_ClearITPendingBit(EXTI_Line8); //
  }
}


#endif
/**
 *
 * @param mode
 * @return
 */
uint8_t exit_key_get(uint8_t mode)//ɨ��4�� IO���Ƿ��а�������
{
  static u8 key_up = 1;
  static uint8_t key_delay = 0,res = 0;
	if(mode)key_up=1;
#if TAG == 1
	if(key_up && (KEY0==0 || KEY1==0 || KEY2==0 || KEY3==0) && (key_delay == 0))
	{
		res = 0;
		key_delay ++;
		return 0;
	}

	if((key_up && (KEY0 == 0 || KEY1 == 0 || KEY2 == 0 || KEY3 == 0)) && key_delay > 0)
	{
		key_up=0;
		if(KEY0==0)res = KEY0_PLAY;
		else if(KEY1==0)res = KEY1_SET;
		else if(KEY2==0)res = KEY2_DOWN;
		else if(KEY3==0)res = KEY3_UP;
	}
	else if(KEY0==1&&KEY1==1&&KEY2==1&&KEY3==1 && key_delay > 0){
		key_up = 1;
		key_delay = 0;
		return res;
  }
#endif
	  return 0;
}



