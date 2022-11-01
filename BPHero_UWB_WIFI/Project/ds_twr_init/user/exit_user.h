#ifndef __EXIT_USER_H_
#define __EXIT_USER_H_

#include "stm32f10x.h"

#define KEY0  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3)
#define KEY1  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4)
#define KEY2  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5)
#define KEY3  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9)

#define KEY0_PLAY  1
#define KEY1_SET   2
#define KEY2_DOWN  3
#define KEY3_UP    4

#define RDATA 	GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_8)	 	//�������������
//����ң��ʶ����(ID),ÿ��ң�����ĸ�ֵ��������һ��,��Ҳ��һ����.
//����ѡ�õ�ң����ʶ����Ϊ0
#define REMOTE_ID 0

#define GPIO_INFRARED	GPIO_Pin_8

typedef enum Key_def
{
    key_set = 0,
    key_exit = 1,
    key_up = 2,
    key_down = 3,
} key_def;


extern uint8_t RmtCnt;			//�������µĴ���

void exit_init(void);
uint8_t exit_get(void);
uint8_t exit_key_get(uint8_t mode);
uint8_t exit_infrared_get(void);

#endif
