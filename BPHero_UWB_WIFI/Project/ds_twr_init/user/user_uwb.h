#ifndef __USER_UWB_H_
#define __USER_UWB_H_

#include "stm32f10x.h"
#include "stm32_eval.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define str(x)  		#x

#define OBJECT_1  		1		//����
#define OBJECT_2  		2		//̨��
#define OBJECT_3  		3		//��ʪ��
#define OBJECT_4  		4		//Ԥ��
#define OBJECT_AMRK_1  'F'
#define OBJECT_AMRK_2  'S'
#define OBJECT_AMRK_3  'T'
#define OBJECT_AMRK_4  'H'

#define OBJECT_AMRK_MASTER  'M'

#define OBJECT_THIS OBJECT_3    //�����ӻ����֣�����ģʽ����Ч���ֱ�ΪOBJECT_1��OBJECT_2��OBJECT_3��OBJECT_4

#define MASTER         1	     //0:�ӻ� 1:����

#define LIM_DIST        1000      //�����տ��ƾ���cm

#if(MASTER == 1)
#define ANTHOR  0     //��ǩģʽ
#define TAG     1     //��վģʽ
#else
#define ANTHOR  1     //��ǩģʽ
#define TAG     0     //��վģʽ
#endif

#define TAG_ID 0x0F
#define MASTER_TAG 0x0F
#define MAX_SLAVE_TAG 0x02
#define SLAVE_TAG_START_INDEX 0x01

#define ANCHOR_MAX_NUM 4
#define ANCHOR_IND     (OBJECT_THIS-1)// 0 1 
//#define ANCHOR_IND ANCHOR_NUM

//�ӻ��벨��
#define OBJECT_INFRARED_1     550
#define OBJECT_INFRARED_2     450
#define OBJECT_INFRARED_3     300
#define OBJECT_INFRARED_4     200

#if ANTHOR == 1
#if OBJECT_THIS == OBJECT_1
#define OBJECT_AMRK OBJECT_AMRK_1
#define OBJECT_INFRARED     OBJECT_INFRARED_1
#elif OBJECT_THIS == OBJECT_2
#define OBJECT_AMRK OBJECT_AMRK_2
#define OBJECT_INFRARED     OBJECT_INFRARED_2
#elif OBJECT_THIS == OBJECT_3
#define OBJECT_AMRK OBJECT_AMRK_3
#define OBJECT_INFRARED     OBJECT_INFRARED_3
#elif OBJECT_THIS == OBJECT_4
#define OBJECT_AMRK OBJECT_AMRK_4
#define OBJECT_INFRARED     OBJECT_INFRARED_4
#endif
#endif

#if TAG == 1
#define OBJECT_AMRK OBJECT_AMRK_MASTER

#endif

#define TX_BUFF_SIZE_USER   24

#define TX_BUFF_DATA_USER   12
#define TX_BUFF_DATA_USER_L   TX_BUFF_DATA_USER
#define TX_BUFF_DATA_USER_X   TX_BUFF_DATA_USER_L + 2
#define TX_BUFF_DATA_USER_Y   TX_BUFF_DATA_USER_X + 2
#define TX_BUFF_DATA_USER_Z   TX_BUFF_DATA_USER_Y + 2


typedef struct
{
    uint8_t head;   //ͷ
    uint8_t data_mark;
    uint8_t object;   //���ƶ���
/*+--------+--------+--------+--------+--------+--------+--------+--------+
 *|  bit7  |  bit6  |  bit5  |  bit4  |  bit3  |  bit2  |  bit1  |  bit0  |
 *|--------|--------|--------|--------|--------|--------|--------|--------|
 *| 0 ��Ч | 0:��Ч | 0:��Ч | 0:��Ч | 0:��Ч | 0:��Ч | 0:��Ч | 0:��Ч |
 *| 1 ���� |1:�Ʊ�־| 1:�׵� | 1:�ʵ� | 1:��ʪ | 1:��ʪ |1:ʪ��־| 1:PWM  |
 *+--------+--------+--------+--------+--------+--------+--------+--------+
*/
    uint8_t cmd;   //��������
    int l;   //����100��
    int x;   //
    int y;   //
    int z;   //
    int temp;   //�¶�100��
    int humi;   //ʪ��100��
    int lumi;   //��ǿ100��
	  int volume; //����100��
    int value;   //���Ʋ���
    int value_1;   //���Ʋ���
  
} user_data_t;

typedef user_data_t data_t;

extern data_t udata;

void dwt_dumpregisters(char *str, size_t strSize);
void Anchor_Array_Init(void);
void Semaphore_Init(void);
int Sum_Tag_Semaphore_request(void);
void Tag_Measure_Dis(void);

void assert_failed(uint8_t* file, uint32_t line);

int filter(int input, int fliter_idx );

void user_dwb_init(void);
void user_uwb_ruing(void);

void dw1000_send_userdata(void );
void print_data(user_data_t *rx_udata);
#endif

