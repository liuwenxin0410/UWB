#ifndef __MENU_H_
#define __MENU_H_

#include "stm32f10x.h"

#define show_SIZE		17

typedef struct
{
    uint8_t menustaus;   //��ǰ����״̬
    uint8_t funcstaus;   //��ǰ����״̬ 0:����״̬;1:���ڿ���״̬��2:�˳�״̬��
    uint8_t oledmark;    //������ʾˢ��0:����;1:����һ;2:������һ;3:�����ж�;5:�����л�;6:δ������ʾ;7:������ʾ��;8:;
    uint8_t oledcmd;   //�ӻ���ʾ���ƽ���״̬
    uint8_t keymark;    //��ǰ������ֵ
    uint8_t infrared_mak;   //��ǰ�������״̬
    uint16_t infrared_count;
    /*+--------+--------+--------+--------+--------+--------+--------+--------+
     *|  bit7  |  bit6  |  bit5  |  bit4  |  bit3  |  bit2  |  bit1  |  bit0  |
     *|--------|--------|--------|--------+--------+--------+--------+--------|
     *| 0:��Ч | 0:��Ч | 0:��Ч |              bit[4:0]                      |
     *| 1:��ʾ |1:�Զ���|1:�ֶ�ѡ|             ����ʱ�����                   |
     *+--------+--------+--------+--------+--------+--------+--------+--------+
    */
    uint8_t oledmark_count;
    /*+--------+--------+--------+--------+--------+--------+--------+--------+
     *|  bit7  |  bit6  |  bit5  |  bit4  |  bit3  |  bit2  |  bit1  |  bit0  |
     *|--------|--------|--------|--------+--------+--------+--------+--------|
     *|          bit[7:4]                 |          bit[3:0]                 |
     *|        ��ǰ���Ӹ���               |        ����ʱ�����               |
     *+--------+--------+--------+--------+--------+--------+--------+--------+
    */
    uint8_t link_num_mark;
} user_menu_DEF_t;

typedef struct
{
    const char *poweron ;
    const char *show1 ;    //
    const char *show2 ;     //
    const char *show3 ;     //
    const char *show4 ;    //
    const char *show5 ;    //
    const char *show6 ;    //
    const char *show7 ;    //
    const char *show8 ;    //
    const char *show9 ;    //
    const char *show10 ;    //
    const char *show11 ;    //
    const char *show12 ;    //
    const char *show13 ;    //
    const char *show14 ;    //
    const char *show15 ;    //
    const char *show16 ;    //
    const char *show17 ;    //
    const char *show18 ;    //
    const char *show19 ;    //
    const char *show20 ;    //
    const char *show21 ;    //
		const char *show22 ;    //
		const char *show23 ;    //
		const char *show24 ;    //
		const char *show25 ;    //
} user_menu_show_t;



typedef user_menu_DEF_t menu_t;//������
typedef user_menu_show_t show_t;

extern menu_t user_menu;

void menu_olde_show(void);

#endif


