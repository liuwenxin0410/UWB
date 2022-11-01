#ifndef __USER_UWB_H_
#define __USER_UWB_H_

#include "stm32f10x.h"
#include "stm32_eval.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define str(x)  		#x

#define OBJECT_1  		1		//风扇
#define OBJECT_2  		2		//台灯
#define OBJECT_3  		3		//加湿器
#define OBJECT_4  		4		//预留
#define OBJECT_AMRK_1  'F'
#define OBJECT_AMRK_2  'S'
#define OBJECT_AMRK_3  'T'
#define OBJECT_AMRK_4  'H'

#define OBJECT_AMRK_MASTER  'M'

#define OBJECT_THIS OBJECT_3    //用作从机区分，主机模式下无效，分别为OBJECT_1，OBJECT_2，OBJECT_3，OBJECT_4

#define MASTER         1	     //0:从机 1:主机

#define LIM_DIST        1000      //最大接收控制距离cm

#if(MASTER == 1)
#define ANTHOR  0     //标签模式
#define TAG     1     //基站模式
#else
#define ANTHOR  1     //标签模式
#define TAG     0     //基站模式
#endif

#define TAG_ID 0x0F
#define MASTER_TAG 0x0F
#define MAX_SLAVE_TAG 0x02
#define SLAVE_TAG_START_INDEX 0x01

#define ANCHOR_MAX_NUM 4
#define ANCHOR_IND     (OBJECT_THIS-1)// 0 1 
//#define ANCHOR_IND ANCHOR_NUM

//从机半波长
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
    uint8_t head;   //头
    uint8_t data_mark;
    uint8_t object;   //控制对象
/*+--------+--------+--------+--------+--------+--------+--------+--------+
 *|  bit7  |  bit6  |  bit5  |  bit4  |  bit3  |  bit2  |  bit1  |  bit0  |
 *|--------|--------|--------|--------|--------|--------|--------|--------|
 *| 0 无效 | 0:无效 | 0:无效 | 0:无效 | 0:无效 | 0:无效 | 0:无效 | 0:无效 |
 *| 1 控制 |1:灯标志| 1:白灯 | 1:彩灯 | 1:加湿 | 1:加湿 |1:湿标志| 1:PWM  |
 *+--------+--------+--------+--------+--------+--------+--------+--------+
*/
    uint8_t cmd;   //控制命令
    int l;   //距离100倍
    int x;   //
    int y;   //
    int z;   //
    int temp;   //温度100倍
    int humi;   //湿度100倍
    int lumi;   //光强100倍
	  int volume; //音量100倍
    int value;   //控制参数
    int value_1;   //控制参数
  
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

