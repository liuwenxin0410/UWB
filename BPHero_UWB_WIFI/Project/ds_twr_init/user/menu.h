#ifndef __MENU_H_
#define __MENU_H_

#include "stm32f10x.h"

#define show_SIZE		17

typedef struct
{
    uint8_t menustaus;   //当前界面状态
    uint8_t funcstaus;   //当前控制状态 0:空闲状态;1:正在控制状态，2:退出状态。
    uint8_t oledmark;    //控制显示刷新0:开机;1:待机一;2:工作中一;3:工作中二;5:按键切换;6:未连接显示;7:待机显示二;8:;
    uint8_t oledcmd;   //从机显示控制界面状态
    uint8_t keymark;    //当前按键键值
    uint8_t infrared_mak;   //当前红外接收状态
    uint16_t infrared_count;
    /*+--------+--------+--------+--------+--------+--------+--------+--------+
     *|  bit7  |  bit6  |  bit5  |  bit4  |  bit3  |  bit2  |  bit1  |  bit0  |
     *|--------|--------|--------|--------+--------+--------+--------+--------|
     *| 0:无效 | 0:无效 | 0:无效 |              bit[4:0]                      |
     *| 1:提示 |1:自动切|1:手动选|             动作时间计数                   |
     *+--------+--------+--------+--------+--------+--------+--------+--------+
    */
    uint8_t oledmark_count;
    /*+--------+--------+--------+--------+--------+--------+--------+--------+
     *|  bit7  |  bit6  |  bit5  |  bit4  |  bit3  |  bit2  |  bit1  |  bit0  |
     *|--------|--------|--------|--------+--------+--------+--------+--------|
     *|          bit[7:4]                 |          bit[3:0]                 |
     *|        当前连接个数               |        动作时间计数               |
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



typedef user_menu_DEF_t menu_t;//重命名
typedef user_menu_show_t show_t;

extern menu_t user_menu;

void menu_olde_show(void);

#endif


