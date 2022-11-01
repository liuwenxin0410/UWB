#include "menu.h"
#include <stdio.h>
#include <string.h>
#include "lcd_oled.h"
#include "user_uwb.h"
#include "SEGGER_RTT.h"
menu_t user_menu  = {0,};
/**
 * 显示文字索引
 */
show_t user_show = {
		"欢迎使用",

		"当前","未连接", "已连接",		//1	2 3

		"风扇", "台灯", "加湿器",		//4 5 6 

		"温度", "湿度", "距离",		//7	8 9

		"光照", "转速", "档位", 		//10 11 12

		"加湿", "正在运行", "主机", 	//13 14 15

		"照明", "模式", "七彩", 		//16 17 18

		"一直", "间歇", "关闭", 		//19 20 21
    "音响","播放","结束", //22 23 24
	   "音量",                      //25
};
#if TAG == 1

extern user_data_t u_control[];

#endif
extern data_t udata;

void menu_olde_show(void){
	static uint8_t buff[24],len = 0,init_flag = 0;
	memset((void*)buff, 0x00, 24);
	oled_clear(0, 0, 128, 8);
#if TAG == 1			//主机显示
	__disable_irq() ;   //关闭总中断
	if(user_menu.oledmark == 0)		//开机显示
	{
		OLED_ShowCH(32,2,(uint8_t*)user_show.poweron);
//		OLED_ShowNum(40,4,1234,4,16);
	}else if(user_menu.oledmark == 1 ){		//一指连接显示

		if(user_menu.link_num_mark / 16 == 0){
			len = snprintf((char*)buff,16, "%s%s",user_show.show1,user_show.show2);
			buff[len] = 0;
			OLED_ShowCH(16,0,buff);
		}
		else{
			OLED_ShowCH(16,0,(uint8_t*)user_show.show3);
			OLED_ShowChar(16+16*3,0,':',16);
			OLED_ShowNum(16+16*3+8,0,(user_menu.link_num_mark / 16),2,16);
		}
		buff[len] = 0;
		OLED_ShowCH(0,2,(uint8_t*)user_show.show7);
		OLED_ShowChar(32,2,':',16);
		OLED_ShowNum(40,2,(udata.temp) / 100,2,16);
		OLED_ShowChar(40+8*2,2,'.',16);
		OLED_ShowNum(40+8*3,2,(udata.temp) % 100,2,16);
		buff[len] = 0;
		OLED_ShowCH(0,4,(uint8_t*)user_show.show8);
		OLED_ShowChar(32,4,(uint8_t)':',16);
		OLED_ShowNum(40,4,(udata.humi) / 100,2,16);
		OLED_ShowChar(40+8*2,4,'.',16);
		OLED_ShowNum(40+8*3,4,(udata.humi) % 100,2,16);
	}else if(user_menu.oledmark == 2){		//按键控制显示
		if(user_menu.funcstaus == 1){
			udata.temp = u_control[user_menu.menustaus].temp;
			udata.humi = u_control[user_menu.menustaus].humi;
			udata.lumi = u_control[user_menu.menustaus].lumi;
			udata.volume = u_control[user_menu.menustaus].volume;
			if(u_control[user_menu.menustaus].head == OBJECT_AMRK_1){	//风扇
				OLED_ShowCH(16*3,0,(uint8_t*)user_show.show4);

				OLED_ShowCH(0,2,(uint8_t*)user_show.show7);
				OLED_ShowChar(32,2,(uint8_t)':',16);
				OLED_ShowNum(40,2,(udata.temp) / 100,2,16);
				OLED_ShowChar(40+8*2,2,'.',16);
				OLED_ShowNum(40+8*3,2,(udata.temp) % 100,2,16);


				OLED_ShowCH(0,4,(uint8_t*)user_show.show8);
				OLED_ShowChar(32,4,(uint8_t)':',16);
				OLED_ShowNum(40,4,(udata.humi) / 100,2,16);
				OLED_ShowChar(40+8*2,4,'.',16);
				OLED_ShowNum(40+8*3,4,(udata.humi) % 100,2,16);

				OLED_ShowCH(0,6,(uint8_t*)user_show.show11);
				OLED_ShowCH(32,6,(uint8_t*)user_show.show12);
				OLED_ShowChar(32 + 32,6,(uint8_t)':',16);
				OLED_ShowNum(40+ 32,6,(udata.value),3,16);
			}
			if(u_control[user_menu.menustaus].head == OBJECT_AMRK_2){	//台灯
				OLED_ShowCH(16*3,0,(uint8_t*)user_show.show5);

				OLED_ShowCH(0,2,(uint8_t*)user_show.show10);
				OLED_ShowChar(32,2,(uint8_t)':',16);
				OLED_ShowNum(40,2,udata.lumi,4,16);

				OLED_ShowCH(0,4,(uint8_t*)user_show.show10);
				OLED_ShowCH(32,4,(uint8_t*)user_show.show12);
				OLED_ShowChar(32 + 32,4,(uint8_t)':',16);
				OLED_ShowNum(40+ 32,4,(udata.value),3,16);

			}
			if(u_control[user_menu.menustaus].head == OBJECT_AMRK_3){	//加湿器
				uint8_t cmd = 0;
				cmd = user_menu.oledcmd;

				OLED_ShowCH(16*2,0,(uint8_t*)user_show.show6);

				OLED_ShowCH(0,2,(uint8_t*)user_show.show8);
				OLED_ShowChar(32,2,(uint8_t)':',16);
				OLED_ShowNum(40,2,(udata.humi) / 100,2,16);
				OLED_ShowChar(40+8*2,2,'.',16);
				OLED_ShowNum(40+8*3,2,(udata.humi) % 100,2,16);

				OLED_ShowCH(0,4,(uint8_t*)user_show.show13);
				OLED_ShowCH(32,4,(uint8_t*)user_show.show17);
				OLED_ShowChar(32 + 32,4,(uint8_t)':',16);
				if((cmd & 0x08) == 0x08){		//08代表长加湿
					OLED_ShowCH(40+32,4,(uint8_t*)user_show.show19);
				}else if((cmd & 0x04) == 0x04){		//4C代表间歇加湿
					OLED_ShowCH(40+32,4,(uint8_t*)user_show.show20);
				}else {		//00代表关闭
					OLED_ShowCH(40+32,4,(uint8_t*)user_show.show21);
				}

				OLED_ShowCH(0,6,(uint8_t*)user_show.show16);
				OLED_ShowCH(32,6,(uint8_t*)user_show.show17);
				OLED_ShowChar(32 + 32,6,(uint8_t)':',16);
				if((cmd & 0x20) == 0x20){		//20代表照明
					OLED_ShowCH(40+32,6,(uint8_t*)user_show.show16);
				}else if((cmd & 0x10) == 0x10){		//10代表七色循环
					OLED_ShowCH(40+32,6,(uint8_t*)user_show.show18);
				}else{		//00代表灯条关闭
					OLED_ShowCH(40+32,6,(uint8_t*)user_show.show21);
				}

			}
			/*********************自己改*******************************************/
				if(u_control[user_menu.menustaus].head == OBJECT_AMRK_4){	//蓝牙音响
				uint8_t cmd = 0;
				cmd = user_menu.oledcmd;

				OLED_ShowCH(16*2,0,(uint8_t*)user_show.show22);

				OLED_ShowCH(0,2,(uint8_t*)user_show.show25);
				OLED_ShowChar(32,2,(uint8_t)':',16);
				OLED_ShowNum(40,2,(udata.volume) / 100,2,16);
				OLED_ShowChar(40+8*2,2,'.',16);
				OLED_ShowNum(40+8*3,2,(udata.volume) % 100,2,16);

				OLED_ShowCH(0,4,(uint8_t*)user_show.show25);
				OLED_ShowCH(32,4,(uint8_t*)user_show.show12);
				OLED_ShowChar(32 + 32,4,(uint8_t)':',16);
				OLED_ShowNum(40+ 32,4,(udata.value),3,16);

			}
			
//			SEGGER_RTT_printf(0, "<%s>[%d]udata.value:%d\r\n",__func__,__LINE__,udata.value);
//			SEGGER_RTT_printf(0, "<%s>[%d]udata.cmd:0x%02x\r\n",__func__,__LINE__,udata.cmd);

		}

	}else if(user_menu.oledmark == 4){
		if(u_control[user_menu.menustaus].head == OBJECT_AMRK_1){	//风扇
			OLED_ShowCH(0+0,0,(uint8_t*)user_show.show4);
			OLED_ShowCH(32+0,0,(uint8_t*)user_show.show14);

		}
		if(u_control[user_menu.menustaus].head == OBJECT_AMRK_2){	//台灯
			OLED_ShowCH(0+0,0,(uint8_t*)user_show.show5);
			OLED_ShowCH(32+0,0,(uint8_t*)user_show.show14);
		}
		if(u_control[user_menu.menustaus].head == OBJECT_AMRK_3){	//加湿器
			OLED_ShowCH(0+0,0,(uint8_t*)user_show.show6);
			OLED_ShowCH(32+8,0,(uint8_t*)user_show.show14);
		}
	/*********************自己改*******************************************/
		if(u_control[user_menu.menustaus].head == OBJECT_AMRK_4){	//蓝牙音响
			OLED_ShowCH(0+0,0,(uint8_t*)user_show.show22);
			OLED_ShowCH(32+8,0,(uint8_t*)user_show.show14);
		}
		udata.x = u_control[user_menu.menustaus].x;
		udata.y = u_control[user_menu.menustaus].y;
		udata.z = u_control[user_menu.menustaus].z;
		udata.l = u_control[user_menu.menustaus].l;

		OLED_ShowChar(0 + 0,2,(uint8_t)'X',16);
		OLED_ShowChar(0 + 8,2,(uint8_t)':',16);
		OLED_ShowNum(16,2,(udata.x),4,16);
		OLED_ShowChar(0 + 64,2,(uint8_t)'Y',16);
		OLED_ShowChar(0 + 64 + 8,2,(uint8_t)':',16);
		OLED_ShowNum(16 + 64,2,(udata.y),4,16);

		OLED_ShowChar(32,4,(uint8_t)'Z',16);
		OLED_ShowChar(32 + 8,4,(uint8_t)':',16);
		OLED_ShowNum(16+32,4,(udata.z),4,16);

		OLED_ShowCH(0+0,6,(uint8_t*)user_show.show9);
		OLED_ShowChar(0 + 32,6,(uint8_t)':',16);
		OLED_ShowNum(0 + 32+8,6,(udata.l),4,16);
		OLED_ShowChar(0+40 +32,6,(uint8_t)'c',16);
		OLED_ShowChar(0+40 +32 + 8,6,(uint8_t)'m',16);
	}else if( user_menu.oledmark == 5 ){		//按键切换控制显示	只有按键模式下会用到
		uint8_t x = 0,y = 0;
		OLED_ShowCH(16*3 , 0 ,(uint8_t*)user_show.show4);
		OLED_ShowCH(16*3 , 2 ,(uint8_t*)user_show.show5);
		OLED_ShowCH(16*3 - 8 , 4 ,(uint8_t*)user_show.show6);
		/************自己改***************/
		OLED_ShowCH(16*3 - 8 , 6 ,(uint8_t*)user_show.show22);

		if( user_menu.menustaus == 0 ) {		//用来区分按键切换时指示到哪一个设备
			y = 0;

		}else if( user_menu.menustaus == 1 ){
			y = 2;
		}else if( user_menu.menustaus == 2 ){
			y = 4;
		}else if( user_menu.menustaus == 3 ){///
			y = 6;
		}
		OLED_ShowChar(0, y, '*', 16);
	}else if( user_menu.oledmark == 6 ){		//按键切换控制显示	只有按键模式下会用到
		OLED_ShowCH(0 + 16 , 0 ,(uint8_t*)user_show.show1);
		OLED_ShowCH(16 + 16 * 2 , 0 ,(uint8_t*)user_show.show2);
	} else if( user_menu.oledmark == 7 ){		//按键切换控制显示	只有按键模式下会用到

		if(user_menu.link_num_mark / 16 == 0){
			len = snprintf((char*)buff,16, "%s%s",user_show.show1,user_show.show2);
			buff[len] = 0;
			OLED_ShowCH(16,0,buff);
		}
		else{
			OLED_ShowCH(16,0,(uint8_t*)user_show.show3);
			OLED_ShowChar(16+16*3,0,':',16);
			OLED_ShowNum(16+16*3+8,0,(user_menu.link_num_mark / 16),2,16);
		}

		OLED_ShowChar(0 + 0,2,(uint8_t)'X',16);
		OLED_ShowChar(0 + 8,2,(uint8_t)':',16);
		OLED_ShowNum(16,2,(udata.x),4,16);
		OLED_ShowChar(0 + 64,2,(uint8_t)'Y',16);
		OLED_ShowChar(0 + 64 + 8,2,(uint8_t)':',16);
		OLED_ShowNum(16 + 64,2,(udata.y),4,16);

		OLED_ShowChar(32,4,(uint8_t)'Z',16);
		OLED_ShowChar(32 + 8,4,(uint8_t)':',16);
		OLED_ShowNum(16+32,4,(udata.z),4,16);

	}
	__enable_irq() ; //开放总中断

	init_flag ++;
	if(init_flag > 50){
		oled_init();            //初始化OLED
		init_flag = 0;
	}

#endif

#if ANTHOR == 1					//从机显示

#endif

#if 0
/*
 *测试字库完整性@{
 */
//	snprintf((char*)buff,16, "%s%s",user_show.show1,user_show.show2);
//	OLED_ShowCH(0,0,buff);
//	snprintf((char*)buff,16, "%s%s",user_show.show3,user_show.show4);
//	OLED_ShowCH(0,2,buff);
//	snprintf((char*)buff,16, "%s%s",user_show.show5,user_show.show6);
//	OLED_ShowCH(0,4,buff);
//	snprintf((char*)buff,16, "%s%s",user_show.show7,user_show.show8);
//	OLED_ShowCH(0,6,buff);

//	snprintf((char*)buff,16, "%s%s",user_show.show9,user_show.show10);
//	OLED_ShowCH(0,0,buff);
//	snprintf((char*)buff,16, "%s%s",user_show.show11,user_show.show12);
//	OLED_ShowCH(0,2,buff);
//	snprintf((char*)buff,16, "%s%s",user_show.show13,user_show.show14);
//	OLED_ShowCH(0,4,buff);
//	snprintf((char*)buff,16, "%s%s",user_show.show15,user_show.show15);
//	OLED_ShowCH(0,6,buff);

//	snprintf((char*)buff,16, "%s%s",user_show.show16,user_show.show17);
//	OLED_ShowCH(0,2,buff);
//	snprintf((char*)buff,16, "%s%s",user_show.show18,user_show.show19);
//	OLED_ShowCH(0,4,buff);
//	snprintf((char*)buff,16, "%s%s",user_show.show20,user_show.show21);
//	OLED_ShowCH(0,6,buff);

/*
 * 测试字库完整性@}
 */

#endif
//	SEGGER_RTT_printf(0, "<%s>[%d]user_menu.oledmark:%d\r\n",__func__,__LINE__,user_menu.oledmark);
}
