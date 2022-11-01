/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/main.c
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */
/*! ----------------------------------------------------------------------------
 *  @file    main.c
 *  @brief   Double-sided two-way ranging (DS TWR) responder example code
 *
 *           This is a simple code example which acts as the responder in a DS TWR distance measurement exchange. This application waits for a "poll"
 *           message (recording the RX time-stamp of the poll) expected from the "DS TWR initiator" example code (companion to this application), and
 *           then sends a "response" message recording its TX time-stamp, after which it waits for a "final" message from the initiator to complete
 *           the exchange. The final message contains the remote initiator's time-stamps of poll TX, response RX and final TX. With this data and the
 *           local time-stamps, (of poll RX, response TX and final RX), this example application works out a value for the time-of-flight over-the-air
 *           and, thus, the estimated distance between the two devices, which it writes to the LCD.
 *
 * @attention
 *
 * Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
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
#include "user_tim.h"
#include "user_uwb.h"
#include "bh1750.h"
#include "waificmd.h"
#include "lis2dh12.h"
#include "exit_user.h"
#include "menu.h"
#include "SEGGER_RTT.h"
//=============================================================//
/**************************************************************/
/********More Information Please Visit Our Website*************/
/***********************bphero.com.cn**************************/
/**********************Version V1.1****************************/
/**************************************************************/
//=============================================================//

#define  CONTROL_KEY			1  		//1:按键切换来控制从机	0:通过红外定位从机来控制

#if TAG == 1
#define MEMBER  4  

user_data_t u_control[MEMBER];
            
#endif

static uint8_t rx_buff[1024];
static u16 len = 0;
static char buff[1024];


extern const uint8_t TabHL1[12];
extern const uint8_t TabHL2[12];
/**
 *
 * @return
 */
int main(void)
{
	static uint8_t app_id = 0xff;
    user_data_t *rx_udata = NULL;
    int lumi = 0;//光照强度
    uint8_t Time_ns = 0,data_num = 0,i = 0,pdata[6],ddata = 0,time_200ms = 0,time_5ms = 0;
    uint8_t data_arr[200];
    drv_lis2dh12_t get;
    float p_temperature = 0;
    float p_humidity = 0;
  
#if TAG == 1
    u_control[0].head = OBJECT_AMRK_1;
    u_control[1].head = OBJECT_AMRK_2;
    u_control[2].head = OBJECT_AMRK_3;
    u_control[3].head = OBJECT_AMRK_4;
    u_control[0].object = 0xff;
    u_control[1].object = 0xff;
    u_control[2].object = 0xff;
    u_control[3].object = 0xff;
    udata.head = OBJECT_AMRK;  
    user_menu.oledmark = 0;
    user_menu.link_num_mark = 0;
#endif
  
#if ANTHOR == 1
    udata.head = OBJECT_AMRK;
    static uint8_t cmd_before = 0;
#endif
    /* Start with board specific hardware init. */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    peripherals_init();
    menu_olde_show();
    OLED_DrawMem();
    TIM3_Int_Init();
    TIM2_100us_Init();
    exit_init();
    usart3_init(115200);
    printf("hello dwm1000!\r\n");
#if 1
    user_dwb_init();
    RTK8266_init();
#endif
    while(1)
    {

    	do{
        user_uwb_ruing();
        if( 1 == TIM_100us )
        {
          /**********************/
          /**接收WIFI数据***/
          /**********************/
            if(USART3_RX_STA&0X8000)//接收到期待的应答结果
            {//得到有效数据
#if TAG == 1
				uint8_t b[10] = {0,0,};
				memcpy(b,strstr((char*)USART3_RX_BUF,(char*)"+IPD,"),10);
				uint8_t obj = 0xff;
				if(b[5] != 0)obj = chartonumber(b[5]);
#endif
	            printf("<%s>[%d]buff:%s\r\n",__func__,__LINE__,USART3_RX_BUF);//调试用，查看接收到的原始数据
//              uint8_t Len = 0;
//              Len = sprintf(buff,"hello!");
//              if(Wifi_UDP == (enum ATK8266_UDP)UDP_NUll)
//              UPD_Choose ("OK",sizeof("OK"));
//              Tx_Data(buff,Wifi_UDP,Len);

              /*
              **用于接收WIFI数据解析@{
              */
	          char* str_f = NULL;
	          str_f = strstr((char*)USART3_RX_BUF,(char*)"#");
	          if(*str_f != '#'){		//如果接收wifi模块返回信息，跳出本次循环
          		USART3_RX_STA = 0;
          		USART3_RX_BUF[USART3_RX_STA&0X7fff] = 0;
                TIM_100us = 0;
          		break;
	          }
#if TAG == 1
	          else if(strncmp((char*)str_f, (char*)"#app", 4) == 0){
	        	app_id = obj;
          		USART3_RX_STA = 0;
          		USART3_RX_BUF[USART3_RX_STA&0X7fff] = 0;
                TIM_100us = 0;
          		break;
	          }
#endif
              data_num = sizeof(user_data_t)/sizeof(uint8_t);
              memcpy(rx_buff,str_f,data_num);
//              printf("<%s>[%d]data_num:%d;rx_buff:%s\r\n",__func__,__LINE__,data_num,rx_buff);

              data_t u_data = {0,};
              int value = 0;
              sscanf((char*)&rx_buff[0],"#%d#%d#%d#%d#%d#%d#%d#%d#%d#%d#%d#%d#%d>",\
            		  &u_data.head,&u_data.value,&u_data.object,&u_data.cmd,\
                         &u_data.l,	&u_data.x, &u_data.y, &u_data.z,\
						 &u_data.temp,&u_data.humi,&u_data.lumi,&value,&u_data.value_1);

              rx_udata  = (user_data_t *)&u_data;
              /*
              **用于接收数据解析@}
              */
              print_data(rx_udata);		//打印接收数据
#if ANTHOR == 1
 
						/*
            **从机接收控制逻辑@{
            */
            udata.cmd = rx_udata->cmd;				//保存当前控制命令和参数
            udata.value = rx_udata->value;
            printf("<%s>[%d]udata.cmd:0x%02x;\r\n",__func__,__LINE__,udata.cmd);

            if(rx_udata->cmd & 0x80)
            {
              user_menu.funcstaus = 1;
              if((rx_udata->cmd & 0x81) == 0x81){
#if OBJECT_THIS == OBJECT_2
            	  user_set_PWM(((rx_udata->value > 0) ? 0 + rx_udata->value * 90 - 1:0 ));    //控制PWM
#elif OBJECT_THIS == OBJECT_1
            	  user_set_PWM(((rx_udata->value > 0) ? 499 + rx_udata->value * 30:0 ));    //控制PWM
#elif OBJECT_THIS == OBJECT_3
            	  user_set_PWM(((rx_udata->value > 0) ? 0 + rx_udata->value * 90 - 1:0));    //控制PWM
#endif
              }else if(((rx_udata->cmd & 0x40) || (rx_udata->cmd & 0x02)) && (cmd_before != rx_udata->cmd)){

					if(((rx_udata->cmd & 0x08) == 0x08 || (rx_udata->cmd & 0x04) == 0x04 || (rx_udata->cmd & 0x02) == 0x02) && \
							((rx_udata->cmd & (0x02 | 0x04 | 0x08)) != (cmd_before & (0x02 | 0x04 | 0x08)))){
						write_pin_time(45);    //控制加湿
					}else if(((rx_udata->cmd & 0x20) == 0x20 || (rx_udata->cmd & 0x10) == 0x10 || (rx_udata->cmd & 0x40) == 0x40) && \
							((rx_udata->cmd & (0x02 | 0x04 | 0x08)) != (cmd_before & (0x20 | 0x10 | 0x40)))){
					  write_pin_time(3500);    //控制灯
					}
					/*else if(((rx_udata->cmd & 0x20) == 0x20 || (rx_udata->cmd & 0x10) == 0x10 || (rx_udata->cmd & 0x40) == 0x40) && \
							((rx_udata->cmd & (0x02 | 0x04 | 0x08)) != (cmd_before & (0x20 | 0x10 | 0x40)))){
					  write_pin_time(3500);    //控制音响
					}*/
					cmd_before = rx_udata->cmd;
				}
            }

//            SEGGER_RTT_printf(0, "<%s>[%d]rx_udata->cmd:0x%02x\r\nudata.value:%d\r\n\r\n",__func__,__LINE__,rx_udata->cmd,udata.value);
            
            if ((rx_udata->cmd & (0x80 | 0x40)) == 0 && user_menu.funcstaus == 1)    //退出控制状态
            {
              user_menu.funcstaus = 0;
            }
            
            /*
            **从机接收控制逻辑@}
            */
#endif   
#if TAG == 1
		  /*
		  **主机用来区分
		  */
            if(app_id != 0xff){
            	len = 0;
            	len += sprintf(buff, "%s",rx_buff);
            	buff[len] = 0;
                Tx_Data(buff,app_id,len);
            }

				rx_udata->object = obj;     //区分从机客户端,将不同从机数据分别放入不同数据库里
				do {
					for (i = 0; i < MEMBER; i++) {
						if (u_control[i].head == rx_udata->head) {
							memcpy((void*) &u_control[i], (void*) rx_udata,
									data_num);
						}
					}

				} while (0);
#endif
				USART3_RX_BUF[USART3_RX_STA&0X7fff] = 0;
                USART3_RX_STA=0;
            }
            TIM_100us = 0;
        }
        if(1 == TIM_200us)
        {
            TIM_200us = 0;
        }
        if(1 == TIM_1ms)
        {
#if ANTHOR == 1 && OBJECT_THIS == OBJECT_2
        	if(time_5ms ++ > 4){
        		write_pin_handle();			//IO口定时电平低电平回调函数
        		time_5ms = 0;
        	}
#endif
            TIM_1ms = 0;
        }
        if(1 == TIM_10ms)
        {
          TIM_10ms = 0;
#if TAG == 1
          user_menu.keymark = exit_key_get(1);    //获取按键值
          if(user_menu.keymark != 0)
          {
        	  if(user_menu.keymark == KEY2_DOWN ){						//下键
        		  if( user_menu.oledmark == 2){		//控制工作界面
					  if(user_menu.menustaus == 2){			//加湿器需要循环控制
						  udata.cmd |= user_menu.oledcmd;
						  udata.cmd |= 0x02;
						  if((udata.cmd & 0x08) == 0x08){		//08代表长加湿
							  udata.cmd &=~ 0x08;
							  udata.cmd |= 0x04;
						  }
						  else if((udata.cmd & 0x04) == 0x04){		//04代表间歇加湿
							  udata.cmd &=~ 0x04;
							  udata.cmd |= 0x00;
						  }
						  else if((udata.cmd & 0x0C) == 0x00){		//00代表关闭
								  udata.cmd |= 0x08;
						  }
						  user_menu.oledcmd = udata.cmd;		//显示逻辑用,同时显示灯和加湿器状态
						  udata.cmd &=~ 0x70;					//同一时刻只能发送控制一个选项
						  SEGGER_RTT_printf(0, "<%s>[%d]udata.cmd:0x%02x\r\n",__func__,__LINE__,udata.cmd);
					  }
					  udata.value -= 1;
					  if(udata.value <= 0)udata.value = 0 ;
        		  }else if(user_menu.oledmark == 5){		//手动切换从机控制
        			  user_menu.menustaus ++;
        			  if(user_menu.menustaus > 4)user_menu.menustaus = 0;
        			  if(user_menu.oledmark_count & 0x20)user_menu.oledmark_count &= ~0x1f;
//                      SEGGER_RTT_printf(0, "<%s>[%d]user_menu.menustaus:%d\r\n",__func__,__LINE__,user_menu.menustaus);
        		  }

			  }else if(user_menu.keymark == KEY3_UP ){						//上键
        		  if( user_menu.oledmark == 2){
					  if(user_menu.menustaus == 2){			//加湿器需要循环控制
						  udata.cmd |= user_menu.oledcmd;
						  udata.cmd |= 0x40;
						  if((udata.cmd & 0x30) == 0x00){		//00代表灯条关闭
							  udata.cmd |= 0x20;
						  }
						  else if((udata.cmd & 0x20) == 0x20){		//20代表白灯照明
							  udata.cmd &=~ 0x20;
							  udata.cmd |= 0x10;
						  }
						  else if((udata.cmd & 0x10) == 0x10){		//10代表七色循环
								  udata.cmd &= ~0x10;
						  }

						  user_menu.oledcmd = udata.cmd;		//显示逻辑用,同时显示灯和加湿器状态
						  udata.cmd &=~ 0x0f;					//同一时刻只能发送控制一个选项
						  SEGGER_RTT_printf(0, "<%s>[%d]udata.cmd:0x%02x\r\n",__func__,__LINE__,udata.cmd);
					  }
					  udata.value += 1;
					  if(udata.value > 10)udata.value = 10 ;
        		  }else if(user_menu.oledmark == 5){		//手动切换控制///////////////////////////////////
        			  user_menu.menustaus --;
        			  if(user_menu.menustaus < 0 || user_menu.menustaus > 3)user_menu.menustaus = 3;
        			  if(user_menu.oledmark_count & 0x20)user_menu.oledmark_count &= ~0x1f;
        		  }
			  }else if(user_menu.keymark == KEY0_PLAY){						//退出按键
                  if(user_menu.oledmark > 1){
    	              udata.cmd = 0x00;
    	              udata.value = 0;
                	  user_menu.funcstaus = 2;
                	  user_menu.oledmark = 1;
            		  user_menu.oledmark_count = 0;
            		  user_menu.oledmark_count |= 0x40;		//进入待机
                  }
			  }else if(user_menu.keymark == KEY1_SET){		//确认按键
                  if(user_menu.oledmark == 1 || user_menu.oledmark == 7){
                	  if(user_menu.funcstaus == 0){		//待机状态下点击确认，进入手动选择控制界面
                		  user_menu.oledmark = 5;
                		  user_menu.oledmark_count = 0;
                		  user_menu.oledmark_count |= 0x20;		//进入手动选择
                	  }
                  }
                  else if(user_menu.oledmark == 2){		//工作中切换菜单显示界面
//    	              udata.cmd = 0x40;
                	  user_menu.oledmark = 3;
                  }
                  else if(user_menu.oledmark == 3){     //工作中切换菜单显示界面
                	  user_menu.oledmark = 2;
                  }
                  else if(user_menu.oledmark == 5){			//手动切换控制
                	  if((u_control[user_menu.menustaus].object != 0xff && user_menu.funcstaus == 0) || 0){	//判断当前选择的从机是否可以控制

						  if(user_menu.menustaus == 2){		//工作状态下发送控制命令并显示工作界面
							  udata.cmd = 0x80;	//默认关闭
						  }
						  else {
							  udata.cmd = 0x81;
							  udata.value = 10;
						  }

						  user_menu.oledmark = 2;
						  user_menu.funcstaus = 1;
                	  }else if(u_control[user_menu.menustaus].object == 0xff){		//当前控制设备没有联网
                		  user_menu.oledmark = 6;
                		  user_menu.oledmark_count |= 0x80;
                	  }
				  }
        	  }
//              printf("<%s>[%d]Key:%d\r\n",__func__,__LINE__,user_menu.keymark);
//              SEGGER_RTT_printf(0, "<%s>[%d]user_menu.menustaus:%d\r\n",__func__,__LINE__,user_menu.menustaus);
//              SEGGER_RTT_printf(0, "<%s>[%d]udata.cmd:0x%02x\r\n",__func__,__LINE__,udata.cmd);
//              SEGGER_RTT_printf(0, "<%s>[%d]udata.value:%d\r\n",__func__,__LINE__,udata.value);
//              SEGGER_RTT_printf(0, "<%s>[%d]user_menu.oledmark:%d\r\n\r\n",__func__,__LINE__,user_menu.oledmark);

              //按键处理
          }
#endif
        }
        if(1 == TIM_20ms)
        {
#if ANTHOR == 1 && OBJECT_THIS == OBJECT_2
			/**
			 ** 获取光照值，只有台灯从机
			 */
			lumi = read_BH1750();
			if(lumi >= 0)
				udata.lumi = lumi;

#else
			udata.lumi = 0;
#endif
			TIM_20ms = 0;
        }
        if(1 == TIM_50ms)
        {
        	/**
        	 ** 获取坐标数据
        	 */
            drv_lis2dh12_get_angle(&get);
            udata.x = (uint16_t)(get.acc_x * 1) / 1;
            udata.y = (uint16_t)(get.acc_y * 1) / 1;
            udata.z = (uint16_t)(get.acc_z * 1) / 1;
            TIM_50ms = 0;
			/**
			 **获取温湿度
			 */
            if(SHT20_read_tem(&p_temperature) != -1){
                udata.temp = (int)(p_temperature*100);
            }
            if(SHT20_read_hum(&p_humidity) != -1){
                udata.humi = (int)(p_humidity*100);
            }
						udata.volume = udata.value;//////////////////////////////////////////////////////////
        }
        if(1 == TIM_100ms)
        {
            TIM_100ms = 0;
          if(user_menu.oledmark == 0){		//从开机界面切换到红外指示显示界面

        	  user_menu.oledmark = 1;
        	  user_menu.oledmark_count |= 0x40;
          }

          if(time_200ms++ > 0){		//200ms
#if TAG == 1
            /*
            **主机控制逻辑@{
            */
#if	CONTROL_KEY == 1

        	  /**
        	   **  获取红外值判断从机
        	   */
		  uint8_t infrared_vl = exit_infrared_get();
		  printf("<%s>[%d]infrared_vl:0x%02x\r\n\r\n",__func__,__LINE__,infrared_vl);

            uint8_t infrared_mak = exit_get();

            if(infrared_vl == 0x73){			//从机标识，实测值
            	infrared_mak = OBJECT_AMRK_1;
            }else if(infrared_vl == 0x50){		//从机标识，实测值
            	infrared_mak = OBJECT_AMRK_2;
            }else if(infrared_vl == 0x7b){		//从机标识，实测值
            	infrared_mak = OBJECT_AMRK_3;
            }else if(infrared_vl == TabHL1[4]){
            	infrared_mak = OBJECT_AMRK_4;//自己改
            }else{
            	infrared_mak = 0;
            }

			if (infrared_mak != 0 && user_menu.funcstaus == 0) {
				printf("<%s>[%d]infrared_mak:%c\r\n", __func__,
						__LINE__, infrared_mak);
				data_num = sizeof(user_data_t) / sizeof(uint8_t);//得到结构体长度（所有命令）
				for (i = 0; i < MEMBER; i++) {
					if (u_control[i].head == infrared_mak) {
						memcpy((void*) &udata, (void*) &u_control[i],//把当前从机的数据库拷贝到当前使用的缓存
								data_num);
						udata.head = OBJECT_AMRK;
						if (udata.l < LIM_DIST
								&& udata.object != 0xff) {//距离小于且wifi已连接
							user_menu.oledmark = 2;
							user_menu.oledmark_count &= ~0x40;
							user_menu.funcstaus = 1;
							user_menu.menustaus = i;
							udata.value = 10;
							udata.cmd = 0x81;
							if (user_menu.menustaus == 2) {	//工作状态下发送控制命令并显示工作界面
								udata.cmd |= 0x80;
							}
						}
					}
				}
			}
#else

#endif
            /*
            **主机控制逻辑@}
            */
            menu_olde_show();		//刷新显示内容
            OLED_DrawMem();			//更新屏幕
#endif

#if ANTHOR == 1
          udata.head = OBJECT_AMRK;
          len = 0;    //发送数据结构
          len += sprintf(buff,"#%d#%d#%d#%d#%d#%d#%d#%d#%d#%d#%d#%d#%d#%d>",udata.head,udata.value,udata.object,udata.cmd,\
          udata.l,udata.x,udata.y,udata.z,udata.temp,udata.humi,udata.lumi,udata.volume,udata.value,udata.value_1);

          buff[len] = 0;
          Tx_Data(buff,0,len);

#endif

#if TAG == 1
          /**
           ** 在控制工作中向从机发送控制数据
           */
            if(user_menu.funcstaus != 0 ){
              udata.head = OBJECT_AMRK;
              if(user_menu.funcstaus == 2){
				  user_menu.funcstaus = 0 ;		//pwm为0，结束控制信号，退出工作控制
				  udata.cmd = 0x00;
				  udata.value = 0;
				  user_menu.oledmark = 1;
				  user_menu.oledmark_count |= 0x40;
              }else if(user_menu.funcstaus == 1){
            	  udata.cmd |= 0x80;
              }

              len = 0;  //发送数据结构
              len += sprintf(buff,"#%d#%d#%d#%d#%d#%d#%d#%d#%d#%d#%d#%d#%d>",udata.head,udata.value,udata.object,udata.cmd,\
              udata.l,udata.x,udata.y,udata.z,udata.temp,udata.humi,udata.lumi,udata.value,udata.value_1);

              buff[len] = 0;
              Tx_Data(buff,udata.object,len);
            }
            /**
             **主机向app发送
             */
            if(app_id != 0xff){
                udata.head = OBJECT_AMRK;
                len = 0;
                len += sprintf(&buff[0],"#%d#%d#%d#%d#%d#%d#%d#%d#%d#%d#%d#%d#%d>",udata.head,udata.value,udata.object,udata.cmd,\
                udata.l,udata.x,udata.y,udata.z,udata.temp,udata.humi,udata.lumi,udata.value,udata.value_1);
                buff[len] = 0;
                Tx_Data(buff,app_id,len);
            }
#endif
            time_200ms = 0;
          }
        }
        if(1 == TIM_500ms)
        {
			if (Time_ns++ >= 2) {
#if ANTHOR == 1
				/**
				 ** 向主机发送红外标识
				 */
				user_infrared_data(TabHL1[OBJECT_THIS],
						TabHL2[OBJECT_THIS]);
#endif

#if TAG == 1
				/**
				 ** 控制从机未连接时显示界面切换
				 */
				if (user_menu.oledmark == 6 && (user_menu.oledmark_count & 0x80)) {		//控制从机未连接时显示界面切换
					user_menu.oledmark = 5;
					user_menu.oledmark_count &= ~0x80;
				}
				/**
				 ** 待机状态下界面切换
				 */
				if((user_menu.oledmark == 7 || user_menu.oledmark == 1) && (user_menu.oledmark_count & 0x40)) {	//待机状态下界面切换
					user_menu.oledmark_count ++;
					if((user_menu.oledmark_count & 0x1f) > 5 && user_menu.oledmark == 7) {
						user_menu.oledmark_count = 0;
						user_menu.oledmark = 1;
					}
					else if((user_menu.oledmark_count & 0x1f) > 5 && user_menu.oledmark == 1) {
						user_menu.oledmark_count = 0;
						user_menu.oledmark = 7;
					}
					user_menu.oledmark_count |= 0x40;
				}
				else if((user_menu.oledmark == 5 ) && (user_menu.oledmark_count & 0x20)) {
					user_menu.oledmark_count ++;			//按下确认建后5s显示手动切换
					if((user_menu.oledmark_count & 0x1f) > 5) {
						user_menu.oledmark_count = 0;
						user_menu.oledmark = 1;
						user_menu.oledmark_count |= 0x40;
					}
				}
				/**
				 ** 获取当前连接个数
				 */
				user_menu.link_num_mark &= ~0xf0;			//获取当前连接个数
				for(i = 0;i < MEMBER;i ++)
				if(u_control[i].object != 0xff)user_menu.link_num_mark += 16;

				/**
				 ** 每10s检测从机连接数量
				 */
				user_menu.link_num_mark ++;//10s内无数据则清空
				if((user_menu.link_num_mark & 0x0f) > 10) {
					app_id = 0xff;
					user_menu.link_num_mark &= ~0xff;
//					for(i = 0;i < MEMBER;i ++)
					for(i = 0;i < 5;i ++)
					u_control[i].object = 0xff;
				}

#endif
				Time_ns = 0;
			}
			TIM_500ms = 0;
		}
    	}while(0);
    }
}




/**
  * @}
  */
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
