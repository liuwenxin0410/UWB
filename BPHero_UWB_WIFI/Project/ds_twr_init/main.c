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

#define  CONTROL_KEY			1  		//1:�����л������ƴӻ�	0:ͨ�����ⶨλ�ӻ�������

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
    int lumi = 0;//����ǿ��
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
          /**����WIFI����***/
          /**********************/
            if(USART3_RX_STA&0X8000)//���յ��ڴ���Ӧ����
            {//�õ���Ч����
#if TAG == 1
				uint8_t b[10] = {0,0,};
				memcpy(b,strstr((char*)USART3_RX_BUF,(char*)"+IPD,"),10);
				uint8_t obj = 0xff;
				if(b[5] != 0)obj = chartonumber(b[5]);
#endif
	            printf("<%s>[%d]buff:%s\r\n",__func__,__LINE__,USART3_RX_BUF);//�����ã��鿴���յ���ԭʼ����
//              uint8_t Len = 0;
//              Len = sprintf(buff,"hello!");
//              if(Wifi_UDP == (enum ATK8266_UDP)UDP_NUll)
//              UPD_Choose ("OK",sizeof("OK"));
//              Tx_Data(buff,Wifi_UDP,Len);

              /*
              **���ڽ���WIFI���ݽ���@{
              */
	          char* str_f = NULL;
	          str_f = strstr((char*)USART3_RX_BUF,(char*)"#");
	          if(*str_f != '#'){		//�������wifiģ�鷵����Ϣ����������ѭ��
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
              **���ڽ������ݽ���@}
              */
              print_data(rx_udata);		//��ӡ��������
#if ANTHOR == 1
 
						/*
            **�ӻ����տ����߼�@{
            */
            udata.cmd = rx_udata->cmd;				//���浱ǰ��������Ͳ���
            udata.value = rx_udata->value;
            printf("<%s>[%d]udata.cmd:0x%02x;\r\n",__func__,__LINE__,udata.cmd);

            if(rx_udata->cmd & 0x80)
            {
              user_menu.funcstaus = 1;
              if((rx_udata->cmd & 0x81) == 0x81){
#if OBJECT_THIS == OBJECT_2
            	  user_set_PWM(((rx_udata->value > 0) ? 0 + rx_udata->value * 90 - 1:0 ));    //����PWM
#elif OBJECT_THIS == OBJECT_1
            	  user_set_PWM(((rx_udata->value > 0) ? 499 + rx_udata->value * 30:0 ));    //����PWM
#elif OBJECT_THIS == OBJECT_3
            	  user_set_PWM(((rx_udata->value > 0) ? 0 + rx_udata->value * 90 - 1:0));    //����PWM
#endif
              }else if(((rx_udata->cmd & 0x40) || (rx_udata->cmd & 0x02)) && (cmd_before != rx_udata->cmd)){

					if(((rx_udata->cmd & 0x08) == 0x08 || (rx_udata->cmd & 0x04) == 0x04 || (rx_udata->cmd & 0x02) == 0x02) && \
							((rx_udata->cmd & (0x02 | 0x04 | 0x08)) != (cmd_before & (0x02 | 0x04 | 0x08)))){
						write_pin_time(45);    //���Ƽ�ʪ
					}else if(((rx_udata->cmd & 0x20) == 0x20 || (rx_udata->cmd & 0x10) == 0x10 || (rx_udata->cmd & 0x40) == 0x40) && \
							((rx_udata->cmd & (0x02 | 0x04 | 0x08)) != (cmd_before & (0x20 | 0x10 | 0x40)))){
					  write_pin_time(3500);    //���Ƶ�
					}
					/*else if(((rx_udata->cmd & 0x20) == 0x20 || (rx_udata->cmd & 0x10) == 0x10 || (rx_udata->cmd & 0x40) == 0x40) && \
							((rx_udata->cmd & (0x02 | 0x04 | 0x08)) != (cmd_before & (0x20 | 0x10 | 0x40)))){
					  write_pin_time(3500);    //��������
					}*/
					cmd_before = rx_udata->cmd;
				}
            }

//            SEGGER_RTT_printf(0, "<%s>[%d]rx_udata->cmd:0x%02x\r\nudata.value:%d\r\n\r\n",__func__,__LINE__,rx_udata->cmd,udata.value);
            
            if ((rx_udata->cmd & (0x80 | 0x40)) == 0 && user_menu.funcstaus == 1)    //�˳�����״̬
            {
              user_menu.funcstaus = 0;
            }
            
            /*
            **�ӻ����տ����߼�@}
            */
#endif   
#if TAG == 1
		  /*
		  **������������
		  */
            if(app_id != 0xff){
            	len = 0;
            	len += sprintf(buff, "%s",rx_buff);
            	buff[len] = 0;
                Tx_Data(buff,app_id,len);
            }

				rx_udata->object = obj;     //���ִӻ��ͻ���,����ͬ�ӻ����ݷֱ���벻ͬ���ݿ���
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
        		write_pin_handle();			//IO�ڶ�ʱ��ƽ�͵�ƽ�ص�����
        		time_5ms = 0;
        	}
#endif
            TIM_1ms = 0;
        }
        if(1 == TIM_10ms)
        {
          TIM_10ms = 0;
#if TAG == 1
          user_menu.keymark = exit_key_get(1);    //��ȡ����ֵ
          if(user_menu.keymark != 0)
          {
        	  if(user_menu.keymark == KEY2_DOWN ){						//�¼�
        		  if( user_menu.oledmark == 2){		//���ƹ�������
					  if(user_menu.menustaus == 2){			//��ʪ����Ҫѭ������
						  udata.cmd |= user_menu.oledcmd;
						  udata.cmd |= 0x02;
						  if((udata.cmd & 0x08) == 0x08){		//08������ʪ
							  udata.cmd &=~ 0x08;
							  udata.cmd |= 0x04;
						  }
						  else if((udata.cmd & 0x04) == 0x04){		//04�����Ъ��ʪ
							  udata.cmd &=~ 0x04;
							  udata.cmd |= 0x00;
						  }
						  else if((udata.cmd & 0x0C) == 0x00){		//00����ر�
								  udata.cmd |= 0x08;
						  }
						  user_menu.oledcmd = udata.cmd;		//��ʾ�߼���,ͬʱ��ʾ�ƺͼ�ʪ��״̬
						  udata.cmd &=~ 0x70;					//ͬһʱ��ֻ�ܷ��Ϳ���һ��ѡ��
						  SEGGER_RTT_printf(0, "<%s>[%d]udata.cmd:0x%02x\r\n",__func__,__LINE__,udata.cmd);
					  }
					  udata.value -= 1;
					  if(udata.value <= 0)udata.value = 0 ;
        		  }else if(user_menu.oledmark == 5){		//�ֶ��л��ӻ�����
        			  user_menu.menustaus ++;
        			  if(user_menu.menustaus > 4)user_menu.menustaus = 0;
        			  if(user_menu.oledmark_count & 0x20)user_menu.oledmark_count &= ~0x1f;
//                      SEGGER_RTT_printf(0, "<%s>[%d]user_menu.menustaus:%d\r\n",__func__,__LINE__,user_menu.menustaus);
        		  }

			  }else if(user_menu.keymark == KEY3_UP ){						//�ϼ�
        		  if( user_menu.oledmark == 2){
					  if(user_menu.menustaus == 2){			//��ʪ����Ҫѭ������
						  udata.cmd |= user_menu.oledcmd;
						  udata.cmd |= 0x40;
						  if((udata.cmd & 0x30) == 0x00){		//00��������ر�
							  udata.cmd |= 0x20;
						  }
						  else if((udata.cmd & 0x20) == 0x20){		//20����׵�����
							  udata.cmd &=~ 0x20;
							  udata.cmd |= 0x10;
						  }
						  else if((udata.cmd & 0x10) == 0x10){		//10������ɫѭ��
								  udata.cmd &= ~0x10;
						  }

						  user_menu.oledcmd = udata.cmd;		//��ʾ�߼���,ͬʱ��ʾ�ƺͼ�ʪ��״̬
						  udata.cmd &=~ 0x0f;					//ͬһʱ��ֻ�ܷ��Ϳ���һ��ѡ��
						  SEGGER_RTT_printf(0, "<%s>[%d]udata.cmd:0x%02x\r\n",__func__,__LINE__,udata.cmd);
					  }
					  udata.value += 1;
					  if(udata.value > 10)udata.value = 10 ;
        		  }else if(user_menu.oledmark == 5){		//�ֶ��л�����///////////////////////////////////
        			  user_menu.menustaus --;
        			  if(user_menu.menustaus < 0 || user_menu.menustaus > 3)user_menu.menustaus = 3;
        			  if(user_menu.oledmark_count & 0x20)user_menu.oledmark_count &= ~0x1f;
        		  }
			  }else if(user_menu.keymark == KEY0_PLAY){						//�˳�����
                  if(user_menu.oledmark > 1){
    	              udata.cmd = 0x00;
    	              udata.value = 0;
                	  user_menu.funcstaus = 2;
                	  user_menu.oledmark = 1;
            		  user_menu.oledmark_count = 0;
            		  user_menu.oledmark_count |= 0x40;		//�������
                  }
			  }else if(user_menu.keymark == KEY1_SET){		//ȷ�ϰ���
                  if(user_menu.oledmark == 1 || user_menu.oledmark == 7){
                	  if(user_menu.funcstaus == 0){		//����״̬�µ��ȷ�ϣ������ֶ�ѡ����ƽ���
                		  user_menu.oledmark = 5;
                		  user_menu.oledmark_count = 0;
                		  user_menu.oledmark_count |= 0x20;		//�����ֶ�ѡ��
                	  }
                  }
                  else if(user_menu.oledmark == 2){		//�������л��˵���ʾ����
//    	              udata.cmd = 0x40;
                	  user_menu.oledmark = 3;
                  }
                  else if(user_menu.oledmark == 3){     //�������л��˵���ʾ����
                	  user_menu.oledmark = 2;
                  }
                  else if(user_menu.oledmark == 5){			//�ֶ��л�����
                	  if((u_control[user_menu.menustaus].object != 0xff && user_menu.funcstaus == 0) || 0){	//�жϵ�ǰѡ��Ĵӻ��Ƿ���Կ���

						  if(user_menu.menustaus == 2){		//����״̬�·��Ϳ��������ʾ��������
							  udata.cmd = 0x80;	//Ĭ�Ϲر�
						  }
						  else {
							  udata.cmd = 0x81;
							  udata.value = 10;
						  }

						  user_menu.oledmark = 2;
						  user_menu.funcstaus = 1;
                	  }else if(u_control[user_menu.menustaus].object == 0xff){		//��ǰ�����豸û������
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

              //��������
          }
#endif
        }
        if(1 == TIM_20ms)
        {
#if ANTHOR == 1 && OBJECT_THIS == OBJECT_2
			/**
			 ** ��ȡ����ֵ��ֻ��̨�ƴӻ�
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
        	 ** ��ȡ��������
        	 */
            drv_lis2dh12_get_angle(&get);
            udata.x = (uint16_t)(get.acc_x * 1) / 1;
            udata.y = (uint16_t)(get.acc_y * 1) / 1;
            udata.z = (uint16_t)(get.acc_z * 1) / 1;
            TIM_50ms = 0;
			/**
			 **��ȡ��ʪ��
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
          if(user_menu.oledmark == 0){		//�ӿ��������л�������ָʾ��ʾ����

        	  user_menu.oledmark = 1;
        	  user_menu.oledmark_count |= 0x40;
          }

          if(time_200ms++ > 0){		//200ms
#if TAG == 1
            /*
            **���������߼�@{
            */
#if	CONTROL_KEY == 1

        	  /**
        	   **  ��ȡ����ֵ�жϴӻ�
        	   */
		  uint8_t infrared_vl = exit_infrared_get();
		  printf("<%s>[%d]infrared_vl:0x%02x\r\n\r\n",__func__,__LINE__,infrared_vl);

            uint8_t infrared_mak = exit_get();

            if(infrared_vl == 0x73){			//�ӻ���ʶ��ʵ��ֵ
            	infrared_mak = OBJECT_AMRK_1;
            }else if(infrared_vl == 0x50){		//�ӻ���ʶ��ʵ��ֵ
            	infrared_mak = OBJECT_AMRK_2;
            }else if(infrared_vl == 0x7b){		//�ӻ���ʶ��ʵ��ֵ
            	infrared_mak = OBJECT_AMRK_3;
            }else if(infrared_vl == TabHL1[4]){
            	infrared_mak = OBJECT_AMRK_4;//�Լ���
            }else{
            	infrared_mak = 0;
            }

			if (infrared_mak != 0 && user_menu.funcstaus == 0) {
				printf("<%s>[%d]infrared_mak:%c\r\n", __func__,
						__LINE__, infrared_mak);
				data_num = sizeof(user_data_t) / sizeof(uint8_t);//�õ��ṹ�峤�ȣ��������
				for (i = 0; i < MEMBER; i++) {
					if (u_control[i].head == infrared_mak) {
						memcpy((void*) &udata, (void*) &u_control[i],//�ѵ�ǰ�ӻ������ݿ⿽������ǰʹ�õĻ���
								data_num);
						udata.head = OBJECT_AMRK;
						if (udata.l < LIM_DIST
								&& udata.object != 0xff) {//����С����wifi������
							user_menu.oledmark = 2;
							user_menu.oledmark_count &= ~0x40;
							user_menu.funcstaus = 1;
							user_menu.menustaus = i;
							udata.value = 10;
							udata.cmd = 0x81;
							if (user_menu.menustaus == 2) {	//����״̬�·��Ϳ��������ʾ��������
								udata.cmd |= 0x80;
							}
						}
					}
				}
			}
#else

#endif
            /*
            **���������߼�@}
            */
            menu_olde_show();		//ˢ����ʾ����
            OLED_DrawMem();			//������Ļ
#endif

#if ANTHOR == 1
          udata.head = OBJECT_AMRK;
          len = 0;    //�������ݽṹ
          len += sprintf(buff,"#%d#%d#%d#%d#%d#%d#%d#%d#%d#%d#%d#%d#%d#%d>",udata.head,udata.value,udata.object,udata.cmd,\
          udata.l,udata.x,udata.y,udata.z,udata.temp,udata.humi,udata.lumi,udata.volume,udata.value,udata.value_1);

          buff[len] = 0;
          Tx_Data(buff,0,len);

#endif

#if TAG == 1
          /**
           ** �ڿ��ƹ�������ӻ����Ϳ�������
           */
            if(user_menu.funcstaus != 0 ){
              udata.head = OBJECT_AMRK;
              if(user_menu.funcstaus == 2){
				  user_menu.funcstaus = 0 ;		//pwmΪ0�����������źţ��˳���������
				  udata.cmd = 0x00;
				  udata.value = 0;
				  user_menu.oledmark = 1;
				  user_menu.oledmark_count |= 0x40;
              }else if(user_menu.funcstaus == 1){
            	  udata.cmd |= 0x80;
              }

              len = 0;  //�������ݽṹ
              len += sprintf(buff,"#%d#%d#%d#%d#%d#%d#%d#%d#%d#%d#%d#%d#%d>",udata.head,udata.value,udata.object,udata.cmd,\
              udata.l,udata.x,udata.y,udata.z,udata.temp,udata.humi,udata.lumi,udata.value,udata.value_1);

              buff[len] = 0;
              Tx_Data(buff,udata.object,len);
            }
            /**
             **������app����
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
				 ** ���������ͺ����ʶ
				 */
				user_infrared_data(TabHL1[OBJECT_THIS],
						TabHL2[OBJECT_THIS]);
#endif

#if TAG == 1
				/**
				 ** ���ƴӻ�δ����ʱ��ʾ�����л�
				 */
				if (user_menu.oledmark == 6 && (user_menu.oledmark_count & 0x80)) {		//���ƴӻ�δ����ʱ��ʾ�����л�
					user_menu.oledmark = 5;
					user_menu.oledmark_count &= ~0x80;
				}
				/**
				 ** ����״̬�½����л�
				 */
				if((user_menu.oledmark == 7 || user_menu.oledmark == 1) && (user_menu.oledmark_count & 0x40)) {	//����״̬�½����л�
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
					user_menu.oledmark_count ++;			//����ȷ�Ͻ���5s��ʾ�ֶ��л�
					if((user_menu.oledmark_count & 0x1f) > 5) {
						user_menu.oledmark_count = 0;
						user_menu.oledmark = 1;
						user_menu.oledmark_count |= 0x40;
					}
				}
				/**
				 ** ��ȡ��ǰ���Ӹ���
				 */
				user_menu.link_num_mark &= ~0xf0;			//��ȡ��ǰ���Ӹ���
				for(i = 0;i < MEMBER;i ++)
				if(u_control[i].object != 0xff)user_menu.link_num_mark += 16;

				/**
				 ** ÿ10s���ӻ���������
				 */
				user_menu.link_num_mark ++;//10s�������������
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
