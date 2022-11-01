#include "waificmd.h"
#include "lcd_oled.h"
#include <stdio.h>

#if(ANTHOR == 1)
const u8 MODE[] = "AT+CWMODE=1";
const u8 SERVER[] = "AT+CIPSERVER=0";
const u8 JAP[] = "AT+CWJAP=\"Yun\",\"12345678\"";
const u8 MUX[] = "AT+CIPMUX=0";
const u8 START[] = "AT+CIPSTART=\"TCP\",\"192.168.4.1\",8060";
const u8 mode[] = "AT+CIPMODE=0";
#endif

#if(TAG == 1)
//WIFI AP模式,模块对外的无线参数,可自行修改.
const u8 MODE[] = "AT+CWMODE=2";	
const u8 SAP[] = "AT+CWSAP=\"Yun\",\"12345678\",1,4";	
const u8 MUX[] = "AT+CIPMUX=1"; 
const u8 SERVER[] = "AT+CIPSERVER=1,8060";
#endif
extern __IO UDP Wifi_UDP = (enum ATK8266_UDP)UDP0;

/******************************************/
//函数名:
//功  能:
//输入值:
//返回值:
//日  期:
/*******************************************/
void RTK8266_init (void)
{
#if(TAG == 1)
  (atk_8266_send_cmd(MODE,"OK",100));
	delay_ms(100);
	(atk_8266_send_cmd(SAP,"OK",100));	
	(atk_8266_send_cmd("AT+RST","OK",200));
	delay_ms(1000);
	(atk_8266_send_cmd(MUX,"OK",200));
	delay_ms(100);
	(atk_8266_send_cmd(SERVER,"OK",200));
	delay_ms(100);
#endif
  
#if(ANTHOR == 1)
	delay_ms(1000);
	(atk_8266_send_cmd("AT","OK",80));
	(atk_8266_send_cmd(MODE,"OK",100));
	(atk_8266_send_cmd("AT+RST","OK",200));
	delay_ms(100);
	(atk_8266_send_cmd(JAP,"WIFI GOT IP",1500));
	delay_ms(80);
	(atk_8266_send_cmd(MUX,"OK",200));
	delay_ms(10);
	(atk_8266_send_cmd(START,"OK",1500));
	delay_ms(100);
	(atk_8266_send_cmd(mode,"OK",200));
	delay_ms(100);
#endif
}

uint8_t string (char *a,char *b,uint8_t Lenth )
{
	uint8_t i;
  for(i = 0;i < Lenth;i ++)
	{
		if (a[i] != b[i])
		{
			return 0;
		}
	}
	return 1;
}

/******************************************/
//函数名:
//功  能:选择客户端序号
//输入值:
//返回值:
//日  期:2017-09-02
/*******************************************/
void UPD_Choose (uint8_t* cmd,uint8_t Lenth)
{
  uint8_t b[10];
	uint8_t c[Lenth];
	uint8_t data[Lenth]; 
	sprintf((char*)data,":%s",cmd);   
	memcpy(b,strstr((char*)USART3_RX_BUF,(char*)"+IPD,"),10);
	memcpy(c,strstr((char*)USART3_RX_BUF,(char*)":"),Lenth);
	if (string ((char*)c,(char*)data,Lenth))
	{
		Wifi_UDP = (enum ATK8266_UDP)chartonumber(b[5]);
	}
}


/******************************************/
//函数名:
//功  能:Flag为0时比较字符串，其它为发送数据
//输入值:
//返回值:
//日  期:2017-08-31
/*******************************************/
uint8_t string_chenk (uint8_t *cmd,uint8_t *ack,uint8_t Lenth,uint8_t Flag)
{
	uint8_t b[Lenth-1];
	Lenth -= 1;
	if(!Flag)
	{
		memcpy(b,strstr((char*)cmd,(char*)ack),Lenth);
		if(string ((char*)b,(char*)ack,Lenth))
			return 1;
		else 
			return 0;
  }
	else
	{
		memcpy(b,strstr((char*)USART3_RX_BUF,(char*)"+IPD,"),sizeof("+IPD,"));
		Tx_Data ((char*)cmd,chartonumber(b[5]),Lenth + 1);
		return 1;
	}
}



/******************************************/
//函数名:
//功  能:发送指定长度数据
//输入值:
//返回值:
//日  期:
/*******************************************/
void Tx_Data (char* data,u8 flag,u8 Lengh)
{
	u8 a[20];
#if(TAG == 1)
	if(flag != 'N')
	{
		sprintf((char*)a,"AT+CIPSEND=%d,%d",flag,Lengh);
		atk_8266_send_cmd((u8*)a,"OK",10);
//		u3_printf("%s\r\n",a);	//发送命令
		atk_8266_send_data((u8*)data,"OK",10);  //发送指定长度的数据
	}
#endif
  
#if(ANTHOR == 1)

		sprintf((char*)a,"AT+CIPSEND=%d",Lengh);
		atk_8266_send_cmd((u8*)a,"OK",10);
//		u3_printf("%s\r\n",a);	//发送命令
		atk_8266_send_data((u8*)data,"OK",10);  //发送指定长度的数据
#endif
}

//ATK-ESP8266发送命令后,检测接收到的应答
//str:期待的应答结果
//返回值:0,没有得到期待的应答结果
//    其他,期待应答结果的位置(str的位置)
u8* atk_8266_check_cmd(u8 *str)
{

    char *strx=0;
    if(USART3_RX_STA&0X8000)		//接收到一次数据了
    {
        USART3_RX_BUF[USART3_RX_STA&0X7FFF]=0;//添加结束符
        strx=strstr((const char*)USART3_RX_BUF,(const char*)str);
//        printf("<%s>[%d]USART3_RX_BUF:%s\r\n",__func__,__LINE__,USART3_RX_BUF);
//        printf("<%s>[%d]str:%s\r\n\r\n",__func__,__LINE__,strx);
    }
    return (u8*)strx;
}

//向ATK-ESP8266发送命令
//cmd:发送的命令字符串
//ack:期待的应答结果,如果为空,则表示不需要等待应答
//waittime:等待时间(单位:10ms)
//返回值:0,发送成功(得到了期待的应答结果)
//       1,发送失败
u8 atk_8266_send_cmd(const u8 *cmd,u8 *ack,u16 waittime)
{
    u8 res=0;
    USART3_RX_STA=0;
    u3_printf("%s\r\n",cmd);	//发送命令
    if(ack&&waittime)		//需要等待应答
    {
        while(--waittime)	//等待倒计时
        {
            delay_ms(10);
            if(USART3_RX_STA&0X8000)//接收到期待的应答结果
            {
                if(atk_8266_check_cmd(ack))
                {
                    break;//得到有效数据
                }
                USART3_RX_STA=0;
            }
        }
        if(waittime==0)res=1;
    }
    return res;
}
//向ATK-ESP8266发送指定数据
//data:发送的数据(不需要添加回车了)
//ack:期待的应答结果,如果为空,则表示不需要等待应答
//waittime:等待时间(单位:10ms)
//返回值:0,发送成功(得到了期待的应答结果)luojian
u8 atk_8266_send_data(u8 *data,u8 *ack,u16 waittime)
{
    u8 res=0;
    USART3_RX_STA=0;
    u3_printf("%s",data);	//发送命令
    if(ack&&waittime)		//需要等待应答
    {
        while(--waittime)	//等待倒计时
        {
            delay_ms(10);
            if(USART3_RX_STA&0X8000)//接收到期待的应答结果
            {
                if(atk_8266_check_cmd(ack))break;//得到有效数据
                USART3_RX_STA=0;
            }
        }
        if(waittime==0)res=1;
    }
    return res;
}

//获取ATK-ESP8266模块的连接状态
//返回值:0,未连接;1,连接成功.
u8 atk_8266_consta_check(void)
{
    u8 *p;
    u8 res;
//	if(atk_8266_quit_trans())return 0;			//退出透传
    atk_8266_send_cmd("AT+CIPSTATUS",":",50);	//发送AT+CIPSTATUS指令,查询连接状态
    p=atk_8266_check_cmd("+CIPSTATUS:");
    res=*p;									//得到连接状态
    return res;
}
//获取ATK-ESP8266模块的AP+STA连接状态
//返回值:0，未连接;1,连接成功
u8 atk_8266_apsta_check(void)
{
//	if(atk_8266_quit_trans())return 0;			//退出透传
    atk_8266_send_cmd("AT+CIPSTATUS",":",50);	//发送AT+CIPSTATUS指令,查询连接状态
    if(atk_8266_check_cmd("+CIPSTATUS:0")&&
            atk_8266_check_cmd("+CIPSTATUS:1")&&
            atk_8266_check_cmd("+CIPSTATUS:2")&&
            atk_8266_check_cmd("+CIPSTATUS:4"))
        return 0;
    else return 1;
}
