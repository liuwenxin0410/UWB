#ifndef __OLED_H
#define __OLED_H
#include "stm32f10x.h"
//#include "sys.h"
#include "stdint.h"
#include "stdlib.h"

#define DE_DELAY 0

#define IIC_TIMEOUT 100

#define delay_ms(x) deca_sleep(x)
//OLED模式设置
//0:4线串行模式
//1:并行8080模式
#define OLED_MODE 0
#define SIZE 16
#define XLevelL		0x00
#define XLevelH		0x10
#define Max_Column	128
#define Max_Row		64
#define	Brightness	0xFF
#define X_WIDTH 	128
#define Y_WIDTH 	64

//-----------------测试LED端口定义----------------
//#define LED_ON GPIO_ResetBits(GPIOD,GPIO_Pin_2)
//#define LED_OFF GPIO_SetBits(GPIOD,GPIO_Pin_2)

//-----------------OLED端口定义----------------


#define OLED_SCLK_Clr() GPIO_ResetBits(GPIOB,GPIO_Pin_13)//CLK
#define OLED_SCLK_Set() GPIO_SetBits(GPIOB,GPIO_Pin_13)

#define OLED_SDIN_Clr() GPIO_ResetBits(GPIOB,GPIO_Pin_15)//DIN
#define OLED_SDIN_Set() GPIO_SetBits(GPIOB,GPIO_Pin_15)

#define OLED_RST_Clr() GPIO_ResetBits(GPIOB,GPIO_Pin_11)//RES
#define OLED_RST_Set() GPIO_SetBits(GPIOB,GPIO_Pin_11)

#define OLED_DC_Clr() GPIO_ResetBits(GPIOB,GPIO_Pin_10)//DC
#define OLED_DC_Set() GPIO_SetBits(GPIOB,GPIO_Pin_10)

#define OLED_CS_Clr()  GPIO_ResetBits(GPIOB,GPIO_Pin_12)//CS
#define OLED_CS_Set()  GPIO_SetBits(GPIOB,GPIO_Pin_12)

#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据



typedef struct
{
    float acc_x;
    float acc_y;
    float acc_z;
    float angle;
} drv_lis2dh12_def_t;

typedef drv_lis2dh12_def_t drv_lis2dh12_t;

//OLED控制用函数
void SHT20_init(void);
int32_t drv_lis2dh12_init(void);

int drv_lis2dh12_get_angle( drv_lis2dh12_t *get);
int SHT20_read_tem(float *p_temperature);
int SHT20_read_hum(float *p_humidity);

uint8_t I2C_Write(I2C_TypeDef *I2Cx,uint8_t I2C_Addr,uint8_t addr,uint8_t *buf,uint16_t num);
uint8_t I2C_ReadOneByte(I2C_TypeDef* I2Cx, unsigned char AddressDevice, unsigned char AddressByte);
 
void OLED_WR_Byte(uint8_t dat,uint8_t cmd);
void OLED_Display_On(void);
void OLED_Display_Off(void);
void oled_init(void);
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t t);
void OLED_Fill(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2,uint8_t dot);
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size);
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size);
void OLED_ShowString(uint8_t x,uint8_t y, uint8_t *p);
uint8_t OLED_ShowCH(uint8_t x,uint8_t y,uint8_t *cn);
void OLED_Set_Pos(unsigned char x, unsigned char y);
void OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t no);
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[]);

void OLED_DrawMem(void);
void oled_clear(uint8_t x,uint8_t y,uint8_t x0,uint8_t y0);
#endif




