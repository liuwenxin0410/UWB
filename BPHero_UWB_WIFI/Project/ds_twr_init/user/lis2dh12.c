#include "lis2dh12.h"
#include "lis2dh12_reg.h"
#include "lcd_oled.h"
#include "stdlib.h"
#include "stdio.h"
#include <string.h>
#include <math.h>

#include "SEGGER_RTT.h"

#define LIS2DH12_FROM_FS_2g_HR_TO_mg(lsb)  (float)((int16_t)lsb>>4) * 1.0f
#define LIS2DH12_FROM_FS_4g_HR_TO_mg(lsb)  (float)((int16_t)lsb>>4) * 2.0f
#define LIS2DH12_FROM_FS_8g_HR_TO_mg(lsb)  (float)((int16_t)lsb>>4) * 4.0f
#define LIS2DH12_FROM_FS_16g_HR_TO_mg(lsb) (float)((int16_t)lsb>>4) * 12.0f 
#define LIS2DH12_FROM_LSB_TO_degC_HR(lsb)  (float)((int16_t)lsb>>6) / 4.0f+25.0f

 
axis_info_t acc_sample;
filter_avg_t acc_data;
extern uint8_t I2C_WriteOneByte(I2C_TypeDef *I2Cx,uint8_t I2C_Addr,uint8_t addr,uint8_t value);

/* iic 通讯相关操作 */
int32_t drv_lis2dh12_iic_write_byte(uint8_t addr, uint8_t data)
{
    I2C_WriteOneByte(I2C1,0x32,addr,data);
    return 0;

}
int32_t drv_lis2dh12_iic_read_byte(uint8_t AddressByte, uint8_t* data)
{
    uint8_t ReceiveData = 0;
    I2C_TypeDef *I2Cx=I2C1;
    uint8_t AddressDevice=0x32;
    //__disable_irq();
#if DE_DELAY
    while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
    I2C_GenerateSTART(I2Cx, ENABLE);
    while( !I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB) ); // wait Generate Start

    I2C_Send7bitAddress(I2Cx, AddressDevice, I2C_Direction_Transmitter);
    while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) ); // wait send Address Device

    I2C_SendData(I2Cx, AddressByte);
    while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED) ); // wait send Address Byte

    I2C_GenerateSTART(I2Cx, ENABLE);
    while( !I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB) ); // wait Generate Start

    I2C_Send7bitAddress(I2Cx, AddressDevice, I2C_Direction_Receiver);
    while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) ); // wait Send Address Device As Receiver

    while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) ); // wait Receive a Byte
    ReceiveData = I2C_ReceiveData(I2Cx);

    I2C_NACKPositionConfig(I2Cx, I2C_NACKPosition_Current); // send not acknowledge
    I2C_AcknowledgeConfig(I2Cx, DISABLE);// disable acknowledge

    I2C_GenerateSTOP(I2Cx, ENABLE);
    while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF)); // wait Generate Stop Condition
#else
    int timeout = 0;
    timeout = IIC_TIMEOUT;
    while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY)&&(timeout --));
    I2C_GenerateSTART(I2Cx, ENABLE);
    timeout = IIC_TIMEOUT;
    while( !I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB)&&(timeout --) ); // wait Generate Start

    I2C_Send7bitAddress(I2Cx, AddressDevice, I2C_Direction_Transmitter);
    timeout = IIC_TIMEOUT;
    while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) &&(timeout --)); // wait send Address Device

    I2C_SendData(I2Cx, AddressByte);
    timeout = IIC_TIMEOUT;
    while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)&&(timeout --) ); // wait send Address Byte

    I2C_GenerateSTART(I2Cx, ENABLE);
    timeout = IIC_TIMEOUT;
    while( !I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB)&&(timeout --) ); // wait Generate Start

    I2C_Send7bitAddress(I2Cx, AddressDevice, I2C_Direction_Receiver);
    timeout = IIC_TIMEOUT;
    while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) &&(timeout --)); // wait Send Address Device As Receiver

    timeout = IIC_TIMEOUT;
    while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) &&(timeout --)); // wait Receive a Byte
    ReceiveData = I2C_ReceiveData(I2Cx);

    I2C_NACKPositionConfig(I2Cx, I2C_NACKPosition_Current); // send not acknowledge
    I2C_AcknowledgeConfig(I2Cx, DISABLE);// disable acknowledge

    I2C_GenerateSTOP(I2Cx, ENABLE);
    timeout = IIC_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF)&&(timeout --)); // wait Generate Stop Condition
#endif
    I2C_AcknowledgeConfig(I2Cx, DISABLE);// disable acknowledge
    __enable_irq();
    *data=ReceiveData;
    //return ReceiveData;

    //*data=I2C_ReadOneByte(I2C1,0x32,addr);
    return 0;
}

#define lis2dh12_iic_write_byte drv_lis2dh12_iic_write_byte


/* ============================================================
func name   :  Lis2dh12_Init
discription :  配置检测阈值与中断
param       :  void
return      :  
Revision    : 
=============================================================== */
int32_t Lis2dh12_Init(void)
{	
	/* Initialization of sensor */
	lis2dh12_iic_write_byte(0x20, 0x37);	/* CTRL_REG1(20h): 关闭sensor，设置进入掉电模式 ODR 25HZ */
//	lis2dh12_iic_write_byte(0x20, 0x57);	/* CTRL_REG1(20h): 关闭sensor，设置进入低功耗模式 ODR 100HZ */
	lis2dh12_iic_write_byte(0x21, 0x03);	/* CTRL_REG2(21h): IA1、IA2 开启高通滤波 bc */
	lis2dh12_iic_write_byte(0x22, 0xc0);	/* CTRL_REG3(22h): 0x80 使能单击中断到INT_1 INT_2 */
	lis2dh12_iic_write_byte(0x23, 0x88);  /* CTRL_REG4(23h): 使能快，数据更新，全量程+/-2G，非常精度模式 */
//	lis2dh12_iic_write_byte(0x25, 0x00);  /* CTRL_REG6(25h): 高电平(上升沿)触发中断 */
	
	/* INT1 翻转检测，中断*/							//0x6a
	lis2dh12_iic_write_byte(0x30, 0x7f);  /* INT1_CFG(30h): 使能，6D X/Y/Z任一超过阈值中断 */
//	lis2dh12_iic_write_byte(0x30, 0x4f);  /* INT1_CFG(30h): 使能，6D X/Y任一超过阈值中断 */
//  lis2dh12_iic_write_byte(0x31, 0x20);  /* INT1_SRC(31h): 设置中断源 */
	
//	lis2dh12_iic_write_byte(0x32, 0x0a);  /* INT1_THS(32h): 设置中断阀值 0x10: 16*2(FS)  0x20: 32*16(FS) 平放时 Z轴*/
//	lis2dh12_iic_write_byte(0x32, 0x0a);  /* INT1_THS(32h): 设置中断阀值 0x10: 16*2(FS)  0x20: 32*16(FS) 躺着时 X轴*/
 
	lis2dh12_iic_write_byte(0x32, 0x02);  	/* INT1_THS(32h): 设置中断阀值 0x10: 16*2(FS)  0x20: 32*16(FS) 立着时 Y轴*/
 
//	lis2dh12_iic_write_byte(0x33, 0x02);  	/* INT1_DURATION(33h): 1LSB=1/ODR  如果ODR=25HZ  那么1LSB=40ms 设置延时 1s,对应25->0x19 */
//	lis2dh12_iic_write_byte(0x33, 0x03);  /* INT1_DURATION(33h): 1LSB=1/ODR  如果ODR=50HZ   那么1LSB=20ms 设置延时 1s,对应50->0x32 */
  lis2dh12_iic_write_byte(0x33, 0x03);  	/* INT1_DURATION(33h): 1LSB=1/ODR  如果ODR=100HZ  那么1LSB=10ms 设置延时 1s,对应100->0x64 */
	
	
//	/* INT2 单击中断 */
////	lis2dh12_iic_write_byte(0x24, 0x01);	/* CTRL_REG5(24h):  */
//	lis2dh12_iic_write_byte(0x25, 0xa0);  /* CTRL_REG6(25h): Click interrupt on INT2 pin */
//	
//	lis2dh12_iic_write_byte(0x38, 0x15);	/* CLICK_CFG (38h): 单击识别中断使能 */
////	lis2dh12_iic_write_byte(0x39, 0x10);
//	lis2dh12_iic_write_byte(0x3a, 0x7f);  /* CLICK_THS (3Ah): 单击阀值 */
//	lis2dh12_iic_write_byte(0x3b, 0xff);  /* TIME_LIMIT (3Bh): 时间限制窗口6 ODR 1LSB=1/ODR 1LSB=1/100HZ,10ms,设置延时1s,对应100—>0x64*/
//	lis2dh12_iic_write_byte(0x3c, 0xff);  /* TIME_LATENCY (3Ch): 中断电平持续时间1 ODR=10ms */
//	lis2dh12_iic_write_byte(0x3d, 0x01);  /* TIME_WINDOW (3Dh):  单击时间窗口 */
	
	/* Start sensor */
//	lis2dh12_iic_write_byte(0x20, 0x37);
	lis2dh12_iic_write_byte(0x20, 0x5f);  /* CTRL_REG1(20h): Start sensor at ODR 100Hz Low-power mode */
	
	return 0;
}

/* lis2dh12 init 配置检测阈值与中断 */
int32_t drv_lis2dh12_init(void)
{
    /* Initialization of sensor */
    lis2dh12_iic_write_byte(0x20, 0x37);	/* CTRL_REG1(20h): 关闭sensor，设置进入掉电模式 ODR 25HZ */
//	lis2dh12_iic_write_byte(0x20, 0x57);	/* CTRL_REG1(20h): 关闭sensor，设置进入低功耗模式 ODR 100HZ */
    lis2dh12_iic_write_byte(0x21, 0x03);	/* CTRL_REG2(21h): IA1、IA2 开启高通滤波 bc */
    lis2dh12_iic_write_byte(0x22, 0xc0);	/* CTRL_REG3(22h): 0x80 使能单击中断到INT_1 INT_2 */
    lis2dh12_iic_write_byte(0x23, 0x88);  /* CTRL_REG4(23h): 使能快，数据更新，全量程+/-2G，非常精度模式 */
//	lis2dh12_iic_write_byte(0x25, 0x00);  /* CTRL_REG6(25h): 高电平(上升沿)触发中断 */

    /* INT1 翻转检测，中断*/							//0x6a
    lis2dh12_iic_write_byte(0x30, 0x7f);  /* INT1_CFG(30h): 使能，6D X/Y/Z任一超过阈值中断 */
//	lis2dh12_iic_write_byte(0x30, 0x4f);  /* INT1_CFG(30h): 使能，6D X/Y任一超过阈值中断 */
//  lis2dh12_iic_write_byte(0x31, 0x20);  /* INT1_SRC(31h): 设置中断源 */

    lis2dh12_iic_write_byte(0x32, 0x02);  	/* INT1_THS(32h): 设置中断阀值 0x10: 16*2(FS)  0x20: 32*16(FS) */

//	lis2dh12_iic_write_byte(0x33, 0x02);  	/* INT1_DURATION(33h): 1LSB=1/ODR  如果ODR=25HZ  那么1LSB=40ms 设置延时 1s,对应25->0x19 */
//	lis2dh12_iic_write_byte(0x33, 0x03);  /* INT1_DURATION(33h): 1LSB=1/ODR  如果ODR=50HZ   那么1LSB=20ms 设置延时 1s,对应50->0x32 */
    lis2dh12_iic_write_byte(0x33, 0x03);  	/* INT1_DURATION(33h): 1LSB=1/ODR  如果ODR=100HZ  那么1LSB=10ms 设置延时 1s,对应100->0x64 */


//	/* INT2 单击中断 */
////	lis2dh12_iic_write_byte(0x24, 0x01);	/* CTRL_REG5(24h):  */
//	lis2dh12_iic_write_byte(0x25, 0xa0);  /* CTRL_REG6(25h): Click interrupt on INT2 pin */
//
//	lis2dh12_iic_write_byte(0x38, 0x15);	/* CLICK_CFG (38h): 单击识别中断使能 */
////	lis2dh12_iic_write_byte(0x39, 0x10);
//	lis2dh12_iic_write_byte(0x3a, 0x7f);  /* CLICK_THS (3Ah): 单击阀值 */
//	lis2dh12_iic_write_byte(0x3b, 0xff);  /* TIME_LIMIT (3Bh): 时间限制窗口6 ODR 1LSB=1/ODR 1LSB=1/100HZ,10ms,设置延时1s,对应100—>0x64*/
//	lis2dh12_iic_write_byte(0x3c, 0xff);  /* TIME_LATENCY (3Ch): 中断电平持续时间1 ODR=10ms */
//	lis2dh12_iic_write_byte(0x3d, 0x01);  /* TIME_WINDOW (3Dh):  单击时间窗口 */

    /* Start sensor */
//	lis2dh12_iic_write_byte(0x20, 0x37);
    lis2dh12_iic_write_byte(0x20, 0x5f);  /* CTRL_REG1(20h): Start sensor at ODR 100Hz Low-power mode */



}


int32_t drv_lis2dh12_initqq(void )
{
    /* Initialization of sensor */
    drv_lis2dh12_iic_write_byte(0x21, 0x01); /* CTRL_REG2(21h): Filtered data and High-pass filter selection */


    drv_lis2dh12_iic_write_byte(0x22, 0x40); /* CTRL_REG3(22h): IA1 interrupt on INT1 pin */


    drv_lis2dh12_iic_write_byte(0x23, 0x80); /* CTRL_REG4(23h): Set Full-scale to +/-2g */


    /* Wakeup recognition enable */
    drv_lis2dh12_iic_write_byte(0x30, 0x2a); /* INT1_CFG(30h): INT1 Configuration */

    drv_lis2dh12_iic_write_byte(0x32, 0x36); /* INT1_THS(32h): INT1 Threshold set */


    drv_lis2dh12_iic_write_byte(0x33, 0x0);/* INT1_DURATION(33h): INT1 Duration set */


    /* Start sensor */
    drv_lis2dh12_iic_write_byte(0x20, 0x57);/* CTRL_REG1(20h): Start sensor at ODR 100Hz Low-power mode */


    return 0;
}

/* 获取当前倾斜角度 */
int drv_lis2dh12_get_angle( drv_lis2dh12_t *get)
{
    ;
    float acc_x, acc_y, acc_z, acc_g;
    float angle_x, angle_y, angle_z, angle_xyz;
    int8_t data[6]= {0};
    uint8_t i;

	for (i=0; i<6; i++){
		drv_lis2dh12_iic_read_byte(0x28 + i, data+i);		//获取X、y、z轴的数据
		//printf("data[i] %d, \r\n", data[i]);
	}
    /* x, y, z 轴加速度 */
    acc_x = (LIS2DH12_FROM_FS_2g_HR_TO_mg(*(int16_t*)data));
    acc_y = (LIS2DH12_FROM_FS_2g_HR_TO_mg(*(int16_t*)(data+2)));
    acc_z = (LIS2DH12_FROM_FS_2g_HR_TO_mg(*(int16_t*)(data+4)));

    get->acc_x = acc_x;
    get->acc_y = acc_y;
    get->acc_z = acc_z;
  
    acc_x = abs(acc_x);
    acc_y = abs(acc_y);
    acc_z = abs(acc_z);

    /* 重力加速度 */
    acc_g = sqrt(pow(acc_x, 2) + pow(acc_y, 2) + pow(acc_z, 2));
    //printf("acc_g:%f\r\n",acc_g);

    //return acc_y;
    if (acc_z > acc_g)
        acc_z = acc_g;

    /* angle_z/90 = asin(acc_z/acc_g)/π/2 */
    angle_z = asin(acc_z/acc_g) * 2 / 3.14 * 90;
    angle_z = 90 - angle_z;
    if(angle_z < 0)
        angle_z = 0;

    get->angle = angle_z;
    
//    SEGGER_RTT_printf(0,"acc_x:%5.3f\r\n",get->acc_x);
//    SEGGER_RTT_printf(0,"acc_y:%5.3f\r\n",get->acc_y);
//    SEGGER_RTT_printf(0,"acc_z:%5.3f\r\n",get->acc_z);
//    SEGGER_RTT_printf(0,"angle:%5.3f\r\n",angle_z);
    return 0;
}





/* ============================================================
func name   :  lis2_Delay_us
discription :  lis2 微妙延时
param       :  us
return      :  
Revision    : 
=============================================================== */
void lis2_Delay_us(uint32_t us)
{
	int n = 11;
	while(--us)
	{
	    while(--n);
	    n = 11;
	}
}

/* ============================================================
func name   :  lis2_Delay_ms
discription :  lis2 毫秒延时
param       :  ms
return      :  
Revision    : 
=============================================================== */
void lis2_Delay_ms(uint32_t ms)
{
	int i = 0;
	for(;i<ms;i++)
	{
	    lis2_Delay_us(1000);
	}
}
/* ============================================================
func name   :  get_acc_value
discription :  获取加速度值
param       :  axis_info_t *sample
return      :  void
Revision    : 
=============================================================== */
void get_acc_value(axis_info_t *sample)
{
  short acc_x, acc_y, acc_z, acc_g;
  float angle_z;
	uint8_t i = 0;
	uint8_t data[6];
	for (i=0; i<6; i++){
		drv_lis2dh12_iic_read_byte(0x28 + i, data+i);		//获取X、y、z轴的数据
		//printf("data[i] %d, \r\n", data[i]);
	}
//  sample->x = abs((int)(LIS2DH12_FROM_FS_2g_HR_TO_mg(*(int16_t*)data))); 
//	sample->y = abs((int)(LIS2DH12_FROM_FS_2g_HR_TO_mg(*(int16_t*)(data+2))));
//	sample->z = abs((int)(LIS2DH12_FROM_FS_2g_HR_TO_mg(*(int16_t*)(data+4))));
	acc_x = sample->x = LIS2DH12_FROM_FS_2g_HR_TO_mg(*(int16_t*)data); 
	acc_y = sample->y = LIS2DH12_FROM_FS_2g_HR_TO_mg(*(int16_t*)(data+2));
	acc_z = sample->z = LIS2DH12_FROM_FS_2g_HR_TO_mg(*(int16_t*)(data+4));
  
  
    acc_x = abs(acc_x);
    acc_y = abs(acc_y);
    acc_z = abs(acc_z);

    /* 重力加速度 */
    acc_g = sqrt(pow(acc_x, 2) + pow(acc_y, 2) + pow(acc_z, 2));
    //printf("acc_g:%f\r\n",acc_g);

    //return acc_y;
    if (acc_z > acc_g)
        acc_z = acc_g;

    /* angle_z/90 = asin(acc_z/acc_g)/π/2 */
    angle_z = asin(acc_z/acc_g) * 2 / 3.14 * 90;
    angle_z = 90 - angle_z;
    if(angle_z < 0)
        angle_z = 0;

    sample->angle = angle_z;
}

/* ============================================================
func name   :  filter_calculate
discription :  均值滤波器---滤波
			   读取xyz数据存入均值滤波器，存满进行计算，滤波后样本存入sample
param       :  filter_avg_t *filter, axis_info_t *sample
return      :  void
Revision    : 
=============================================================== */
void filter_calculate(filter_avg_t *filter, axis_info_t *sample)
{
	uint8_t i = 0;
	short x_sum = 0, y_sum = 0, z_sum = 0;	
	
	for (i=0; i<FILTER_CNT; i++) 
	{
		get_acc_value(sample);

		filter->info[i].x = sample->x;
		filter->info[i].y = sample->y;
		filter->info[i].z = sample->z;
		
		x_sum += filter->info[i].x;
		y_sum += filter->info[i].y;
		z_sum += filter->info[i].z;
		
//		printf("acc_x:%d,  acc_y:%d,  acc_z:%d \n",filter->info[i].x,filter->info[i].y,filter->info[i].z);
	}
	sample->x = x_sum / FILTER_CNT;
	sample->y = y_sum / FILTER_CNT;
	sample->z = z_sum / FILTER_CNT;	
	printf("\r\n acc_info.acc_x:%d, acc_info.acc_y:%d, acc_info.acc_z:%d \r\n",sample->x, sample->y, sample->z);
  SEGGER_RTT_printf(0,"\r\n acc_info.acc_x:%d, acc_info.acc_y:%d, acc_info.acc_z:%d \r\n",sample->x, sample->y, sample->z);
}
 
 
/* ============================================================
func name   :  new_angle_calculate
discription :  计算新角度
param       :  axis_info_t *sample
return      :  void
Revision    : 
=============================================================== */
void new_angle_calculate(axis_info_t *sample)
{
	sample->new_angle_x = atan((short)sample->x/(short)sqrt(pow(sample->y, 2)+pow(sample->z, 2))) * DEGREE_CAL;
	sample->new_angle_y = atan((short)sample->y/(short)sqrt(pow(sample->x, 2)+pow(sample->z, 2))) * DEGREE_CAL;
	sample->new_angle_z = atan((short)sample->z/(short)sqrt(pow(sample->x, 2)+pow(sample->y, 2))) * DEGREE_CAL;
	if (sample->new_angle_z < 0)
	{
		sample->new_angle_x = 180-sample->new_angle_x;
		sample->new_angle_y = 180-sample->new_angle_y;
	}
	printf("sample->new_angle_x:%d, sample->new_angle_y:%d, sample->new_angle_z:%d \r\n",sample->new_angle_x, sample->new_angle_y, sample->new_angle_z);
SEGGER_RTT_printf(0,"sample->new_angle_x:%d, sample->new_angle_y:%d, sample->new_angle_z:%d \r\n",sample->new_angle_x, sample->new_angle_y, sample->new_angle_z);
}

/* ============================================================
func name   :  old_angle_calculate
discription :  计算旧角度
param       :  axis_info_t *sample
return      :  void
Revision    : 
=============================================================== */
void old_angle_calculate(axis_info_t *sample)
{
	sample->old_angle_x = atan((short)sample->x/(short)sqrt(pow(sample->y, 2)+pow(sample->z, 2))) * DEGREE_CAL;
	sample->old_angle_y = atan((short)sample->y/(short)sqrt(pow(sample->x, 2)+pow(sample->z, 2))) * DEGREE_CAL;
	sample->old_angle_z = atan((short)sample->z/(short)sqrt(pow(sample->x, 2)+pow(sample->y, 2))) * DEGREE_CAL;
	if (sample->old_angle_z < 0)
	{
		sample->old_angle_x = 180-sample->old_angle_x;
		sample->old_angle_y = 180-sample->old_angle_y;
	}
	printf("sample->old_angle_x:%d, sample->old_angle_y:%d, sample->old_angle_z:%d \r\n",sample->old_angle_x, sample->old_angle_y, sample->old_angle_z);
SEGGER_RTT_printf(0,"sample->old_angle_x:%d, sample->old_angle_y:%d, sample->old_angle_z:%d \r\n",sample->old_angle_x, sample->old_angle_y, sample->old_angle_z);
}

