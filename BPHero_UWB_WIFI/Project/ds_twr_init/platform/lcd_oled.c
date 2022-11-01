#include "lcd_oled.h"
#include "stdlib.h"
#include "string.h"
#include "lcd_oledfont.h"
#include "deca_sleep.h"

#include <math.h>
//OLED���Դ�
//��Ÿ�ʽ����.
//[0]0 1 2 3 ... 127
//[1]0 1 2 3 ... 127
//[2]0 1 2 3 ... 127
//[3]0 1 2 3 ... 127
//[4]0 1 2 3 ... 127
//[5]0 1 2 3 ... 127
//[6]0 1 2 3 ... 127
//[7]0 1 2 3 ... 127
#define Memory_refresh    1
#define Screen_cache_SIZE Max_Column*Max_Row/8

static uint8_t Screen_cache[Screen_cache_SIZE];


///I2C related
/*******************************************************************************
* Function Name  : I2C_delay
* Description    : �ӳ�ʱ��
* Input          : None
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
static void I2C_delay(uint16_t cnt)
{
    while(cnt--);
}

int IIc_flag = 1 ;//����1�� ������0
/*******************************************************************************
* Function Name  : I2C_AcknowledgePolling
* Description    : �ȴ���ȡI2C���߿���Ȩ �ж�æ״̬
* Input          : - I2Cx:I2C�Ĵ�����ַ
*                  - I2C_Addr:��������ַ
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
static void I2C_AcknowledgePolling(I2C_TypeDef *I2Cx,uint8_t I2C_Addr)
{
    vu16 SR1_Tmp;
    uint16_t timeout = 10000;
    timeout = IIC_TIMEOUT;
    do
    {
        I2C_GenerateSTART(I2Cx, ENABLE); /*��ʼλ*/
        /*��SR1*/
        SR1_Tmp = I2C_ReadRegister(I2Cx, I2C_Register_SR1);
        /*������ַ(д)*/
        I2C_Send7bitAddress(I2Cx, I2C_Addr, I2C_Direction_Transmitter);

    }
    while(!(I2C_ReadRegister(I2Cx, I2C_Register_SR1) & 0x0002)&&(timeout--));
    I2C_ClearFlag(I2Cx, I2C_FLAG_AF);
    I2C_GenerateSTOP(I2Cx, ENABLE);  /*ֹͣλ*/
}


/*******************************************************************************
* Function Name  : I2C_WriteOneByte
* Description    : ͨ��ָ��I2C�ӿ�д��һ���ֽ�����
* Input          : - I2Cx:I2C�Ĵ�����ַ
*                  - I2C_Addr:��������ַ
*                  - addr:Ԥд���ֽڵ�ַ
*                  - value:д������
* Output         : None
* Return         : �ɹ�����0
* Attention      : None
*******************************************************************************/
uint8_t I2C_WriteOneByte(I2C_TypeDef *I2Cx,uint8_t I2C_Addr,uint8_t addr,uint8_t value)
{
    uint16_t timeout = 0;
    /* ��ʼλ */
    I2C_GenerateSTART(I2Cx, ENABLE);
    timeout = IIC_TIMEOUT;
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT) &&(timeout--));
    I2C_Send7bitAddress(I2Cx, I2C_Addr, I2C_Direction_Transmitter);

    if(timeout == 0)
        IIc_flag = 0;

    timeout = IIC_TIMEOUT;
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)&&(timeout--));
    /*���͵�ַ*/
    I2C_SendData(I2Cx, addr);
    timeout = IIC_TIMEOUT;
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)&&(timeout--));
    /* дһ���ֽ�*/
    I2C_SendData(I2Cx, value);
    timeout = IIC_TIMEOUT;
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)&&(timeout--));
    /* ֹͣλ*/
    I2C_GenerateSTOP(I2Cx, ENABLE);
    /*stop bit flag*/
    timeout = IIC_TIMEOUT;
    while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF)&&(timeout--) );

    //I2C_AcknowledgePolling(I2Cx,I2C_Addr);
    I2C_AcknowledgeConfig(I2Cx, DISABLE);// disable acknowledge
    I2C_delay(80);
    return 0;
}


uint8_t I2C_ReadOneByte(I2C_TypeDef* I2Cx, unsigned char AddressDevice, unsigned char AddressByte)
{
    uint8_t ReceiveData = 0;

    int16_t timeout = 0;
    timeout = IIC_TIMEOUT;
    while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY) &&(timeout--));
    I2C_GenerateSTART(I2Cx, ENABLE);

    timeout = IIC_TIMEOUT;
    while( !I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB) &&(timeout--) ); // wait Generate Start

    I2C_Send7bitAddress(I2Cx, AddressDevice, I2C_Direction_Transmitter);
    timeout = IIC_TIMEOUT;
    while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) &&(timeout--) ); // wait send Address Device

    I2C_SendData(I2Cx, AddressByte);
    timeout = IIC_TIMEOUT;
    while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED) &&(timeout--) ); // wait send Address Byte

    I2C_GenerateSTART(I2Cx, ENABLE);
    timeout = IIC_TIMEOUT;
    while( !I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB)&&(timeout--) ); // wait Generate Start

    I2C_Send7bitAddress(I2Cx, AddressDevice, I2C_Direction_Receiver);
    timeout = IIC_TIMEOUT;
    while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)&&(timeout--) ); // wait Send Address Device As Receiver

    timeout = IIC_TIMEOUT;
    while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) &&(timeout--) ); // wait Receive a Byte
    ReceiveData = I2C_ReceiveData(I2Cx);

    I2C_NACKPositionConfig(I2Cx, I2C_NACKPosition_Current); // send not acknowledge
    I2C_AcknowledgeConfig(I2Cx, DISABLE);// disable acknowledge

    I2C_GenerateSTOP(I2Cx, ENABLE);
    timeout = IIC_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF) &&(timeout--) ); // wait Generate Stop Condition

    I2C_AcknowledgeConfig(I2Cx, DISABLE);// disable acknowledge

    return ReceiveData;
};


/*******************************************************************************
* Function Name  : I2C_Write
* Description    : ͨ��ָ��I2C�ӿ�д�����ֽ�����
* Input          : - I2Cx:I2C�Ĵ�����ַ
*                  - I2C_Addr:��������ַ
*                  - addr:Ԥд���ֽڵ�ַ
*                  - *buf:Ԥд�����ݴ洢λ��
*                  - num:д���ֽ���
* Output         : None
* Return         : �ɹ�����0
* Attention      : None
*******************************************************************************/
uint8_t I2C_Write(I2C_TypeDef *I2Cx,uint8_t I2C_Addr,uint8_t addr,uint8_t *buf,uint16_t num)
{
    uint8_t err=0;

    while(num--)
    {
        if(I2C_WriteOneByte(I2Cx, I2C_Addr,addr++,*buf++))
        {
            err++;
        }
    }
    if(err)
        return 1;
    else
        return 0;
}


/*
//��������
//slaveAddr���ӻ�7λ��ַ���ڰ�λ�ķ�����Զ�����
//pdata��Ҫ���͵����ݵ�ַ
//len��Ҫ���͵����ݳ���
*/
uint8_t hal_i2c1_send_data1(I2C_TypeDef *I2Cx,uint8_t I2C_Addr, uint8_t pdata)
{

    uint16_t timeout = 0;
    /* ��ʼλ */
    I2C_GenerateSTART(I2Cx, ENABLE);
    timeout = IIC_TIMEOUT;
    //__disable_irq();
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT) &&(timeout--));
    //printf("\r\nstart\r\n");

    I2C_Send7bitAddress(I2Cx, I2C_Addr, I2C_Direction_Transmitter);
    if(timeout == 0)
        IIc_flag = 0;
    timeout = IIC_TIMEOUT;
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) &&(timeout--) );//�����豸
    //printf("\r\nweak up\r\n");

    /*��������*/

    I2C_SendData(I2Cx,pdata);
    timeout = IIC_TIMEOUT;
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED) &&(timeout--) );

    //printf("\r\nsend data\r\n");


    /* ֹͣλ*/
    I2C_GenerateSTOP(I2Cx, ENABLE);
    /*stop bit flag*/
    timeout = IIC_TIMEOUT;
    while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF) &&(timeout--));

    //I2C_AcknowledgePolling(I2Cx,I2C_Addr);
    I2C_AcknowledgeConfig(I2Cx, DISABLE);// disable acknowledge
    __enable_irq();
    I2C_delay(9000);
    //I2C_delay(0xfff0);
    //printf("\r\nstop\r\n");
    return 0;

}
uint8_t hal_i2c1_send_data(I2C_TypeDef *I2Cx,uint8_t I2C_Addr, uint8_t *pdata,uint32_t u32_len)
{
    /*��������*/
    for(uint32_t u32_count=0; u32_count<(u32_len); u32_count++) {


        hal_i2c1_send_data1(I2Cx,I2C_Addr,*pdata);
        pdata++;
        I2C_delay(10000);
        //I2C_delay(0xfff0);
        //printf("\r\nsend ok\r\n");
    };

}
int hal_i2c1_read_data(I2C_TypeDef *I2Cx, unsigned char AddressDevice, uint8_t *pdata,uint32_t u32_len)
{
    uint8_t ReceiveData = 0;
    int timeout = 0;
    timeout = IIC_TIMEOUT;
    //printf("\r\nread start\r\n");
    //__disable_irq();
    while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY) &&(timeout--) );
    I2C_GenerateSTART(I2Cx, ENABLE);
    timeout = IIC_TIMEOUT;
    while( !I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB) &&(timeout--) ); // wait Generate Start
    //printf("\r\nread start\r\n");

    timeout = IIC_TIMEOUT;
    I2C_Send7bitAddress(I2Cx, AddressDevice, I2C_Direction_Transmitter);
    while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) &&(timeout--) ); // wait send Address Device
    //printf("\r\nread weak up timeout:%d\r\n",timeout);
    if(timeout<=0)
    {
        I2C_GenerateSTOP(I2Cx, ENABLE);
#if DE_DELAY
        while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF)); // wait Generate Stop Condition
#else
        timeout = IIC_TIMEOUT;
        while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF)&&(timeout --)); // wait Generate Stop Condition
#endif
        I2C_AcknowledgeConfig(I2Cx, DISABLE);// disable acknowledge
        //printf("\r\nread weak up timeout:%d\r\n",timeout);
        return -1;
    }

    timeout = IIC_TIMEOUT;
    I2C_GenerateSTART(I2Cx, ENABLE);
    while( !I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB)&&(timeout--) ); // wait Generate Start

    timeout = IIC_TIMEOUT;
    I2C_Send7bitAddress(I2Cx, AddressDevice, I2C_Direction_Receiver);
    while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)&&(timeout--) ); // wait Send Address Device As Receiver
    //printf("\r\nread rec\r\n");

    /*��������*/
    for(uint32_t u32_count=0; u32_count<(u32_len-1); u32_count++) {
        timeout = IIC_TIMEOUT;
        while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) &&(timeout--) ); // wait Receive a Byte
        *pdata = I2C_ReceiveData(I2Cx);
        pdata++;

        I2C_NACKPositionConfig(I2Cx, I2C_NACKPosition_Next); //
        I2C_AcknowledgeConfig(I2Cx, ENABLE);// disable acknowledge
        //printf("\r\nread Ack\r\n");
    };


//__disable_irq();
    *pdata = I2C_ReceiveData(I2Cx);

    I2C_NACKPositionConfig(I2Cx, I2C_NACKPosition_Current); // send not acknowledge
    I2C_AcknowledgeConfig(I2Cx, DISABLE);// disable acknowledge
    //printf("\r\nread no Ack\r\n");

    I2C_GenerateSTOP(I2Cx, ENABLE);
#if DE_DELAY
    while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF)); // wait Generate Stop Condition
#else
    timeout = IIC_TIMEOUT;
    (I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF)&&(timeout --)); // wait Generate Stop Condition
#endif
    I2C_AcknowledgeConfig(I2Cx, DISABLE);// disable acknowledge
    //printf("\r\nread ok\r\n");
    __enable_irq();
    return ReceiveData;

}


#define SHT20_ADDR					0x80//0x40
#define SHT20_WRITE_REG				0xE6	//д�Ĵ���ָ��
#define SHT20_READ_REG				0xE7	//��ȡ�Ĵ���ָ��
#define TRIG_TEMP_MEASUREMENT_POLL	0xF3	//�����¶Ȳ���(�Ǳ�������)
#define TRIG_HUMI_MEASUREMENT_POLL	0xF5	//����ʪ�Ȳ���(�Ǳ�������)
#define TRIG_RESET					0xFE	//��λָ��

//��λ�����ô�����
void SHT20_init(void) {

    uint8_t u8_cmd_buf[2]= {0},u8_reg=0;

    //��λ
    u8_cmd_buf[0]=TRIG_RESET;	 //���͸�λָ��

    hal_i2c1_send_data(I2C1,SHT20_ADDR,u8_cmd_buf,1);

    //deca_sleep(100);	//��λʱ��С��15ms
    I2C_delay(9000);

    //���Ĵ���
    u8_cmd_buf[0]=SHT20_READ_REG;	 //���Ͷ�ȡ�Ĵ���ָ��
    hal_i2c1_send_data(I2C1,SHT20_ADDR,u8_cmd_buf,1);


    hal_i2c1_read_data(I2C1,SHT20_ADDR,&u8_reg,1);
    //printf("u8_reg:%X\r\n",u8_reg);


    //printf("SHT20_WRITE_REG\r\n");
    //д�Ĵ���
    //u8_cmd_buf[0]=SHT20_WRITE_REG;	 //����д�Ĵ���ָ��
    //u8_cmd_buf[1]=0x02;				 //�Ĵ�����ֵ
    //hal_i2c1_send_data(I2C1,SHT20_ADDR,u8_cmd_buf,2);
    I2C_WriteOneByte(I2C1,0x80,SHT20_WRITE_REG,0x2);


    //���Ĵ���
    u8_cmd_buf[0]=SHT20_READ_REG;	 //���Ͷ�ȡ�Ĵ���ָ��
    hal_i2c1_send_data(I2C1,SHT20_ADDR,u8_cmd_buf,1);

    hal_i2c1_read_data(I2C1,SHT20_ADDR,&u8_reg,1);
    //printf("u8_reg1:%X\r\n",u8_reg);


    return;
}

//��ȡ�¶�(����������ģʽ)
int SHT20_read_tem(float *p_temperature) {
#if 1
	uint8_t u8_cmd_buf[2] = { 0 }, u8_data_buf[3] = { 0 }, u8_count = 0;

	u8_cmd_buf[0] = TRIG_TEMP_MEASUREMENT_POLL;	 //�����¶Ȳ���(�Ǳ�������)
	hal_i2c1_send_data(I2C1, SHT20_ADDR, u8_cmd_buf, 1);

	int h_ret = 1;
	u8_count = 0;
	while (1) {
		I2C_delay(0xfffe);
		h_ret = hal_i2c1_read_data(I2C1, SHT20_ADDR, u8_data_buf, 3);//һֱ��ȡ��ֱ����ȡ�ɹ����߳�ʱ
		if (0 == h_ret) {
			//Ӧ��ɹ�����ѭ��
			break;
		}

		if (u8_count > 20) {
			printf("SHT20_read_tem time out 03");
			return -1;
		}
		u8_count++;
	}

	u8_data_buf[1] &= ~0x0003;

	//��������
	*p_temperature = (u8_data_buf[0] << 8) | u8_data_buf[1];
	*p_temperature = ((*p_temperature) * 0.00268127) - 46.85;

	return 0;
#else
    static uint8_t tem_count = 0;
    float p_temp = 0.0;
    int h_ret = -1;
    uint8_t u8_cmd_buf[2]= {0},u8_data_buf[3]= {0};

    if((tem_count & 0x80) == 0){
		u8_cmd_buf[0]=TRIG_TEMP_MEASUREMENT_POLL;	 //�����¶Ȳ���(�Ǳ�������)
		hal_i2c1_send_data(I2C1,SHT20_ADDR,u8_cmd_buf,1);

		tem_count |= 0x80;
		return -1;
    }

	if (tem_count & 0x80) {
		tem_count++;
		h_ret = hal_i2c1_read_data(I2C1, SHT20_ADDR, u8_data_buf, 3);//һֱ��ȡ��ֱ����ȡ�ɹ����߳�ʱ
		if (0 == h_ret) {
			u8_data_buf[1] &= ~0x0003;

			//��������
			p_temp = (u8_data_buf[0] << 8) | u8_data_buf[1];
			*p_temperature = ((p_temp) * 0.00268127) - 46.85;

			tem_count = 0;
			//Ӧ��ɹ�����ѭ��
			return 0;
		} else {
			if ((tem_count & 0x7f) > 10) {
				printf("SHT20_read_tem time out 03\r\n");
				tem_count = 0;
			}
			return -1;
		}
	}
#endif
}

//��ȡʪ��
int SHT20_read_hum(float *p_humidity){
#if 1
	uint8_t u8_cmd_buf[2]={0},u8_data_buf[3]={0},u8_count=0;

	u8_cmd_buf[0]=TRIG_HUMI_MEASUREMENT_POLL;	 //����ʪ�Ȳ���(�Ǳ�������)
	hal_i2c1_send_data(I2C1,SHT20_ADDR,u8_cmd_buf,1);


	u8_count=0;
	int h_ret=1;
	while(1){
		I2C_delay(0xfffe);
		h_ret=hal_i2c1_read_data(I2C1,SHT20_ADDR,u8_data_buf,3);//һֱ��ȡ��ֱ����ȡ�ɹ����߳�ʱ
		if(0==h_ret){
			//Ӧ��ɹ�����ѭ��
			break;
		}
		if(u8_count>20){
			printf(0,"SHT20_read_tem time out 03");
			return -1;
		}
		u8_count++;
	}

	u8_data_buf[1] &= ~0x0003;

	//��������
	*p_humidity=(u8_data_buf[0]<<8)|u8_data_buf[1];
	*p_humidity=((*p_humidity)*0.00190735)-6;

	return 0;
#else
    static uint8_t hum_count=0;
    float p_humi = 0.0;
    int h_ret = -1;
    uint8_t u8_cmd_buf[2]= {0},u8_data_buf[3]= {0};

	if((hum_count & 0x80) == 0){
		u8_cmd_buf[0]=TRIG_HUMI_MEASUREMENT_POLL;	 //����ʪ�Ȳ���(�Ǳ�������)
		hal_i2c1_send_data(I2C1,SHT20_ADDR,u8_cmd_buf,1);
		hum_count |= 0x80 ;
		return -1;
	}else if (hum_count & 0x80) {
		hum_count++;
		h_ret = hal_i2c1_read_data(I2C1, SHT20_ADDR, u8_data_buf, 3);//һֱ��ȡ��ֱ����ȡ�ɹ����߳�ʱ
		if (0 == h_ret) {
			u8_data_buf[1] &= ~0x0003;
			//��������
			p_humi = (u8_data_buf[0] << 8) | u8_data_buf[1];
			*p_humidity = ((p_humi) * 0.00190735) - 6;
			hum_count = 0;
			return 0;
		} else {
			if ((hum_count & 0x7f) > 1) {
//				printf("SHT20_read_tem time out 03\r\n");
				hum_count = 0;
			}
			return -1;
		}
	}
#endif
}


#ifdef STM32F10X_MD
void OLED_WR_Byte(uint8_t dat,uint8_t cmd)
{
    if(IIc_flag == 1)
    {
        if(cmd)
        {
            I2C_WriteOneByte(I2C1,0x78,0x40,dat);
        }
        else
        {
            I2C_WriteOneByte(I2C1,0x78,0x00,dat);
        }
    }
}

#else
//STM32F103RCT6 SPI OLED
///I2C related end


#if OLED_MODE==1
//��SSD1106д��һ���ֽڡ�
//dat:Ҫд�������/����
//cmd:����/�����־ 0,��ʾ����;1,��ʾ����;
void OLED_WR_Byte(uint8_t dat,uint8_t cmd)
{
    DATAOUT(dat);
    if(cmd)
        OLED_DC_Set();
    else
        OLED_DC_Clr();
    OLED_CS_Clr();
    OLED_WR_Clr();
    OLED_WR_Set();
    OLED_CS_Set();
    OLED_DC_Set();
}
#else
//��SSD1106д��һ���ֽڡ�
//dat:Ҫд�������/����
//cmd:����/�����־ 0,��ʾ����;1,��ʾ����;
void OLED_WR_Byte(uint8_t dat,uint8_t cmd)
{
    uint8_t i;
    if(cmd)
        OLED_DC_Set();
    else
        OLED_DC_Clr();
    OLED_CS_Clr();
    for(i=0; i<8; i++)
    {
        OLED_SCLK_Clr();
        if(dat&0x80)
            OLED_SDIN_Set();
        else
            OLED_SDIN_Clr();
        OLED_SCLK_Set();
        dat<<=1;
    }
    OLED_CS_Set();
    OLED_DC_Set();
}

#endif
#endif
void OLED_Set_Pos(unsigned char x, unsigned char y)
{
    OLED_WR_Byte(0xb0+y,OLED_CMD);
    OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);
    OLED_WR_Byte((x&0x0f)|0x01,OLED_CMD);
}
//����OLED��ʾ
void OLED_Display_On(void)
{
    OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC����
    OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
    OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}
//�ر�OLED��ʾ
void OLED_Display_Off(void)
{
    OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC����
    OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
    OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}
//��������,������,������Ļ�Ǻ�ɫ��!��û����һ��!!!
void OLED_Clear(void)
{
    uint8_t i,n;
#if Memory_refresh == 0
    for(i=0; i<8; i++)
    {
        OLED_WR_Byte (0xb0+i,OLED_CMD);    //����ҳ��ַ��0~7��
        OLED_WR_Byte (0x00,OLED_CMD);      //������ʾλ�á��е͵�ַ
        OLED_WR_Byte (0x10,OLED_CMD);      //������ʾλ�á��иߵ�ַ
        for(n=0; n<128; n++)OLED_WR_Byte(0,OLED_DATA);
    } //������ʾ
#else
    memset((uint8_t*)Screen_cache,0,Screen_cache_SIZE);
    OLED_DrawMem();
#endif
}

void oled_clear(uint8_t x,uint8_t y,uint8_t x0,uint8_t y0)
{
    uint8_t i,n;
    for(n = y;n < y0; n ++){
        for(i = x;i < x0; i ++){
        	Screen_cache[n*128+i] = 0x00;
        }
    }
}

//������ˢ�µ���Ļ��
void OLED_DrawMem(void)
{
    uint8_t i,n;
    for(i=0; i<8; i++)
    {
#if Memory_refresh == 1
        OLED_WR_Byte (0xb0+i,OLED_CMD);    //����ҳ��ַ��0~7��
        OLED_WR_Byte (0x00,OLED_CMD);      //������ʾλ�á��е͵�ַ
        OLED_WR_Byte (0x10,OLED_CMD);      //������ʾλ�á��иߵ�ַ
        for(n=0; n<128; n++)OLED_WR_Byte(Screen_cache[i*Max_Column+n],OLED_DATA);
#endif
    } //������ʾ
}
#if 0
//��ָ��λ����ʾһ���ַ�,���������ַ�
//x:0~127
//y:0~63
//mode:0,������ʾ;1,������ʾ
//size:ѡ������ 16/12
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t size)
{
    unsigned char c=0,i=0,j = 0;
    c=chr-' ';//�õ�ƫ�ƺ��ֵ
    if(x>Max_Column-1)
    {
        x=0;
        y=y+2;
    }
    if(SIZE ==16)
    {
#if Memory_refresh == 0
        OLED_Set_Pos(x,y);
        for(i=0; i<8; i++)
            OLED_WR_Byte(F8X16[c*16+i],OLED_DATA);
        OLED_Set_Pos(x,y+1);
        for(i=0; i<8; i++)
            OLED_WR_Byte(F8X16[c*16+i+8],OLED_DATA);
#else
        memcpy((char*)&Screen_cache[y*Max_Column+x],(char*)&F8X16[c*16], 8);
        memcpy((char*)&Screen_cache[(y+1)*Max_Column+x],(char*)&F8X16[c*16+8], 8);
#endif
    }
    else
    {
#if Memory_refresh == 0
        OLED_Set_Pos(x,y+1);
        for(i=0; i<6; i++)
            OLED_WR_Byte(F6x8[c][i],OLED_DATA);

#else
        memcpy((char*)&Screen_cache[(y+1)*Max_Column+x],(char*)&F6x8[c][0], 6);
#endif
    }
}
#else
//��ָ��λ����ʾһ���ַ�,���������ַ�
//x:0~127
//y:0~63
//mode:0,������ʾ;1,������ʾ
//size:ѡ������ 16/12
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size)
{
	unsigned char c=0,i=0,Nom;
		c=chr-' ';//�õ�ƫ�ƺ��ֵ
		if(x>Max_Column-1){x=0;y=y+2;}
		if(size ==16)
		{
			for(Nom=0;Nom<95;Nom++)
			{
				if(F8X16[Nom].Index1[0] == chr)
				{

#if Memory_refresh == 0
				OLED_Set_Pos(x,y);
				for(i=0;i<8;i++)
				OLED_WR_Byte(F8X16[Nom].Msk1[i],OLED_DATA);
				OLED_Set_Pos(x,y+1);
				for(i=0;i<8;i++)
				OLED_WR_Byte(F8X16[Nom].Msk1[i + 8],OLED_DATA);
#else
				memcpy((char*)&Screen_cache[y*Max_Column+x],(char*)&F8X16[Nom].Msk1[0], 8);
				memcpy((char*)&Screen_cache[(y+1)*Max_Column+x],(char*)&F8X16[Nom].Msk1[8], 8);
#endif
				}
			}
		}
		else {
#if Memory_refresh == 0
			OLED_Set_Pos(x,y+1);
			for(i=0;i<6;i++)
			OLED_WR_Byte(F6x8[c][i],OLED_DATA);

#else
			memcpy((char*)&Screen_cache[(y+1)*Max_Column+x],(char*)&F6x8[c][0], 6);
#endif
		}
}
#endif
//m^n����
uint32_t oled_pow(uint8_t m,uint8_t n)
{
    uint32_t result=1;
    while(n--)result*=m;
    return result;
}
//��ʾ2������
//x,y :�������
//len :���ֵ�λ��
//size:�����С
//mode:ģʽ 0,���ģʽ;1,����ģʽ
//num:��ֵ(0~4294967295);
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size)
{
    uint8_t t,temp;
    uint8_t enshow=0;
    for(t=0; t<len; t++)
    {
        temp=(num/oled_pow(10,len-t-1))%10;
        if(enshow==0&&t<(len-1))
        {
            if(temp==0)
            {
                OLED_ShowChar(x+(size/2)*t,y,' ',size);
                continue;
            }
            else enshow=1;

        }
        OLED_ShowChar(x+(size/2)*t,y,temp+'0',size);
    }
}
//��ʾһ���ַ��Ŵ�
void OLED_ShowString(uint8_t x,uint8_t y,uint8_t *chr)
{
    unsigned char j=0;
    while (chr[j]!='\0')
    {
        OLED_ShowChar(x,y,chr[j],SIZE);
        x+=8;
        if(x>120)
        {
            x=0;
            y+=2;
        }
        j++;
    }
}
//��ʾ����
void OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t no)
{
    uint8_t t,adder=0;
#if Memory_refresh == 0
    OLED_Set_Pos(x,y);
    for(t=0; t<16; t++)
    {
        OLED_WR_Byte(CN16CHAR[no].Msk[t],OLED_DATA);
        adder+=1;
    }
    OLED_Set_Pos(x,y+1);
    for(t=0; t<16; t++)
    {
        OLED_WR_Byte(CN16CHAR[no].Msk[t + 16],OLED_DATA);
        adder+=1;
    }

#else
    memcpy((char*)&Screen_cache[y*Max_Column+x],(char*)&CN16CHAR[no].Msk[0], 16);
    memcpy((char*)&Screen_cache[(y+1)*Max_Column+x],(char*)&CN16CHAR[no].Msk[16], 16);
  
#endif
}

u8 OLED_ShowCH(u8 x,u8 y,u8 *cn)
{
	u8 t,adder=0, wordNum;
	if(y > 64)return 0;
	if(x > 128)return 0;

	while ( *cn)	 //��C�������ַ��������ԡ�\0����β
	{
		for (wordNum=0; wordNum<95; wordNum++)
		{
		    //--��ѯҪд�������ֿ��е�λ��--//
			if (((CN16CHAR[wordNum].Index[0] == *cn)
			     &&(CN16CHAR[wordNum].Index[1] == *(cn+1))) && (*cn > 0x80))
			{
				OLED_ShowCHinese(x,y,wordNum);
				x += 16;
			}//if�鵽�ֽ���
			if ((*cn > 20 && 126 > *cn))
			{
				OLED_ShowChar(x,y,*cn,SIZE);
				x += 8;
			}
		} //for���ֽ���
		cn += 1;
	}	//while����
	return 1;
}

/***********������������ʾ��ʾBMPͼƬ128��64��ʼ������(x,y),x�ķ�Χ0��127��yΪҳ�ķ�Χ0��7*****************/
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[])
{
    unsigned int j=0;
    unsigned char x,y;

    if(y1%8==0) y=y1/8;
    else y=y1/8+1;
    for(y=y0; y<y1; y++)
    {
        OLED_Set_Pos(x0,y);
        for(x=x0; x<x1; x++)
        {
            OLED_WR_Byte(BMP[j++],OLED_DATA);
        }
    }
}

void oled_init(void){
    OLED_WR_Byte(0xAE,OLED_CMD);//--turn off oled panel
    OLED_WR_Byte(0x00,OLED_CMD);//---set low column address
    OLED_WR_Byte(0x10,OLED_CMD);//---set high column address
    OLED_WR_Byte(0x40,OLED_CMD);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
    OLED_WR_Byte(0x81,OLED_CMD);//--set contrast control register
    OLED_WR_Byte(0xCF,OLED_CMD); // Set SEG Output Current Brightness
    OLED_WR_Byte(0xA1,OLED_CMD);//--Set SEG/Column Mapping     0xa0���ҷ��� 0xa1����
    OLED_WR_Byte(0xC8,OLED_CMD);//Set COM/Row Scan Direction   0xc0���·��� 0xc8����
    OLED_WR_Byte(0xA6,OLED_CMD);//--set normal display
    OLED_WR_Byte(0xA8,OLED_CMD);//--set multiplex ratio(1 to 64)
    OLED_WR_Byte(0x3f,OLED_CMD);//--1/64 duty
    OLED_WR_Byte(0xD3,OLED_CMD);//-set display offset   Shift Mapping RAM Counter (0x00~0x3F)
    OLED_WR_Byte(0x00,OLED_CMD);//-not offset
    OLED_WR_Byte(0xd5,OLED_CMD);//--set display clock divide ratio/oscillator frequency
    OLED_WR_Byte(0x80,OLED_CMD);//--set divide ratio, Set Clock as 100 Frames/Sec
    OLED_WR_Byte(0xD9,OLED_CMD);//--set pre-charge period
    OLED_WR_Byte(0xF1,OLED_CMD);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
    OLED_WR_Byte(0xDA,OLED_CMD);//--set com pins hardware configuration
    OLED_WR_Byte(0x12,OLED_CMD);
    OLED_WR_Byte(0xDB,OLED_CMD);//--set vcomh
    OLED_WR_Byte(0x40,OLED_CMD);//Set VCOM Deselect Level
    OLED_WR_Byte(0x20,OLED_CMD);//-Set Page Addressing Mode (0x00/0x01/0x02)
    OLED_WR_Byte(0x02,OLED_CMD);//
    OLED_WR_Byte(0x8D,OLED_CMD);//--set Charge Pump enable/disable
    OLED_WR_Byte(0x14,OLED_CMD);//--set(0x10) disable
    OLED_WR_Byte(0xA4,OLED_CMD);// Disable Entire Display On (0xa4/0xa5)
    OLED_WR_Byte(0xA6,OLED_CMD);// Disable Inverse Display On (0xa6/a7)
    OLED_WR_Byte(0xAF,OLED_CMD);//--turn on oled panel

    OLED_WR_Byte(0xAF,OLED_CMD); /*display ON*/
}

//��ʼ��SSD1306
void OLED_Init(void)
{
#ifdef STM32F10X_HD
//STM32F103RC SPI OLED
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);    //ʹ��A�˿�ʱ��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;         //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//�ٶ�50MHz
    GPIO_Init(GPIOB, &GPIO_InitStructure);    //��ʼ��GPIOD3,6
    GPIO_SetBits(GPIOB,GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_15);

    OLED_RST_Set();
    deca_sleep(100);
    OLED_RST_Clr();
    deca_sleep(200);
    OLED_RST_Set();
#endif

    OLED_WR_Byte(0xAE,OLED_CMD);//--turn off oled panel
    OLED_WR_Byte(0x00,OLED_CMD);//---set low column address
    OLED_WR_Byte(0x10,OLED_CMD);//---set high column address
    OLED_WR_Byte(0x40,OLED_CMD);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
    OLED_WR_Byte(0x81,OLED_CMD);//--set contrast control register
    OLED_WR_Byte(0xCF,OLED_CMD); // Set SEG Output Current Brightness
    OLED_WR_Byte(0xA1,OLED_CMD);//--Set SEG/Column Mapping     0xa0���ҷ��� 0xa1����
    OLED_WR_Byte(0xC8,OLED_CMD);//Set COM/Row Scan Direction   0xc0���·��� 0xc8����
    OLED_WR_Byte(0xA6,OLED_CMD);//--set normal display
    OLED_WR_Byte(0xA8,OLED_CMD);//--set multiplex ratio(1 to 64)
    OLED_WR_Byte(0x3f,OLED_CMD);//--1/64 duty
    OLED_WR_Byte(0xD3,OLED_CMD);//-set display offset   Shift Mapping RAM Counter (0x00~0x3F)
    OLED_WR_Byte(0x00,OLED_CMD);//-not offset
    OLED_WR_Byte(0xd5,OLED_CMD);//--set display clock divide ratio/oscillator frequency
    OLED_WR_Byte(0x80,OLED_CMD);//--set divide ratio, Set Clock as 100 Frames/Sec
    OLED_WR_Byte(0xD9,OLED_CMD);//--set pre-charge period
    OLED_WR_Byte(0xF1,OLED_CMD);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
    OLED_WR_Byte(0xDA,OLED_CMD);//--set com pins hardware configuration
    OLED_WR_Byte(0x12,OLED_CMD);
    OLED_WR_Byte(0xDB,OLED_CMD);//--set vcomh
    OLED_WR_Byte(0x40,OLED_CMD);//Set VCOM Deselect Level
    OLED_WR_Byte(0x20,OLED_CMD);//-Set Page Addressing Mode (0x00/0x01/0x02)
    OLED_WR_Byte(0x02,OLED_CMD);//
    OLED_WR_Byte(0x8D,OLED_CMD);//--set Charge Pump enable/disable
    OLED_WR_Byte(0x14,OLED_CMD);//--set(0x10) disable
    OLED_WR_Byte(0xA4,OLED_CMD);// Disable Entire Display On (0xa4/0xa5)
    OLED_WR_Byte(0xA6,OLED_CMD);// Disable Inverse Display On (0xa6/a7)
    OLED_WR_Byte(0xAF,OLED_CMD);//--turn on oled panel

    OLED_WR_Byte(0xAF,OLED_CMD); /*display ON*/
    OLED_Clear();
    OLED_Set_Pos(0,0);
}
