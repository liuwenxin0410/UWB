#include "bh1750.h"

#include "lcd_oled.h"
#include "stdlib.h"
#include "string.h"

#define SlaveAddress 0x46 //ADDR��0��BH1750��I2C������ַΪ0x46=0100011+0
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

void BH1750_Write(u8 REG_Address)
{
    uint16_t timeout = 0;
    I2C_TypeDef *I2Cx = I2C1;
    /* ��ʼλ */
    I2C_GenerateSTART(I2Cx, ENABLE);
    timeout = IIC_TIMEOUT;
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT) &&(timeout--));
    I2C_Send7bitAddress(I2Cx, SlaveAddress, I2C_Direction_Transmitter);

    timeout = IIC_TIMEOUT;
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)&&(timeout--));
    /*���͵�ַ*/
    I2C_SendData(I2Cx, REG_Address);
    timeout = IIC_TIMEOUT;
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)&&(timeout--));
    /* ֹͣλ*/
    I2C_GenerateSTOP(I2Cx, ENABLE);
    /*stop bit flag*/
    timeout = IIC_TIMEOUT;
    while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF)&&(timeout--) );

    I2C_AcknowledgeConfig(I2Cx, DISABLE);// disable acknowledge
    I2C_delay(80);
}

u16 BH1750_Read_Data(void)
{

    u16 hbyte=0,lbyte=0;

    hbyte=I2C_ReadOneByte(I2C1,SlaveAddress+1,1);

    lbyte=I2C_ReadOneByte(I2C1,SlaveAddress+1,0);

    return ((hbyte<<8)|lbyte);
}

int read_BH1750(void)
{
  static uint8_t flag = 0;
  uint8_t data = 0;
  int dis_data = 0; //double data;                      //����
  if((flag & 0x80) == 0)
  {
    BH1750_Write(0x01);   //�����ϵ�����(0x01)
    BH1750_Write(0x10);   //���͸߷ֱ���������������(0x10)
    flag |= 0x80;
    return -1;
  }

  if((flag & 0x80) && ((flag & 0x7f) < 10))   //20msִ��//    delay_ms(200); //�ȴ�������������ʵ��ʱ180ms�����ˣ���ʱ200msֻ��Ԥ����һ��ʱ�䣬��֤ͨѶ����һʧ
  {
    flag ++;
    return -1;
  }
  else
  {
	data = BH1750_Read_Data() ;
    dis_data = 256 * data;
    dis_data +=  data;
    flag = 0;
	return dis_data;
  }
}

