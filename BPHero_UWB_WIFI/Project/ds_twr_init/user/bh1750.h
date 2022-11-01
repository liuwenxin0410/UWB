#ifndef __BH1750_H_
#define __BH1750_H_

#define SlaveAddress 0x46 //ADDR置0，BH1750的I2C从属地址为0x46=0100011+0

int read_BH1750(void);


#endif


