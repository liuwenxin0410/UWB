#ifndef __WAIFICMD_H_
#define __WAIFICMD_H_

#include "usart3.h"
#include <string.h>
#include <stddef.h>

#include "user_uwb.h"

#define chartonumber(x) (x - '0')

__IO typedef enum ATK8266_UDP
{
    UDP_NUll = 'N',
    UDP0 = 0,
    UDP1 = 1,
    UDP2 = 2,
    UDP3 = 3,
    UDP4 = 4,
    UDP5 = 5,
    UDP6 = 6,
    UDP7 = 7,
    UDP8 = 8,
    UDP9 = 9
} UDP;

extern __IO UDP Wifi_UDP;


void RTK8266_init (void);
void Tx_Data (char* data,u8 flag,u8 Lengh);
void UPD_Choose (uint8_t* cmd,uint8_t Lenth);
u8* atk_8266_check_cmd(u8 *str);
u8 atk_8266_send_cmd(const u8 *cmd,u8 *ack,u16 waittime);
u8 atk_8266_send_data(u8 *data,u8 *ack,u16 waittime);
u8 atk_8266_consta_check(void);
u8 atk_8266_apsta_check(void);
u8 atk_8266_quit_trans(void);
void atk_8266_at_response(u8 mode);
#endif

