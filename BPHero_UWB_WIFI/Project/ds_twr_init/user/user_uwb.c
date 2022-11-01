#include "user_uwb.h"
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
#include "user_tim.h"
#include "SEGGER_RTT.h"
#define UWB_TIMEOUT   50000
/* Example application name and version to display on LCD screen. */
#define RNG_DELAY_MS 5

/* Default communication configuration. We use here EVK1000's default mode (mode 3). */
static dwt_config_t config =
{
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_1024,   /* Preamble length. */
    DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* Use non-standard SFD (Boolean) */
    DWT_BR_110K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

/* Default antenna delay values for 64 MHz PRF. See NOTE 1 below. */
//#define TX_ANT_DLY 16436
//#define RX_ANT_DLY 16436
#define TX_ANT_DLY 0
#define RX_ANT_DLY 32950

//#define RX_ANT_DLY 33130


/* Frames used in the ranging process. See NOTE 2 below. */
static uint8 rx_poll_msg[] =  {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
/*标签发送信息*/
static uint8 tx_resp_msg[] =  {0x41, 0x88, 0, 0x0, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8 rx_final_msg[] = {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
/*标签发送确认距离信息*/
//static uint8 distance_msg[] = {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xAA, 0, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 distance_msg[] = {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xAA, 0, 0,0, 0, 0};


/*基站发送信息*/
static uint8 tx_poll_msg[] =  {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8 rx_resp_msg[] =  {0x41, 0x88, 0, 0x0, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8 tx_final_msg[] = {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 angle_msg[] =    {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xFE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 Semaphore_Release[] =    {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0, 0};
static uint8 Tag_Statistics[] =                      {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xE1, 0, 0, 0};
static uint8 Master_Release_Semaphore[] =            {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xE2, 0, 0, 0};
static uint8 Tag_Statistics_response[] =             {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xE3, 0, 0, 0};
static uint8 Master_Release_Semaphore_comfirm[] =    {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xE4, 0, 0, 0};

static uint8_t tx_msg[] = {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0 ,0};




/* Length of the common part of the message (up to and including the function code, see NOTE 2 below). */
#define ALL_MSG_COMMON_LEN 10
/* Index to access some of the fields in the frames involved in the process. */
#define ALL_MSG_SN_IDX 2
#define ALL_MSG_TAG_IDX 3
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TS_LEN 4
#define ANGLE_MSG_IDX 10
#define LOCATION_FLAG_IDX 11
#define LOCATION_INFO_LEN_IDX 12
#define LOCATION_INFO_START_IDX 13
#define ANGLE_MSG_MAX_LEN 30

/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;
static uint8 frame_seq_nb_semaphore = 0;

/* Buffer to store received messages.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 48
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference, so reader can examine it at a breakpoint. */
static uint32 status_reg = 0;

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 ? and 1 ? = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.46 ms with above configuration. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 2600
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500
/* Receive final timeout. See NOTE 5 below. */
#define FINAL_RX_TIMEOUT_UUS 3300


/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 150
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.66 ms with above configuration. */
#define RESP_RX_TO_FINAL_TX_DLY_UUS 2800 //2700 will fail
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 2700



/* Timestamps of frames transmission/reception.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef signed long long int64;
typedef unsigned long long uint64;
static uint64 poll_rx_ts;
static uint64 resp_tx_ts;
static uint64 final_rx_ts;

static uint64 poll_tx_ts;
static uint64 resp_rx_ts;
static uint64 final_tx_ts;

/* Speed of light in air, in metres per second. */
#ifndef SPEED_OF_LIGHT
#define SPEED_OF_LIGHT 299702547
#endif

/* Hold copies of computed time of flight and distance here for reference, so reader can examine it at a breakpoint. */
static double tof;
static double distance;

/* String used to display measured distance on LCD screen (16 characters maximum). */
char dist_str[16] = {0};

/* Declaration of static functions. */
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts);
static void final_msg_set_ts(uint8 *ts_field, uint64 ts);
static void compute_angle_send_to_anthor0(int distance1, int distance2, int distance3);
static void distance_mange(void);
void USART_puts(uint8_t *s,uint8_t len);


uint8 Semaphore[MAX_SLAVE_TAG];


vec3d AnchorList[ANCHOR_MAX_NUM];
vec3d tag_best_solution;
int Anthordistance[ANCHOR_MAX_NUM];
int Anthordistance_count[ANCHOR_MAX_NUM];

#define ANCHOR_REFRESH_COUNT 10



static uint8_t ANCHOR_ID = 0;
data_t udata = {0,0,0,0};
data_t udata_slave[ANCHOR_MAX_NUM] = {{0,},{0,}};


/* Private macro ---------- ---------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


static uint8 Semaphore_Enable = 0 ;
static uint8 Waiting_TAG_Release_Semaphore = 0;

static void distance_mange(void);
static void compute_angle_send_to_anthor0(int distance1, int distance2,int distance3);
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts);
static void final_msg_set_ts(uint8 *ts_field, uint64 ts);

void dwt_dumpregisters(char *str, size_t strSize)
{
    uint32 reg = 0;
    uint8 buff[5];
    int i;
    int cnt ;

#if (0)
    //first print all single registers
    for(i=0; i<0x3F; i++)
    {
        dwt_readfromdevice(i, 0, 5, buff) ;
        str += cnt = sprintf(str,"reg[%02X]=%02X%02X%02X%02X%02X",i,buff[4], buff[3], buff[2], buff[1], buff[0] ) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x20
    for(i=0; i<=32; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x20,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x20,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x21
    for(i=0; i<=44; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x21,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x21,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x23
    for(i=0; i<=0x20; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x23,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x23,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }
#else
    //reg 0x24
    for(i=0; i<=12; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x24,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x24,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x27
    for(i=0; i<=44; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x27,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x27,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x28
    for(i=0; i<=64; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x28,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x28,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x2A
    for(i=0; i<20; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x2A,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x2A,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x2B
    for(i=0; i<24; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x2B,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x2B,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x2f
    for(i=0; i<40; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x2f,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x2f,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x31
    for(i=0; i<84; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x31,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x31,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x36 = PMSC_ID
    for(i=0; i<=48; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x36,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x36,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }
#endif
}

void Anchor_Array_Init(void)
{
    int anchor_index = 0;
    for(anchor_index = 0; anchor_index < ANCHOR_MAX_NUM; anchor_index++)
    {
        Anthordistance[anchor_index] = 0;
        Anthordistance_count[anchor_index] = 0;
    }
}
void Semaphore_Init(void)
{
    int tag_index = 0 ;
    for(tag_index = 0; tag_index <MAX_SLAVE_TAG; tag_index++)
    {
        Semaphore[tag_index]  = 0;
    }
}

int Sum_Tag_Semaphore_request(void)
{
    int tag_index = 0 ;
    int sum_request = 0;
    for(tag_index = SLAVE_TAG_START_INDEX; tag_index <MAX_SLAVE_TAG; tag_index++)
    {
        sum_request+=Semaphore[tag_index];
    }
    return sum_request;
}

#if 1
void Tag_Measure_Dis(void)
{
    uint8 dest_anthor = 0,frame_len = 0;
    float final_distance = 0;
//    do
    for(dest_anthor = 0 ;  dest_anthor < ANCHOR_MAX_NUM; dest_anthor++)
//    for(dest_anthor = 0 ;  dest_anthor < 1; dest_anthor++)
    {
        dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
        dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
        /* Write frame data to DW1000 and prepare transmission. See NOTE 7 below. */
        tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
//        tx_poll_msg[ALL_MSG_SN_IDX] = dest_anthor;
        tx_poll_msg[ALL_MSG_TAG_IDX] = TAG_ID;//基站收到标签的信息，里面有TAG_ID,在基站回复标签的时候，也需要指定TAG_ID,只有TAG_ID一致才做处理

        dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
        dwt_writetxfctrl(sizeof(tx_poll_msg), 0);

        /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
         * set by dwt_setrxaftertxdelay() has elapsed. */
        dwt_starttx(DWT_START_TX_IMMEDIATE| DWT_RESPONSE_EXPECTED);

        //GPIO_SetBits(GPIOA,GPIO_Pin_2);
        //TODO
        dwt_rxenable(0);//这个后加的，默认tx后应该自动切换rx，但是目前debug 发现并没有自动打开，这里强制打开rx

        /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 8 below. */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
        { };
        GPIO_SetBits(GPIOA,GPIO_Pin_1);

        if (status_reg & SYS_STATUS_RXFCG)
        {
            /* Clear good RX frame event and TX frame sent in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

            /* A frame has been received, read it into the local buffer. */
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
            if (frame_len <= RX_BUF_LEN)
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }
            
            ANCHOR_ID = rx_buffer[ALL_MSG_SN_IDX];

            if(rx_buffer[ALL_MSG_TAG_IDX] != TAG_ID)//检测TAG_ID
                continue;
            rx_buffer[ALL_MSG_TAG_IDX] = 0;

            /* As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
            rx_buffer[ALL_MSG_SN_IDX] = 0;

            if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
            {
                uint32 final_tx_time;

                /* Retrieve poll transmission and response reception timestamp. */
                poll_tx_ts = get_tx_timestamp_u64();
                resp_rx_ts = get_rx_timestamp_u64();

                /* Compute final message transmission time. See NOTE 9 below. */
                final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                dwt_setdelayedtrxtime(final_tx_time);

                /* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
                final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFE)) << 8) + TX_ANT_DLY;

                /* Write all timestamps in the final message. See NOTE 10 below. */
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

                /* Write and send final message. See NOTE 7 below. */
                tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
//                tx_final_msg[ALL_MSG_SN_IDX] = dest_anthor;
                tx_final_msg[ALL_MSG_TAG_IDX] = TAG_ID;
                dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0);
                dwt_writetxfctrl(sizeof(tx_final_msg), 0);

                //TODO maybe need longer time
                dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
                dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS*2);
                dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED );

                while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
                { };

                /* Increment frame sequence number after transmission of the poll message (modulo 256). */
                if (status_reg & SYS_STATUS_RXFCG)
                {
                    /* Clear good/fail RX frame event in the DW1000 status register. */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
                    /* A frame has been received, read it into the local buffer. */
                    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
                    if (frame_len <= RX_BUF_LEN)
                    {
                        dwt_readrxdata(rx_buffer, frame_len, 0);
                    }
                    
                    ANCHOR_ID = rx_buffer[ALL_MSG_SN_IDX];

                    if(rx_buffer[ALL_MSG_TAG_IDX] != TAG_ID)
                        continue;
                    rx_buffer[ALL_MSG_TAG_IDX] = 0;

                    /*As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
                    rx_buffer[ALL_MSG_SN_IDX] = 0;

                    if (memcmp(rx_buffer, distance_msg, ALL_MSG_COMMON_LEN) == 0)
                    {
                      SEGGER_RTT_printf(0,"<%s>[%d]\r\nANCHOR_ID:%d--L:%d\r\n",__func__,__LINE__,ANCHOR_ID,(rx_buffer[10]*100 + rx_buffer[11]));
                      
                    }
                    else if(memcmp(rx_buffer, tx_msg, ALL_MSG_COMMON_LEN) == 0)
                    {
                      udata_slave[ANCHOR_ID].data_mark = 1;
                      udata_slave[ANCHOR_ID].l = rx_buffer[TX_BUFF_DATA_USER_L] * 256 + rx_buffer[TX_BUFF_DATA_USER_L + 1];
                      udata_slave[ANCHOR_ID].x = (int)(rx_buffer[TX_BUFF_DATA_USER_X] * 256 + rx_buffer[TX_BUFF_DATA_USER_X + 1]);
                      udata_slave[ANCHOR_ID].y = (int)(rx_buffer[TX_BUFF_DATA_USER_Y] * 256 + rx_buffer[TX_BUFF_DATA_USER_Y + 1]);
                      udata_slave[ANCHOR_ID].z = (int)(rx_buffer[TX_BUFF_DATA_USER_Z] * 256 + rx_buffer[TX_BUFF_DATA_USER_Z + 1]);
                      
                      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                      
                      SEGGER_RTT_printf(0,"<%s>[%d]\r\nANCHOR_ID:%d--L:%d  X:%d  Y:%d  Z:%d\r\n",__func__,__LINE__,ANCHOR_ID,
                      udata_slave[ANCHOR_ID].l,udata_slave[ANCHOR_ID].x,\
                      udata_slave[ANCHOR_ID].y,udata_slave[ANCHOR_ID].z);
//                      return;
                    }
                }
                else
                {
                    /* Clear RX error events in the DW1000 status register. */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                }
            }
            else if(memcmp(rx_buffer, tx_msg, ALL_MSG_COMMON_LEN) == 0)
            {
              udata_slave[ANCHOR_ID].data_mark = 1;
              udata_slave[ANCHOR_ID].l = rx_buffer[TX_BUFF_DATA_USER_L] * 256 + rx_buffer[TX_BUFF_DATA_USER_L + 1];
              udata_slave[ANCHOR_ID].x = (int)(rx_buffer[TX_BUFF_DATA_USER_X] * 256 + rx_buffer[TX_BUFF_DATA_USER_X + 1]);
              udata_slave[ANCHOR_ID].y = (int)(rx_buffer[TX_BUFF_DATA_USER_Y] * 256 + rx_buffer[TX_BUFF_DATA_USER_Y + 1]);
              udata_slave[ANCHOR_ID].z = (int)(rx_buffer[TX_BUFF_DATA_USER_Z] * 256 + rx_buffer[TX_BUFF_DATA_USER_Z + 1]);
              
              
//              SEGGER_RTT_printf(0,"<%s>[%d]\r\nANCHOR_ID:%d--L:%3.2f  X:%3.2f  Y:%3.2f  Z:%3.2f\r\n",__func__,__LINE__,ANCHOR_ID,udata_slave[ANCHOR_ID].l/100,udata_slave[ANCHOR_ID].x/100,\
//              udata_slave[ANCHOR_ID].y/100,udata_slave[ANCHOR_ID].z/100);
                SEGGER_RTT_printf(0,"<%s>[%d]\r\nANCHOR_ID:%d--L:%d  X:%d  Y:%d  Z:%d\r\n",__func__,__LINE__,ANCHOR_ID,
                    udata_slave[ANCHOR_ID].l,udata_slave[ANCHOR_ID].x,\
                    udata_slave[ANCHOR_ID].y,udata_slave[ANCHOR_ID].z);
              
              dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
//              return;
              
            }
        }
        else
        {
            /* Clear RX error events in the DW1000 status register. */
            // sprintf(dist_str, "%08x",status_reg);
            // OLED_ShowString(0, 2,"           ");
            // OLED_ShowString(0, 2,dist_str);
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        }
        /* Execute a delay between ranging exchanges. */
        // deca_sleep(RNG_DELAY_MS);
        frame_seq_nb++;
    }
    while(0);
}


#endif
/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
//   /*!< At this stage the microcontroller clock setting is already configured,
//        this is done through SystemInit() function which is called from startup
//        file (startup_stm32f10x_xx.s) before to branch to application main.
//        To reconfigure the default setting of SystemInit() function, refer to
//        system_stm32f10x.c file
//      */

double final_distance =  0;

#define Filter_N 5  //max filter use in this system
#define Filter_D 5  //each filter contain "Filter_D" data
int Value_Buf[Filter_N][Filter_D]= {0};
int filter_index[Filter_N] = {0};
int filter(int input, int fliter_idx )
{
    char count = 0;
    int sum = 0;
    if(input > 0)
    {
        Value_Buf[fliter_idx][filter_index[fliter_idx]++]=input;
        if(filter_index[fliter_idx] == Filter_D) filter_index[fliter_idx] = 0;

        for(count = 0; count<Filter_D; count++)
        {
            sum += Value_Buf[fliter_idx][count];
        }
        return (int)(sum/Filter_D);
    }
    else
    {
        for(count = 0; count<Filter_D; count++)
        {
            sum += Value_Buf[fliter_idx][count];
        }
        return (int)(sum/Filter_D);
    }

}

#define DISTANCE3 0.27

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_tx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_get_ts()
 *
 * @brief Read a given timestamp value from the final message. In the timestamp fields of the final message, the least
 *        significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to read
 *         ts  timestamp value
 *
 * @return none
 */
static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts)
{
    int i;
    *ts = 0;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        *ts += ts_field[i] << (i * 8);
    }
}
/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_set_ts()
 *
 * @brief Fill a given timestamp field in the final message with the given value. In the timestamp fields of the final
 *        message, the least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to fill
 *         ts  timestamp value
 *
 * @return none
 */
static void final_msg_set_ts(uint8 *ts_field, uint64 ts)
{
    int i;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        ts_field[i] = (uint8) ts;
        ts >>= 8;
    }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: SEGGER_RTT_printf(0,"Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}
#endif

PUTCHAR_PROTOTYPE
{
    /* Place your implementation of fputc here */
    /* 清SR寄存器中的TC标志 */

    USART_ClearFlag(EVAL_COM1,USART_FLAG_TC);
    /* e.g. write a character to the USART */
    USART_SendData(EVAL_COM1, (uint8_t) ch);
    /* Loop until the end of transmission */
    while (USART_GetFlagStatus(EVAL_COM1, USART_FLAG_TC) == RESET)
    {}
    return ch;
}


void user_dwb_init(void)
{
    /* Reset and initialise DW1000.
     * For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for optimum
     * performance. */
    reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */

    spi_set_rate_low();
    if(dwt_initialise(DWT_LOADUCODE) == -1)
    {
        SEGGER_RTT_printf(0,"dwm1000 init fail!\r\n");
//        OLED_ShowString(0,0,"INIT FAIL");
        uint8_t i = 5;
        while (i -- )
        {
            STM_EVAL_LEDOn(LED1);
            deca_sleep(100);
            STM_EVAL_LEDOff(LED1);
            deca_sleep(100);
        }
    }
    spi_set_rate_high();

    /* Configure DW1000. See NOTE 6 below. */
    dwt_configure(&config);
    dwt_setleds(1);
    /* Apply default antenna delay value. See NOTE 1 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
//    OLED_ShowString(0,0,"INIT PASS");

    SEGGER_RTT_printf(0,"init pass!\r\n");
    printf("init pass!\r\n");

    AnchorList[0].x =0.12;
    AnchorList[0].y =0.34;
    AnchorList[0].z =0;

    AnchorList[1].x =0.25;
    AnchorList[1].y =0;
    AnchorList[1].z =0;

    AnchorList[2].x =0;
    AnchorList[2].y =0;
    AnchorList[2].z =0;
    int rx_ant_delay =32880;
    int index = 0 ;
    
//		tx_poll_msg[6] = ANCHOR_IND;//ANCHOR_ID;	//UWB POLL 包数据
//		rx_resp_msg[6] = ANCHOR_IND;//ANCHOR_ID;	//UWB RESPONSE 包数据
//		tx_final_msg[6] = ANCHOR_IND;//ANCHOR_ID;//UWB Fianl 包数据
//		
//		rx_poll_msg[6] = ANCHOR_IND;//ANCHOR_ID;
//		tx_resp_msg[6] = ANCHOR_IND;//ANCHOR_ID;
//		rx_final_msg[6] = ANCHOR_IND;//ANCHOR_ID;
//		
//		tx_poll_msg[5] = TAG_ID;//UWB POLL 包数据
//		rx_resp_msg[5] = TAG_ID;//UWB RESPONSE 包数据
//		tx_final_msg[5] = TAG_ID;//UWB Fianl 包数据
    
#if ANTHOR == 1
    Anchor_Array_Init();
    /* Loop forever initiating ranging exchanges. */
//    OLED_ShowString(0,0,"DS TWR ANTHOR");
//    OLED_ShowString(0,2,"Distance:");

    KalMan_PramInit();
#endif

#if TAG == 1
    /* Set expected response's delay and timeout. See NOTE 4 and 5 below.
     * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
//    if(TAG_ID == MASTER_TAG)
//    {
//        OLED_ShowString(0,0,"DS MASTER TAG:");
//    }
//    else
//    {
//        OLED_ShowString(0,0,"DS SLAVE TAG:");
//    }

//    OLED_ShowString(0,2,"Distance:");

    if(TAG_ID ==  MASTER_TAG)
    {
        Semaphore_Enable = 1 ;
        Semaphore_Init();
        Waiting_TAG_Release_Semaphore = 0;
    }
    else
    {
        Semaphore_Enable = 0 ;
    }
    //Master TAG0
#endif
}
#if 1
void user_uwb_ruing(void )
{
    static int coun_ary[100];
    static uint8_t coun_ary_c;

    static uint8 status  = 0,TAG_ID_rx = 0;
    uint32_t time_out = 0,time_out_1 = 0;
    static uint8_t anthor_flag = 0,print_count = 0;
    uint8 anthor_index = 0;
    uint8 tag_index = 0;

    int frame_len = 0;
    coun_ary_c = 0;
#if ANTHOR == 1

    if(status == 0)
    {
        /* Clear reception timeout to start next ranging process. */
        dwt_setrxtimeout(0);
        /* Activate reception immediately. */
        dwt_rxenable(0);
    }

    /* Poll for reception of a frame or error/timeout. See NOTE 7 below. */
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
    {
      delay_ms(1);
    };
    
    if (status_reg & SYS_STATUS_RXFCG)
    {
        /* Clear good RX frame event in the DW1000 status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

        /* A frame has been received, read it into the local buffer. */
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
        if (frame_len <= RX_BUFFER_LEN)
        {
            dwt_readrxdata(rx_buffer, frame_len, 0);
        }
        /* Check that the frame is a poll sent by "DS TWR initiator" example.
         * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */

        if(rx_buffer[ALL_MSG_SN_IDX]%ANCHOR_MAX_NUM != ANCHOR_IND)
        {
            status = 0;
            delay_ms(2);
            return;
        }
        status = 1;
        anthor_index = rx_buffer[ALL_MSG_SN_IDX]%ANCHOR_MAX_NUM;
        tag_index = rx_buffer[ALL_MSG_TAG_IDX];

        rx_buffer[ALL_MSG_SN_IDX] = 0;
        rx_buffer[ALL_MSG_TAG_IDX] = 0;
//        TAG_ID_rx = rx_buffer[5];
//        rx_poll_msg[5] = TAG_ID_rx;//为多标签通讯服务，防止一次通讯中接收到不同ID标签的数据
//        tx_resp_msg[5] = TAG_ID_rx;
//        rx_final_msg[5] = TAG_ID_rx;
        if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0)
        {
            /* Retrieve poll reception timestamp. */
            poll_rx_ts = get_rx_timestamp_u64();           

            /* Set expected delay and timeout for final message reception. */
            dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
            dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);

            /* Write and send the response message. See NOTE 9 below.*/
            tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
//            tx_resp_msg[ALL_MSG_SN_IDX] = ANCHOR_IND;
            tx_resp_msg[ALL_MSG_SN_IDX] = anthor_index;
            tx_resp_msg[ALL_MSG_TAG_IDX] = tag_index;
            dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
            dwt_writetxfctrl(sizeof(tx_resp_msg), 0);
            dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
        }
        else if(memcmp(rx_buffer, rx_final_msg, ALL_MSG_COMMON_LEN) == 0)
        {
            uint32 poll_tx_ts, resp_rx_ts, final_tx_ts;
            uint32 poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
            double Ra, Rb, Da, Db;
            int64 tof_dtu;

            /* Retrieve response transmission and final reception timestamps. */
            resp_tx_ts = get_tx_timestamp_u64();
            final_rx_ts = get_rx_timestamp_u64();

            /* Get timestamps embedded in the final message. */
            final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
            final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
            final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

            /* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. See NOTE 10 below. */
            poll_rx_ts_32 = (uint32)poll_rx_ts;
            resp_tx_ts_32 = (uint32)resp_tx_ts;
            final_rx_ts_32 = (uint32)final_rx_ts;
            Ra = (double)(resp_rx_ts - poll_tx_ts);
            Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
            Da = (double)(final_tx_ts - resp_rx_ts);
            Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
            tof_dtu = (int64)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

            tof = tof_dtu * DWT_TIME_UNITS;
            distance = tof * SPEED_OF_LIGHT;
            distance = distance - dwt_getrangebias(config.chan,(float)distance, config.prf);//距离减去矫正系数
            // sprintf(dist_str, "dis: %3.2f m", distance);
            printf("before kalman fliter Distance:%3.2f m,rx_buffer[12]:0x%02x\r\n",distance,rx_buffer[12]);
            //kalman filter
            distance =  KalMan_Update(&distance);
            sprintf(dist_str, "dis: %3.2f m", distance);
            printf("after kalman fliter Distance:%3.2f m,rx_buffer[12]:0x%02x\r\n",distance,rx_buffer[12]);
			SEGGER_RTT_printf(0, "<%s>[%d]%s\r\n",__func__,__LINE__,dist_str);

            //将计算结果发送给TAG
            int temp = (int)(distance*100);
            distance_msg[10] = temp/100;
            // a=x;  //自动类型转换，取整数部分
            distance_msg[11] = temp%100;  //乘100后对100取余，得到2位小数点后数字
            distance_msg[12] = anthor_index;

            distance_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
//            distance_msg[ALL_MSG_SN_IDX] = ANCHOR_IND;
          tx_resp_msg[ALL_MSG_SN_IDX] = anthor_index;
            distance_msg[ALL_MSG_TAG_IDX] = tag_index;
            dwt_writetxdata(sizeof(distance_msg), distance_msg, 0);
            dwt_writetxfctrl(sizeof(distance_msg), 0);

            udata.l = distance * 100;
            udata.data_mark = 1;

            /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
             * set by dwt_setrxaftertxdelay() has elapsed. */
            dwt_starttx(DWT_START_TX_IMMEDIATE );
          //  while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
          //  { };
            /* Display computed distance on LCD. */
            // OLED_ShowString(0,6,"                ");
            // sprintf(dist_str, "DIST: %3.2f m", distance);
            //lcd_display_str(dist_str);
//            OLED_ShowString(0,6,dist_str);
            status  = 0;
        }
        else
        {
            /* Clear RX error events in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
            status  = 0;
        }

    }
    else
    {
        /* Clear RX error events in the DW1000 status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        status  = 0;
    }
    

#endif

#if TAG == 1
        {
            GPIO_ResetBits(GPIOA,GPIO_Pin_1);
            GPIO_ResetBits(GPIOA,GPIO_Pin_2);

            //send message to anthor,TAG<->ANTHOR
            Tag_Measure_Dis();//measuer distance between tag and all anthor
        }
        
        
        
#endif


//    SEGGER_RTT_printf(0,"Debug __LINE__:\r\n");
//    for(int i = 0;i < coun_ary_c;i ++)
//    {
//      SEGGER_RTT_printf(0," %d,",coun_ary[i]);
//    }
//    SEGGER_RTT_printf(0,"\r\n\r\n");
    coun_ary_c = 0;

}

#endif

void dw1000_send_userdata(void )
{
    /* Clear reception timeout to start next ranging process. */
    dwt_setrxtimeout(0);

    tx_msg[ALL_MSG_SN_IDX] = ANCHOR_IND;
    tx_msg[ALL_MSG_TAG_IDX] = TAG_ID;
    tx_msg[TX_BUFF_DATA_USER_L    ] = (uint16_t)udata.l / 256;
    tx_msg[TX_BUFF_DATA_USER_L + 1] = (uint16_t)udata.l % 256;
    tx_msg[TX_BUFF_DATA_USER_X    ] = (uint16_t)udata.x / 256;
    tx_msg[TX_BUFF_DATA_USER_X + 1] = (uint16_t)udata.x % 256;
    tx_msg[TX_BUFF_DATA_USER_Y    ] = (uint16_t)udata.y / 256;
    tx_msg[TX_BUFF_DATA_USER_Y + 1] = (uint16_t)udata.y % 256;
    tx_msg[TX_BUFF_DATA_USER_Z    ] = (uint16_t)udata.z / 256;
    tx_msg[TX_BUFF_DATA_USER_Z + 1] = (uint16_t)udata.z % 256;
    dwt_writetxdata(sizeof(tx_msg), tx_msg, 0);
    dwt_writetxfctrl(sizeof(tx_msg), 0);
    dwt_starttx(DWT_START_TX_IMMEDIATE );
  
    /* Activate reception immediately. */
    dwt_rxenable(0);
  
//    printf("<%s>[%d]\r\n",__func__,__LINE__);
}

void dw1000_rx_userdata(void )
{
    /* Clear reception timeout to start next ranging process. */
    dwt_setrxtimeout(0);
    /* Activate reception immediately. */
    dwt_rxenable(0);

    /* Poll for reception of a frame or error/timeout. See NOTE 7 below. */
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
    { };
}

void print_data(user_data_t *rx_udata)
{
#if 0
	printf("\r\n+++################################+++\r\n");
    printf("<%s>[%d]%s:%c\r\n",__func__,__LINE__,str(rx_udata->head),rx_udata->head);
    printf("<%s>[%d]%s:%d\r\n",__func__,__LINE__,str(rx_udata->data_mark),rx_udata->data_mark);
    printf("<%s>[%d]%s:%d\r\n",__func__,__LINE__,str(rx_udata->object),rx_udata->object);
    printf("<%s>[%d]%s:%d\r\n",__func__,__LINE__,str(rx_udata->cmd),rx_udata->cmd);
    printf("<%s>[%d]%s:%d\r\n",__func__,__LINE__,str(rx_udata->l),rx_udata->l);
    printf("<%s>[%d]%s:%d\r\n",__func__,__LINE__,str(rx_udata->x),rx_udata->x);
    printf("<%s>[%d]%s:%d\r\n",__func__,__LINE__,str(rx_udata->y),rx_udata->y);
    printf("<%s>[%d]%s:%d\r\n",__func__,__LINE__,str(rx_udata->z),rx_udata->z);
    printf("<%s>[%d]%s:%d\r\n",__func__,__LINE__,str(rx_udata->humi),rx_udata->humi);
    printf("<%s>[%d]%s:%d\r\n",__func__,__LINE__,str(rx_udata->temp),rx_udata->temp);
    printf("<%s>[%d]%s:%d\r\n",__func__,__LINE__,str(rx_udata->lumi),rx_udata->lumi);
    printf("<%s>[%d]%s:%d\r\n",__func__,__LINE__,str(rx_udata->value),rx_udata->value);
	printf("\r\n---################################---\r\n");
#endif
}


/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. The sum of the values is the TX to RX antenna delay, experimentally determined by a calibration process. Here we use a hard coded typical value
 *    but, in a real application, each device should have its own antenna delay properly calibrated to get the best possible precision when performing
 *    range measurements.
 * 2. The messages here are similar to those used in the DecaRanging ARM application (shipped with EVK1000 kit). They comply with the IEEE
 *    802.15.4 standard MAC data frame encoding and they are following the ISO/IEC:24730-62:2013 standard. The messages used are:
 *     - a poll message sent by the initiator to trigger the ranging exchange.
 *     - a response message sent by the responder allowing the initiator to go on with the process
 *     - a final message sent by the initiator to complete the exchange and provide all information needed by the responder to compute the
 *       time-of-flight (distance) estimate.
 *    The first 10 bytes of those frame are common and are composed of the following fields:
 *     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: PAN ID (0xDECA).
 *     - byte 5/6: destination address, see NOTE 3 below.
 *     - byte 7/8: source address, see NOTE 3 below.
 *     - byte 9: function code (specific values to indicate which message it is in the ranging process).
 *    The remaining bytes are specific to each message as follows:
 *    Poll message:
 *     - no more data
 *    Response message:
 *     - byte 10: activity code (0x02 to tell the initiator to go on with the ranging exchange).
 *     - byte 11/12: activity parameter, not used for activity code 0x02.
 *    Final message:
 *     - byte 10 -> 13: poll message transmission timestamp.
 *     - byte 14 -> 17: response message reception timestamp.
 *     - byte 18 -> 21: final message transmission timestamp.
 *    All messages end with a 2-byte checksum automatically set by DW1000.
 * 3. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
 *    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
 *    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
 * 4. Delays between frames have been chosen here to ensure proper synchronisation of transmission and reception of the frames between the initiator
 *    and the responder and to ensure a correct accuracy of the computed distance. The user is referred to DecaRanging ARM Source Code Guide for more
 *    details about the timings involved in the ranging process.
 * 5. This timeout is for complete reception of a frame, i.e. timeout duration must take into account the length of the expected frame. Here the value
 *    is arbitrary but chosen large enough to make sure that there is enough time to receive the complete final frame sent by the responder at the
 *    110k data rate used (around 3.5 ms).
 * 6. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW1000 OTP memory.
 * 7. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
 *    refer to DW1000 User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
 *    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
 *    bytes.
 * 8. Timestamps and delayed transmission time are both expressed in device time units so we just have to add the desired response delay to poll RX
 *    timestamp to get response transmission time. The delayed transmission time resolution is 512 device time units which means that the lower 9 bits
 *    of the obtained value must be zeroed. This also allows to encode the 40-bit value in a 32-bit words by shifting the all-zero lower 8 bits.
 * 9. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW1000. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
 *    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()). It is also to be noted that, when using
 *    delayed send, the time set for transmission must be far enough in the future so that the DW1000 IC has the time to process and start the
 *    transmission of the frame at the wanted time. If the transmission command is issued too late compared to when the frame is supposed to be sent,
 *    this is indicated by an error code returned by dwt_starttx() API call. Here it is not tested, as the values of the delays between frames have
 *    been carefully defined to avoid this situation.
 * 10. The high order byte of each 40-bit time-stamps is discarded here. This is acceptable as, on each device, those time-stamps are not separated by
 *     more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays can be handled by a 32-bit
 *     subtraction.
 * 11. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *     DW1000 API Guide for more details on the DW1000 driver functions.
 ****************************************************************************************************************************************************/
