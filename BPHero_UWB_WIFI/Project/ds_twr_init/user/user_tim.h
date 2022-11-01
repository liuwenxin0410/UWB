#ifndef __USER_TIM_H_
#define __USER_TIM_H_

#include "stm32f10x.h"

typedef enum signstatus{
	sign_no  = 0,		//无信号发送
	sign_zero  = 1,		//0发送
	sign_one  = 2,		//1发送
	sign_head = 3,		//引导码
}sign_t;

typedef struct
{
	/*
	 * 发送状态指示高四位1信号，低四位0信号
	 */
    uint32_t tim_reg;
    uint16_t tim_pulse;			//32kHZ调制占空比
    uint8_t tim_zero_sign;		//发送0时候的计数阀值,560/26=22,560/26=22
    uint8_t tim_one_sign;		//发送1时候的计数阀值,560/26=22,1685/22=77
    sign_t tim_signstatus;		//当前正在发送的电平状态
    uint32_t tim_count;

    uint8_t tim_data;
    uint8_t tim_data_f;
    uint8_t tim_work;			//工作状态，为0不工作，其它:工作
} user_infrared_t;


typedef struct
{
    uint8_t tim100us;
    uint8_t tim200us;
    uint8_t tim1ms;
    uint8_t tim10ms;
    uint8_t tim20ms;
    uint8_t tim50ms;
    uint8_t tim100ms;
    uint8_t tim500ms;
    uint8_t tim_mark;
    uint32_t tim_count;
} user_tim_flag_t;

typedef user_infrared_t infrared_t;
typedef user_tim_flag_t USER_TIM;
extern __attribute((aligned (8))) __IO USER_TIM  USERE_TIME_USECOUNT;

#define TIM_CLEAN()   (void*)0
#define TIM_100us USERE_TIME_USECOUNT.tim100us
#define TIM_200us USERE_TIME_USECOUNT.tim200us
#define TIM_1ms   USERE_TIME_USECOUNT.tim1ms
#define TIM_10ms  USERE_TIME_USECOUNT.tim10ms
#define TIM_20ms  USERE_TIME_USECOUNT.tim20ms
#define TIM_50ms  USERE_TIME_USECOUNT.tim50ms
#define TIM_100ms USERE_TIME_USECOUNT.tim100ms
#define TIM_500ms USERE_TIME_USECOUNT.tim500ms
#if 0
#define user_set_PWM(x) TIM_SetCompare2(TIM3,x)    //控制PWM
#else
#define user_set_PWM(x) TIM_SetCompare4(TIM3,x)    //控制PWM
#endif
void Tim4_Init (u16 arr,u16 psc);
void TIM3_Int_Init(void);
void TIM2_100us_Init(void);
void user_infrared_data(uint8_t data,uint8_t data_f);
uint32_t get_count(void);
uint8_t get_flag(void);
void write_pin_time(uint16_t ms);
void write_pin_handle(void);
#endif

