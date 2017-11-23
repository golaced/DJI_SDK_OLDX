

#include "led_fc.h"
#include "include.h"
#include "mpu6050.h"
#include "hml5833l.h"
void LEDRGB_COLOR(u8 color);
void LED_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(ANO_RCC_LED,ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	#if USE_VER_FINAL
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;
	#else
	GPIO_InitStructure.GPIO_Pin   = ANO_Pin_LED1| ANO_Pin_LED2| ANO_Pin_LED3;
	#endif
	GPIO_Init(ANO_GPIO_LED, &GPIO_InitStructure);

  #if USE_VER_FINAL	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5|GPIO_Pin_6;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	#endif
	 //SEL
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOA,ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_Pin   =GPIO_Pin_15 ;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	LEDRGB_COLOR(BLUE);
	Delay_ms(500);
	LEDRGB_COLOR(RED);
	Delay_ms(500);
	LEDRGB_COLOR(GREEN);
  Delay_ms(500);

}



/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/


void LEDRGB(u8 sel,u8 on)
{
#if USE_VER_FINAL
switch(sel)
{
case RED:
if(!on)
GPIO_ResetBits(GPIOC,GPIO_Pin_7);
else
GPIO_SetBits(GPIOC,GPIO_Pin_7);
break;
case GREEN:
if(!on)
GPIO_ResetBits(GPIOA,GPIO_Pin_5);
else
GPIO_SetBits(GPIOA,GPIO_Pin_5);
break;
case BLUE:
if(!on)
GPIO_ResetBits(GPIOA,GPIO_Pin_6);
else
GPIO_SetBits(GPIOA,GPIO_Pin_6);
break;
}
#else	
switch(sel)
{
case RED:
if(!on)
GPIO_ResetBits(GPIOE,GPIO_Pin_3);
else
GPIO_SetBits(GPIOE,GPIO_Pin_3);
break;
case GREEN:
if(!on)
GPIO_ResetBits(GPIOE,GPIO_Pin_4);
else
GPIO_SetBits(GPIOE,GPIO_Pin_4);
break;
case BLUE:
if(!on)
GPIO_ResetBits(GPIOE,GPIO_Pin_2);
else
GPIO_SetBits(GPIOE,GPIO_Pin_2);
break;
case 12:
if(!on)
GPIO_ResetBits(GPIOC,GPIO_Pin_1);
else
GPIO_SetBits(GPIOC,GPIO_Pin_1);
break;
}
#endif
}
u8 LED[3]={0};
void LEDRGB_COLOR(u8 color)
{
switch (color)
{
case RED:
LED[0]=1;
LED[1]=0;
LED[2]=0;
LEDRGB(RED,1);
LEDRGB(BLUE,0);
LEDRGB(GREEN,0);
break;
case BLUE:
LED[0]=0;
LED[1]=1;
LED[2]=0;	
LEDRGB(RED,0);
LEDRGB(BLUE,1);
LEDRGB(GREEN,0);
break;
case GREEN:
LED[0]=0;
LED[1]=0;
LED[2]=1;	
LEDRGB(RED,0);
LEDRGB(BLUE,0);
LEDRGB(GREEN,1);
break;
case WHITE:
LED[0]=1;
LED[1]=1;
LED[2]=1;
LEDRGB(RED,1);
LEDRGB(BLUE,1);
LEDRGB(GREEN,1);
break;
case BLACK:
LED[0]=0;
LED[1]=0;
LED[2]=0;	
LEDRGB(RED,0);
LEDRGB(BLUE,0);
LEDRGB(GREEN,0);
break;
case YELLOW:
LED[0]=1;
LED[1]=0;
LED[2]=1;	
LEDRGB(RED,1);
LEDRGB(BLUE,0);
LEDRGB(GREEN,1);
break;
}
}
#define IDLE 0
#define CAL_MPU 1
#define CAL_M  2
#include "circle.h"
void LEDRGB_STATE(void)
{
static u8 main_state;
static u8 mpu_state,m_state,idle_state;
static u16 cnt,cnt_idle;
u8 mode_control;
	
	
mode_control=mode.mode_fly;
switch(main_state)
{ 
	
	case IDLE:
	if(mpu6050.Gyro_CALIBRATE)
	{idle_state=0;main_state=CAL_MPU;

	}
	else if(Mag_CALIBRATED)
	{idle_state=0;main_state=CAL_M;

	}
	break;
	case CAL_MPU:
  break;
	case CAL_M:
  break;
}
//   | | | |    | | | |   | | | |   | | | |   | | | |
//    ARM          GPS1     GPS2      GPS3      MODE  
#define RGB_DELAY 3
static u8 cnt_gps;
static u8 flag_cnt_gps;
if(cnt_gps++>1){cnt_gps=0;
	flag_cnt_gps=!flag_cnt_gps;
}
if(state_set_point!=0)
{
idle_state=0;
switch(state_set_point)
{//ARM
	case 1:
		    if(flag_cnt_gps)
				LEDRGB_COLOR(BLACK);		
				else
				LEDRGB_COLOR(RED);
	break;
	case 3:
		 if(flag_cnt_gps)
				LEDRGB_COLOR(BLACK);		
				else
				LEDRGB_COLOR(BLUE);
	break;
	case 5:
		 if(flag_cnt_gps)
				LEDRGB_COLOR(BLACK);		
				else
				LEDRGB_COLOR(WHITE);
	break;
	default:
		LEDRGB_COLOR(BLACK);	
  break;
}

}
else {
main_state=0;
switch(idle_state)
{//ARM
	case 0:
		if(main_state==IDLE)
			{idle_state=1;cnt_idle=0;}
	break;
	case 1:
	 if(state_v==SG_LOW_CHECK)
				LEDRGB_COLOR(WHITE); 
		 else
				LEDRGB_COLOR(RED);
	if(cnt_idle++>0.1/0.05)
	{idle_state=2;cnt_idle=0;}
	break;
	case 2:
		LEDRGB_COLOR(BLACK);	
	if(cnt_idle++>0.1/0.05)
	{idle_state=3;cnt_idle=0;}
	break;
	
	case 3:
	if(circle.connect&&m100.mems_board_connect)
	LEDRGB_COLOR(WHITE); 
	else if(m100.mems_board_connect) 
	LEDRGB_COLOR(BLUE); 
	else
	LEDRGB_COLOR(RED);
	if(cnt_idle++>0.1/0.05)
	{idle_state=4;cnt_idle=0;}
	break;
	case 4:
		LEDRGB_COLOR(BLACK);	
	if(cnt_idle++>0.1/0.05)
	{idle_state=5;cnt_idle=0;}
	break;
	
	case 5:
	 if(m100_data_refresh&&!dji_rc_miss&&m100.GPS_STATUS>=1)
				LEDRGB_COLOR(WHITE); 
		 else if(m100.refresh&&!dji_rc_miss)
			  LEDRGB_COLOR(BLUE);
		 else
				LEDRGB_COLOR(RED);
	if(cnt_idle++>0.1/0.05)
	{idle_state=6;cnt_idle=0;}
	break;
	case 6:
		LEDRGB_COLOR(BLACK);	
	if(cnt_idle++>1.2/0.05)
	{idle_state=0;cnt_idle=0;}
	break;
}
}
}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

