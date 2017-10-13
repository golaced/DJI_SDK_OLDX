/*! @file main.cpp
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  An exmaple program of DJI-onboard-SDK portable for stm32
 *
 *  Copyright 2016 DJI. All right reserved.
 *
 *******************************************************************************
 *                                                                             *
 *          --------               --------                 --------           *
 *         |        |   USART2    |        |    USART3     |        |          *
 *         |   PC   | <---------> | stm32  |  <----------> |  M100  |          *
 *         |        | (USB-TTL)   |        |               |        |          *
 *         |        |             |        |               |        |          *
 *          --------               --------                 --------           *
 *                                                                             *
 *                                                                             *
 *******************************************************************************
 * */

#include "main.h"
#include "math.h"
#include "stm32f4xx.h"
#include "stm32f4xx_iwdg.h"
#undef USE_ENCRYPT
/*-----------------------DJI_LIB VARIABLE-----------------------------*/
using namespace DJI::onboardSDK;

HardDriver* driver = new STM32F4;
CoreAPI defaultAPI = CoreAPI(driver);
CoreAPI *coreApi = &defaultAPI;

Flight flight = Flight(coreApi);
FlightData flightData;

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//独立看门狗 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/6/12
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 

//初始化独立看门狗
//prer:分频数:0~7(只有低3位有效!)
//rlr:自动重装载值,0~0XFFF.
//分频因子=4*2^prer.但最大值只能是256!
//rlr:重装载寄存器值:低11位有效.
//时间计算(大概):Tout=((4*2^prer)*rlr)/32 (ms).
void IWDG_Init(u8 prer,u16 rlr)
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //使能对IWDG->PR IWDG->RLR的写
	
	IWDG_SetPrescaler(prer); //设置IWDG分频系数

	IWDG_SetReload(rlr);   //设置IWDG装载值

	IWDG_ReloadCounter(); //reload
	
	IWDG_Enable();       //使能看门狗
}

//喂独立看门狗
void IWDG_Feed(void)
{
	IWDG_ReloadCounter();//reload
}

#include "led.h"


void LEDRGB(u8 sel,u8 on)
{
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
GPIO_ResetBits(GPIOD,GPIO_Pin_15);
else
GPIO_SetBits(GPIOD,GPIO_Pin_15);
break;
}
}

void LEDRGB_COLOR(u8 color)
{
switch (color)
{
case RED:
LEDRGB(RED,1);
LEDRGB(BLUE,0);
LEDRGB(GREEN,0);
break;
case BLUE:
LEDRGB(RED,0);
LEDRGB(BLUE,1);
LEDRGB(GREEN,0);
break;
case GREEN:
LEDRGB(RED,0);
LEDRGB(BLUE,0);
LEDRGB(GREEN,1);
break;
case WHITE:
LEDRGB(RED,1);
LEDRGB(BLUE,1);
LEDRGB(GREEN,1);
break;
case BLACK:
LEDRGB(RED,0);
LEDRGB(BLUE,0);
LEDRGB(GREEN,0);
break;
case YELLOW:
LEDRGB(RED,1);
LEDRGB(BLUE,0);
LEDRGB(GREEN,1);
break;
}
}
void LED_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(ANO_RCC_LED,ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_Pin   = ANO_Pin_LED1| ANO_Pin_LED2| ANO_Pin_LED3;
	GPIO_Init(ANO_GPIO_LED, &GPIO_InitStructure);
	
	
	 //SEL
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_Pin   =GPIO_Pin_15 ;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	LEDRGB_COLOR(BLUE);
	delay_nms(500);
	LEDRGB_COLOR(RED);
	delay_nms(500);
	LEDRGB_COLOR(GREEN);
  delay_nms(500);

}
VirtualRC virtualrc = VirtualRC(coreApi);
VirtualRCData myVRCdata =
{ 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024,
    1024, 1024 };

extern TerminalCommand myTerminal;
extern LocalNavigationStatus droneState;
extern uint8_t myFreq[16];
u16 set1=10;
int main()
{static u16 cnt;
  BSPinit();
  delay_nms(30);
//  printf("This is the example App to test DJI onboard SDK on STM32F4Discovery Board! \r\n");
//  printf("Refer to \r\n");
//  printf("https://developer.dji.com/onboard-sdk/documentation/github-platform-docs/STM32/README.html \r\n");
//  printf("for supported commands!\r\n");
//  printf("Board Initialization Done!\r\n");
  //delay_nms(1000);

  //! Change the version string to your platform/version as defined in DJI_Version.h
  coreApi->setVersion(SDK_VERSION);
  delay_nms(50);
  //printf("API Version Set Done!\r\n");
  LED_Init();
  uint32_t runOnce = 1;
  uint32_t next500MilTick;
	IWDG_Init(4,1000); //与分频数为64,重载值为500,溢出时间为1s	
	
  while (1)
  {
    // One time automatic activation
    if (runOnce)
    {
      runOnce = 0;
      coreApi->setBroadcastFreq(myFreq);
      delay_nms(50);

      // The Local navigation example will run in broadcast call back function,
      // immediate after GPS position is updated
      coreApi->setBroadcastCallback(myRecvCallback,(DJI::UserData)(&droneState));

      User_Activate();      
      delay_nms(50);

      next500MilTick = driver->getTimeStamp() + 10;
			
    }

    if (driver->getTimeStamp() >= next500MilTick)
    {
      next500MilTick = driver->getTimeStamp() + 10;

      // Handle user commands from mobile device
      mobileCommandHandler(coreApi, &flight);

      // Handle user commands from serial (USART2)

			 myTerminal.terminalCommandHandler(coreApi, &flight);
//			if(!rst_board)//M100 conncet
//				IWDG_Feed();//喂狗
    }

		//#define   DEG2RAD 0.01745329252
     

		//if(cnt++>set1-1){cnt=0;
		
		
		
    coreApi->sendPoll();
  }
}

