/*! @file BspUsart.h
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Usart helper functions and ISR for board STM32F4Discovery
 *
 *  Copyright 2016 DJI. All right reserved.
 *
 * */

#ifndef _BSPUSART_H
#define _BSPUSART_H
#include <stm32f4xx.h>
#include "stdio.h"
void USARTxNVIC_Config(void);
void UsartConfig(void);
void NVIC_Config(void);
void Rx_buff_Handler() ;
void Ultra_Duty();
extern int Laser_distance,Laser_ST,ultra_distance;
#endif  //_USART_H
