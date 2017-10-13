#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "beep.h"
#include "key.h"


//ALIENTEK 探索者STM32F407开发板 实验4
//串口通信实验 -库函数版本
//技术支持：www.openedv.com
//淘宝店铺：http://eboard.taobao.com
//广州市星翼电子科技有限公司  
//作者：正点原子 @ALIENTEK


int main(void)
{ 
 
	u8 t;
	u8 len;	
	u16 times=0;  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);		//延时初始化 
	delay_ms(100);	
	Sonar_Init(9600);
	delay_ms(100);	
  Laser_Init(115200);//LINK
	delay_ms(100);	
	Usart2_Init(115200);
	delay_ms(100);	
	DJ_Init(115200);
	while(1)
	{
		
	Send_MEMS();	
	if(times++>5){times=0	;
	Ultra_Duty();	
	}
	delay_ms(10);	
	}
}

