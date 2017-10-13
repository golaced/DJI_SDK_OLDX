#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "beep.h"
#include "key.h"


//ALIENTEK ̽����STM32F407������ ʵ��4
//����ͨ��ʵ�� -�⺯���汾
//����֧�֣�www.openedv.com
//�Ա����̣�http://eboard.taobao.com
//������������ӿƼ����޹�˾  
//���ߣ�����ԭ�� @ALIENTEK


int main(void)
{ 
 
	u8 t;
	u8 len;	
	u16 times=0;  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);		//��ʱ��ʼ�� 
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

