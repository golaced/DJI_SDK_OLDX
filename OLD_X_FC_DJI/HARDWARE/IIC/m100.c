#include "include.h"
#include "circle.h"
#include "usart.h"


void m100_rst(u16 delay)
{
UsartSend_M100(0xFA);
UsartSend_M100(0xFB);
UsartSend_M100(0xF1);
UsartSend_M100(dji_rst);
UsartSend_M100(0xFE);	

delay_ms(delay);
}

void m100_disarm(u16 delay)
{
UsartSend_M100(0xFA);
UsartSend_M100(0xFB);
UsartSend_M100(0x03);
UsartSend_M100(0x00);
UsartSend_M100(0xFE);	

delay_ms(delay);
}

void m100_arm(u16 delay)
{
UsartSend_M100(0xFA);
UsartSend_M100(0xFB);
UsartSend_M100(0x03);
UsartSend_M100(0x01);
UsartSend_M100(0xFE);	

delay_ms(delay);
}

void m100_activate(u16 delay)
{
UsartSend_M100(0xFA);
UsartSend_M100(0xFB);
UsartSend_M100(0x01);
UsartSend_M100(0xFE);	

delay_ms(delay);
}

void m100_activate_long(u8 times)
{
	u8 i;
	for(i=0;i<times;i++)
	{
		m100_activate(20);
	}
	delay_ms(100);
}

void m100_obtain_control(u16 delay)
{
UsartSend_M100(0xFA);
UsartSend_M100(0xFB);
UsartSend_M100(0x02);
UsartSend_M100(0x01);
UsartSend_M100(0xFE);	

delay_ms(delay);
}

void m100_obtain_control_long(u8 times)
{
	u8 i;
	for(i=0;i<times;i++)
	{
		m100_obtain_control(20);
	}
		delay_ms(100);
}

void m100_dis_control(u16 delay)
{
UsartSend_M100(0xFA);
UsartSend_M100(0xFB);
UsartSend_M100(0x02);
UsartSend_M100(0x00);
UsartSend_M100(0xFE);	

delay_ms(delay);
}

void m100_rth(u16 delay)
{
UsartSend_M100(0xFA);
UsartSend_M100(0xFB);
UsartSend_M100(0x05);
UsartSend_M100(0x01);
UsartSend_M100(0xFE);	

delay_ms(delay);
}

void m100_take_off(u16 delay)
{
UsartSend_M100(0xFA);
UsartSend_M100(0xFB);
UsartSend_M100(0x05);
UsartSend_M100(0x02);
UsartSend_M100(0xFE);	

delay_ms(delay);
}

void m100_take_off_long(u8 times)
{
	u8 i;
	for(i=0;i<times;i++)
	{
		m100_take_off(20);
	}
	
		delay_ms(100);
}

void m100_land_control(u16 delay)
{
UsartSend_M100(0xFA);
UsartSend_M100(0xFB);
UsartSend_M100(0x05);
UsartSend_M100(0x03);
UsartSend_M100(0xFE);	

delay_ms(delay);
}

void m100_land_control_long(u8 times)
{
	u8 i;
	for(i=0;i<times;i++)
	{
		m100_land_control(20);
	}
	
		delay_ms(100);
}

	
void m100_dis_control_long(u8 times)
{
	u8 i;
	for(i=0;i<times;i++)
	{
		m100_dis_control(20);
	}
	
		delay_ms(100);
}

void m100_data(u16 delay)
{
UsartSend_M100(0xFA);
UsartSend_M100(0xFB);
UsartSend_M100(0x08);
UsartSend_M100(0x00);
UsartSend_M100(0xFE);	

delay_ms(delay);
}

void m100_vRc_on_A(u16 delay)
{
UsartSend_M100(0xFA);
UsartSend_M100(0xFB);
UsartSend_M100(0x06);
UsartSend_M100(0x01);
UsartSend_M100(0xFE);	

delay_ms(delay);
}

void m100_vRc_on_F(u16 delay)
{
UsartSend_M100(0xFA);
UsartSend_M100(0xFB);
UsartSend_M100(0x06);
UsartSend_M100(0x02);
UsartSend_M100(0xFE);	

delay_ms(delay);
}

void m100_vRc_off(u16 delay)
{
UsartSend_M100(0xFA);
UsartSend_M100(0xFB);
UsartSend_M100(0x06);
UsartSend_M100(0x00);
UsartSend_M100(0xFE);	

delay_ms(delay);
}
#include "filter.h"

float m100_x_g = 0;
float m100_y_g = 0;
float m100_z_g = 0;
float m100_yaw_g = 0;

void m100_contrl(float x,float y,float z,float yaw,u8 mode)
{
UsartSend_M100(0xFA);
UsartSend_M100(0xFB);
UsartSend_M100(0x04);
UsartSend_M100(0x01);

UsartSend_M100(mode);//01001010 flag???
//	UsartSend_M100(0x4A);
	vs16 _temp;
	
	m100_x_g = x;
	m100_y_g = y;
	m100_z_g = z;
	m100_yaw_g = yaw;
	
	float temp1 = Moving_Median(24,3, my_deathzoom_2(x-1500, 5));
	_temp = (vs16)(temp1)/100.0*3*100/5;//x;
	UsartSend_M100(BYTE1(_temp));//x
	UsartSend_M100(BYTE0(_temp));//
	
	temp1 =  Moving_Median(25,3,my_deathzoom_2(y-1500, 5));
	_temp = (vs16)(temp1)/100.0*3*100/5;
	UsartSend_M100(BYTE1(_temp));//y
	UsartSend_M100(BYTE0(_temp));
	
	temp1 =  Moving_Median(26,3,my_deathzoom_2(z-1500, 5));
	_temp = (vs16)(temp1)/100.0*3*100/5;
	UsartSend_M100(BYTE1(_temp));//z
	UsartSend_M100(BYTE0(_temp));
	
	temp1 =  Moving_Median(27,3,my_deathzoom_2(yaw-1500, 5));
	_temp = (vs16)(temp1)/100.0*3*100/2;
	UsartSend_M100(BYTE1(_temp));//yaw
	UsartSend_M100(BYTE0(_temp));

//	UsartSend_M100(0x00);
//	UsartSend_M100(0x64);
//	UsartSend_M100(0x00);UsartSend_M100(0xC8);UsartSend_M100(0x00);UsartSend_M100(0x00);UsartSend_M100(0x00);UsartSend_M100(0x00);
	
	
UsartSend_M100(0xFE);	

}

static void Send_Data_M100(u8 *dataToSend , u8 length)
{
u16 i;
  for(i=0;i<length;i++)
     UsartSend_M100(dataToSend[i]);
}


void px4_control_publish(float x,float y,float z,float yaw,u8 mode)
{
  u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
  data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x04;//功能字
	data_to_send[_cnt++]=0;//数据量
	
	_temp = mode;
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(x*1000);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(y*1000);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(z*1000);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	Send_Data_M100(data_to_send, _cnt);
}
