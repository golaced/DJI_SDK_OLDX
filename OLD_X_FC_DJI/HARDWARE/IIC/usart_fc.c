
#include "include.h"
#include "usart_fc.h"
#include "ultrasonic.h"
#include "hml5833l.h"
#include "mpu6050.h"
#include "ms5611.h"
#include "rc.h"
#include "att.h"
#include "height_ctrl.h"
#include "alt_kf.h"
#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"
#include "flow.h"
#include "gps.h"
#include "circle.h"
#include "data_transfer.h"
#include "led_fc.h"
void DATA_M100(u8 *data_buf,u8 num);
void Data_Receive_Anl3(u8 *data_buf,u8 num);
u8 Rx_Buf3[256];	//串口接收缓存
u8 RxBuffer3[50];
u8 RxState3 = 0;
u8 RxBufferNum3 = 0;
u8 RxBufferCnt3 = 0;
u8 RxLen3 = 0;
static u8 _data_len3 = 0,_data_cnt3 = 0;

void Usart2_Init(u32 br_num)//--GOL-link
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
	
	//配置PD5作为USART2　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
	//配置PD6作为USART2　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 

	

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = br_num;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART2, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART2, ENABLE);  //使能串口1 
	
	USART_ClearFlag(USART2, USART_FLAG_TC);
	

	//使能USART2接收中断
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	//使能USART2
	USART_Cmd(USART2, ENABLE); 
}
RESULT color;
CIRCLE qr;
float dt_flow_rx;
u16 data_rate_gol_link;
PID_STA HPID,SPID,FIX_PID,NAV_PID;
struct _PID_SET pid;
RC_GETDATA Rc_Get;
struct _plane plane;
struct _slam slam;
struct _IMU_NAV imu_nav;
_MODE mode;
u8 LOCK, KEY[8],KEY_SEL[4];

u8 NAV_BOARD_CONNECT=0;
 void Data_Receive_Anl(u8 *data_buf,u8 num)
{
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
  if(*(data_buf+2)==0x86)//FLOW_MINE_frame
  {
	 dt_flow_rx= Get_Cycle_T(GET_T_FLOW_UART);	
   NAV_BOARD_CONNECT=1;
	 imu_nav.flow.speed.west = -(float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5));///10.;//UKF
	 imu_nav.flow.speed.east = -(float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7));///10.; 
	 imu_nav.flow.speed.x = -(float)((int16_t)(*(data_buf+8)<<8)|*(data_buf+9))/100.;//Origin
	 imu_nav.flow.speed.y = -(float)((int16_t)(*(data_buf+10)<<8)|*(data_buf+11))/100.;
	 imu_nav.flow.speed_h.x = (float)((int16_t)(*(data_buf+12)<<8)|*(data_buf+13))/100.;//Origin
	 imu_nav.flow.speed_h.y = (float)((int16_t)(*(data_buf+14)<<8)|*(data_buf+15))/100.;
   //imu_nav.flow.position.flow_qual = ((int16_t)(*(data_buf+12)<<8)|*(data_buf+13));
		
	 color.x = ((int16_t)(*(data_buf+14)<<8)|*(data_buf+15));
	 color.y = ((int16_t)(*(data_buf+16)<<8)|*(data_buf+17));
	 color.h = ((int16_t)(*(data_buf+18)<<8)|*(data_buf+19));
   color.check = *(data_buf+20);
	 imu_nav.flow.position.flow_qual = *(data_buf+21);
		

	}
  else if(*(data_buf+2)==0x01)//FLOW_MINE_frame
  {
	 dt_flow_rx= Get_Cycle_T(GET_T_FLOW_UART);	
   NAV_BOARD_CONNECT=1;
	 imu_nav.flow.speed.west = (float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5))/1000;///10.;
	 imu_nav.flow.speed.east = (float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7))/1000;///10.; 
	 imu_nav.flow.speed.x = (float)((int16_t)(*(data_buf+8)<<8)|*(data_buf+9))/1000;
	 imu_nav.flow.speed.y = (float)((int16_t)(*(data_buf+10)<<8)|*(data_buf+11))/1000;


	}			
}
 u8 SONAR_HEAD_CHECK[2];
u32 imu_loss_cnt;
u16 S_head;
float Pitch_DJ,Roll_DJ;
float ALT_POS_SONAR_HEAD,ALT_POS_SONAR_HEAD_LASER_SCANER;

u8 laser_sel=1;

u8 TxBuffer[256];
u8 TxCounter=0;
u8 count=0; 

u8 Rx_Buf[256];	//串口接收缓存
u8 RxBuffer[50];
u8 RxState = 0;
u8 RxBufferNum = 0;
u8 RxBufferCnt = 0;
u8 RxLen = 0;
u8 com_data ;
static u8 _data_len = 0,_data_cnt = 0;
void USART2_IRQHandler(void)
{ OSIntEnter(); 
	u8 com_data;
	
	if(USART2->SR & USART_SR_ORE)//ORE中断
	{
		com_data = USART2->DR;
	}

  //接收中断
	if( USART_GetITStatus(USART2,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);//清除中断标志

		com_data = USART2->DR;
				if(RxState==0&&com_data==0xAA)
		{
			RxState=1;
			RxBuffer[0]=com_data;
		}
		else if(RxState==1&&com_data==0xAF)
		{
			RxState=2;
			RxBuffer[1]=com_data;
		}
		else if(RxState==2&&com_data>0&&com_data<0XF1)
		{
			RxState=3;
			RxBuffer[2]=com_data;
		}
		else if(RxState==3&&com_data<50)
		{
			RxState = 4;
			RxBuffer[3]=com_data;
			_data_len = com_data;
			_data_cnt = 0;
		}
		else if(RxState==4&&_data_len>0)
		{
			_data_len--;
			RxBuffer[4+_data_cnt++]=com_data;
			if(_data_len==0)
				RxState = 5;
		}
		else if(RxState==5)
		{
			RxState = 0;
			RxBuffer[4+_data_cnt]=com_data;
			//Data_IMU(RxBuffer,_data_cnt+5);
			DATA_M100(RxBuffer,_data_cnt+5);
		}
		else
			RxState = 0;
	
		//ANO_DT_Data_Receive_Prepare(com_data);
	}
	//发送（进入移位）中断
	if( USART_GetITStatus(USART2,USART_IT_TXE ) )
	{
				
		USART2->DR = TxBuffer[TxCounter++]; //写DR清除中断标志          
		if(TxCounter == count)
		{
			USART2->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}
	}

   OSIntExit(); 

}

void UsartSend_GOL_LINK(uint8_t ch)
{


	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
USART_SendData(USART2, ch); 
}

static void Send_Data_GOL_LINK(u8 *dataToSend , u8 length)
{
u16 i;
  for(i=0;i<length;i++)
     UsartSend_GOL_LINK(dataToSend[i]);
}


//------------------------------------------------------GOL_LINK----------------------------------------------------

void Uart5_Init(u32 br_num)//-----video sonar F
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);
	
	//配置PC12作为UART5　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
	//配置PD2作为UART5　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
	
	//配置UART5
	//中断被屏蔽了
	USART_InitStructure.USART_BaudRate = br_num;       //波特率可以通过地面站配置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
	USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
	USART_Init(UART5, &USART_InitStructure);
	


	//使能UART5接收中断
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
	//使能USART5
	USART_Cmd(UART5, ENABLE); 
}
u8 Tx5Buffer[256];
u8 Tx5Counter=0;
u8 count5=0; 

u8 rx_uart5[20];
float Rol_yun,Pit_yun,Yaw_yun;
float Rol_yun_rate,Pit_yun_rate,Yaw_yun_rate;
void UART5_IRQHandler(void)
{ OSIntEnter(); 
	u8 com_data;
	u8 sum;
	static u8 state,cnt;
	if(UART5->SR & USART_SR_ORE)//ORE中断
	{
		com_data = UART5->DR;
	}

  //接收中断
	if( USART_GetITStatus(UART5,USART_IT_RXNE) )
	{
		
		USART_ClearITPendingBit(UART5,USART_IT_RXNE);//清除中断标志
		com_data = UART5->DR;
		//Ultra_Get(com_data);
				if(RxState3==0&&com_data==0xAA)
		{
			RxState3=1;
			RxBuffer3[0]=com_data;
		}
		else if(RxState3==1&&com_data==0xAF)
		{
			RxState3=2;
			RxBuffer3[1]=com_data;
		}
		else if(RxState3==2&&com_data>0&&com_data<0XF1)
		{
			RxState3=3;
			RxBuffer3[2]=com_data;
		}
		else if(RxState3==3&&com_data<50)
		{
			RxState3 = 4;
			RxBuffer3[3]=com_data;
			_data_len3 = com_data;
			_data_cnt3 = 0;
		}
		else if(RxState3==4&&_data_len3>0)
		{
			_data_len3--;
			RxBuffer3[4+_data_cnt3++]=com_data;
			if(_data_len3==0)
				RxState3 = 5;
		}
		else if(RxState3==5)
		{
			RxState3 = 0;
			RxBuffer3[4+_data_cnt3]=com_data;
			Data_Receive_Anl3(RxBuffer3,_data_cnt3+5);
		}
		else
			RxState3 = 0;
		
	
		
	}
	//发送（进入移位）中断
	if( USART_GetITStatus(UART5,USART_IT_TXE ) )
	{
				
		UART5->DR = TxBuffer[TxCounter++]; //写DR清除中断标志          
		if(TxCounter == count)
		{
			UART5->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}


		USART_ClearITPendingBit(UART5,USART_IT_TXE);
	}
  
   OSIntExit(); 

}

void Uart5_Send(unsigned char *DataToSend ,u8 data_num)
{

while(USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);
USART_SendData(UART5, data_num); ;//USART1, ch); 

}

void Usart1_Init(u32 br_num)//-------UPload_board1  ???????
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //??USART2??
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);	
	
//	//???????
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	
	//??PD5??USART2 Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
	//??PD6??USART2 Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 

   //USART1 ?????
	USART_InitStructure.USART_BaudRate = br_num;//?????
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//???8?????
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//?????
	USART_InitStructure.USART_Parity = USART_Parity_No;//??????
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//????????
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//????
  USART_Init(USART1, &USART_InitStructure); //?????1
	
  USART_Cmd(USART1, ENABLE);  //????1 
	
	USART_ClearFlag(USART1, USART_FLAG_TC);

	//??USART2????
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	//??USART2
	USART_Cmd(USART1, ENABLE); 
	//Send_Data_UP_LINK(BLE_UP1,20);

}


void Usart4_Init(u32 br_num)//-------SD_board
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);
	
	//配置PD5作为USART2　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
	//配置PD6作为USART2　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 

	

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = br_num;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(UART4, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(UART4, ENABLE);  //使能串口1 
	
	USART_ClearFlag(UART4, USART_FLAG_TC);
	

	//使能USART2接收中断
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
	//使能USART2
	USART_Cmd(UART4, ENABLE); 
}



#include "gps.h"

nmea_msg gpsx,gps_data; 		
u16 cnt_m100_data_refresh;
u8 m100_data_refresh;
 void DATA_M100(u8 *data_buf,u8 num)
{double zen,xiao;
 static u8 flag;
	vs16 rc_value_temp;
	static float m100_hr,m100_attr[3];
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
	#if USE_PX4
  if(*(data_buf+2)==0x04)//Px4
  { dji_miss_cnt=0;
		DJI_CONNECT=1;
		flag=!flag;
		LEDRGB(12,flag);
		m100.rx_dt=Get_Cycle_T(GET_T_M100); 	
	  m100.Pit=(float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5))/10.;
		m100.Rol=(float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7))/10.;
		m100.Yaw=(float)((int16_t)(*(data_buf+8)<<8)|*(data_buf+9))/10.;
		m100.H=(float)((int16_t)(*(data_buf+10)<<8)|*(data_buf+11))/1000.;
		
		if(m100.H!=m100_hr||m100_attr[0]!=m100.Pit||m100_attr[1]!=m100.Rol||m100_attr[2]!=m100.Yaw)
		{cnt_m100_data_refresh=0;
		 m100_data_refresh=1;
		}
		m100.refresh=m100_data_refresh;
		m100_hr=m100.H;
		m100_attr[0]=m100.Pit;
		m100_attr[1]=m100.Rol;
		m100_attr[2]=m100.Yaw;
		
		ultra_distance=(float)((int16_t)(*(data_buf+12)<<8)|*(data_buf+13));
		zen=(*(data_buf+14)<<8)|*(data_buf+15);
		xiao=(double)((u32)(*(data_buf+16)<<24)|(*(data_buf+17)<<16)|(*(data_buf+18)<<8)|*(data_buf+19))/1000000000.;
		m100.Lat=zen+xiao;
		zen=(*(data_buf+20)<<8)|*(data_buf+21);
		xiao=(double)((u32)(*(data_buf+22)<<24)|(*(data_buf+23)<<16)|(*(data_buf+24)<<8)|*(data_buf+25))/1000000000.;
		m100.Lon=zen+xiao;
		
		m100.Bat=(float)((int16_t)(*(data_buf+26)<<8)|*(data_buf+27))/100.;
		m100.Rc_pit=(float)((int16_t)(*(data_buf+28)<<8)|*(data_buf+29));
		m100.Rc_rol=(float)((int16_t)(*(data_buf+30)<<8)|*(data_buf+31));
		m100.Rc_yaw=(float)((int16_t)(*(data_buf+32)<<8)|*(data_buf+33));
		m100.Rc_thr=(float)((int16_t)(*(data_buf+34)<<8)|*(data_buf+35));
		m100.Rc_mode=(float)((int16_t)(*(data_buf+36)<<8)|*(data_buf+37));
		m100.Rc_gear=(float)((int16_t)(*(data_buf+38)<<8)|*(data_buf+39));
		m100.STATUS=*(data_buf+40);		
		m100.GPS_STATUS=*(data_buf+41);
		m100.spd[0]=(float)((int16_t)(*(data_buf+42)<<8)|*(data_buf+43))/1000.;
		m100.spd[1]=(float)((int16_t)(*(data_buf+44)<<8)|*(data_buf+45))/1000.;
		m100.H_Spd=m100.spd[2]=(float)((int16_t)(*(data_buf+46)<<8)|*(data_buf+47))/1000.;
	}else if(*(data_buf+2)==0x05)
	{
	if(*(data_buf+4)==66)
  {circle.check=qr.check=track.connect=qr.connect=circle.connect=0;}
  else{	qr.connect=circle.connect=1;
	track.check=circle.check=qr.check=*(data_buf+4);	
	}
	qr.x=(float)((int16_t)(*(data_buf+5)<<8)|*(data_buf+6));
	qr.y=(float)((int16_t)(*(data_buf+7)<<8)|*(data_buf+8));
	qr.z=(float)((int16_t)(*(data_buf+9)<<8)|*(data_buf+10));
	circle.x=(float)((int16_t)(*(data_buf+11)<<8)|*(data_buf+12));
	circle.y=(float)((int16_t)(*(data_buf+13)<<8)|*(data_buf+14));
	qr.center_x=(float)((int16_t)(*(data_buf+15)<<8)|*(data_buf+16));
	qr.center_y=(float)((int16_t)(*(data_buf+17)<<8)|*(data_buf+18));
	qr.yaw=(float)((int16_t)(*(data_buf+19)<<8)|*(data_buf+20));
	}
	#else//M100
	if(*(data_buf+2)==0x01)//
  { dji_miss_cnt=0;
		DJI_CONNECT=1;
		flag=!flag;
		LEDRGB(12,flag);
	  m100.Pit=(float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5))/10.;
		m100.Rol=(float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7))/10.;
		m100.Yaw=-1*(float)((int16_t)(*(data_buf+8)<<8)|*(data_buf+9))/10.;
		m100.H=(float)((int16_t)(*(data_buf+10)<<8)|*(data_buf+11))/1000.;
		
		if(m100.H!=m100_hr||m100_attr[0]!=m100.Pit||m100_attr[1]!=m100.Rol||m100_attr[2]!=m100.Yaw)
		{cnt_m100_data_refresh=0;
		 m100_data_refresh=1;
		}
		m100.refresh=m100_data_refresh;
		m100_hr=m100.H;
		m100_attr[0]=m100.Pit;
		m100_attr[1]=m100.Rol;
		m100_attr[2]=m100.Yaw;
		
		m100.H_Spd=(float)((int16_t)(*(data_buf+12)<<8)|*(data_buf+13))/1000.;
		zen=(*(data_buf+14)<<8)|*(data_buf+15);
		xiao=(double)((u32)(*(data_buf+16)<<24)|(*(data_buf+17)<<16)|(*(data_buf+18)<<8)|*(data_buf+19))/1000000000.;
		m100.Lat=zen+xiao;
		zen=(*(data_buf+20)<<8)|*(data_buf+21);
		xiao=(double)((u32)(*(data_buf+22)<<24)|(*(data_buf+23)<<16)|(*(data_buf+24)<<8)|*(data_buf+25))/1000000000.;
		m100.Lon=zen+xiao;
		
		m100.Bat=(float)((int16_t)(*(data_buf+26)<<8)|*(data_buf+27))/100.;
		m100.Rc_pit=(float)((int16_t)(*(data_buf+28)<<8)|*(data_buf+29));
		m100.Rc_rol=(float)((int16_t)(*(data_buf+30)<<8)|*(data_buf+31));
		m100.Rc_yaw=(float)((int16_t)(*(data_buf+32)<<8)|*(data_buf+33));
		m100.Rc_thr=(float)((int16_t)(*(data_buf+34)<<8)|*(data_buf+35));
		m100.Rc_mode=(float)((int16_t)(*(data_buf+36)<<8)|*(data_buf+37));
		m100.Rc_gear=(float)((int16_t)(*(data_buf+38)<<8)|*(data_buf+39));
		m100.STATUS=*(data_buf+40);		
		m100.GPS_STATUS=*(data_buf+41);
		m100.spd[0]=(float)((int16_t)(*(data_buf+42)<<8)|*(data_buf+43))/1000.;
		m100.spd[1]=(float)((int16_t)(*(data_buf+44)<<8)|*(data_buf+45))/1000.;
		m100.spd[2]=(float)((int16_t)(*(data_buf+46)<<8)|*(data_buf+47))/1000.;
	} 
  #endif	
}

//  NRF board ??
float time_rc;
u8 rc_board_connect=1;
u16 rc_board_connect_lose_cnt=0;
RC_GETDATA Rc_Get_PWM,Rc_Get_SBUS;
void Data_Receive_Anl4(u8 *data_buf,u8 num)
{
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))	{Rc_Get_PWM.Heart_error++;	return;}		//??sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))	{Rc_Get_PWM.Heart_error++;	return;}			//????
	if(Rc_Get_PWM.update&&Rc_Get_PWM.THROTTLE>600)
	Feed_Rc_Dog(2);//???????

  if(*(data_buf+2)==0x03)//RC_PWM
  { rc_board_connect=1;
		rc_board_connect_lose_cnt=0;
		Rc_Get_PWM.PITCH1=((int16_t)(*(data_buf+4)<<8)|*(data_buf+5));
		Rc_Get_PWM.ROLL1=((int16_t)(*(data_buf+6)<<8)|*(data_buf+7));
		Rc_Get_PWM.THROTTLE=((int16_t)(*(data_buf+8)<<8)|*(data_buf+9));
		Rc_Get_PWM.YAW1=((int16_t)(*(data_buf+10)<<8)|*(data_buf+11));
		
		Rc_Get_PWM.AUX1=((int16_t)(*(data_buf+12)<<8)|*(data_buf+13));
		Rc_Get_PWM.AUX2=((int16_t)(*(data_buf+14)<<8)|*(data_buf+15));
		Rc_Get_PWM.AUX3=((int16_t)(*(data_buf+16)<<8)|*(data_buf+17));
		Rc_Get_PWM.AUX4=((int16_t)(*(data_buf+18)<<8)|*(data_buf+19));
		Rc_Get_PWM.AUX5=((int16_t)(*(data_buf+20)<<8)|*(data_buf+21));
		Rc_Get_PWM.update=*(data_buf+22);
		Rc_Get_PWM.POS_MODE=Rc_Get_PWM.AUX3;
		Rc_Get_PWM.HEIGHT_MODE=Rc_Get_PWM.AUX4;
		Rc_Get_PWM.RST=Rc_Get_PWM.AUX2;
		Rc_Get_PWM.Heart=*(data_buf+23);	
		Rc_Get_PWM.Heart_rx++;
	}	
}
u8 TxBuffer4[256];
u8 TxCounter4=0;
u8 count4=0; 

u8 Rx_Buf4[256];	//串口接收缓存
u8 RxBuffer4[50];
u8 RxState4 = 0;
u8 RxBufferNum4 = 0;
u8 RxBufferCnt4 = 0;
u8 RxLen4= 0;
static u8 _data_len4 = 0,_data_cnt4 = 0;
void UART4_IRQHandler(void)
{ OSIntEnter(); 
	u8 com_data;
	
	if(UART4->SR & USART_SR_ORE)//ORE中断
	{
		com_data = UART4->DR;
	}

  //接收中断
	if( USART_GetITStatus(UART4,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(UART4,USART_IT_RXNE);//清除中断标志

		com_data = UART4->DR;
			if(RxState4==0&&com_data==0xAA)
		{
			RxState4=1;
			RxBuffer4[0]=com_data;
		}
		else if(RxState4==1&&com_data==0xAF)
		{
			RxState4=2;
			RxBuffer4[1]=com_data;
		}
		else if(RxState4==2&&com_data>0&&com_data<0XF1)
		{
			RxState4=3;
			RxBuffer4[2]=com_data;
		}
		else if(RxState4==3&&com_data<50)
		{
			RxState4 = 4;
			RxBuffer4[3]=com_data;
			_data_len4 = com_data;
			_data_cnt4 = 0;
		}
		else if(RxState4==4&&_data_len4>0)
		{
			_data_len4--;
			RxBuffer4[4+_data_cnt4++]=com_data;
			if(_data_len4==0)
				RxState4 = 5;
		}
		else if(RxState4==5)
		{
			RxState4 = 0;
			RxBuffer4[4+_data_cnt4]=com_data;
			Data_Receive_Anl4(RxBuffer4,_data_cnt4+5);
		}
		else
			RxState4 = 0;
		
	}
	//发送（进入移位）中断
	if( USART_GetITStatus(UART4,USART_IT_TXE ) )
	{
				
		UART4->DR = TxBuffer[TxCounter++]; //写DR清除中断标志          
		if(TxCounter == count)
		{
			UART4->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}
	}
  
   OSIntExit(); 

}

void UsartSend_M100(uint8_t ch)
{

while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
USART_SendData(USART2, ch);  
}


void UsartSend_SD(uint8_t ch)
{

while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
USART_SendData(USART2, ch); ;//USART1, ch); 
}

static void Send_Data_SD(u8 *dataToSend , u8 length)
{
u16 i;
  for(i=0;i<length;i++)
     UsartSend_SD(dataToSend[i]);
}

void UART2_Put_Char(unsigned char DataToSend)
{

while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
USART_SendData(USART1, DataToSend); 

}

void Usart3_Init(u32 br_num)//-------GPS_board
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
	
	//配置PD5作为USART2　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
	//配置PD6作为USART2　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 

	
   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = br_num;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART3, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART3, ENABLE);  //使能串口1 
	
	USART_ClearFlag(USART3, USART_FLAG_TC);
	

	//使能USART2接收中断
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	//使能USART2
	USART_Cmd(USART3, ENABLE); 
}
#include "circle.h"
#include "filter.h"
u16 laser_o[5];
float k_laser=0.8;
float rate_gps_board;
 void Data_Receive_Anl3(u8 *data_buf,u8 num)
{
	double zen,xiao;
	static float m100_h_off;
	static float m100_hr,m100_attr[3]; 
	vs16 rc_value_temp,rc_value_temp1;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
	if(*(data_buf+2)==0x02)//SLAM_frame
  {
	circle.connect=1;
	circle.lose_cnt=0;
	circle.check=(*(data_buf+4));///10.;
  rc_value_temp=((int16_t)(*(data_buf+5)<<8)|*(data_buf+6));
	if(rc_value_temp<320)
	circle.x=rc_value_temp;//Moving_Median(16,3,rc_value_temp);
	circle.y=(int16_t)(*(data_buf+7));//Moving_Median(17,3,((int16_t)(*(data_buf+7))));
	circle.control[0]=(int8_t)(*(data_buf+8));
	circle.control[1]=(int8_t)(*(data_buf+9));
	circle.r=(int8_t)(*(data_buf+10));
	track.check=(int8_t)(*(data_buf+11));
	}	
  else if(*(data_buf+2)==0x03)//Num
  {
	circle.connect=1;
	circle.lose_cnt=0;
	track.check=circle.check=(*(data_buf+4));///10.;
	rc_value_temp=((int16_t)(*(data_buf+5)<<8)|*(data_buf+6));
	if(rc_value_temp<320)
	circle.x=rc_value_temp;//Moving_Median(16,3,rc_value_temp);
	circle.y=(int16_t)(*(data_buf+7));//Moving_Median(17,3,((int16_t)(*(data_buf+7))));
	circle.control[0]=(int8_t)(*(data_buf+8));
	circle.control[1]=(int8_t)(*(data_buf+9));
	circle.r=(int8_t)(*(data_buf+10));
	}			
	else if(*(data_buf+2)==0x04)//Mouse
  {
	mouse.connect=1;
	mouse.lose_cnt=0;
	mouse.check=(*(data_buf+4));///10.;
	rc_value_temp1=((int16_t)(*(data_buf+5)<<8)|*(data_buf+6));
	if(rc_value_temp<320)
	mouse.x=rc_value_temp1;//Moving_Median(16,3,rc_value_temp);
	mouse.y=(int16_t)(*(data_buf+7));//Moving_Median(17,3,((int16_t)(*(data_buf+7))));
	mouse.control[0]=(int8_t)(*(data_buf+8));
	mouse.control[1]=(int8_t)(*(data_buf+9));
	mouse.r=(int8_t)(*(data_buf+10));
	//track.check=(int8_t)(*(data_buf+11));
	}		
		else if(*(data_buf+2)==0x05)//TAR
	{			
		if(tar_need_to_check_odroid[0]&&state_v==SU_CHECK_TAR)		
		tar_need_to_check_odroid[1]=*(data_buf+5);
		else
		tar_need_to_check_odroid[1]=66;
		
		tar_need_to_check_odroid[0]=*(data_buf+4);
	
		
  }
		else if(*(data_buf+2)==0x06)//MAP
		{	
		for(i=0;i<6;i++){
		circle.map[i][0]=(*(data_buf+4+i*4));//check  N
		circle.map[i][1]=((int16_t)(*(data_buf+5+i*4)<<8)|*(data_buf+6+i*4));//x
		circle.map[i][2]=(int16_t)(*(data_buf+7+i*4));//y
		//circle.map[i][3]=(int16_t)(*(data_buf+8+i*4));//r
		}
		}
	/*else if(*(data_buf+2)==0x21)//Qr land
  {
	qr.connect=circle.connect=1;
	qr.lose_cnt=0;
	track.check=circle.check=qr.check=*(data_buf+4);	
	qr.x=(int16_t)(*(data_buf+5)<<8)|*(data_buf+6);
	qr.y=(int16_t)(*(data_buf+7)<<8)|*(data_buf+8);
	qr.z=(int16_t)(*(data_buf+9)<<8)|*(data_buf+10);
	qr.pit=(int16_t)(*(data_buf+11)<<8)|*(data_buf+12);
	qr.rol=(int16_t)(*(data_buf+13)<<8)|*(data_buf+14);
	qr.yaw=(int16_t)(*(data_buf+15)<<8)|*(data_buf+16);
	
	if(qr.check){
		int temp=((int16_t)(*(data_buf+17)<<8)|*(data_buf+18));
		if(ABS(temp)<1000)
		circle.x=qr.pix_x=temp;
		temp=((int16_t)(*(data_buf+19)<<8)|*(data_buf+20));
		if(ABS(temp)<1000)
		circle.y=qr.pix_y=temp;	
	}
	qr.center_x=-((int16_t)(*(data_buf+25)<<8)|*(data_buf+26));	
	qr.center_y=-((int16_t)(*(data_buf+27)<<8)|*(data_buf+28));
	}*/
}
 

void USART1_IRQHandler(void)
{ OSIntEnter(); 
	u8 com_data;
	if(USART1->SR & USART_SR_ORE)//ORE??
	{
		com_data = USART1->DR;
	}

  //????
	if( USART_GetITStatus(USART1,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);//??????
   
		com_data = USART1->DR;
		ANO_DT_Data_Receive_Prepare(com_data);
	}
	//??(????)??
	if( USART_GetITStatus(USART1,USART_IT_TXE ) )
	{
				
		USART_ClearITPendingBit(USART1,USART_IT_TXE);
	}

  OSIntExit(); 

}

void CharToLong(char Dest[],char Source[])
{
	 *Dest 		= Source[3];
	 *(Dest+1) 	= Source[2];
	 *(Dest+2) 	= Source[1];
	 *(Dest+3) 	= Source[0];
}

struct STime		stcTime={0};
struct SAcc 		stcAcc={0};
struct SGyro 		stcGyro={0};
struct SAngle 		stcAngle={0};
struct SMag 		stcMag={0};
struct SDStatus 	stcDStatus={0};
struct SPress 		stcPress={0};
struct SLonLat 		stcLonLat={0};
struct SGPSV 		stcGPSV={0};

void CopeSerialData(unsigned char ucData)
{
	static unsigned char ucRxBuffer[12];
	static unsigned char ucRxCnt = 0;	
	
	ucRxBuffer[ucRxCnt++]=ucData;
	if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt=0;
		return;																																	  
	}
	if (ucRxCnt<11) {return;}//数据不满11个，则返回
	else
	{
		switch(ucRxBuffer[1])
		{
			case 0x50: stcTime.ucYear 		= ucRxBuffer[2];
						stcTime.ucMonth 	= ucRxBuffer[3];
						stcTime.ucDay 		= ucRxBuffer[4];
						stcTime.ucHour 		= ucRxBuffer[5];
						stcTime.ucMinute 	= ucRxBuffer[6];
						stcTime.ucSecond 	= ucRxBuffer[7];
						stcTime.usMiliSecond=((unsigned short)ucRxBuffer[9]<<8)|ucRxBuffer[8];
						break;
			case 0x51:	stcAcc.a[0] = ((unsigned short)ucRxBuffer[3]<<8)|ucRxBuffer[2];
						stcAcc.a[1] = ((unsigned short)ucRxBuffer[5]<<8)|ucRxBuffer[4];
						stcAcc.a[2] = ((unsigned short)ucRxBuffer[7]<<8)|ucRxBuffer[6];
						break;
			case 0x52:	stcGyro.w[0] = ((unsigned short)ucRxBuffer[3]<<8)|ucRxBuffer[2];
						stcGyro.w[1] = ((unsigned short)ucRxBuffer[5]<<8)|ucRxBuffer[4];
						stcGyro.w[2] = ((unsigned short)ucRxBuffer[7]<<8)|ucRxBuffer[6];
			      Rol_yun_rate=(float)(stcGyro.w[1])/32768*2000;
				    Pit_yun_rate=(float)(stcGyro.w[0])/32768*2000;
					  Yaw_yun_rate=(float)(stcGyro.w[2])/32768*2000;
						break;
			case 0x53:	stcAngle.Angle[0] = ((unsigned short)ucRxBuffer[3]<<8)|ucRxBuffer[2];
						stcAngle.Angle[1] = ((unsigned short)ucRxBuffer[5]<<8)|ucRxBuffer[4];
						stcAngle.Angle[2] = ((unsigned short)ucRxBuffer[7]<<8)|ucRxBuffer[6];
						stcAngle.T = ((unsigned short)ucRxBuffer[9]<<8)|ucRxBuffer[8];
			      Rol_yun=(float)(stcAngle.Angle[1] )/32768*180;
				    Pit_yun=(float)(stcAngle.Angle[0] )/32768*180;
					  Yaw_yun=(float)(stcAngle.Angle[2] )/32768*180;
						break;
			case 0x54:	stcMag.h[0] = ((unsigned short)ucRxBuffer[3]<<8)|ucRxBuffer[2];
						stcMag.h[1] = ((unsigned short)ucRxBuffer[5]<<8)|ucRxBuffer[4];
						stcMag.h[2] = ((unsigned short)ucRxBuffer[7]<<8)|ucRxBuffer[6];
						stcAngle.T = ((unsigned short)ucRxBuffer[9]<<8)|ucRxBuffer[8];
						break;
			case 0x55:	stcDStatus.sDStatus[0] = ((unsigned short)ucRxBuffer[3]<<8)|ucRxBuffer[2];
						stcDStatus.sDStatus[1] = ((unsigned short)ucRxBuffer[5]<<8)|ucRxBuffer[4];
						stcDStatus.sDStatus[2] = ((unsigned short)ucRxBuffer[7]<<8)|ucRxBuffer[6];
						stcDStatus.sDStatus[3] = ((unsigned short)ucRxBuffer[9]<<8)|ucRxBuffer[8];
						break;
			case 0x56:	ucRxBuffer[2] = 0x12;ucRxBuffer[3] = 0x34;ucRxBuffer[4] = 0x56;ucRxBuffer[5] = 0x78;
						CharToLong((char*)&stcPress.lPressure,(char*)&ucRxBuffer[2]);
						CharToLong((char*)&stcPress.lAltitude,(char*)&ucRxBuffer[6]);
						break;
			case 0x57:	CharToLong((char*)&stcLonLat.lLon,(char*)&ucRxBuffer[2]);
						CharToLong((char*)&stcLonLat.lLat,(char*)&ucRxBuffer[6]);
						break;
			case 0x58:	stcGPSV.sGPSHeight = ((unsigned short)ucRxBuffer[3]<<8)|ucRxBuffer[2];
						stcGPSV.sGPSYaw = ((unsigned short)ucRxBuffer[5]<<8)|ucRxBuffer[4];
						CharToLong((char*)&stcGPSV.lGPSVelocity,(char*)&ucRxBuffer[6]);
						break;
		}
		ucRxCnt=0;
	}
}


 void Data_Receive_Anl41(u8 *data_buf,u8 num)
{
	double zen,xiao;
	static float m100_hr,m100_attr[3];
	vs16 rc_value_temp,rc_value_temp1;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	u8 sum_in=*(data_buf+num-1);
	if(!(sum==*(data_buf+num-1)))		
		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
	if(*(data_buf+2)==0x31)//SLAM_frame
  {
	m100.mems_board_connect=1;
	m100.mems_loss_cnt=0;
	ultra_distance=(float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5));
	ALT_POS_SONAR_HEAD=((int16_t)(*(data_buf+6)<<8)|*(data_buf+7))/1000.;
	S_head=((int16_t)(*(data_buf+8)<<8)|*(data_buf+9));
	Rol_yun=(float)((int16_t)(*(data_buf+10)<<8)|*(data_buf+11))/100.;
	Pit_yun=(float)((int16_t)(*(data_buf+12)<<8)|*(data_buf+13))/100.;
	Yaw_yun=(float)((int16_t)(*(data_buf+14)<<8)|*(data_buf+15))/100.;
	Rol_yun_rate=(float)((int16_t)(*(data_buf+16)<<8)|*(data_buf+17))/100.;
	Pit_yun_rate=(float)((int16_t)(*(data_buf+18)<<8)|*(data_buf+19))/100.;
	Yaw_yun_rate=(float)((int16_t)(*(data_buf+20)<<8)|*(data_buf+21))/100.;
	SONAR_HEAD_CHECK[0]=1;
	}	
	else 	if(*(data_buf+2)==0x01)//
  { px4.loss_cnt=0;
		px4.connect=1;
	 	px4.rx_dt=Get_Cycle_T(GET_T_PX4); 	

	  px4.Pit=(float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5))/10.;
		px4.Rol=(float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7))/10.;
		px4.Yaw=To_180_degrees(1*(float)((int16_t)(*(data_buf+8)<<8)|*(data_buf+9))/10.);
		
		px4.H=(float)((int16_t)(*(data_buf+10)<<8)|*(data_buf+11))/1000.;
		
		if(px4.H!=m100_hr||m100_attr[0]!=px4.Pit||m100_attr[1]!=px4.Rol||m100_attr[2]!=px4.Yaw)
		{px4.cnt_m100_data_refresh=0;
		 px4.m100_data_refresh=1;
		}
		m100_hr=px4.H;
		m100_attr[0]=px4.Pit;
		m100_attr[1]=px4.Rol;
		m100_attr[2]=px4.Yaw;
		
		px4.H_Spd=(float)((int16_t)(*(data_buf+12)<<8)|*(data_buf+13))/1000.;
		zen=(*(data_buf+14)<<8)|*(data_buf+15);
		xiao=(double)((u32)(*(data_buf+16)<<24)|(*(data_buf+17)<<16)|(*(data_buf+18)<<8)|*(data_buf+19))/1000000000.;
		px4.Lat=zen+xiao;
		zen=(*(data_buf+20)<<8)|*(data_buf+21);
		xiao=(double)((u32)(*(data_buf+22)<<24)|(*(data_buf+23)<<16)|(*(data_buf+24)<<8)|*(data_buf+25))/1000000000.;
		px4.Lon=zen+xiao;
		
		px4.Bat=(float)((int16_t)(*(data_buf+26)<<8)|*(data_buf+27));
		px4.Rc_rol=(float)((int16_t)(*(data_buf+28)<<8)|*(data_buf+29));//rol
		px4.Rc_yaw=(float)((int16_t)(*(data_buf+30)<<8)|*(data_buf+31));//yaw
		px4.Rc_gear=(float)((int16_t)(*(data_buf+32)<<8)|*(data_buf+33));//gear
		px4.Rc_mode=(float)((int16_t)(*(data_buf+34)<<8)|*(data_buf+35));//mode
		px4.Rc_thr=(float)((int16_t)(*(data_buf+36)<<8)|*(data_buf+37));//thr
		px4.Rc_pit=(float)((int16_t)(*(data_buf+38)<<8)|*(data_buf+39));//pit
		px4.STATUS=*(data_buf+40);		
		if(px4.Lat!=0&&px4.Lon!=0)
		px4.GPS_STATUS=3;
    else		
		px4.GPS_STATUS=*(data_buf+41);
		px4.spd[0]=(float)((int16_t)(*(data_buf+42)<<8)|*(data_buf+43))/1000.;
		px4.spd[1]=(float)((int16_t)(*(data_buf+44)<<8)|*(data_buf+45))/1000.;
		px4.spd[2]=(float)((int16_t)(*(data_buf+46)<<8)|*(data_buf+47))/1000.;
	}
	/*else if(*(data_buf+2)==0x21)//Qr land
  {
	qr.connect=circle.connect=1;
	qr.lose_cnt=0;
	track.check=circle.check=qr.check=*(data_buf+4);	
	qr.x=(int16_t)(*(data_buf+5)<<8)|*(data_buf+6);
	qr.y=(int16_t)(*(data_buf+7)<<8)|*(data_buf+8);
	qr.z=(int16_t)(*(data_buf+9)<<8)|*(data_buf+10);
	qr.pit=(int16_t)(*(data_buf+11)<<8)|*(data_buf+12);
	qr.rol=(int16_t)(*(data_buf+13)<<8)|*(data_buf+14);
	qr.yaw=(int16_t)(*(data_buf+15)<<8)|*(data_buf+16);
	
	if(qr.check){
		int temp=((int16_t)(*(data_buf+17)<<8)|*(data_buf+18));
		if(ABS(temp)<1000)
		circle.x=qr.pix_x=temp;
		temp=((int16_t)(*(data_buf+19)<<8)|*(data_buf+20));
		if(ABS(temp)<1000)
		circle.y=qr.pix_y=temp;	
	}
	qr.center_x=-((int16_t)(*(data_buf+25)<<8)|*(data_buf+26));	
	qr.center_y=-((int16_t)(*(data_buf+27)<<8)|*(data_buf+28));
	}*/
}

u8 Rx_Buf41[256];	//串口接收缓存
u8 RxBuffer41[50];
u8 RxState41 = 0;
u8 RxBufferNum41 = 0;
u8 RxBufferCnt41 = 0;
u8 RxLen41= 0;
static u8 _data_len41 = 0,_data_cnt41 = 0;
void USART3_IRQHandler(void)
{  OSIntEnter();  
	u8 com_data;
	static u8 state,cnt;
	u8 sum;
	if(USART3->SR & USART_SR_ORE)//ORE中断
	{
		com_data = USART3->DR;
	}

  //接收中断
	if( USART_GetITStatus(USART3,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);//清除中断标志

		com_data = USART3->DR;
		#if PX4_VER1
		UsartSend_GOL_LINK(com_data);
		#endif
		//CopeSerialData(com_data);
    if(RxState41==0&&com_data==0xAA)
		{
			RxState41=1;
			RxBuffer41[0]=com_data;
		}
		else if(RxState41==1&&com_data==0xAF)
		{
			RxState41=2;
			RxBuffer41[1]=com_data;
		}
		else if(RxState41==2&&com_data>0&&com_data<0XF1)
		{
			RxState41=3;
			RxBuffer41[2]=com_data;
		}
		else if(RxState41==3&&com_data<50)
		{
			RxState41 = 4;
			RxBuffer41[3]=com_data;
			_data_len41 = com_data;
			_data_cnt41 = 0;
		}
		else if(RxState41==4&&_data_len41>0)
		{
			_data_len41--;
			RxBuffer41[4+_data_cnt41++]=com_data;
			if(_data_len41==0)
				RxState41 = 5;
		}
		else if(RxState41==5)
		{
			RxState41 = 0;
			RxBuffer41[4+_data_cnt41]=com_data;
			Data_Receive_Anl41(RxBuffer41,_data_cnt41+5);
		}
		else
			RxState41 = 0;
	}
	//发送（进入移位）中断
	if( USART_GetITStatus(USART3,USART_IT_TXE ) )
	{
				
		USART3->DR = TxBuffer[TxCounter++]; //写DR清除中断标志          
		if(TxCounter == count)
		{
			USART3->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}

		USART_ClearITPendingBit(USART3,USART_IT_TXE);
	}
 OSIntExit();        
}

void UsartSend_GPS(uint8_t ch)
{

while(USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);
USART_SendData(UART5, ch); ;//USART1, ch); 
}

static void Send_Data_GPS(u8 *dataToSend , u8 length)
{
u16 i;
  for(i=0;i<length;i++)
     UsartSend_GPS(dataToSend[i]);
}


void Send_IMU_TO_GPS(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
  data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x01;//功能字
	data_to_send[_cnt++]=0;//数据量
	if(state_v_test!=0)
	_temp = state_v_test;//ultra_distance;
	else
	_temp = state_v;//ultra_distance;	
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = tar_need_to_check_odroid[2];//ultra_distance;
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(m100.Pit*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(m100.Rol*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(m100.Yaw*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	for(i=0;i<10;i++){
	_temp = tar_buf[i];//ultra_distance;
	data_to_send[_cnt++]=BYTE0(_temp);
	}
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	Send_Data_GPS(data_to_send, _cnt);
}

void CPU_LINK_TASK(void)
{
static u8 cnt[4];
static u8 flag;
if(cnt[0]++>0)
{cnt[0]=0;
 Send_IMU_TO_GPS();
}
}



//-------------------------NAV_BOARD_LINK

//------------------------------------------------------GOL_LINK----------------------------------------------------

void UsartSend_GOL_LINK_NAV(uint8_t ch)
{


	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
USART_SendData(USART3, ch); 
}

static void Send_Data_GOL_LINK_NAV(u8 *dataToSend , u8 length)
{
u16 i;
  for(i=0;i<length;i++)
     UsartSend_GOL_LINK_NAV(dataToSend[i]);
}


u8 SendBuff1[SEND_BUF_SIZE1];	//发送数据缓冲区

u8 SendBuff1_cnt;
void Usart1_Send_DMA(u8 *dataToSend , u8 length)
{u8 i;
for	(i=0;i<length;i++)
SendBuff1[SendBuff1_cnt++]=dataToSend[i];
}

int16_t BLE_DEBUG[16];
void data_per_uart1(int16_t ax,int16_t ay, int16_t az, int16_t gx,int16_t  gy, int16_t gz,int16_t hx, int16_t hy, int16_t hz,
	int16_t yaw,int16_t pitch,int16_t roll,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec)
{
u16 i=0; 	
unsigned int temp=0xaF+9;

char ctemp;	
BLE_DEBUG[1]=ax;
BLE_DEBUG[2]=ay;
BLE_DEBUG[3]=az;	
BLE_DEBUG[4]=gx;
BLE_DEBUG[5]=gy;
BLE_DEBUG[6]=gz;
BLE_DEBUG[7]=hx;
BLE_DEBUG[8]=hy;
BLE_DEBUG[9]=hz;
BLE_DEBUG[10]=yaw;
BLE_DEBUG[11]=pitch;
BLE_DEBUG[12]=roll;
BLE_DEBUG[13]=alt;
BLE_DEBUG[13]=tempr;
BLE_DEBUG[14]=press;
BLE_DEBUG[15]=IMUpersec;

}



u8 SendBuff2[SEND_BUF_SIZE2];	//发送数据缓冲区
void data_per_uart2(void)
{
	u8 i;	u8 sum = 0;
	u8 _cnt=0;
	vs16 _temp;
  SendBuff2[_cnt++]=0xAA;
	SendBuff2[_cnt++]=0xAF;
	SendBuff2[_cnt++]=0x01;//功能字
	SendBuff2[_cnt++]=0;//数据量
	
	_temp = (vs16)(Pitch*10);//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Roll*10);//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Yaw*10);//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	
	_temp = mpu6050.Acc.x;//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
 	_temp = mpu6050.Acc.y;//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp = mpu6050.Acc.z;//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);

	_temp = q_nav[0]*1000;//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
 	_temp = q_nav[1]*1000;//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  q_nav[2]*1000;//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  q_nav[3]*1000;//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	
	
	
	_temp =  0;//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  0;//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	
	
	
	SendBuff2[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += SendBuff2[i];
	SendBuff2[_cnt++] = sum;
}


u8 SendBuff3[SEND_BUF_SIZE3];	//发送数据缓冲区
void data_per_uart3(void)
{
	u8 i;	u8 sum = 0;
	u8 _cnt=0;
	vs16 _temp;
  SendBuff3[_cnt++]=0xAA;
	SendBuff3[_cnt++]=0xAF;
	SendBuff3[_cnt++]=0x01;//功能字
	SendBuff3[_cnt++]=0;//数据量
	
	_temp = (vs16)(Pitch*10);//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Roll*10);//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Yaw*10);//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	
	_temp = mpu6050.Acc.x;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
 	_temp = mpu6050.Acc.y;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	_temp = mpu6050.Acc.z;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);

	_temp = q_nav[0]*1000;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
 	_temp = q_nav[1]*1000;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	_temp =  q_nav[2]*1000;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	_temp =  q_nav[3]*1000;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	
	SendBuff3[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += SendBuff3[i];
	SendBuff3[_cnt++] = sum;
}


u8 SendBuff4[SEND_BUF_SIZE4];	//???????
u16 nrf_uart_cnt;
int sd_save[25*3]={0};
void data_per_uart4(u8 sel)
{
	u8 i;	u8 sum = 0;
	u16 _cnt=0,cnt_reg;
	vs16 _temp;
  

switch(sel){
	case SEND_M100:
	cnt_reg=nrf_uart_cnt;
	SendBuff4[nrf_uart_cnt++]=0xAA;
	SendBuff4[nrf_uart_cnt++]=0xAF;
	SendBuff4[nrf_uart_cnt++]=0x51;//???
	SendBuff4[nrf_uart_cnt++]=0;//???
	_temp=m100.Pit*10;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
  _temp=m100.Rol*10;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=m100.Yaw*10;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	
	_temp=m100.H*1000;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=m100.H_Spd*1000;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	
	_temp=m100.Lat;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=(m100.Lat-(int)m100.Lat)*1000000000;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE3(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE2(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=m100.Lon;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=(m100.Lon-(int)m100.Lon)*1000000000;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE3(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE2(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=m100.Bat*100;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=m100.Rc_pit;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=m100.Rc_rol;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=m100.Rc_yaw;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=m100.Rc_thr;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=m100.Rc_mode;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=m100.Rc_gear;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=m100.m100_connect;
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=m100.GPS_STATUS;
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=mode.en_sd_save;
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);

	
	SendBuff4[cnt_reg+3] =(nrf_uart_cnt-cnt_reg)-4;
		for( i=cnt_reg;i<nrf_uart_cnt;i++)
	sum += SendBuff4[i];
	SendBuff4[nrf_uart_cnt++] = sum;
	break;
	case SEND_ALT:
		cnt_reg=nrf_uart_cnt;
	SendBuff4[nrf_uart_cnt++]=0xAA;
	SendBuff4[nrf_uart_cnt++]=0xAF;
	SendBuff4[nrf_uart_cnt++]=0x03;//???
	SendBuff4[nrf_uart_cnt++]=0;//???
//	_temp = exp_height;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
//	_temp = ultra_dis_lpf;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	//_temp = ultra_ctrl_out;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	//_temp = wz_speed;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
 //	_temp = thr_value	;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = ALT_POS_BMP*1000	;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = ALT_POS_SONAR2*1000	;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
  _temp = mode.en_sd_save;
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp =0;// ALT_POS_BMP_EKF*1000	;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = ALT_VEL_BMP_EKF*1000	;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = ultra_distance	;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	
	
	SendBuff4[cnt_reg+3] =(nrf_uart_cnt-cnt_reg)-4;
	for( i=cnt_reg;i<nrf_uart_cnt;i++)
	sum += SendBuff4[i];
	SendBuff4[nrf_uart_cnt++] = sum;
	break;
	case SEND_IMU:
		cnt_reg=nrf_uart_cnt;
	SendBuff4[nrf_uart_cnt++]=0xAA;
	SendBuff4[nrf_uart_cnt++]=0xAF;
	SendBuff4[nrf_uart_cnt++]=0x05;//???
	SendBuff4[nrf_uart_cnt++]=0;//???
	_temp = Roll*10;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = Pitch*10;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = Yaw*10	;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=(mpu6050.Gyro.x);
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=(mpu6050.Gyro.y);
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=(mpu6050.Gyro.z);
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Acc.x;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Acc.y;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Acc.z;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=m100.q[0]*1000;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=m100.q[1]*1000;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=m100.q[2]*1000;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=m100.q[3]*1000;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);

	_temp=ak8975.Mag_Adc.x;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=ak8975.Mag_Adc.y;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=ak8975.Mag_Adc.z;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=ak8975.Mag_Val.x;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
		_temp=ak8975.Mag_Val.y;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
		_temp=ak8975.Mag_Val.z;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	
		_temp=mode.en_sd_save;
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	
	SendBuff4[cnt_reg+3] =(nrf_uart_cnt-cnt_reg)-4;
	for( i=cnt_reg;i<nrf_uart_cnt;i++)
	sum += SendBuff4[i];
	SendBuff4[nrf_uart_cnt++] = sum;
	break;
	case SEND_FLOW:
		cnt_reg=nrf_uart_cnt;
	SendBuff4[nrf_uart_cnt++]=0xAA;
	SendBuff4[nrf_uart_cnt++]=0xAF;
	SendBuff4[nrf_uart_cnt++]=0x06;//???
	SendBuff4[nrf_uart_cnt++]=0;//???
	_temp=imu_nav.flow.speed.x_f*1000;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=imu_nav.flow.speed.y_f*1000;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=imu_nav.flow.speed.x*1000;//origin
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=imu_nav.flow.speed.y*1000;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	
	_temp=target_position[LAT]*100;//
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=target_position[LON]*100;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=now_position[LAT]*100;//
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=now_position[LON]*100;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	
	_temp=0;//flow_matlab_data[0]*1000;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=0;//flow_matlab_data[1]*1000;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=0;//flow_matlab_data[2]*1000;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=0;//flow_matlab_data[3]*1000;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);

	//_temp=baroAlt_fc;//baro_matlab_data[0];
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	//_temp=acc_bmp*1000;//baro_matlab_data[1];
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=circle.check&&circle.connect;///10.;
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=circle.x;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=circle.y;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=circle.z;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=circle.pit;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=circle.rol;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=circle.yaw;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=0;//k_scale_pix;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	
	SendBuff4[cnt_reg+3] =(nrf_uart_cnt-cnt_reg)-4;
	for( i=cnt_reg;i<nrf_uart_cnt;i++)
	sum += SendBuff4[i];
	SendBuff4[nrf_uart_cnt++] = sum;
	break;
  case SEND_PID:
		cnt_reg=nrf_uart_cnt;
	SendBuff4[nrf_uart_cnt++]=0xAA;
	SendBuff4[nrf_uart_cnt++]=0xAF;
	SendBuff4[nrf_uart_cnt++]=0x07;//???
	SendBuff4[nrf_uart_cnt++]=0;//???
	_temp=SPID.OP;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=SPID.OI;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=SPID.OD;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	
	_temp=SPID.IP;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=SPID.II;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=SPID.ID;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	
	_temp=SPID.YP;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=SPID.YI;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=SPID.YD;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	
	_temp=HPID.OP;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=HPID.OI;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=HPID.OD;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=mode.en_sd_save;
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	
	SendBuff4[cnt_reg+3] =(nrf_uart_cnt-cnt_reg)-4;
	for( i=cnt_reg;i<nrf_uart_cnt;i++)
	sum += SendBuff4[i];
	SendBuff4[nrf_uart_cnt++] = sum;
	break;
	case SEND_DEBUG:
		cnt_reg=nrf_uart_cnt;
	SendBuff4[nrf_uart_cnt++]=0xAA;
	SendBuff4[nrf_uart_cnt++]=0xAF;
	SendBuff4[nrf_uart_cnt++]=0x08;//???
	SendBuff4[nrf_uart_cnt++]=0;//???
	for(i=0;i<16;i++){
	_temp=0;//BLE_DEBUG[i];//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	}
	
	SendBuff4[cnt_reg+3] =(nrf_uart_cnt-cnt_reg)-4;
	for( i=cnt_reg;i<nrf_uart_cnt;i++)
	sum += SendBuff4[i];
	SendBuff4[nrf_uart_cnt++] = sum;
	break;
	
	case SEND_MARKER:
	cnt_reg=nrf_uart_cnt;
	SendBuff4[nrf_uart_cnt++]=0xAA;
	SendBuff4[nrf_uart_cnt++]=0xAF;
	SendBuff4[nrf_uart_cnt++]=0x09;//???
	SendBuff4[nrf_uart_cnt++]=0;//???
	for(i=0;i<4;i++){
	_temp=0;//mark_map[i][0];//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=0;//mark_map[i][1];//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=0;//mark_map[i][2];//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	}
  _temp=0;//mark_map[0][3];//filter
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=0;//mark_map[1][3];//filter
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=0;//mark_map[2][3];//filter
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=0;//mark_map[3][3];//filter
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	
  SendBuff4[cnt_reg+3] =(nrf_uart_cnt-cnt_reg)-4;
	for( i=cnt_reg;i<nrf_uart_cnt;i++)
	sum += SendBuff4[i];
	SendBuff4[nrf_uart_cnt++] = sum;
	break;
	case SEND_SD_SAVE1://<--------------------------------sd general save 
	cnt_reg=nrf_uart_cnt;
	SendBuff4[nrf_uart_cnt++]=0xAA;
	SendBuff4[nrf_uart_cnt++]=0xAF;
	SendBuff4[nrf_uart_cnt++]=0x81;//???
	SendBuff4[nrf_uart_cnt++]=0;//???
	for(i=0;i<20;i++){
	_temp=sd_save[i];//id
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	}

  SendBuff4[cnt_reg+3] =(nrf_uart_cnt-cnt_reg)-4;
	for( i=cnt_reg;i<nrf_uart_cnt;i++)
	sum += SendBuff4[i];
	SendBuff4[nrf_uart_cnt++] = sum;	
	break;
  case SEND_SD_SAVE2:
	cnt_reg=nrf_uart_cnt;
	SendBuff4[nrf_uart_cnt++]=0xAA;
	SendBuff4[nrf_uart_cnt++]=0xAF;
	SendBuff4[nrf_uart_cnt++]=0x82;//???
	SendBuff4[nrf_uart_cnt++]=0;//???
	for(i=20;i<20*2;i++){
	_temp=sd_save[i];//id
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	}

  SendBuff4[cnt_reg+3] =(nrf_uart_cnt-cnt_reg)-4;
	for( i=cnt_reg;i<nrf_uart_cnt;i++)
	sum += SendBuff4[i];
	SendBuff4[nrf_uart_cnt++] = sum;	
	break;
	case SEND_SD_SAVE3:
	cnt_reg=nrf_uart_cnt;
	SendBuff4[nrf_uart_cnt++]=0xAA;
	SendBuff4[nrf_uart_cnt++]=0xAF;
	SendBuff4[nrf_uart_cnt++]=0x83;//???
	SendBuff4[nrf_uart_cnt++]=0;//???

	for(i=20*2;i<20*3;i++){
	_temp=sd_save[i];//id
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	}
	SendBuff4[nrf_uart_cnt++]=mode.en_sd_save;
	SendBuff4[nrf_uart_cnt++]=mode.cal_rc;
	SendBuff4[nrf_uart_cnt++]=mode.mems_state;
	
	if((m100_data_refresh&&!dji_rc_miss&&m100.GPS_STATUS>=1))
	SendBuff4[nrf_uart_cnt++]=9;
	else if(m100_data_refresh&&!dji_rc_miss)
	SendBuff4[nrf_uart_cnt++]=6;
	else
	SendBuff4[nrf_uart_cnt++]=0;
	
	SendBuff4[nrf_uart_cnt++]=1;//module.acc_imu==2&&module.gyro_imu==2&&module.hml_imu==2;
	
	SendBuff4[nrf_uart_cnt++]=LED[0];
	SendBuff4[nrf_uart_cnt++]=LED[1];
	SendBuff4[nrf_uart_cnt++]=LED[2];
	
  SendBuff4[cnt_reg+3] =(nrf_uart_cnt-cnt_reg)-4;
	for( i=cnt_reg;i<nrf_uart_cnt;i++)
	sum += SendBuff4[i];
	SendBuff4[nrf_uart_cnt++] = sum;	
	break;
	case SEND_QR:
	cnt_reg=nrf_uart_cnt;
	SendBuff4[nrf_uart_cnt++]=0xAA;
	SendBuff4[nrf_uart_cnt++]=0xAF;
	SendBuff4[nrf_uart_cnt++]=0x61;//???
	SendBuff4[nrf_uart_cnt++]=0;//???

  static int x,y,z;
	if(circle.x!=0)
		x=circle.x*10;
	if(circle.y!=0)
		y=-circle.y*10;
	if(circle.z!=0)
		z=circle.z*10;
	//qr
	_temp = (int)(x);
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (int)(y);
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (int)(z);
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	//exp
	_temp = 0;//(int)(nav_pos_ctrl[0].exp*1000);
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = 0;//(int)(nav_pos_ctrl[1].exp*1000);
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = 0;//(int)(exp_height);
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	//kf
	_temp = 0;//(int)(POS_UKF_X*1000);
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = 0;//(int)(POS_UKF_Y*1000);
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (int)(ALT_POS_SONAR2*1000);
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	
	
  SendBuff4[cnt_reg+3] =(nrf_uart_cnt-cnt_reg)-4;
	for( i=cnt_reg;i<nrf_uart_cnt;i++)
	sum += SendBuff4[i];
	SendBuff4[nrf_uart_cnt++] = sum;	
	break;
	
	
	default:break;
}
}
#include "rc.h"
u8 sd_pub_sel=1;
void sd_publish(void)
{u8 cnt=0;
//	switch(sd_pub_sel){
//	case 0:	//mark slam
//	//sd 1	
//	sd_save[cnt++]=mark_map[0][0];
//	sd_save[cnt++]=mark_map[0][1];
//	sd_save[cnt++]=mark_map[0][2];	
//	sd_save[cnt++]=mark_map[0][3];
//	sd_save[cnt++]=mark_map[0][4];

//	sd_save[cnt++]=mark_map[1][0];
//	sd_save[cnt++]=mark_map[1][1];
//	sd_save[cnt++]=mark_map[1][2];	
//	sd_save[cnt++]=mark_map[1][3];
//	sd_save[cnt++]=mark_map[1][4];

//	sd_save[cnt++]=mark_map[2][0];
//	sd_save[cnt++]=mark_map[2][1];
//	sd_save[cnt++]=mark_map[2][2];	
//	sd_save[cnt++]=mark_map[2][3];
//	sd_save[cnt++]=mark_map[2][4];

//	sd_save[cnt++]=mark_map[3][0];
//	sd_save[cnt++]=mark_map[3][1];
//	sd_save[cnt++]=mark_map[3][2];	
//	sd_save[cnt++]=mark_map[3][3];
//	sd_save[cnt++]=mark_map[3][4];

//	//sd 2  20~39
//	sd_save[cnt++]=mark_map[4][0];
//	sd_save[cnt++]=mark_map[4][1];
//	sd_save[cnt++]=mark_map[4][2];	
//	sd_save[cnt++]=mark_map[4][3];
//	sd_save[cnt++]=mark_map[4][4];

//	sd_save[cnt++]=mark_map[5][0];
//	sd_save[cnt++]=mark_map[5][1];
//	sd_save[cnt++]=mark_map[5][2];	
//	sd_save[cnt++]=mark_map[5][3];
//	sd_save[cnt++]=mark_map[5][4];

//	sd_save[cnt++]=circle.check&&circle.connect;
//	sd_save[cnt++]=circle.x;
//	sd_save[cnt++]=circle.y;
//	sd_save[cnt++]=circle.z;
//	sd_save[cnt++]=circle.pit;

//	sd_save[cnt++]=circle.rol;
//	sd_save[cnt++]=circle.yaw;
//	sd_save[cnt++]=flow_matlab_data[0]*1000;	
//	sd_save[cnt++]=flow_matlab_data[1]*1000;
//	sd_save[cnt++]=ALT_POS_SONAR2*1000;
//	//sd 3 40~59

//	sd_save[cnt++]=circle.spdy;//flow_matlab_data[2]*1000;	
//	sd_save[cnt++]=circle.spdx;//flow_matlab_data[3]*1000;
//	sd_save[cnt++]=0;
//	sd_save[cnt++]=mpu6050.Acc.x;
//	sd_save[cnt++]=mpu6050.Acc.y;

//	sd_save[cnt++]=mpu6050.Acc.z;
//	sd_save[cnt++]=mpu6050.Gyro.x;
//	sd_save[cnt++]=mpu6050.Gyro.y;
//	sd_save[cnt++]=mpu6050.Gyro.z;
//	sd_save[cnt++]=ak8975.Mag_Val.x;

//	sd_save[cnt++]=ak8975.Mag_Val.y;
//	sd_save[cnt++]=ak8975.Mag_Val.z;
//	sd_save[cnt++]=Pit_fc*100;
//	sd_save[cnt++]=Rol_fc*100;
//	sd_save[cnt++]=Yaw_fc*100;

//	sd_save[cnt++]=VEL_UKF_X*100;//flow_matlab_data[2]*1000;
//	sd_save[cnt++]=VEL_UKF_Y*100;//flow_matlab_data[3]*1000;
//	sd_save[cnt++]=POS_UKF_X*100;
//	sd_save[cnt++]=POS_UKF_Y*100;
//	sd_save[cnt++]=0;
//	break;
//	///
//	case 1:	//track 
//	//sd 1	
//	sd_save[cnt++]=Pit_fc*100;
//	sd_save[cnt++]=Rol_fc*100;
//	sd_save[cnt++]=Yaw_fc*100;	
//	sd_save[cnt++]=0;
//	sd_save[cnt++]=0;

//	sd_save[cnt++]=ALT_POS_SONAR2*1000;
//	sd_save[cnt++]=POS_UKF_Y*1000;
//	sd_save[cnt++]=POS_UKF_X*1000;	
//	sd_save[cnt++]=VEL_UKF_Y*1000;
//	sd_save[cnt++]=VEL_UKF_X*1000;

//	sd_save[cnt++]=acc_body[Y];
//	sd_save[cnt++]=acc_body[X];
//	sd_save[cnt++]=0;	
//	sd_save[cnt++]=0;
//	sd_save[cnt++]=0;

//	sd_save[cnt++]=robot.connect&&robot.mark_check;
//	sd_save[cnt++]=robot.camera_x;
//	sd_save[cnt++]=robot.camera_y;	
//	sd_save[cnt++]=robot.camera_z;
//	sd_save[cnt++]=robot.yaw;

//	//sd 2  20~39
//	sd_save[cnt++]=robot.rol;
//	sd_save[cnt++]=robot.pit;
//	sd_save[cnt++]=robot.track_x;	
//	sd_save[cnt++]=robot.track_y;
//	sd_save[cnt++]=robot.track_r;

//	sd_save[cnt++]=robot.mark_x;
//	sd_save[cnt++]=robot.mark_y;
//	sd_save[cnt++]=robot.mark_r;	
//	sd_save[cnt++]=aux.ero[Xr];
//	sd_save[cnt++]=aux.ero[Yr];

//	sd_save[cnt++]=aux.att[0]*10;
//	sd_save[cnt++]=aux.att[1]*10;
//	sd_save[cnt++]=0;//spd obo
//	sd_save[cnt++]=0;
//	sd_save[cnt++]=0;

//	sd_save[cnt++]=robot.mark_map[0][0];
//	sd_save[cnt++]=robot.mark_map[0][1];
//	sd_save[cnt++]=robot.mark_map[0][2];	
//	sd_save[cnt++]=robot.mark_map[0][3];
//	sd_save[cnt++]=robot.mark_map[0][4];	
//	//sd 3 40~59

//	sd_save[cnt++]=robot.mark_map[1][0];
//	sd_save[cnt++]=robot.mark_map[1][1];
//	sd_save[cnt++]=robot.mark_map[1][2];	
//	sd_save[cnt++]=robot.mark_map[1][3];
//	sd_save[cnt++]=robot.mark_map[1][4];	

//	sd_save[cnt++]=robot.mark_map[2][0];
//	sd_save[cnt++]=robot.mark_map[2][1];
//	sd_save[cnt++]=robot.mark_map[2][2];	
//	sd_save[cnt++]=robot.mark_map[2][3];
//	sd_save[cnt++]=robot.mark_map[2][4];		

//	sd_save[cnt++]=robot.mark_map[3][0];
//	sd_save[cnt++]=robot.mark_map[3][1];
//	sd_save[cnt++]=robot.mark_map[3][2];	
//	sd_save[cnt++]=robot.mark_map[3][3];
//	sd_save[cnt++]=robot.mark_map[3][4];	

//	sd_save[cnt++]=ultra_ctrl.exp;
//	sd_save[cnt++]=ultra_ctrl.now;
//	sd_save[cnt++]=wz_speed_pid_v.exp;	
//	sd_save[cnt++]=wz_speed_pid_v.now;
//	sd_save[cnt++]=0;	
//	break;
//	}
}	


void clear_nrf_uart(void)
{u16 i;
for(i=0;i<SEND_BUF_SIZE4;i++)
SendBuff4[i]=0;

}

//flow
u8 mav_flow_sel;
FLOW flow;
FLOW_RAD flow_rad;

u8 FLOW_STATE[4];
u8 flow_buf[27];
u8 flow_buf_rad[45];
   float ByteToFloat(unsigned char* byteArry)
{
  return *((float*)byteArry);
}

void FLOW_MAVLINK(unsigned char data)
{
/*
红色的是起始标志位（stx），在v1.0版本中以“FE”作为起始标志。这个标志位在mavlink消息帧接收端进行消息解码时有用处。

第二个格子代表的是灰色部分（payload，称作有效载荷，要用的数据在有效载荷里面）的字节长度（len），范围从0到255之间。在mavlink消息帧接收端可以用它和实际收到的有效载荷的长度比较，以验证有效载荷的长度是否正确。

第三个格子代表的是本次消息帧的序号（seq），每次发完一个消息，这个字节的内容会加1，加到255后会从0重新开始。这个序号用于mavlink消息帧接收端计算消息丢失比例用的，相当于是信号强度。

第四个格子代表了发送本条消息帧的设备的系统编号（sys），使用PIXHAWK刷PX4固件时默认的系统编号为1，用于mavlink消息帧接收端识别是哪个设备发来的消息。

第五个格子代表了发送本条消息帧的设备的单元编号（comp），使用PIXHAWK刷PX4固件时默认的单元编号为50，用于mavlink消息帧接收端识别是设备的哪个单元发来的消息（暂时没什么用） 。

第六个格子代表了有效载荷中消息包的编号（msg），注意它和序号是不同的，这个字节很重要，mavlink消息帧接收端要根据这个编号来确定有效载荷里到底放了什么消息包并根据编号选择对应的方式来处理有效载荷里的信息包。
      26*/		 
// FE 1A| A2 X X| 64
	
static u8 s_flow=0,data_cnt=0;
float sonar_new;
static float  temp,sonar_lp;
u8 cnt_offset=0;	
u8 get_one_fame=0;
char floattobyte[4];
		switch(s_flow)
	 {
    case 0: if(data==0xFE)
			s_flow=1;
			break;
		case 1: if(data==0x1A||data==0x2C)
				{ s_flow=2;}
			else
			s_flow=0;
			break;
	  case 2:
			if(data_cnt<4)
			{s_flow=2; FLOW_STATE[data_cnt++]=data;}
		  else
			{data_cnt=0;s_flow=3;flow_buf[data_cnt++]=data;}
		 break;
		case 3:
		 if(FLOW_STATE[3]==100){
			if(data_cnt<26)
			{s_flow=3; flow_buf[data_cnt++]=data;}
		  else
			{data_cnt=0;s_flow=4;}
		}
		else if(FLOW_STATE[3]==106){
			if(data_cnt<44)
			{s_flow=3; flow_buf_rad[data_cnt++]=data;}
		  else
			{data_cnt=0;s_flow=4;}
		}
		else
			{data_cnt=0;s_flow=0;}
			 break;
		case 4:get_one_fame=1;s_flow=0;data_cnt=0;break;
		default:s_flow=0;data_cnt=0;break;
	 }//--end of s_uart
		

	 if(get_one_fame)
	 { mav_flow_sel=2;
		 if(FLOW_STATE[3]==100){
		flow.time_sec=(flow_buf[7]<<64)|(flow_buf[6]<<56)|(flow_buf[5]<<48)|(flow_buf[4]<<40)
		 |(flow_buf[3]<<32)|(flow_buf[2]<<16)|(flow_buf[1]<<8)|(flow_buf[0]);
  	 floattobyte[0]=flow_buf[8];
		 floattobyte[1]=flow_buf[9];
		 floattobyte[2]=flow_buf[10];
		 floattobyte[3]=flow_buf[11];
		flow.flow_comp_x.originf =ByteToFloat(floattobyte);
		 floattobyte[0]=flow_buf[12];
		 floattobyte[1]=flow_buf[13];
		 floattobyte[2]=flow_buf[14];
		 floattobyte[3]=flow_buf[15];
		flow.flow_comp_y.originf =ByteToFloat(floattobyte);
		 floattobyte[0]=flow_buf[16];
		 floattobyte[1]=flow_buf[17];
		 floattobyte[2]=flow_buf[18];
		 floattobyte[3]=flow_buf[19];
//	   if(!GOL_LINK_CONNECT||height_ctrl_mode!=2||x_pred==0)
//			 sonar_new=1;
//		 else
//			 sonar_new=x_pred;//ByteToFloat(floattobyte);//ground_distance	float	Ground distance in m. Positive value: distance known. Negative value: Unknown distance     
	  
		 flow.hight.originf= 0.05f * sonar_new + 0.95f * sonar_lp;
		 sonar_lp = sonar_new;
		 flow.flow_x.origin=(int16_t)((flow_buf[20])|(flow_buf[21]<<8));
	   flow.flow_y.origin=(int16_t)((flow_buf[22])|(flow_buf[23]<<8));
		 flow.id=flow_buf[24];
	   flow.quality=flow_buf[25]; //Optical flow quality / confidence. 0: bad, 255: maximum quality
	//	flow_fix.flow_comp_x.average 		 = IIR_I_Filter(flow_fix.flow_comp_x.originf , InPut_IIR[0], OutPut_IIR[0], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
	//	flow.flow_y.average 		 = IIR_I_Filter(flow.flow_y.origin , InPut_IIR[1], OutPut_IIR[1], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
	//	flow.flow_comp_x.average = IIR_I_Filter(flow.flow_comp_x.originf , InPut_IIR[2], OutPut_IIR[2], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
	//	flow.flow_comp_y.average = IIR_I_Filter(flow.flow_comp_y.originf , InPut_IIR[3], OutPut_IIR[3], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
    flow.new_data_flag	=1;//LED1=!LED1;
		 }
	 else if(FLOW_STATE[3]==106)
	 {
	 	flow_rad.time_usec=(flow_buf_rad[7]<<64)|(flow_buf_rad[6]<<56)|(flow_buf_rad[5]<<48)|(flow_buf_rad[4]<<40)
		 |(flow_buf_rad[3]<<32)|(flow_buf_rad[2]<<16)|(flow_buf_rad[1]<<8)|(flow_buf_rad[0]);
  	 flow_rad.integration_time_us=(flow_buf_rad[11]<<32)|(flow_buf_rad[10]<<16)|(flow_buf_rad[9]<<8)|(flow_buf_rad[8]);
		 floattobyte[0]=flow_buf_rad[12];
		 floattobyte[1]=flow_buf_rad[13];
		 floattobyte[2]=flow_buf_rad[14];
		 floattobyte[3]=flow_buf_rad[15];
		 flow_rad.integrated_x=ByteToFloat(floattobyte);
		 floattobyte[0]=flow_buf_rad[16];
		 floattobyte[1]=flow_buf_rad[17];
		 floattobyte[2]=flow_buf_rad[18];
		 floattobyte[3]=flow_buf_rad[19];
		 flow_rad.integrated_y=ByteToFloat(floattobyte);
		 floattobyte[0]=flow_buf_rad[20];
		 floattobyte[1]=flow_buf_rad[21];
		 floattobyte[2]=flow_buf_rad[22];
		 floattobyte[3]=flow_buf_rad[23];
		 flow_rad.integrated_xgyro=ByteToFloat(floattobyte);
		 floattobyte[0]=flow_buf_rad[24];
		 floattobyte[1]=flow_buf_rad[25];
		 floattobyte[2]=flow_buf_rad[26];
		 floattobyte[3]=flow_buf_rad[27];
		 flow_rad.integrated_ygyro=ByteToFloat(floattobyte);
		 floattobyte[0]=flow_buf_rad[28];
		 floattobyte[1]=flow_buf_rad[29];
		 floattobyte[2]=flow_buf_rad[30];
		 floattobyte[3]=flow_buf_rad[31];
		 flow_rad.integrated_zgyro=ByteToFloat(floattobyte);
		 flow_rad.time_delta_distance_us=(flow_buf_rad[35]<<32)|(flow_buf_rad[34]<<16)|(flow_buf_rad[33]<<8)|(flow_buf_rad[32]);
		 floattobyte[0]=flow_buf_rad[36];
		 floattobyte[1]=flow_buf_rad[37];
		 floattobyte[2]=flow_buf_rad[38];
		 floattobyte[3]=flow_buf_rad[39];
		 flow_rad.distance=ByteToFloat(floattobyte);
		 flow_rad.temperature=(flow_buf_rad[41]<<8)|(flow_buf_rad[40]);
		 flow_rad.sensor_id=(flow_buf_rad[42]);
		 flow_rad.quality=(flow_buf_rad[43]);
		 
    flow_task_uart();
		 
	 }	
	 }
}
/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

