/*! @file Receive.cpp
 *
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  This function parses Rx buffer and execute commands sent from computer.
 *
 *  Copyright 2016 DJI. All right reserved.
 *
 * */
#include "DJI_API.h"
#include "Receive.h"
#include "math.h"
#include "LocalNavigation.h"
#include "main.h"
#include "BspUsart.h"
M100 m100;
using namespace DJI::onboardSDK;
extern FlightData flightData;

/*
 * @brief Helper function to assemble two bytes into a float number
 */
static float32_t hex2Float(uint8_t HighByte, uint8_t LowByte)
{
	
	//  float32_t high = (float32_t) (HighByte & 0x7f);
  //float32_t low  = (float32_t) LowByte;
  if (HighByte & 0x80)//MSB is 1 means a negative number
  {
    return (HighByte*256.0f + LowByte - 65536)/100.0f;
  }
  else
  {
    return (HighByte*256.0f + LowByte)/100.0f;
  }
	
	
//  float32_t high = (float32_t) (HighByte & 0x7f);
//  float32_t low  = (float32_t) LowByte;
//  if (HighByte & 0x80)//MSB is 1 means a negative number
//  {
//    return -(high*256.0f + low)/100.0f;
//  }
//  else
//  {
//    return (high*256.0f + low)/100.0f;
//  }
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

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

float lat1;
float lon1;

void SEND_DJI1(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
	u32 _temp32;
	double test1;
  data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x01;//功能字
	data_to_send[_cnt++]=0;//数据量
	
	_temp = (vs16)(m100.Pitch*10);//Pitch;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(m100.Roll*10);//Roll;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(m100.Yaw*10);//Yaw;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)(m100.altitude*1000);//altitude;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(m100.h_spd*1000);//height velocity;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	test1=123.987654321;
	_temp = (vs32)(m100.latitude);//latitude;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp32 = (vs32)((m100.latitude-(int)(m100.latitude))*1000000000);//latitude;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);
	
	_temp = (vs32)(m100.longitude);//latitude;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp32 = (vs32)((m100.longitude-(int)(m100.longitude))*1000000000);//latitude;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);

	_temp = (vs16)(m100.Bat);//Bat;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp = (vs16)(m100.Rc_p);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(m100.Rc_r);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(m100.Rc_y);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(m100.Rc_t);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(m100.Rc_m);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(m100.Rc_g);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)(m100.State&&activate_s);//ultra_distance;
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(m100.GPS_S);//GPS_S;
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(m100.spd[0]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(m100.spd[1]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(m100.spd[2]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
//	_temp = ultra_distance;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = Laser_distance;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = Laser_ST;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_GOL_LINK(data_to_send, _cnt);
}




#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )

TerminalCommand myTerminal;
u8 en_dji_upload=0;
u8 rst_board;
void TerminalCommand::terminalCommandHandler(CoreAPI* api, Flight* flight)
{
  if (cmdReadyFlag == 1)
  {
    cmdReadyFlag = 0;
  }
  else
  { // No full command has been received yet.
    return;
  }

  if ((cmdIn[0] != 0xFA) || (cmdIn[1] != 0xFB))
  { // Command header doesn't match
    return;
  }


  switch(cmdIn[2])
  {
		case 0xF1:
		rst_board=cmdIn[4];
	 break;
  case 0x00:
    api->getDroneVersion();
    break;

  case 0x01:
    User_Activate();
    break;

  case 0x02:
    api->setControl(cmdIn[3]);
    break;

  case 0x03:
    flight->setArm(cmdIn[3]);
    break;

  case 0x04:
    if (cmdIn[3] == 0x01)
    {
      flightData.flag = cmdIn[4];
      flightData.x = LIMIT(hex2Float(cmdIn[5], cmdIn[6]), -3, 3);
      flightData.y = LIMIT(hex2Float(cmdIn[7], cmdIn[8]), -3, 3);
      flightData.z = LIMIT(hex2Float(cmdIn[9], cmdIn[10]), -3, 3);
			flightData.yaw = LIMIT(hex2Float(cmdIn[11], cmdIn[12]), -3, 3);

      flight->setFlight(&flightData);
			 m100.height= m100.altitude=api->getBroadcastData().pos.altitude;
			lat1=api->getBroadcastData().pos.latitude;
			lon1=api->getBroadcastData().pos.longitude;
				m100.latitude=api->getBroadcastData().pos.latitude/DEG2RAD(1);
				m100.longitude=api->getBroadcastData().pos.longitude/DEG2RAD(1);
				m100.h_spd = api->getBroadcastData().gps.Hmsl;//.pos.height;
				m100.q[0]=api->getBroadcastData().q.q0;
				m100.q[1]=api->getBroadcastData().q.q1;
				m100.q[2]=api->getBroadcastData().q.q2;
				m100.q[3]=api->getBroadcastData().q.q3;
				m100.Pitch=api->getBroadcastData().gimbal.pitch;
				m100.Roll=api->getBroadcastData().gimbal.roll;
				m100.Yaw=api->getBroadcastData().gimbal.yaw;
				m100.Bat=api->getBroadcastData().battery;	
				m100.GPS_S=api->getBroadcastData().pos.health;	
				m100.State=api->getBroadcastData().ctrlInfo.flightStatus;	
				m100.spd[0]=api->getBroadcastData().v.x;
				m100.spd[1]=api->getBroadcastData().v.y;
				m100.Rc_p=api->getBroadcastData().rc.pitch;		
				m100.Rc_r=api->getBroadcastData().rc.roll;
				m100.Rc_y=api->getBroadcastData().rc.yaw;
				m100.Rc_t=api->getBroadcastData().rc.throttle;
				m100.Rc_m=api->getBroadcastData().rc.mode;		
				m100.Rc_g=api->getBroadcastData().rc.gear;				
				float q0=m100.q[0],q1=m100.q[1],q2=m100.q[2],q3=m100.q[3];
				m100.Pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; 					// pitch
				m100.Roll =  atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; 	// roll
				m100.Yaw = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1)* 57.3; // yaw 
				SEND_DJI1();
    }
    else if (cmdIn[3] == 0x02)
    {//for display ur hex2int
//      printf("roll_or_x =%f\n", hex2Float(cmdIn[5], cmdIn[6]));
//      printf("pitch_or_y =%f\n", hex2Float(cmdIn[7], cmdIn[8]));
//      printf("thr_z =%f\n", hex2Float(cmdIn[9], cmdIn[10]));
//      printf("yaw =%f\n\n", hex2Float(cmdIn[11], cmdIn[12]));
    }
    break;

  case 0x05:
    switch(cmdIn[3])
    {
    case 0x01:
      flight->task(Flight::TASK_GOHOME);
      break;
    case 0x02:
      flight->task(Flight::TASK_TAKEOFF);
      break;
    case 0x03:
      flight->task(Flight::TASK_LANDING);
      break;
    }
    break;

  case 0x06:
    if (cmdIn[3] == 0x00)              //0x06 0x00 to stop VRC
    {
      VRCResetData();
      TIM_Cmd(TIM1, DISABLE);
    }
    else if (cmdIn[3] == 0x02)            //0x06 0x01 to turn VRC to F gear
    {
      VRC_TakeControl();
      TIM_Cmd(TIM1, ENABLE);
    }
    else if (cmdIn[3] == 0x01)          //0x06 0x02 to reset data
    {
      VRCResetData();
      TIM_Cmd(TIM1, ENABLE);
    };
    break;

  case 0x07:
    if(cmdIn[3] == 0x00)
    {
      tryHotpoint(cmdIn[4], cmdIn[5], cmdIn[6]);
    }
    else
    {
      stopHotpoint();
    }
    break;
  case 0x08:
        m100.height= m100.altitude=api->getBroadcastData().pos.altitude;
				m100.latitude=api->getBroadcastData().pos.latitude/DEG2RAD(1);
				m100.longitude=api->getBroadcastData().pos.longitude/DEG2RAD(1);
				m100.h_spd = api->getBroadcastData().pos.height;
				m100.q[0]=api->getBroadcastData().q.q0;
				m100.q[1]=api->getBroadcastData().q.q1;
				m100.q[2]=api->getBroadcastData().q.q2;
				m100.q[3]=api->getBroadcastData().q.q3;
				m100.Pitch=api->getBroadcastData().gimbal.pitch;
				m100.Roll=api->getBroadcastData().gimbal.roll;
				m100.Yaw=api->getBroadcastData().gimbal.yaw;
				m100.Bat=api->getBroadcastData().battery;	
				m100.GPS_S=api->getBroadcastData().pos.health;	
				m100.State=api->getBroadcastData().status;	
					
				m100.Rc_p=api->getBroadcastData().rc.pitch;		
				m100.Rc_r=api->getBroadcastData().rc.roll;
				m100.Rc_y=api->getBroadcastData().rc.yaw;
				m100.Rc_t=api->getBroadcastData().rc.throttle;
				m100.Rc_m=api->getBroadcastData().rc.mode;		
				m100.Rc_g=api->getBroadcastData().rc.gear;			

				m100.spd[0]=api->getBroadcastData().v.x;
				m100.spd[1]=api->getBroadcastData().v.y;
				m100.spd[2]=api->getBroadcastData().v.z;
				float q0=m100.q[0],q1=m100.q[1],q2=m100.q[2],q3=m100.q[3];
				m100.Pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; 					// pitch
				m100.Roll =  atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; 	// roll
				m100.Yaw = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1)* 57.3; // yaw 
				SEND_DJI1();//}
    break;				

  case 0x09:
    if (cmdIn[3] == 0x01)
    {
      startLocalNavExample();
    }
    else if(cmdIn[3] == 0x00)
    {
      stopLocalNavExample();
    }
    break;

  default:
    break;
  }
}






#ifdef __cplusplus
extern "C"
{
#endif //__cplusplus
	
void USART2_IRQHandler(void)
{
	static u8 cnt;
	if(USART2->SR & USART_SR_ORE)//ORE中断
	{
		uint8_t com_data = USART2->DR;
	}
	
  if (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == SET)
  {	USART_ClearITPendingBit(USART2,USART_IT_RXNE);//清除中断标志
    uint8_t oneByte = USART_ReceiveData(USART2);
	
    if (myTerminal.rxIndex == 0)
    {
      if (oneByte == 0xFA)
      {
        myTerminal.cmdIn[myTerminal.rxIndex] = oneByte;
        myTerminal.rxIndex = 1;
      }
      else
      {;}
    }
    else
    {
      if (oneByte == 0xFE)    //receive a 0xFE would lead to a command-execution
      {
        myTerminal.cmdIn[myTerminal.rxIndex] = oneByte;
        myTerminal.rxLength = myTerminal.rxIndex + 1;
        myTerminal.rxIndex = 0;
        myTerminal.cmdReadyFlag = 1;
				if(cnt++>20){cnt=0;
					Ultra_Duty();
				}
				IWDG_Feed();//喂狗
      }
      else
      {
        myTerminal.cmdIn[myTerminal.rxIndex] = oneByte;
        myTerminal.rxIndex++;
      }
    }
  }
}
#ifdef __cplusplus
}
#endif //__cplusplus


