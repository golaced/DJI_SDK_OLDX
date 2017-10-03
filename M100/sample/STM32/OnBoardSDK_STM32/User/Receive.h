/*! @file Receive.h
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

#ifndef RECEIVE_H
#define	RECEIVE_H
#include "main.h"

#define MAX_RECEIVE 32
extern u8 en_dji_upload,rst_board;
extern u16 Rc[4];;
class TerminalCommand
{
public:
  uint32_t cmdReadyFlag; //Rx_Handle_Flag
  uint8_t  cmdIn[MAX_RECEIVE]; //Rx_buff
  int32_t  rxIndex;  //Rx_adr
  int32_t  rxLength; //Rx_length

  void terminalCommandHandler(CoreAPI* api, Flight* flight);
  TerminalCommand():cmdReadyFlag(0),rxIndex(0),rxLength(0){;}
};


typedef struct
{
	float altitude;
	double latitude;
	double longitude;
	float Pitch;
	float Roll;
	float Yaw;
	float q[4];
	float height,h_spd;
	float spd[3];
	float Bat;
	int State;
	int Rc_p,Rc_r,Rc_t,Rc_g,Rc_m,Rc_y;
	u8 GPS_S;
}M100;

extern M100 m100;
void SEND_DJI1();
#endif //RECEIVE_H

