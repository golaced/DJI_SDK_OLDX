///*
// * my_serial.cpp
// *
// *  Created on: Sep 13, 2016
// *      Author: odroid
// */
#include "my_serial.h"
#include "AttitudePosition.h"
#include <iostream>
using namespace std;
//

//#ifdef _WIN32
//serial::Serial serial_port("COM1", 230400, serial::Timeout::simpleTimeout(1000));
//#else
serial::Serial serial_port("/dev/ttyUSB0", 230400, serial::Timeout::simpleTimeout(1000));
//#endif // _WIN32

unsigned char data_to_send[50];
int Length = 0;

void serialSent()
{

	int _cnt = 0, i = 0, sum = 0;
    int Locate[5] = {0,0,0,0,0};

	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0xAF;
	data_to_send[_cnt++] = 0x21;
	data_to_send[_cnt++] = 0;

	data_to_send[_cnt++] = Markers.size();
	data_to_send[_cnt++] = int(coordinate_camera.x) >> 8;
	data_to_send[_cnt++] = int(coordinate_camera.x) % 256;
	data_to_send[_cnt++] = int(coordinate_camera.y) >> 8;
	data_to_send[_cnt++] = int(coordinate_camera.y) % 256;
	data_to_send[_cnt++] = int(coordinate_camera.z) >> 8;
	data_to_send[_cnt++] = int(coordinate_camera.z) % 256;
	data_to_send[_cnt++] = int(atti_camera.Pit) >> 8;
	data_to_send[_cnt++] = int(atti_camera.Pit) % 256;
	data_to_send[_cnt++] = int(atti_camera.Rol) >> 8;
	data_to_send[_cnt++] = int(atti_camera.Rol) % 256;
	data_to_send[_cnt++] = int(atti_camera.Yaw) >> 8;
	data_to_send[_cnt++] = int(atti_camera.Yaw) % 256;


    if(Markers.size() == 0)
    {
        data_to_send[_cnt++] = 0;
        data_to_send[_cnt++] = 0;
        data_to_send[_cnt++] = 0;
        data_to_send[_cnt++] = 0;
	data_to_send[_cnt++] = 0;
        data_to_send[_cnt++] = 0;
    }
    else
    {
        data_to_send[_cnt++] = int(markerorigin_img.val[0]) >> 8;
        data_to_send[_cnt++] = int(markerorigin_img.val[0]) % 256;
        data_to_send[_cnt++] = int(markerorigin_img.val[1]) >> 8;
        data_to_send[_cnt++] = int(markerorigin_img.val[1]) % 256;
    }

    for(size_t i=0;i<objectLocate.size();i++)
    {
        Locate[objectLocate[i]] = bool(objectLocate[i]);
    }

    data_to_send[_cnt++] = Locate[1];
    data_to_send[_cnt++] = Locate[2];
    data_to_send[_cnt++] = Locate[3];
    data_to_send[_cnt++] = Locate[4];
    
    data_to_send[_cnt++] = int(markerorigin_img.val[2]) >> 8;
    data_to_send[_cnt++] = int(markerorigin_img.val[2]) % 256;
    data_to_send[_cnt++] = int(markerorigin_img.val[3]) >> 8;
    data_to_send[_cnt++] = int(markerorigin_img.val[3]) % 256;

	data_to_send[3] = _cnt - 4;

	for (i = 0; i < _cnt; i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	Length = _cnt;

	serial_port.write(data_to_send, Length);
}
