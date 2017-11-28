/*
 * my_serial.cpp
 *
 *  Created on: Sep 13, 2016
 *      Author: odroid
 */
#include "my_serial.h"
#include "inifile.h"
#include "lancelot_functions.h"

using namespace std;

serial::Serial my_serial;

float Pitch = 0.0, Yaw = 0.0, Roll = 0.0;
unsigned char data_to_send[50];
int Length = 0;

bool is_print_character;

int get_serial_config(std::string &serial_port_name, int &serial_baudrate)
{
	inifile::IniFile m_inifile;
	m_inifile.load(expand_user("~") + "/Lancelot/config/config.ini");
	int ret = 0;
	serial_port_name = m_inifile.getStringValue("Serial", "serial_port", ret);
	serial_baudrate = m_inifile.getIntValue("Serial", "serial_baudrate", ret);

	return ret;
}

void uartReadThread()
{
	std::string serial_port_name;
	int serial_baudrate = 0;
	get_serial_config(serial_port_name, serial_baudrate);

	cout << "serial_port_name:" << serial_port_name << endl;
	cout << "serial_baudrate:" << serial_baudrate << endl;

	my_serial.setPort(serial_port_name);
	my_serial.setBaudrate(serial_baudrate);
	serial::Timeout timeout(serial::Timeout::simpleTimeout(1000));
	my_serial.setTimeout(timeout);
	my_serial.open();
	if (!my_serial.isOpen())
	{
		cout << "serial open failed!" << endl;
		return;
	}
	size_t data_length = 0;
	unsigned char sum = 0;
	unsigned char data_buf[50] = {0};
	int16_t temp = 0;
	while (true)
	{
		if (flag_LX_target == 0)
		{
			usleep(1000 * delay_ms);
			continue;
		}
		sum = 0;
		data_length = my_serial.available();
		//cout << "data_length:" << data_length << endl;
		if (data_length)
		{
			my_serial.read(data_buf, data_length);
			for (size_t i = 0; i < (data_length - 1); i++)
			{
				sum += *(data_buf + i);
			}

			if (!(sum == *(data_buf + data_length - 1)))
				continue; //判断sum
			if (!(*(data_buf) == 0xAA && *(data_buf + 1) == 0xAF))
				continue; //判断帧头
			if (*(data_buf + 2) == 0x01)
			{
				state_num = data_buf[4];
				target_num = data_buf[5];

				temp = data_buf[6];
				temp <<= 8;
				temp |= data_buf[7];
				Pitch = (float)temp / 10.0f;

				temp = data_buf[8];
				temp <<= 8;
				temp |= data_buf[9];
				Roll = (float)temp / 10.0f;

				temp = data_buf[10];
				temp <<= 8;
				temp |= data_buf[11];
				Yaw = (float)temp / 10.0f;

				for (size_t i = 0; i < 10; i++)
				{
					ignore_char[i] = data_buf[12 + i];
				}
			}
		}
		if (isTerminal)
		{
			break;
		}
		usleep(1000 * delay_ms);
	}
	cout << "thread quit" << endl;
}

void enumerate_ports()
{
	vector<serial::PortInfo> devices_found = serial::list_ports();

	vector<serial::PortInfo>::iterator iter = devices_found.begin();

	while (iter != devices_found.end())
	{
		serial::PortInfo device = *iter++;

		printf("(%s, %s, %s)\n", device.port.c_str(),
			   device.description.c_str(), device.hardware_id.c_str());
	}
}

int mouse_x = 0, mouse_y = 0;

void uartSent(int _type)
{
	switch (_type)
	{
	case UART_SENT_TYPE_MOUSE:
	{
		if (LBtnDown)
		{
			Data_pre_mouse(mouse_x, mouse_y);
		}
		else
		{
			Data_pre_mouse(0, 0);
		}

		break;
	}
	case UART_SENT_TYPE_CHARACTER:
	{
		Data_pre_character();
		break;
	}
	case UART_SENT_TYPE_TARGET:
	{
		Data_pre_target();
		break;
	}
	case UART_SENT_TYPE_SCAN:
	{
		Data_pre_scan_target();
		break;
	}

	default:
		break;
	}

	my_serial.write(data_to_send, Length);
}

void Data_pre_character()
{
	int _cnt = 0, i = 0, sum = 0;
	int char_to_recognition = 0;
	unsigned int _temp;
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0xAF;
	data_to_send[_cnt++] = 0x03;
	data_to_send[_cnt++] = 0;

	if (flag_LX_target == 1)
	{
		char_to_recognition = target_num;
	}
	else
	{
		char_to_recognition = char_num;
	}

	if (number_position_send.number_[0] != char_to_recognition + 48)
	{
		data_to_send[_cnt++] = 0;
	}
	else
	{
		data_to_send[_cnt++] = 1;
		data_to_send[_cnt++] = int(number_position_send.position_.x) / 255;
		data_to_send[_cnt++] = int(number_position_send.position_.x) % 255;
		data_to_send[_cnt++] = int(number_position_send.position_.y);
	}

	data_to_send[3] = _cnt - 4;

	for (i = 0; i < _cnt; i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	Length = _cnt;
}

void Data_pre_mouse(int x, int y)
{
	int _cnt = 0, i = 0, sum = 0;
	unsigned int _temp;
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0xAF;
	data_to_send[_cnt++] = 0x04; //���������������������������
	data_to_send[_cnt++] = 0;	//���������������������������

	data_to_send[_cnt++] = int(LBtnDown);
	data_to_send[_cnt++] = int(x) / 255;
	data_to_send[_cnt++] = int(x) % 255;
	data_to_send[_cnt++] = y;

	data_to_send[3] = _cnt - 4;

	for (i = 0; i < _cnt; i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	Length = _cnt;
}

void Data_pre_target()
{
	int _cnt = 0, i = 0, sum = 0;
	unsigned int _temp;
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0xAF;
	data_to_send[_cnt++] = 0x05;
	data_to_send[_cnt++] = 0;

	data_to_send[_cnt++] = int(have_target);
	data_to_send[_cnt++] = target_num;

	data_to_send[3] = _cnt - 4;

	for (i = 0; i < _cnt; i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	Length = _cnt;
}

void Data_pre_scan_target()
{
	int _cnt = 0, i = 0, sum = 0;
	unsigned int _temp;
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0xAF;
	data_to_send[_cnt++] = 0x06;
	data_to_send[_cnt++] = 0;

	for (size_t i = 0; i < target_global.size(); i++)
	{
		data_to_send[_cnt++] = target_global[i].number_[0] - '0';
		data_to_send[_cnt++] = int(target_global[i].position_.x) / 255;
		data_to_send[_cnt++] = int(target_global[i].position_.x) % 255;
		data_to_send[_cnt++] = int(target_global[i].position_.y);
	}

	data_to_send[3] = _cnt - 4;

	for (i = 0; i < _cnt; i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	Length = _cnt;
}