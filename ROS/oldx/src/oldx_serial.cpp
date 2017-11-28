#include <ros/ros.h>
#include <serial/serial.h>
#include <serial/v8stdint.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/ByteMultiArray.h>
#include <geometry_msgs/TwistStamped.h>
#include "oldx.h"

using std::cout;
using std::endl;

serial::Serial m_serial;

float x = 0.0, y = 0.0, z = 0.0, yaw = 0.0, yaw_rad = 0.0;
float x_min=0.0,x_max=0.0,y_min=0.0,y_max=0.0,z_min=0.0,z_max=0.0,yaw_min=0.0,yaw_max=0.0;

std_msgs::ByteMultiArray data_to_send;
void oldx_send_cb(const std_msgs::ByteMultiArray::ConstPtr &msg)
{
	data_to_send.data.clear();
	data_to_send = *msg;
}

void SEND_PX4(void)
{
	// cout<<"*****"<<endl;
	m_serial.write((unsigned char *)data_to_send.data.data(), data_to_send.data.size());
}

bool checkcommand(const unsigned char *data_buf, int data_length)
{
	if (!(*(data_buf) == 0xFA && *(data_buf + 1) == 0xFB && *(data_buf + 2) == 0x04 && *(data_buf + 3) == 0x01))
	{
		return false; //判断帧头
	}
	if (*(data_buf + data_length - 1) != 0xFE)
	{
		return false;
	}
	return true;
}
void decode(const unsigned char *data_buf, int data_length)
{
	int16_t temp = 0; //temp==>mm, x==>m
	unsigned char mode = data_buf[4];
	temp = data_buf[5];
	temp <<= 8;
	temp |= data_buf[6];
	x = (float)temp / 1000;
	temp = data_buf[7];
	temp <<= 8;
	temp |= data_buf[8];
	y = (float)temp / 1000;
	temp = data_buf[9];
	temp <<= 8;
	temp |= data_buf[10];
	z = (float)temp / 1000;
	temp = data_buf[11];
	temp <<= 8;
	temp |= data_buf[12];
	yaw = (float)temp / 100; //degree

	x = LIMIT(x, x_min, x_max);
	y = LIMIT(y, y_min, y_max);
	z = LIMIT(z, z_min, z_max);
	yaw = LIMIT(yaw, yaw_min, yaw_max);
	yaw_rad = -1 * yaw / 57.3;

#ifdef DEBUG_COUT
	cout << x << " " << y << " " << z << " " << yaw << endl;
#endif
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "oldx_serial");
	ros::NodeHandle nh;
	ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("oldx/velocity", 1);
	ros::Subscriber sub = nh.subscribe("oldx/oldx_send", 1, oldx_send_cb);

	std::string port_name;
	int baudrate = 230400;
	serial::Timeout timeout(serial::Timeout::simpleTimeout(1000));
	ros::param::get("~port_name", port_name);
	ros::param::get("~baudrate", baudrate);
	ros::param::get("~x_min", x_min);
	ros::param::get("~x_max", x_max);
	ros::param::get("~y_min", y_min);
	ros::param::get("~y_max", y_max);
	ros::param::get("~z_min", z_min);
	ros::param::get("~z_max", z_max);
	ros::param::get("~yaw_min", yaw_min);
	ros::param::get("~yaw_max", yaw_max);

	ROS_INFO("serial port name:%s", port_name.c_str());
	ROS_INFO("serial baudrate:%d", baudrate);

	m_serial.setPort(port_name);
	m_serial.setBaudrate(baudrate);
	m_serial.setTimeout(timeout);
	m_serial.open();
	if (!m_serial.isOpen())
	{
		cout << "serial open failed!" << endl;
		return -1;
	}

	size_t data_length = 0;
	unsigned char sum = 0;
	unsigned char data_buf[BUFF_SIZE] = {0};
	int16_t temp = 0;

	geometry_msgs::TwistStamped velocity;

	ros::Rate loop_rate(200);

	while (ros::ok())
	{
		data_length = m_serial.available();
		if (data_length)
		{
			m_serial.read(data_buf, data_length);
			if (checkcommand(data_buf, data_length))
			{
				decode(data_buf, data_length);
			}
		}

		velocity.twist.linear.x = x;
		velocity.twist.linear.y = y;
		velocity.twist.linear.z = z;

		velocity.twist.angular.z = yaw_rad;

		velocity_pub.publish(velocity);

		SEND_PX4();

		ros::spinOnce();

		loop_rate.sleep();
	}

	cout << "serial quit" << endl;

	return 0;
}