#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/ByteMultiArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/WaypointPull.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/WaypointSetCurrent.h>
#include <mavros_msgs/RCIn.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include "oldx.h"

#if ROS_VERSION_GE(ROS_VERSION_MAJOR, ROS_VERSION_MINOR, ROS_VERSION_PATCH, 1, 12, 7)
#include <sensor_msgs/BatteryState.h>
#else
#include <mavros_msgs/BatteryStatus.h>
#endif

using std::cout;
using std::endl;

double current_yaw;
double current_yaw_rad,current_yaw_x;

std_msgs::ByteMultiArray data_to_publish;

struct px4_t
{
	double Pitch;
	double Roll;
	double Yaw;
	int16_t Bat;
	int16_t Rc_g;
	int16_t Rc_m;
	int16_t Rc_p;
	int16_t Rc_r;
	int16_t Rc_t;
	int16_t Rc_y;
	int16_t GPS_S;
	int16_t State;

	double altitude;
	double h_spd;

	double longitude;
	double latitude;

	double spd[3];
	
};
px4_t px4;

mavros_msgs::RCIn rcIn;
void get_rc_in(const mavros_msgs::RCIn::ConstPtr &msg)
{
    rcIn = *msg;
	px4.Rc_p = rcIn.channels[0];
	px4.Rc_m = rcIn.channels[1];
	px4.Rc_g = rcIn.channels[2];
	px4.Rc_r = rcIn.channels[3];
	px4.Rc_t = rcIn.channels[4];
	px4.Rc_y = rcIn.channels[5];
}

double current_battery_percentage = 1.0;
#if ROS_VERSION_GE(ROS_VERSION_MAJOR, ROS_VERSION_MINOR, ROS_VERSION_PATCH, 1, 12, 7)
sensor_msgs::BatteryState current_battery;
void get_battery(const sensor_msgs::BatteryState::ConstPtr &msg)
{
    current_battery = *msg;
    current_battery_percentage = current_battery.percentage;
	px4.Bat = current_battery_percentage*100;
    //ROS_INFO("Battery: %f", current_battery_percentage);
}
#else
mavros_msgs::BatteryStatus current_battery;
void get_battery(const mavros_msgs::BatteryStatus::ConstPtr &msg)
{
    current_battery = *msg;
    current_battery_percentage = current_battery.remaining;
	px4.Bat = current_battery_percentage*100;
    //ROS_INFO("Battery: %f", current_battery_percentage);
}
#endif

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
	px4.State = current_state.connected;
}

geometry_msgs::PoseStamped current_local_pose;
void get_local_pose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_local_pose = *msg;
	px4.altitude = current_local_pose.pose.position.z;
}

geometry_msgs::TwistStamped current_local_pose_velocity;
void get_local_pose_velocity(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    current_local_pose_velocity = *msg;
	px4.h_spd = current_local_pose_velocity.twist.linear.z;
	px4.spd[0]=current_local_pose_velocity.twist.linear.x*cos(current_yaw_rad)-current_local_pose_velocity.twist.linear.y*sin(current_yaw_rad);
	px4.spd[1]=current_local_pose_velocity.twist.linear.y*cos(current_yaw_rad)+current_local_pose_velocity.twist.linear.x*sin(current_yaw_rad);
	px4.spd[2]=current_local_pose_velocity.twist.linear.z;
}

sensor_msgs::Imu current_imu;

void imu_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
    current_imu = *msg;
    double q0,q1,q2,q3;
    q0 = current_imu.orientation.w;
    q1 = current_imu.orientation.x;
    q2 = current_imu.orientation.y;
    q3 = current_imu.orientation.z;

    double Rol = atan2(2 * (q0 * q1 + q2 * q3), 1 + 2 * (q1 * q1 + q2 * q2)) * 57.3f;
    double Pit = asin(2 * (-q1 * q3 + q0 * q2)) * 57.3f;
    double Yaw = atan2(2 * (-q1 * q2 - q0 * q3), 1 - 2 * (q0 * q0 + q1 * q1)) * 57.3f;
    current_yaw_x=atan2(2. * (q0 * q3 + q1 * q2), 1. - 2. * (q2 * q2 + q3 * q3)) * 57.3f;
    current_yaw = 90 - current_yaw_x;
    if (current_yaw < 0)
    {
        current_yaw += 360;
    }
    current_yaw_rad = current_yaw / 57.3f;

	px4.Pitch = Pit;
	px4.Roll = Rol;
	px4.Yaw = current_yaw;
}

sensor_msgs::NavSatFix current_gps;
void get_gps_cb(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    current_gps = *msg;
	px4.GPS_S = current_gps.status.status;
	px4.longitude = current_gps.longitude;
	px4.latitude = current_gps.latitude;
    //ROS_INFO("%.20f %.20f %lf",current_gps.longitude,current_gps.latitude,current_gps.altitude);
}

void format_data_to_send(void)
{	
#ifdef DEBUG_COUT
	cout<<"--------------"<<endl;

	cout<<"spd:"<<px4.spd[0]<<" "<<px4.spd[1]<<" "<<px4.spd[2]<<" "<<endl;
	cout<<"h_spd:"<<px4.h_spd<<endl;
	cout<<"rc:"<<px4.Rc_p<<" "<<px4.Rc_m<<" "<<px4.Rc_g<<" "<<px4.Rc_r<<" "<<px4.Rc_t<<" "<<px4.Rc_y<<" "<<endl;
	cout<<"px4.GPS:"<<px4.longitude<<" "<<px4.latitude<<endl;
	cout<<"px4.GPS state:"<<px4.GPS_S<<" "<<endl;
	cout<<"P:"<<px4.Pitch<<" R:"<<px4.Roll<<" Y:"<<px4.Yaw<<" "<<endl;
	cout<<"alt:"<<px4.altitude<<" "<<endl;
	cout<<"State:"<<px4.State<<" "<<endl;
	cout<<"Bat:"<<px4.Bat<<" "<<endl;

	cout<<"--------------"<<endl;
#endif

	u8 sum = 0;
	u8 data_to_send[BUFF_SIZE];
	u8 _cnt=0;
	vs16 _temp;
	u32 _temp32;

  	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	

	//imu_pub
	_temp = (vs16)(px4.Pitch*10);//Pitch;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(px4.Roll*10);//Roll;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(px4.Yaw*10);//Yaw;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	//local_position
	_temp = (vs16)(px4.altitude*1000);//altitude;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(px4.h_spd*1000);//height velocity;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	//global_position 
	_temp = (vs32)(px4.latitude);//latitude;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp32 = (vs32)((px4.latitude-(int)(px4.latitude))*1000000000);//latitude;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);

	_temp = (vs32)(px4.longitude);//latitude;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp32 = (vs32)((px4.longitude-(int)(px4.longitude))*1000000000);//latitude;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);

	_temp = (vs16)(px4.Bat);//Bat;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp = (vs16)(px4.Rc_p);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(px4.Rc_r);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(px4.Rc_y);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(px4.Rc_t);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(px4.Rc_m);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(px4.Rc_g);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)(px4.State);//ultra_distance;
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(px4.GPS_S);//GPS_S;
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(px4.spd[0]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(px4.spd[1]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(px4.spd[2]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;

	for(int i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	data_to_publish.data.clear();
	for (int i = 0; i < _cnt; i++)
	{
		data_to_publish.data.push_back(data_to_send[i]);
	}
	
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "px4_info");
    ros::NodeHandle nh;
    ros::Publisher px4_info_pub = nh.advertise<std_msgs::ByteMultiArray>("oldx/oldx_send", 1);
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 1, state_cb);
	ros::Subscriber battery_sub = nh.subscribe("mavros/battery", 1, get_battery);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
		("mavros/local_position/pose", 1, get_local_pose);
	ros::Subscriber local_pos_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
		("mavros/local_position/velocity", 1, get_local_pose_velocity);
    ros::Subscriber get_gps_sub = nh.subscribe<sensor_msgs::NavSatFix>
    	("/mavros/global_position/global", 1, get_gps_cb);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>
    	("/mavros/imu/data",1,imu_cb);
	
	ros::Rate loop_rate(100);

	while (ros::ok())
	{
		format_data_to_send();
		px4_info_pub.publish(data_to_publish);

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}