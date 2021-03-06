/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/CommandHome.h>
#include <iostream>
using namespace std;


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pose;
void get_local_pose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose = *msg;
    // ROS_INFO("ori:%f %f %f %f", current_pose.pose.orientation.w, current_pose.pose.orientation.x, 
    //         current_pose.pose.orientation.y, current_pose.pose.orientation.z);
    // ROS_INFO("pos: %f %f %f", current_pose.pose.position.x, current_pose.pose.position.y, 
    //         current_pose.pose.position.z);        
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "set_velocity_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);
    ros::Publisher attitude_velocity_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_attitude/cmd_vel", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::ServiceClient set_home_client = nh.serviceClient<mavros_msgs::CommandHome>("mavros/cmd/set_home");

    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, get_local_pose);
    

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::TwistStamped velocity_tw;
    velocity_tw.twist.linear.z=0.5;
    velocity_tw.twist.linear.x=0;
    velocity_tw.twist.linear.y=0;
    velocity_tw.twist.angular.x = 0.0;
    velocity_tw.twist.angular.y = 0.0;
    velocity_tw.twist.angular.z = 0.0;

    geometry_msgs::TwistStamped attitude_velocity_tw;
    attitude_velocity_tw.twist.linear.z=1;
    attitude_velocity_tw.twist.linear.x=1;
    attitude_velocity_tw.twist.linear.y=1;
    attitude_velocity_tw.twist.angular.z=-1;
    attitude_velocity_tw.twist.angular.x=1;
    attitude_velocity_tw.twist.angular.y=1;

    //send a few setpoints before starting
    for(int i = 20; ros::ok() && i > 0; --i){
        velocity_pub.publish(velocity_tw);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while (ros::ok())
    {
        std::cout << current_state.mode << std::endl;
        // if (current_state.mode != "OFFBOARD" &&
        //     (ros::Time::now() - last_request > ros::Duration(1.0)))
        // {
        //     if( set_mode_client.call(offb_set_mode) &&
        //         offb_set_mode.response.success){
        //         ROS_INFO("Offboard enabled");
        //     }
        //     last_request = ros::Time::now();
        // } else {
        //     if( !current_state.armed &&
        //         (ros::Time::now() - last_request > ros::Duration(1.0))){
        //         if( arming_client.call(arm_cmd) &&
        //             arm_cmd.response.success){
        //             ROS_INFO("Vehicle armed");
        //         }
        //         last_request = ros::Time::now();
        //     }
        // }

        if (current_pose.pose.position.z>3)
        {
            velocity_tw.twist.linear.z = 0.0;
            velocity_tw.twist.angular.z = 0.5;
        }

        //pose.pose.position.x = r * cos(n/200);
        //pose.pose.position.y = r * sin(n/200);
        

        velocity_pub.publish(velocity_tw);
        // attitude_velocity_pub.publish(attitude_velocity_tw);
        // ROS_INFO("%f %f %f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
