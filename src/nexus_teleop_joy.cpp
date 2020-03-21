/*
*
* File: nexus_teleop_joy.cpp
* Purpose: ros joystick listener node.
* Version: 1.0.0
* File Date: 21-03-2020
* Release Date: 21-03-2020
* URL: https://github.com/MartinStokroos/nexus_base_ros
* License: MIT License
*
*
* Copyright (c) M.Stokroos 2020
*
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy,
* modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
* The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
* WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
* COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <iostream>
#include <math.h>
#include "nexus_base_ros/EmergencyStopEnable.h"
#include "nexus_base_ros/ArmingEnable.h"

#define QUEUE_SIZE 16 //subscriber buffer size

using namespace std;




class TeleopJoy{
public:
  TeleopJoy();
private:
  void callBack(const sensor_msgs::Joy::ConstPtr& joy);
  ros::NodeHandle n;
  ros::Publisher pub;
  ros::Subscriber sub;
  ros::ServiceClient emergency_cli;
  ros::ServiceClient arming_cli;
  int i_velLinear , i_velAngular;
  const float max_vel_x = 1.0; // [m/s]
  const float max_vel_y = 1.0; // [m/s]
  const float max_vel_th = M_PI_4; // quarter pi [rad/sec]
  bool armed = true;
};



TeleopJoy::TeleopJoy()
{   i_velLinear = 1;
	i_velAngular = 0; 
	n.param("axis_linear", i_velLinear, i_velLinear);
	n.param("axis_angular", i_velAngular, i_velAngular);
	pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	sub = n.subscribe<sensor_msgs::Joy>("joy", QUEUE_SIZE, &TeleopJoy::callBack, this);
	emergency_cli = n.serviceClient<nexus_base_ros::EmergencyStopEnable>("emergency_stop_enable");
	arming_cli = n.serviceClient<nexus_base_ros::ArmingEnable>("arming_enable");
}



void TeleopJoy::callBack(const sensor_msgs::Joy::ConstPtr& joy)
{
	geometry_msgs::Twist vel;
	nexus_base_ros::EmergencyStopEnable srv1;
	nexus_base_ros::ArmingEnable srv2;
	
	if(armed) {
		vel.linear.x = max_vel_x*joy->axes[3]; // right handle up/down
		vel.linear.y = -max_vel_y*joy->axes[2]; // right handle left-to-right
		vel.angular.z = -max_vel_th*joy->axes[0]; // left handle left-to-right
	}
	else {
		vel.linear.x = 0;
		vel.linear.y = 0;
		vel.angular.z = 0;
	}

	if(joy->buttons[0]){
		ROS_INFO("Blue button pressed.\n");
	}

	if(joy->buttons[1]){
		ROS_INFO("Green button pressed: ");
		srv2.request.enable = true;
		if (arming_cli.call(srv2)) {
			armed = true;
			ROS_INFO("nexus-base armed: %d\n", (int)srv2.response.success);
			}
		else {
			armed = false;
			ROS_ERROR("Failed to call the service Arming!\n");
		}

	}

	if(joy->buttons[2]){
		ROS_INFO("Red button pressed: ");
		srv1.request.enable = true;
		if (emergency_cli.call(srv1)) {
			armed = false;
			ROS_INFO("emergency stop: %d\n", (int)srv1.response.success);
			}
		else {
			armed = false;
			ROS_ERROR("Failed to call the service Emergency Stop!\n");
		}
	}

	if(joy->buttons[3]){
		ROS_INFO("Yellow button pressed.\n");
	}

	if(joy->buttons[4]){
		ROS_INFO("LB pressed.\n");
	}

	if(joy->buttons[5]){
		ROS_INFO("RB pressed.\n");
	}

	if(joy->buttons[6]){
		ROS_INFO("LT pressed.\n");
	}

	if(joy->buttons[7]){
		ROS_INFO("RT pressed.\n");
	}

	if(joy->buttons[8]){
		ROS_INFO("Back button pressed.\n");
	}

	if(joy->buttons[9]){
		ROS_INFO("Start button pressed.\n");
	}

	pub.publish(vel);
}




int main(int argc, char** argv)
{
	ros::init(argc, argv, "nexus_teleop_joy");
	TeleopJoy teleop_robot;
	ros::spin();
}
