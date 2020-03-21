/*
 *
 * File: nexus_base_controller.cpp
 * Purpose: ros nexus base controller node.
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
 *
 *
 *                           -----------------------------
 *                          |                             |
 *            left front M0 |                             | M3 right front
 *                          |                             |
 *                           -----------------------------
 *                          |                             |
 *                          |                             |
 *                          |                             |
 *                          |                             |
 *                          |                             |
 *                          |                             |
 *                          |                             |
 *                          |                             |
 *             left rear M1 |                             | M2 right rear
 *                          |                             |
 *                           -----------------------------
 */


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>
#include "nexus_base_ros/Encoders.h"
#include "nexus_base_ros/Motors.h"
#include "pid_controller.h"


#define LOOP_RATE 20
#define QUEUE_SIZE 1 //subscriber buffer size
#define WHEEL_RADIUS 0.05 // [m]
#define DISTANCE_LEFT_TO_RIGHT_WHEEL .3 // [m]
#define DISTANCE_FRONT_TO_REAR_WHEEL .3 // [m]
#define WHEEL_SEPARATION_WIDTH (DISTANCE_LEFT_TO_RIGHT_WHEEL/2.0)
#define WHEEL_SEPARATION_LENGTH (DISTANCE_FRONT_TO_REAR_WHEEL/2.0)
#define ENC_CPR 12.0	// encoder counts/rev.
#define GEAR_REDUC 64.0	// gears reduction ratio
#define TS (1/20.0)	// loop period in wheel-base Arduino (via parameter sever?)
#define CPP2RADPS (2.0*M_PI/(TS*ENC_CPR*GEAR_REDUC))	// counts per loop period to rad/s conversion factor.
#define DEADBAND 10 // Stops actuating motors when: -DEADBAND < actuation < DEADBAND
		    // too large values will lead to instabillity 

using namespace std;



class NexusBaseController
{
public:
	NexusBaseController();

private:
	void cmdVelCallBack(const geometry_msgs::Twist::ConstPtr& twist_aux);
	void rawVelCallBack(const nexus_base_ros::Encoders::ConstPtr& rawvel_aux);

	ros::NodeHandle nh_;
	ros::Publisher cmd_motor_pub_;
	ros::Publisher odom_pub_;
	ros::Subscriber cmd_vel_sub_;
	ros::Subscriber raw_vel_sub_;
	ros::Time last_time;
	nav_msgs::Odometry odom;
	tf2::Quaternion odom_quat;
	geometry_msgs::TransformStamped odom_trans;

	const float Kp = 4.0; //controller coeff. (via parameter server?)
	const float Ki = 25.0;
	const float Kd = 0.75;
	const float minOutput = -150;
	const float maxOutput = 150;
	double cmd_wheel_left_front_; // [rad/s]
	double cmd_wheel_left_rear_; // [rad/s]
	double cmd_wheel_right_rear_; // [rad/s]
	double cmd_wheel_right_front_; // [rad/s]
	double prev_cmd_wheel_left_front_;
	double prev_cmd_wheel_left_rear_;
	double prev_cmd_wheel_right_rear_;
	double prev_cmd_wheel_right_front_;
	double vel_wheel_left_front_; // [rad/s]
	double vel_wheel_left_rear_; // [rad/s]
	double vel_wheel_right_rear_; // [rad/s]
	double vel_wheel_right_front_; // [rad/s]
	double prev_vel_wheel_left_front_;
	double prev_vel_wheel_left_rear_;
	double prev_vel_wheel_right_rear_;
	double prev_vel_wheel_right_front_;
	double prev_cmd_motor_left_front_;
	double prev_cmd_motor_left_rear_;
	double prev_cmd_motor_right_rear_;
	double prev_cmd_motor_right_front_;
	double linear_velocity_x_;
	double linear_velocity_y_;
	double angular_velocity_z_;
	double dt_;
	double x_pos_;
	double y_pos_;
	double heading_;
	//instantiate PIDs
	PIDControl myPID_wheel_left_front_ = PIDControl(Kp, Ki, Kd, TS, minOutput, maxOutput, AUTOMATIC, DIRECT);
	PIDControl myPID_wheel_left_rear_ = PIDControl(Kp, Ki, Kd, TS, minOutput, maxOutput, AUTOMATIC, DIRECT);
	PIDControl myPID_wheel_right_rear_ = PIDControl(Kp, Ki, Kd, TS, minOutput, maxOutput, AUTOMATIC, DIRECT);
	PIDControl myPID_wheel_right_front_ = PIDControl(Kp, Ki, Kd, TS, minOutput, maxOutput, AUTOMATIC, DIRECT);
};





NexusBaseController::NexusBaseController():
	cmd_wheel_left_front_(0),
	cmd_wheel_left_rear_(0),
	cmd_wheel_right_rear_(0),
	cmd_wheel_right_front_(0),
	prev_cmd_wheel_left_front_(0),
	prev_cmd_wheel_left_rear_(0),
	prev_cmd_wheel_right_rear_(0),
	prev_cmd_wheel_right_front_(0),
	vel_wheel_left_front_(0),
	vel_wheel_left_rear_(0),
	vel_wheel_right_rear_(0),
	vel_wheel_right_front_(0),
	prev_vel_wheel_left_front_(0),
	prev_vel_wheel_left_rear_(0),
	prev_vel_wheel_right_rear_(0),
	prev_vel_wheel_right_front_(0),
	linear_velocity_x_(0),
	linear_velocity_y_(0),
	angular_velocity_z_(0),
	dt_(0),
	x_pos_(0),
	y_pos_(0),
	heading_(0)
{
	cmd_motor_pub_ = nh_.advertise<nexus_base_ros::Motors>("cmd_motor", QUEUE_SIZE);
	odom_pub_ = nh_.advertise<nav_msgs::Odometry>("sensor_odom", QUEUE_SIZE);
	cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", QUEUE_SIZE, &NexusBaseController::cmdVelCallBack, this);
	raw_vel_sub_ = nh_.subscribe<nexus_base_ros::Encoders>("wheel_vel", QUEUE_SIZE, &NexusBaseController::rawVelCallBack, this);
}	




void NexusBaseController::cmdVelCallBack(const geometry_msgs::Twist::ConstPtr& twist_aux)
{
	// - Forward kinematics -
	// Taken from David Kohanbash, Drive Kinematics: Skid Steer & 
	// Mecanum (ROS Twist included), http://robotsforroboticists.com/drive-kinematics/
	cmd_wheel_left_front_ = -(1/WHEEL_RADIUS) * (-twist_aux->linear.x - twist_aux->linear.y - 
		(WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*twist_aux->angular.z);
	cmd_wheel_left_rear_ = -(1/WHEEL_RADIUS) * (-twist_aux->linear.x + twist_aux->linear.y - 
		(WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*twist_aux->angular.z);
	cmd_wheel_right_rear_ = (1/WHEEL_RADIUS) * (-twist_aux->linear.x - twist_aux->linear.y + 
		(WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*twist_aux->angular.z); 
	cmd_wheel_right_front_ = (1/WHEEL_RADIUS) * (-twist_aux->linear.x + twist_aux->linear.y + 
		(WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*twist_aux->angular.z);

/*	// print to console for debugging purpose
	if( (cmd_wheel_left_front_!=prev_cmd_wheel_left_front_) || 
		(cmd_wheel_right_front_!=prev_cmd_wheel_right_front_) || 
		(cmd_wheel_left_rear_!=prev_cmd_wheel_left_rear_) || 
		(cmd_wheel_right_rear_!=prev_cmd_wheel_right_rear_)) {
		ROS_INFO("\ncmd_FL=%.2f\ncmd_FR=%.2f\ncmd_RL=%.2f\ncmd_RR=%.2f\n", 
			cmd_wheel_left_front_, cmd_wheel_right_front_, cmd_wheel_left_rear_, 
			cmd_wheel_right_rear_);
	}
*/
	//store the old values
	prev_cmd_wheel_left_front_ = cmd_wheel_left_front_;
	prev_cmd_wheel_left_rear_ = cmd_wheel_left_rear_;
	prev_cmd_wheel_right_rear_ = cmd_wheel_right_rear_;
	prev_cmd_wheel_right_front_ = cmd_wheel_right_front_;
}





void NexusBaseController::rawVelCallBack(const nexus_base_ros::Encoders::ConstPtr& rawvel_aux)
{
	nexus_base_ros::Motors cmd_motor;
	geometry_msgs::Twist twist;
        nav_msgs::Odometry odom;
	
	// timing
	ros::Time current_time = ros::Time::now();
	dt_ = (current_time - last_time).toSec();
	last_time = current_time;
	//ROS_INFO("dt=%.2f\n", dt_);	//check loop time

	// read encoders. Units are in encoder-ticks/loop-period
	vel_wheel_left_front_ = rawvel_aux->enc0 * CPP2RADPS; //flip sign for the left side
	vel_wheel_left_rear_ = rawvel_aux->enc1 * CPP2RADPS; //flip sign for the left side
	vel_wheel_right_rear_ = rawvel_aux->enc2 * CPP2RADPS;
	vel_wheel_right_front_ = rawvel_aux->enc3 * CPP2RADPS;

	// print to console for debugging purpose
/*	if( (vel_wheel_left_front_ != prev_vel_wheel_left_front_) || 
		(vel_wheel_right_front_ != prev_vel_wheel_right_front_) || 
		(vel_wheel_left_rear_ != prev_vel_wheel_left_rear_) || 
		(vel_wheel_right_rear_ != prev_vel_wheel_right_rear_)) {
		ROS_INFO("\nvel_FL=%.2f\nvel_RL=%.2f\nvel_RR=%.2f\nvel_FR=%.2f\n", 
			vel_wheel_left_front_, vel_wheel_left_rear_, vel_wheel_right_rear_, 
			vel_wheel_right_front_);
	}
	prev_vel_wheel_left_front_ = vel_wheel_left_front_; // store the old values
	prev_vel_wheel_right_rear_ = vel_wheel_right_rear_;
	prev_vel_wheel_left_rear_ = vel_wheel_left_rear_;
	prev_vel_wheel_right_front_ = vel_wheel_right_front_;
*/
	// - Inverse Kinematics -
	// Taken from David Kohanbash, Drive Kinematics: Skid Steer & 
	// Mecanum (ROS Twist included), http://robotsforroboticists.com/drive-kinematics/
	linear_velocity_x_ = -(-vel_wheel_left_front_ + vel_wheel_right_front_ - vel_wheel_left_rear_ + 
		vel_wheel_right_rear_) * (WHEEL_RADIUS/4);
	linear_velocity_y_ = (vel_wheel_left_front_ + vel_wheel_right_front_ - vel_wheel_left_rear_ - 
		vel_wheel_right_rear_) * (WHEEL_RADIUS/4);
	angular_velocity_z_ = (vel_wheel_left_front_ + vel_wheel_right_front_ + vel_wheel_left_rear_ + 
		vel_wheel_right_rear_) * (WHEEL_RADIUS/(4 * (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)));

	double delta_heading = angular_velocity_z_ * dt_; // [radians]
    	double delta_x = (linear_velocity_x_ * cos(heading_) - linear_velocity_y_ * sin(heading_)) * dt_; // [m]
    	double delta_y = (linear_velocity_x_ * sin(heading_) + linear_velocity_y_ * cos(heading_)) * dt_; // [m]

	//calculate current position of the robot
    	x_pos_ += delta_x;
    	y_pos_ += delta_y;
    	heading_ += delta_heading;
	//ROS_INFO("\nx=%.1f, y=%.1f, heading=%.2f\n", x_pos_, y_pos_, heading_);

	// uncomment for feed forward (open loop) testing.
/*	cmd_motor.motor0 = (short) lround(10*cmd_wheel_left_front_);
	cmd_motor.motor1 = (short) lround(10*cmd_wheel_left_rear_);
	cmd_motor.motor2 = (short) lround(10*cmd_wheel_right_rear_);
	cmd_motor.motor3 = (short) lround(10*cmd_wheel_right_front_);
*/
	// do PID control
	myPID_wheel_left_front_.PIDSetpointSet(cmd_wheel_left_front_);
	myPID_wheel_left_front_.PIDInputSet(vel_wheel_left_front_);
	myPID_wheel_left_front_.PIDCompute();
	myPID_wheel_left_rear_.PIDSetpointSet(cmd_wheel_left_rear_);
	myPID_wheel_left_rear_.PIDInputSet(vel_wheel_left_rear_);
	myPID_wheel_left_rear_.PIDCompute();
	myPID_wheel_right_rear_.PIDSetpointSet(cmd_wheel_right_rear_);
	myPID_wheel_right_rear_.PIDInputSet(vel_wheel_right_rear_);
	myPID_wheel_right_rear_.PIDCompute();
	myPID_wheel_right_front_.PIDSetpointSet(cmd_wheel_right_front_);
	myPID_wheel_right_front_.PIDInputSet(vel_wheel_right_front_);
	myPID_wheel_right_front_.PIDCompute();
	// uncomment for closed loop feedback.
	cmd_motor.motor0 = (short) round(myPID_wheel_left_front_.PIDOutputGet());
	cmd_motor.motor1 = (short) round(myPID_wheel_left_rear_.PIDOutputGet());
	cmd_motor.motor2 = (short) round(myPID_wheel_right_rear_.PIDOutputGet());
	cmd_motor.motor3 = (short) round(myPID_wheel_right_front_.PIDOutputGet());

	//publish to motors when actuation is outside DEADBAND margin only.
	if( cmd_motor.motor0 <= -DEADBAND || cmd_motor.motor0 >= DEADBAND ||
		cmd_motor.motor1 <= -DEADBAND || cmd_motor.motor1 >= DEADBAND ||
		cmd_motor.motor2 <= -DEADBAND || cmd_motor.motor2 >= DEADBAND ||
		cmd_motor.motor3 <= -DEADBAND || cmd_motor.motor3 >= DEADBAND )
		{
		cmd_motor_pub_.publish(cmd_motor);
		}

/*	// publish to motors only when actuation value changes.
	if( (cmd_motor.motor0 != prev_cmd_wheel_left_front_) || 
		(cmd_motor.motor1 != prev_cmd_wheel_left_rear_) || 
		(cmd_motor.motor2 != prev_cmd_wheel_right_rear_) || 
		(cmd_motor.motor3 != prev_cmd_wheel_right_front_) )
		{
		cmd_motor_pub_.publish(cmd_motor);
		}
	prev_cmd_motor_left_front_ = cmd_motor.motor0; // store the old values
	prev_cmd_motor_left_rear_ = cmd_motor.motor1;
	prev_cmd_motor_right_rear_ = cmd_motor.motor2;
	prev_cmd_motor_right_front_ = cmd_motor.motor3;
*/

	// The code below for odometry has been copied from the Linobot project, https://github.com/linorobot/linorobot
	// calculate robot's heading in quaternion angle
	// ROS has a function to calculate yaw in quaternion angle
	odom_quat.setRPY(0,0,heading_);
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_footprint";
	// robot's position in x,y, and z
	odom_trans.transform.translation.x = x_pos_;
	odom_trans.transform.translation.y = y_pos_;
	odom_trans.transform.translation.z = 0.0;
	// robot's heading in quaternion
	odom_trans.transform.rotation.x = odom_quat.x();
	odom_trans.transform.rotation.y = odom_quat.y();
	odom_trans.transform.rotation.z = odom_quat.z();
	odom_trans.transform.rotation.w = odom_quat.w();
	odom_trans.header.stamp = current_time;
	// publish robot's tf using odom_trans object
	// odom_broadcaster_.sendTransform(odom_trans);
	odom.header.stamp = current_time;
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_footprint";
	// robot's position in x,y, and z
	odom.pose.pose.position.x = x_pos_;
	odom.pose.pose.position.y = y_pos_;
	odom.pose.pose.position.z = 0.0;
	// robot's heading in quaternion
	odom.pose.pose.orientation.x = odom_quat.x();
	odom.pose.pose.orientation.y = odom_quat.y();
	odom.pose.pose.orientation.z = odom_quat.z();
	odom.pose.pose.orientation.w = odom_quat.w();
	odom.pose.covariance[0] = 0.001;
	odom.pose.covariance[7] = 0.001;
	odom.pose.covariance[35] = 0.001;
	// linear speed from encoders
	odom.twist.twist.linear.x = linear_velocity_x_;
	odom.twist.twist.linear.y = linear_velocity_y_;
	odom.twist.twist.linear.z = 0.0;
	odom.twist.twist.angular.x = 0.0;
	odom.twist.twist.angular.y = 0.0;
	// angular speed from encoders
	odom.twist.twist.angular.z = angular_velocity_z_;
	odom.twist.covariance[0] = 0.0001;
	odom.twist.covariance[7] = 0.0001;
	odom.twist.covariance[35] = 0.0001;
	odom_pub_.publish(odom);

	// check execution time. Make sure there is no overrun.
	//current_time = ros::Time::now();
	//double exect = 1000*(current_time - last_time).toSec();
	//ROS_INFO("exec. time(ms)=%.2f\n", exect);
}





int main(int argc, char** argv)
{
	ros::init(argc, argv, "nexus_base_controller");
	NexusBaseController nexus_base_controller;

	ros::spin();
}
