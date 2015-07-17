#ifndef ARDUINO_NODE_H
#define ARDUINO_NODE_H

#define ROSNAME 		"CarController"
#define JOY_VALUES		"joy"
#define SPEED_VALUES		"speed"
#define ODOMETRY_VALUES 	"odometry"
#define ROS_RATE		20
#define STEERING_SPEED_MAX	15.0
#define USE_JOY			1

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include "Car.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>


class ArduinoNode : public Car
{
public:
	ArduinoNode(ros::NodeHandle & n);
	void callbackCarData (const sensor_msgs::Joy _data) ;
	void callbackSpeedData (const geometry_msgs::Twist _data) ;
	void publishOdometry(odometry od);

private:
	ros::NodeHandle n_;
	ros::Subscriber sub_joy_data_;
	ros::Subscriber sub_speed_data_;
	ros::Publisher 	pub_odometry_;
};

#endif 
