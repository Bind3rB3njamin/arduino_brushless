#include "ArduinoNode.h"
#include <ros/ros.h>



int main(int argc, char **argv) {

    ros::init(argc, argv, ROSNAME);
    ros::NodeHandle n;
    ArduinoNode arduinoNode(n);
    ros::Rate rate(ROS_RATE);

	
    while (ros::ok()) {
	//Update set car values
	arduinoNode.update();
	//Publishes the odometry from car.cpp
	odometry odom = arduinoNode.getOdometry();
	arduinoNode.publishOdometry(odom);
	/// calls all callbacks waiting in the queue
        ros::spinOnce();
        /// sleep for the time remaining to let us hit our publish rate
        rate.sleep();
    }
    return 0;
}

	






