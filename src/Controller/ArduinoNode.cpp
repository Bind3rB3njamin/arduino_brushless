#include "ArduinoNode.h"   
#include <tf/transform_broadcaster.h>



int main(int argc, char **argv) {

    ros::init(argc, argv, ROSNAME);
    ros::NodeHandle n;
    ArduinoNode arduinoNode(n);
    ros::Rate rate(ROS_RATE);

	
    while (ros::ok()) {
	//Update set car values
	arduinoNode.update();
	//Publishes the odometry from car.cpp
	arduinoNode.publishOdometry(arduinoNode.getOdometry());
        /// calls all callbacks waiting in the queue
        ros::spinOnce();
        /// sleep for the time remaining to let us hit our publish rate
        rate.sleep();
    }
    return 0;
}

	
ArduinoNode::ArduinoNode(ros::NodeHandle & n) : Car()
{
    /// subscriber to twist messages
	sub_joy_data_ = n.subscribe(JOY_VALUES, 1, &ArduinoNode::callbackCarData, this);
	sub_speed_data_ = n.subscribe(SPEED_VALUES, 1, &ArduinoNode::callbackSpeedData, this);
	pub_odometry_ = n.advertise<nav_msgs::Odometry>(ODOMETRY_VALUES, 1);
	
}


void ArduinoNode::callbackCarData (const sensor_msgs::Joy _data) 
{	
	if(USE_JOY)
	{
		//axis 2 == left right
		//axis 3 == forward back
		setJoyValues(-_data.axes[3], _data.axes[0]);
	}
}

void ArduinoNode::callbackSpeedData (const geometry_msgs::Twist _data) 
{
	if(!USE_JOY)
	{
		setValues(_data.linear.x, _data.angular.z);
	}
	
}



void ArduinoNode::publishOdometry(odometry od)
{
	//Publishing transform
	
  	tf::Transform transform;
  	transform.setOrigin( tf::Vector3(od.pos.x, od.pos.y, 0.0) );
  	tf::Quaternion q;
  	q.setRPY(0, 0, od.pos.theta);
  	transform.setRotation(q);
	//br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "odom"));

	//Publishing odometry
	nav_msgs::Odometry odo;
	odo.header.seq = od.frameID;
	odo.header.stamp = ros::Time::now();
	odo.header.frame_id = "base_link";
	odo.child_frame_id = od.frameID;
	//poseWithCovarianz;	Position; 	PositionLinear;
	odo.pose.pose.position.x = od.pos.x;
	odo.pose.pose.position.y = od.pos.y;
	odo.pose.pose.position.z = 0;
	//					PositionQuaternion
	//1 0 0 0 standard 1 0 0 
	/*double c1 = cos(0/2);			//rot y
	double c2 = cos(od.pos.theta/2);	//rot z
	double c3 = cos(0/2);   		//rot x
	
	double s1 = sin(0/2);
	double s2 = sin(od.pos.theta/2);
	double s3 = sin(0/2);

	odo.pose.pose.orientation.w = c1*c2*c3 - s1*s2*s3;
	odo.pose.pose.orientation.x = s1*s2*c3 + c1*c2*s3;
	odo.pose.pose.orientation.y = s1*c2*c3 + c1*s2*s3;
	odo.pose.pose.orientation.z = c1*s2*c3 - s1*c2*s3;
*/

	odo.pose.pose.orientation.w = q.w();
	odo.pose.pose.orientation.x = q.x();
	odo.pose.pose.orientation.y = q.y();
	odo.pose.pose.orientation.z = q.z();
	//COvarianz missing

	//TwistWIth Covarianz; 	Twist;	TwistLinear;
	odo.twist.twist.linear.x = od.speed.x;
	odo.twist.twist.linear.y = od.speed.y;
	odo.twist.twist.linear.z = od.speed.z;
	
	odo.twist.twist.angular.x = od.angularSpeed.x;
	odo.twist.twist.angular.y = od.angularSpeed.y;
	odo.twist.twist.angular.z = od.angularSpeed.z;

	//Covarianz missing

	
	pub_odometry_.publish(odo);
}
