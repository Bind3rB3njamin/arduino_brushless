#include "ArduinoNode.h"   

	
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
	
	static tf::TransformBroadcaster br;
  	tf::Transform transform;
  	transform.setOrigin( tf::Vector3(od.pos.x, od.pos.y, 0.0) );
  	tf::Quaternion q;
  	q.setRPY(0, 0, od.pos.theta);
  	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "odom"));

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
