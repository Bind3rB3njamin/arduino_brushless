#ifndef SERVO_H
#define SERVO_H


//car
#define CAR_LENGTH 			0.4

//steering
#define STEERING_ANGLE_MAX		0.392699082
#define STEERING_ANGLE_MIN		-0.392699082


//motor
#define MOTOR_SPEED_MAX			200.0
#define MOTOR_SPEED_MIN			-200.0
#define MOTOR_SPEED_ZERO		0.0
#define MOTOR_JOY_MIN			-15
#define MOTOR_JOY_MAX			15

#define DEVICE				"/dev/ttyACM0"
#define BAUD				115200

#define DEG(X)	(X * 180 / M_PI)
#define RAD(X)	(X * M_PI / 180)


#include "threadedReader.h"
#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <string>
#include <tf/transform_broadcaster.h>

typedef struct s_vector3d
{
	double x;
	double y;
	double z;
}vector3d;

typedef struct s_robotPose
{
	double x;
	double y;
	double theta;
}robotPose;

typedef struct s_odometry
{
	int frameID;
	vector3d speed;
	vector3d angularSpeed; 
	robotPose pos;
	vector3d varianzSpeed;
	vector3d varianzAngularSpeed; 
	robotPose varianzPos;
}odometry;

/**
 * Class
 */
class Car
{

public:
    /// Konstruktor
	Car();
	void setValues(float speed, float turn);
	void setJoyValues(float speed, float angle);
	void setSpeed(float speed);
	void setAngle(float angle);
	void update();
	odometry getOdometry();
private:
	double _angle;
	double _speed;
	Serial _serial;
	threadedReader _threadedReader;
	void setServoPosition(unsigned int target, unsigned int position);
	bool _speedChanged;
	bool _angleChanged;
	odometry _odometry;
};



#endif // Servo_H 
