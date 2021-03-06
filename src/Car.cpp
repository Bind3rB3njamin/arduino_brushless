#include "Car.h"

Car::Car() : _serial(DEVICE,BAUD), _threadedReader(DEVICE,BAUD)
{
	_threadedReader.start();
	_speedChanged = true;
	_angleChanged = true;	
}
 

void Car::setValues(float speed, float turn)
{
	float angle = 0.0;
	if(turn > 0)
		angle = M_PI/2;
	else if(turn < 0)
		angle = -M_PI/2;

	if(speed != 0)
	{
		if(abs(turn/speed*CAR_LENGTH) < 1)
			angle = asinf(CAR_LENGTH * turn / speed);
	}

	
	setSpeed(speed);
	setAngle(angle);
}

void Car::setJoyValues(float speed, float angle)
{
	setSpeed(speed * MOTOR_JOY_MAX);
	setAngle(angle * STEERING_ANGLE_MAX);
}

void Car::setSpeed(float speed)
{

	if(speed == 0)
	{
	}
	else if(speed < MOTOR_JOY_MIN)
	{
		speed = -MOTOR_SPEED_MIN;
	}
	else if(speed > MOTOR_JOY_MAX)
	{
		
		speed = -MOTOR_SPEED_MAX;
	}
	else if(speed >= 0)
	{
		speed = -speed / MOTOR_JOY_MAX * (MOTOR_SPEED_MAX);
	}
	else
	{
		speed = -speed / MOTOR_JOY_MIN * (MOTOR_SPEED_MIN);
	}
	
				
	
	if(_speed != speed)
		_speedChanged = true;

	_speed = speed;
}
void Car::setAngle(float angle)
{
	
	if(angle < STEERING_ANGLE_MIN)
	{
		angle = DEG(STEERING_ANGLE_MIN);
	}
	else if(angle > STEERING_ANGLE_MAX)
	{
		angle = DEG(STEERING_ANGLE_MAX);
	}
	else if(angle >= 0)
	{
		angle = DEG(angle);
	}
	else
	{
		angle = DEG(angle);
	}
	

	if(_angle != angle)
		_angleChanged = true;
	
	_angle = angle;
}

void Car::update()
{
	//everyRosTurn read Odometry
	std::string cmd = "#f";
	_serial.writeString((char*)cmd.c_str(), cmd.size());

	//Read and compare Values
	while(!_threadedReader.empty())
	{
		std::string cmd = _threadedReader.getNextLine();
		std::size_t cmdStart = cmd.find("#");

		if(cmdStart != std::string::npos)
		{	
			cmd = cmd.substr(cmdStart);

			char c_cmd[cmd.size() + 1];
			strncpy(c_cmd, cmd.c_str(), cmd.size()+1); 

			if(c_cmd[1] == 'v')
			{
				if(c_cmd[2] == 's')
				{
					char* pEnd;
					double setSpeed = strtod(&c_cmd[3], &pEnd);
					if(abs(setSpeed - _speed) < 0.01)
						_speedChanged = false;
				}
			}
			else if(c_cmd[1] == 'a')
			{
				if(c_cmd[2] == 's')
				{
					char* pEnd;
					double setAngle = strtod(&c_cmd[3], &pEnd);
					if(abs(setAngle - _angle) < 0.01)
						_angleChanged = false;
				}
			}
			else if(c_cmd[1] == 'O' && c_cmd[2] == 'V' && c_cmd[3] == 'D')
			{
				std::vector<std::string> tokens;
				boost::algorithm::split(tokens, cmd, boost::algorithm::is_any_of(" "));
		
				if(tokens.size() >= 12)
				{
					_odometry.frameID = std::stoi(tokens[1]);
					_odometry.speed.x = std::stod(tokens[2]);
					_odometry.speed.y = 0;
					_odometry.speed.z = 0;
					_odometry.angularSpeed.x = std::stod(tokens[3]);
					_odometry.angularSpeed.y = 0;
					_odometry.angularSpeed.z = 0;
					_odometry.pos.x = std::stod(tokens[4]);
					_odometry.pos.y = std::stod(tokens[5]);
					_odometry.pos.theta = std::stod(tokens[6]);
					_odometry.varianzSpeed.x = std::stod(tokens[7]);
					_odometry.varianzSpeed.y = 0;
					_odometry.varianzSpeed.z = 0;
					_odometry.varianzAngularSpeed.x = std::stod(tokens[8]);
					_odometry.varianzAngularSpeed.y = 0;
					_odometry.varianzAngularSpeed.z = 0;
					_odometry.varianzPos.x = std::stod(tokens[9]);
					_odometry.varianzPos.y = std::stod(tokens[10]);
					_odometry.varianzPos.theta = std::stod(tokens[11]);

				}
			}

		}
	}

	if(_angleChanged)
	{
		std::string cmd = "#as " + std::to_string(round(_angle * 100)/100);
		_serial.writeString((char*)cmd.c_str(), cmd.size());
	}
	if(_speedChanged)
	{
		std::string cmd = "#vs " + std::to_string(round(_speed * 100)/100);
		_serial.writeString((char*)cmd.c_str(), cmd.size());
	}
}

odometry Car::getOdometry()
{
	return _odometry;
}
