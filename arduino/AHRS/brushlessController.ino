#define SERVO_ZERO_ANGLE           90
#define SERVO_MIN_ANGLE            60
#define SERVO_MAX_ANGLE            120
#define SERVO_PORT                 10
#define WHEEL_DIAMETER             0.0657 * M_PI
#define TRANSMISSION               61.2
#define WHEEL_BASE                 0.3
#define M_PI                       3.14159
#define ANGLE_FORWARD_CORRECTION   0.7


#include <Brushless_Motorshield.h>
#include <Servo.h> 


Servo servo;
float servo_pos = SERVO_ZERO_ANGLE;
float odom_theta = 0;
float odom_x = 0;
float odom_y = 0;
float var_velocity = -1;
float var_angular_velocity = -1;
float var_x = -1;
float var_y = -1;
float var_theta = -1;
int frameNum = 0;
float last_odom_theta = 0;
float last_odom_x = 0;
float last_odom_y = 0;

void initController()
{
  // Init serial output
 Brushless_Motorshield::init();
 servo.attach(SERVO_PORT); 
}

void printServoAngle()
{
  BufferedUART::write_s("#a ");
  BufferedUART::write_f(-servo_pos + SERVO_ZERO_ANGLE,3);
  BufferedUART::write_c('\n');
}

void setServoAngle()
{
  servo_pos = SERVO_ZERO_ANGLE - BufferedUART::read_f();
  if(servo_pos < SERVO_MIN_ANGLE)
    servo_pos = SERVO_MIN_ANGLE;
  if(servo_pos > SERVO_MAX_ANGLE)
    servo_pos = SERVO_MAX_ANGLE;
  servo.write(servo_pos);
  BufferedUART::write_s("#as ");
  BufferedUART::write_f(-servo_pos + SERVO_ZERO_ANGLE,3);
  BufferedUART::write_c('\n');
}

void setVelocity()
{
  float num = BufferedUART::read_f();
  Brushless_Motorshield::setVelocity(num);
  BufferedUART::write_s("#vs ");
  BufferedUART::write_f(num,3);   
  BufferedUART::write_c('\n');
}

void printVelocity()
{
    float num = Brushless_Motorshield::getVelocity();
    BufferedUART::write_s("#v ");
    BufferedUART::write_f(num,3);
    BufferedUART::write_c('\n');
}

void updateOdometry(float timeDelay_ms)
{
  float velocity = Brushless_Motorshield::getVelocity();
  velocity = velocity / TRANSMISSION * WHEEL_DIAMETER;
  
  float wheelAngle = (servo_pos - SERVO_ZERO_ANGLE);
  if(velocity > 0)
    wheelAngle *= ANGLE_FORWARD_CORRECTION;
    
  float turning_circle = WHEEL_BASE / sin(wheelAngle/180*M_PI);
  float angular_velocity = velocity/turning_circle;
  
  float radius;
  if(angular_velocity == 0)
    radius = velocity * timeDelay_ms / 1000;
  else
    radius = turning_circle;
  
  
  if(angular_velocity != 0)
  {
    odom_theta = odom_theta - angular_velocity * timeDelay_ms / 1000;
    odom_x = odom_x + (sin(odom_theta + angular_velocity * timeDelay_ms / 1000) - sin(odom_theta)) * turning_circle;
    odom_y = odom_y - (cos(odom_theta + angular_velocity * timeDelay_ms / 1000) - cos(odom_theta)) * turning_circle;
  }
  else
  {
    odom_theta = odom_theta;
    odom_x = odom_x + cos(odom_theta) * velocity * timeDelay_ms/1000;
    odom_y = odom_y + sin(odom_theta) * velocity * timeDelay_ms/1000;
  }
       
  if(odom_theta > 2 * M_PI)
    odom_theta -= 2 * M_PI;
       
  if(odom_theta < -2 * M_PI)
    odom_theta += 2 * M_PI;
}

void printOdometryMsg()
{
  float velocity = Brushless_Motorshield::getVelocity();
  velocity = velocity / TRANSMISSION * WHEEL_DIAMETER;
  
  float wheelAngle = (servo_pos - SERVO_ZERO_ANGLE);
  if(velocity > 0)
    wheelAngle *= ANGLE_FORWARD_CORRECTION;
    
  float turning_circle = WHEEL_BASE / sin(wheelAngle/180*M_PI);
  float angular_velocity = velocity/turning_circle;
       
    
  BufferedUART::write_s("#OVD ");
  BufferedUART::write_i(frameNum);   
  BufferedUART::write_c(' '); 
  BufferedUART::write_f(velocity,3);   
  BufferedUART::write_c(' '); 
  BufferedUART::write_f(angular_velocity,3); 
  BufferedUART::write_c(' '); 
  BufferedUART::write_f((odom_x),3); 
  BufferedUART::write_c(' '); 
  BufferedUART::write_f((odom_y),3); 
  BufferedUART::write_c(' '); 
  BufferedUART::write_f((odom_theta),3); 
  BufferedUART::write_c(' '); 
  BufferedUART::write_f(var_velocity,3); 
  BufferedUART::write_c(' '); 
  BufferedUART::write_f(var_angular_velocity,3); 
  BufferedUART::write_c(' '); 
  BufferedUART::write_f(var_x,3); 
  BufferedUART::write_c(' '); 
  BufferedUART::write_f(var_y,3); 
  BufferedUART::write_c(' '); 
  BufferedUART::write_f(var_theta,3);  
  BufferedUART::write_c('\n'); 
  last_odom_theta = odom_theta;
  last_odom_x = odom_x;
  last_odom_y = odom_y;
  frameNum++;
}
