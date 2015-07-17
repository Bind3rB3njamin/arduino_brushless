#define VELOCITY_COMAND_CAHR      'v'
#define SERVO_ANGLE_COMAND_CHAR   'a'
//#define STREAMING_COMMAND         's'

//#define ENABLE                    'e'
//#define DISABLE                   'd'
#define GET_COMAND_CHAR           'g'
#define SET_COMAND_CHAR           's'

#define SERVO_ZERO_ANGLE           90
#define SERVO_MIN_ANGLE            60
#define SERVO_MAX_ANGLE            120
#define SERVO_PORT                 10

#include <BufferedUART.h>
#include <Brushless_Motorshield.h>
#include <Servo.h> 



boolean set_velocity = false;
boolean print_velocity = false;
boolean set_servo_angle = false;
boolean print_servo_angle = false;

Servo servo;
float servo_pos = SERVO_ZERO_ANGLE;

void setup()
{
  // Init serial output
 Brushless_Motorshield::init();
  BufferedUART::init();
  servo.attach(SERVO_PORT); 
  BufferedUART::write_s("reset\n");
}


// Main loop
void loop()
{
  // Read incoming control messages
  if (BufferedUART::bufferedLength() >= 2)
  {
    if (BufferedUART::read_c() == '#') // Start of new control message
    {
      char command = BufferedUART::read_c(); // Commands
      //Select command
      ////////////////////////
      // velocity
      ////////////////////////
      if (command == VELOCITY_COMAND_CAHR) // speed message
      {
          char mode = BufferedUART::read_c();
          if(mode==SET_COMAND_CHAR)
            set_velocity = true;
          else if(mode==GET_COMAND_CHAR)
            print_velocity = true;
        
      }
      ////////////////////////
      // servo_angle
      ////////////////////////
      else if(command==SERVO_ANGLE_COMAND_CHAR)
      {
          char mode = BufferedUART::read_c();
          if(mode==SET_COMAND_CHAR)
            set_servo_angle = true;
          else if(mode==GET_COMAND_CHAR)
            print_servo_angle = true;       
      }
    }
  }
  if(print_velocity)
  {  
    float num = Brushless_Motorshield::getVelocity();
    BufferedUART::write_s("#v ");
    BufferedUART::write_f(num,3);
    BufferedUART::write_c('\n');
    print_velocity = false;
  }
  if(set_velocity)
  {
    float num = BufferedUART::read_f();
    Brushless_Motorshield::setVelocity(num);
    set_velocity = false;
    BufferedUART::write_s("#vs ");
    BufferedUART::write_f(num,3);   
    BufferedUART::write_c('\n');
  }
  
  if(print_servo_angle)
  {  
    BufferedUART::write_s("#a ");
    BufferedUART::write_f(servo_pos - SERVO_ZERO_ANGLE,3);
    BufferedUART::write_c('\n');//
    print_servo_angle = false;
  }
  if(set_servo_angle)
  {
    servo_pos = SERVO_ZERO_ANGLE + BufferedUART::read_f();
    if(servo_pos < SERVO_MIN_ANGLE)
      servo_pos = SERVO_MIN_ANGLE;
    if(servo_pos > SERVO_MAX_ANGLE)
      servo_pos = SERVO_MAX_ANGLE;
    servo.write(servo_pos);
    set_servo_angle = false;
    BufferedUART::write_s("#as ");
    BufferedUART::write_f(servo_pos - SERVO_ZERO_ANGLE,3);
    BufferedUART::write_c('\n');
  }
}

