#include "SimpleRSLK.h" //required libraries
#include <ROMI_MOTOR_POWER.h>

#define MS 1000 //conversion for milliseconds
#define halfSpd 50 //valid values for speed range from 0-100
#define fullSpd 100

void setup() 
{
  setupRSLK();//required to run any robot commands
  enableMotor(RIGHT_MOTOR);//required to use the motor
  enableMotor(LEFT_MOTOR);
}

void loop() 
{
  setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);//set the motor direction to forward
  setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
  setMotorSpeed(RIGHT_MOTOR, halfSpd);//run at half speed
  setMotorSpeed(LEFT_MOTOR, halfSpd);
  delayMicroseconds(1000*MS);//for 1 second

  setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);//set the motor direction to backward
  setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
  setMotorSpeed(RIGHT_MOTOR, halfSpd);//run at half speed
  setMotorSpeed(LEFT_MOTOR, halfSpd);
  delayMicroseconds(1000*MS);//for 1 second
}
