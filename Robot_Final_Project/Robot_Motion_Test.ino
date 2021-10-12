#include "SimpleRSLK.h" //required libraries
#include <ROMI_MOTOR_POWER.h>

#define MS 1000 //conversion for milliseconds
#define halfSpd 50 //valid values for speed range from 0-100
#define fullSpd 100

void setup() 
{
  setupRSLK();//required to run any robot commands
}

void loop() 
{
  enableMotor(RIGHT_MOTOR);//required to use the motor
  enableMotor(LEFT_MOTOR);
  
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
  
  setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);//rotate counterclockwise
  setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
  setMotorSpeed(RIGHT_MOTOR, halfSpd);
  setMotorSpeed(LEFT_MOTOR, halfSpd);
  delayMicroseconds(1000*MS);//for one second
  
  setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);//rotate clockwise
  setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
  setMotorSpeed(RIGHT_MOTOR, halfSpd);
  setMotorSpeed(LEFT_MOTOR, halfSpd);
  delayMicroseconds(1000*MS);//for one second
  
  setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);//turn counterclockwise
  setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
  setMotorSpeed(RIGHT_MOTOR, fullSpd);
  setMotorSpeed(LEFT_MOTOR, halfSpd);
  delayMicroseconds(1000*MS);//for one second
  
  setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);//turn clockwise
  setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
  setMotorSpeed(RIGHT_MOTOR, halfSpd);
  setMotorSpeed(LEFT_MOTOR, fullSpd);
  delayMicroseconds(1000*MS);//for one second
  
  disableMotor(RIGHT_MOTOR);//disable the motors
  disableMotor(LEFT_MOTOR);
  
  for(int angle = 0; angle <= 180; angle += 10)//open and close servo
  {
    gripper.write(angle);
    delayMicroseconds(100*MS);
  }
  for(int angle = 180; angle >= 0; angle -= 10)
  {
    gripper.write(angle);
    delayMicroseconds(100*MS);
  }
}
}
