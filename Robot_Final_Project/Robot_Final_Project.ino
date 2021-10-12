#include "SimpleRSLK.h"
#include <ROMI_MOTOR_POWER.h>

#define MS 1000
#define halfSpd 50
#define fullSpd 100

void setup() 
{
  setupRSLK();
  enableMotor(RIGHT_MOTOR);
  enableMotor(LEFT_MOTOR);
}

void loop() 
{
  setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
  setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
  setMotorSpeed(RIGHT_MOTOR, halfSpd);
  setMotorSpeed(LEFT_MOTOR, halfSpd);
  delayMicroseconds(1000*MS);

  setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
  setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
  setMotorSpeed(RIGHT_MOTOR, halfSpd);
  setMotorSpeed(LEFT_MOTOR, halfSpd);
  delayMicroseconds(1000*MS);
}
