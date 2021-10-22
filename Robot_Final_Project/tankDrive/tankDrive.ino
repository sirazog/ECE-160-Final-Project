#include "SimpleRSLK.h"
#include <ROMI_MOTOR_POWER.h>
#include "Servo.h"
#include "PS2X_lib.h"

Servo gripper;
int angle = 0;

//using recommended pinout
#define PS2_DAT         14 //P1.7 <-> brown wire
#define PS2_CMD         15 //P1.6 <-> orange wire
#define PS2_SEL         34 //P2.3 <-> yellow wire (also called attention)
#define PS2_CLK         35 //P6.7 <-> blue wire
#define PS2X_DEBUG
#define PS2X_COM_DEBUG
#define pressures false
#define rumble false

#define stickMax 255
#define stickHalf 255/2

PS2X ps2x;

byte vibrate = 0;

#define MS 1000 //conversion for miliseconds
void setup()
{
  Serial.begin(57600);
  delayMicroseconds(500 * MS);

  setupRSLK();
  enableMotor(LEFT_MOTOR);
  enableMotor(RIGHT_MOTOR);

  gripper.attach(SRV_0);
  gripper.write(angle);

  ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);//configure the pins
}

void loop()
{
  ps2x.read_gamepad(false, vibrate);//read controller and set large motor to not spin
  //Serial.print("LFT: ");
  //Serial.println(ps2x.Analog(PSS_LY));
  analogDrive(ps2x.Analog(PSS_LY), ps2x.Analog(PSS_RY));//run the analogDrive function with the analog stick inputs
  if (ps2x.Button(PSB_L1))//run moveGripper based on which button is pushed
  {
    moveGripper(false);
  }
  else if (ps2x.Button(PSB_R1))
  {
    moveGripper(true);
  }
}

void analogDrive(int rightStick, int leftStick)
{
  Serial.println("driving");
  if (leftStick < stickHalf+5) //if the left stick is pushed downward
  {
    Serial.println("left back");
    enableMotor(LEFT_MOTOR);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);//set motor direction to backward
    setMotorSpeed(LEFT_MOTOR, map(leftStick, stickHalf, stickMax, 0, 100));//and set the motor speed
  }
  else if (leftStick >= stickHalf-5) //if the left stick is pushed forward
  {
    Serial.println("left forward");
    enableMotor(LEFT_MOTOR);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);//same code but forward
    setMotorSpeed(LEFT_MOTOR, map(leftStick, stickHalf, 0, 0, 100));
  } else {
    disableMotor(LEFT_MOTOR);
  }

  //same code for the right stick and motor
  if (rightStick < stickHalf+5) //if the left stick is pusheed downward
  {
    Serial.println("right back");
    enableMotor(RIGHT_MOTOR);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);//set motor direction to backward
    setMotorSpeed(RIGHT_MOTOR, map(rightStick, stickHalf, stickMax, 0, 100));//and set the motor speed
  }
  else if (rightStick >= stickHalf-5) //if the left stick is pushed forward
  {
    Serial.println("right forward");
    enableMotor(RIGHT_MOTOR);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);//same code but forward
    setMotorSpeed(RIGHT_MOTOR, map(rightStick, stickHalf, 0, 0, 100));
  } else {
    disableMotor(RIGHT_MOTOR);
  }
}

void moveGripper(bool forward)
{
  if (forward)
  {
    if (angle < 180)
    {
      angle++;
      gripper.write(angle);
    }
  }
  else
  {
    if (angle > 0)
    {
      angle--;
      gripper.write(angle);
    }
  }
}
