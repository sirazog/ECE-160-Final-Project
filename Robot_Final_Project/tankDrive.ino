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

PS2X ps2x;

#define MS 1000 //conversion for miliseconds
void setup()
{
  Serial.begin(57600);
  delay(500 * MS);

  setupRSLK();
  enableMotor(LEFT_MOTOR);
  enableMotor(RIGHT_MOTOR);

  gripper.attach(SRV_0);
  gripper.write(angle);

  ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);//configure the pins
}

void loop()
{
  ps2x.read_gamepad(false, 0);//read controller and set large motor to not spin
  analogDrive(ps2x.Analog(PSS_LY), ps2x.Analog(PSS_RY));
  if (ps2x.Button(PSB_L1))
  {
    moveGripper(false);
  }
  else if (ps2x.Button(PSB_R1))
  {
    moveGripper(true);
  }
}

void analogDrive(int leftStick, int rightStick)
{
  if (ps2x.Analog(PSS_LY) < 512) //if the left stick is pusheed downward
  {
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);//set motor direction to backward
    setMotorSpeed(LEFT_MOTOR, map(ps2x.Analog(PSS_LY), 512, 0, 0, 100));//and set the motor speed
  }
  else if (ps2x.Analog(PSS_LY) >= 512) //if the left stick is pushed forward
  {
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);//same code but forward
    setMotorSpeed(LEFT_MOTOR, map(ps2x.Analog(PSS_LY), 0, 1028, 0, 100));
  }

  //same code for the right stick and motor
  if (ps2x.Analog(PSS_RY) < 512) //if the left stick is pusheed downward
  {
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);//set motor direction to backward
    setMotorSpeed(RIGHT_MOTOR, map(ps2x.Analog(PSS_LY), 512, 0, 0, 100));//and set the motor speed
  }
  else if (ps2x.Analog(PSS_RY) >= 512) //if the left stick is pushed forward
  {
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);//same code but forward
    setMotorSpeed(RIGHT_MOTOR, map(ps2x.Analog(PSS_LY), 0, 1028, 0, 100));
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
