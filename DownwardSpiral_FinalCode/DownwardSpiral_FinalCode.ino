/*
   DownwardSpiral_FinalCode.ino
   Dan Joshwa
   Alex Herzog
   Eric Johnsen
   Heath Thomas
   11/9/21

   Hardware:
      RSLK robot kit (including line follower)
      IR distance sensor
      RC-06 Bluetooth Module
      PS2 Receiver and controller
      Servo with 3d printed claw attachment

   This code is meant to be the final code for the spelunker rescue competition
   The robot starts by autonomatically calibrating its line following sensor (please ensure that the robot is on the tournament surface)
   Then the robot will start in its control state
   switching to the autonomous state can be done with the select button on the ps2 controller

   Functions:
      floorCalibration() - From line follower example, part of calibration process 
      simpleCalibrate() - From line follower example, part of calibration process
      autonomous() - Makes the robot move autonomously using the line follower and IR distance sensor
      controlled() - Makes the robot move via input from the ps2 controller and both the analogDrive() and moveGripper() functions
      analogDrive() - Actually sends output to the motors via the ps2 inputs
      moveGripper() - Opens or closes the gripper
      drop() - Autonomously drops the spelunker

*/
#include <Bump_Switch.h>;//include necessary libraries
#include <Encoder.h>
#include <GP2Y0A21_Sensor.h>
#include <QTRSensors.h>
#include <Romi_Motor_Power.h>
#include <RSLK_Pins.h>
#include <SimpleRSLK.h>
#include "SimpleRSLK.h"
#include "Servo.h"
#include "PS2X_lib.h"

uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS];
uint16_t sensorMinVal[LS_NUM_SENSORS];

//speeds chosen for autonomous movement
uint16_t normalSpeed = 15;
uint16_t fastSpeed = 20;
uint16_t forwardSpeed = 27;

/* Valid values are either:
    DARK_LINE  if your floor is lighter than your line
    LIGHT_LINE if your floor is darker than your line
*/
uint8_t lineColor = LIGHT_LINE;

Servo gripper;
int angle = 40;

//using recommended pinout for the ps2 receiver
#define PS2_DAT         14 //P1.7 <-> brown wire
#define PS2_CMD         15 //P1.6 <-> orange wire
#define PS2_SEL         34 //P2.3 <-> yellow wire (also called attention)
#define PS2_CLK         35 //P6.7 <-> blue wire
#define PS2X_DEBUG
#define PS2X_COM_DEBUG
#define pressures false
#define rumble false

//analog stick values
#define stickMax 255
#define stickHalf 255/2

PS2X ps2x;

byte vibrate = 0;

#define MS 1000 //conversion for miliseconds

//definitions for the state machine
#define IDLE 0
#define AUTO 1
#define CONTROL 2
#define BACKAUTO 3

int STATE = CONTROL;


void setup()
{
  Serial.begin(115200);//regular serial port
  Serial1.begin(9600);//bluetooth serial port
  delayMicroseconds(500 * MS);
  gripper.attach(SRV_0);
  gripper.write(angle);

  ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);//configure the pins
  ps2x.read_gamepad(false, vibrate);//read controller and set large motor to not spin

  setupRSLK();
  setupLed(RED_LED);
  clearMinMax(sensorMinVal, sensorMaxVal);
  enableMotor(BOTH_MOTORS);
}

/*
 *  From line follower example, part of calibration process 
 *  INPUTS: NONE
 *  OUTPUTS: NONE
 */
void floorCalibration() {
  /* Place Robot On Floor (no line) */
  delayMicroseconds(MS * 3000);

  Serial.println("Running calibration on floor");//replace serial prints with comments
  simpleCalibrate();
  Serial.println("Reading floor values complete");

  btnMsg = "Push left button on Launchpad to begin line following.\n";
  btnMsg += "Make sure the robot is on the line.\n";
  delayMicroseconds(MS * 1000);

  enableMotor(BOTH_MOTORS);
}

/*
 *  From line follower example, part of calibration process 
 *  INPUTS: NONE
 *  OUTPUTS: NONE
 */
void simpleCalibrate() {
  /* Set both motors direction MOTOR_DIR_FORWARD */
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
  /* Enable both motors */
  enableMotor(BOTH_MOTORS);
  /* Set both motors speed 20 */
  setMotorSpeed(BOTH_MOTORS, 20);

  for (int x = 0; x < 100; x++) {
    readLineSensor(sensorVal);
    setSensorMinMax(sensorVal, sensorMinVal, sensorMaxVal);
  }

  /* Disable both motors */
  disableMotor(BOTH_MOTORS);
}

bool isCalibrationComplete = false;//will be changed to true at the end of callibration

void loop()
{
  ps2x.read_gamepad(false, vibrate);//read controller and set large motor to not spin

  /* Check if calibration was completed */
  if (isCalibrationComplete == false) {
    floorCalibration();
    isCalibrationComplete = true;
  }

  readLineSensor(sensorVal);
  readCalLineSensor(sensorVal,
                    sensorCalVal,
                    sensorMinVal,
                    sensorMaxVal,
                    lineColor);


  /*
   * State machine code (starts in controlled state)
   * There are only 2 states, controlled and autonomous
   * if the select button on the controller is pressed, the robot switches to whichever state it isn't currently in
   */
  switch (STATE)
  {
    case AUTO:
      autonomous();
      if (ps2x.Button(PSB_SELECT))//if the select button is pressed
      {
        STATE = CONTROL;//switch to controlled state
      }
      break;
    case CONTROL:
      controlled();
      if (ps2x.Button(PSB_SELECT))//and vise verse
      {
        STATE = AUTO;
      }
      break;
    default:
      STATE = AUTO;
      break;
  }
  Serial1.println((6787 / (analogRead(SHRP_DIST_C_PIN) - 3) - 4));//used to check the distance read by the sensor
}

/*
 * This function makes the robot move autonomously using the line follower and IR distance sensor
 * INPUTS: NONE
 * OUTPUTS: NONE 
 */
void autonomous()
{
  int dist = (6787 / (analogRead(SHRP_DIST_L_PIN) - 3) - 4);
  uint32_t linePos = getLinePosition(sensorCalVal, lineColor);
  Serial.println("autonomous");


  if (linePos > 2000 && linePos < 3000)//if the robot is veering right
  { 
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);

    setMotorSpeed(LEFT_MOTOR, fastSpeed);//adjust with the left motor
    setMotorSpeed(RIGHT_MOTOR, normalSpeed);
  }
  else if (linePos > 3500 && linePos < 4000)//if the robot is veering left
  { 
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorSpeed(LEFT_MOTOR, normalSpeed);
    setMotorSpeed(RIGHT_MOTOR, fastSpeed);//adjust with the right motor
  }
  else if (linePos < 2000 && linePos != 0)//if the robot needs to turn left
  {
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);//make the left motor move backwards
    setMotorSpeed(LEFT_MOTOR, fastSpeed);
    setMotorSpeed(RIGHT_MOTOR, fastSpeed);
  }
  else if (linePos > 4000 && linePos != 0)//if the robot needs to turn right
  {
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);//make the right motor move backwards
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorSpeed(LEFT_MOTOR, fastSpeed);
    setMotorSpeed(RIGHT_MOTOR, fastSpeed);
  }
  else //if the robot is centered enogh on the line
  {
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);//both motors move forward
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorSpeed(LEFT_MOTOR, forwardSpeed);//and a faster speed
    setMotorSpeed(RIGHT_MOTOR, forwardSpeed);
    if (dist < 5)//if the robot is moving foward and sees a wall nearby (at the drop zone)
    {
      drop();//run the drop code
    }
  }
}

void controlled()
{
  Serial.println("controlled");

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
  if (leftStick < stickHalf + 5) //if the left stick is pushed downward
  {
    //    Serial.println("left back");
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);//set motor direction to backward
    setMotorSpeed(LEFT_MOTOR, map(leftStick, stickHalf, stickMax, 0, 30));//and set the motor speed
  }
  else if (leftStick >= stickHalf - 5) //if the left stick is pushed forward
  {
    //    Serial.println("left forward");
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);//same code but forward
    setMotorSpeed(LEFT_MOTOR, map(leftStick, stickHalf, 0, 0, 30));
  }

  //same code for the right stick and motor
  if (rightStick < stickHalf + 5) //if the left stick is pusheed downward
  {
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);//set motor direction to backward
    setMotorSpeed(RIGHT_MOTOR, map(rightStick, stickHalf, stickMax, 0, 30));//and set the motor speed
  }
  else if (rightStick >= stickHalf - 5) //if the left stick is pushed forward
  {
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);//same code but forward
    setMotorSpeed(RIGHT_MOTOR, map(rightStick, stickHalf, 0, 0, 30));
  }
}

void moveGripper(bool forward)
{
  if (forward)
  {
    if (angle < 105)
    {
      angle++;
      gripper.write(angle);
    }
  }
  else
  {
    if (angle > 20)
    {
      angle--;
      gripper.write(angle);
    }
  }
}

void drop()
{
  int dist = (6787 / (analogRead(SHRP_DIST_L_PIN) - 3) - 4);//read distance sensor
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_BACKWARD);//move backwards
  while (dist < 9)//while the robot isn't far enough
  {
    dist = (6787 / (analogRead(SHRP_DIST_L_PIN) - 3) - 4);//keep reading the distance
    Serial1.println(dist);//we printed to double check the values
    setMotorSpeed(BOTH_MOTORS, 20);//move backwards slowly
  }
  //after the robot is far away enough from the wall
  setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);//switch one motor to forward so it turns
  delayMicroseconds(MS * 1300);//for 1300 ms
  setMotorSpeed(BOTH_MOTORS, 0);//then turn off both motors
  gripper.write(20);//open the claw
  STATE = AUTO;//set state to auto so the robot just follows the line out
}
