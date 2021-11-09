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

   This code is
*/
#include <Bump_Switch.h>
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

#define stickMax 255
#define stickHalf 255/2

PS2X ps2x;

byte vibrate = 0;

#define MS 1000 //conversion for miliseconds

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


  switch (STATE)
  {
    case AUTO:
      autonomous();
      if (ps2x.Button(PSB_SELECT))//was PSB_R2
      {
        STATE = CONTROL;
      }
      break;
    case CONTROL:
      controlled();
      if (ps2x.Button(PSB_SELECT))
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

void autonomous()
{
  int dist = (6787 / (analogRead(SHRP_DIST_L_PIN) - 3) - 4);
  uint32_t linePos = getLinePosition(sensorCalVal, lineColor);
  Serial.println("autonomous");


  if (linePos > 2000 && linePos < 3000)
  { //veer right
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);

    setMotorSpeed(LEFT_MOTOR, fastSpeed);
    setMotorSpeed(RIGHT_MOTOR, normalSpeed);
  }
  else if (linePos > 3500 && linePos < 4000)
  { //veer left
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorSpeed(LEFT_MOTOR, normalSpeed);
    setMotorSpeed(RIGHT_MOTOR, fastSpeed);
  }
  else if (linePos < 2000 && linePos != 0)//turn left
  {
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorSpeed(LEFT_MOTOR, fastSpeed);
    setMotorSpeed(RIGHT_MOTOR, fastSpeed);
  }
  else if (linePos > 4000 && linePos != 0)//turn right
  {
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorSpeed(LEFT_MOTOR, fastSpeed);
    setMotorSpeed(RIGHT_MOTOR, fastSpeed);
  }
  else 
  {
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorSpeed(LEFT_MOTOR, forwardSpeed);
    setMotorSpeed(RIGHT_MOTOR, forwardSpeed);
    if (dist < 5)
    {
      drop();
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
  int dist = (6787 / (analogRead(SHRP_DIST_L_PIN) - 3) - 4);
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_BACKWARD);
  while (dist < 9)
  {
    dist = (6787 / (analogRead(SHRP_DIST_L_PIN) - 3) - 4);
    Serial1.println(dist);
    setMotorSpeed(BOTH_MOTORS, 20);
  }
  setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
  delayMicroseconds(MS * 1300);
  setMotorSpeed(BOTH_MOTORS, 0);
  gripper.write(20);
  STATE = AUTO;
}
