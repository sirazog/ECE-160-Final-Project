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

#define IDLE 0
#define AUTO 1
#define CONTROL 2
#define BACKAUTO 3

int STATE = CONTROL;

//bool turned = false;//we change this to true after the first turn so we know when to stop the robot at the end
//bool finishedTurn = false;

void setup()
{
  Serial.begin(115200);
  Serial1.begin(9600);
  delayMicroseconds(500 * 1000);
  Serial1.println("test");
  gripper.attach(SRV_0);
  gripper.write(angle);

  ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);//configure the pins
  ps2x.read_gamepad(false, vibrate);//read controller and set large motor to not spin

  setupRSLK();
  /* Left button on Launchpad */
  //setupwaitBtn(ps2x.Button(PSB_L2));
  /* Red led in rgb led */
  setupLed(RED_LED);
  clearMinMax(sensorMinVal, sensorMaxVal);
  enableMotor(BOTH_MOTORS);
}

void floorCalibration() {
  /* Place Robot On Floor (no line) */
  delayMicroseconds(1000 * 2000);
  String btnMsg = "Push left button on Launchpad to begin calibration.\n";
  btnMsg += "Make sure the robot is on the floor away from the line.\n";
  /* Wait until button is pressed to start robot */
  //waitBtnPressed(ps2x.Button(PSB_L2), btnMsg, RED_LED);

  delayMicroseconds(1000 * 1000);

  Serial.println("Running calibration on floor");
  simpleCalibrate();
  Serial.println("Reading floor values complete");

  btnMsg = "Push left button on Launchpad to begin line following.\n";
  btnMsg += "Make sure the robot is on the line.\n";
  /* Wait until button is pressed to start robot */
  //waitBtnPressed(ps2x.Button(PSB_L2), btnMsg, RED_LED);
  delayMicroseconds(1000 * 1000);

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

bool isCalibrationComplete = false;

void loop()
{
  //setSensorMinMax(sensorVal, sensorMinVal, sensorMaxVal);
  ps2x.read_gamepad(false, vibrate);//read controller and set large motor to not spin

  /* Run this setup only once */
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
      else if (ps2x.Button(PSB_START))
      {
        STATE = BACKAUTO;
      }
      //Serial.println(STATE);
      //autonomous();
      break;
    case CONTROL:
      controlled();
      if (ps2x.Button(PSB_SELECT))
      {
        STATE = AUTO;
        //        turned = false;
        //        finishedturn = false;
      }
      else if (ps2x.Button(PSB_START))
      {
        STATE = BACKAUTO;
      }
      //Serial.println(STATE);
      //      controlled();
      break;
    case BACKAUTO:
      if (ps2x.Button(PSB_SELECT))
      {
        STATE = CONTROL;
      }
      autonomousRetrieval();
      break;
    default:
      STATE = CONTROL;
      break;
  }
  //Serial1.println(STATE);
  Serial1.println((6787 / (analogRead(SHRP_DIST_C_PIN) - 3) - 4));
  //  delayMicroseconds(1000*50);
}

void autonomous()
{
  int dist = (6787 / (analogRead(SHRP_DIST_L_PIN) - 3) - 4);
  uint32_t linePos = getLinePosition(sensorCalVal, lineColor);
  Serial.println("autonomous");


  if (linePos > 2000 && linePos < 3000) {
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);

    setMotorSpeed(LEFT_MOTOR, normalSpeed);
    setMotorSpeed(RIGHT_MOTOR, fastSpeed);
  } else if (linePos > 3500 && linePos < 4000) {
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorSpeed(LEFT_MOTOR, fastSpeed);
    setMotorSpeed(RIGHT_MOTOR, normalSpeed);
  }
  else if (linePos < 2000 && linePos != 0)
  {
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorSpeed(LEFT_MOTOR, fastSpeed);
    setMotorSpeed(RIGHT_MOTOR, fastSpeed);
    //    turned = true;
  }
  else if (linePos > 4000 && linePos != 0)
  {
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorSpeed(LEFT_MOTOR, fastSpeed);
    setMotorSpeed(RIGHT_MOTOR, fastSpeed);
    //    turned = true;
  }
  else {
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorSpeed(LEFT_MOTOR, forwardSpeed);
    setMotorSpeed(RIGHT_MOTOR, forwardSpeed);
    if (dist < 5)
    {
      drop();
    }
  }

  //  Serial1.println(dist);
  /*
    Serial1.println(turned);
    if (turned == true && finishedTurn == true)
    {
    Serial1.println("WE'VE TURNED AND ARE STRAIGHT NOW");
    if (dist < 20 && dist != 0)
    {
      disableMotor(BOTH_MOTORS);
      Serial1.println("STOP MOTORS");
    }
    }
  */
  //Serial.println(linePos);
}
void autonomousRetrieval()
{
  int backDist = (6787 / (analogRead(SHRP_DIST_C_PIN) - 3) - 4);//needs to be changed to other dist sensor
  uint32_t linePos = getLinePosition(sensorCalVal, lineColor);
  Serial.println("autonomous");


  if (linePos > 2000 && linePos < 3000) {
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);

    setMotorSpeed(LEFT_MOTOR, fastSpeed);
    setMotorSpeed(RIGHT_MOTOR, normalSpeed);
  } else if (linePos > 3500 && linePos < 4000) {
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorSpeed(LEFT_MOTOR, normalSpeed);
    setMotorSpeed(RIGHT_MOTOR, fastSpeed);
  }
  else if (linePos < 2000 && linePos != 0)
  {
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorSpeed(LEFT_MOTOR, fastSpeed);
    setMotorSpeed(RIGHT_MOTOR, fastSpeed);
    //    turned = true;
  }
  else if (linePos > 4000 && linePos != 0)
  {
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorSpeed(LEFT_MOTOR, fastSpeed);
    setMotorSpeed(RIGHT_MOTOR, fastSpeed);
    //    turned = true;
  }
  else {
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorSpeed(LEFT_MOTOR, forwardSpeed);
    setMotorSpeed(RIGHT_MOTOR, forwardSpeed);
    if (backDist < 5)
    {
      gripper.write(105);
      STATE = AUTO;
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
  //  Serial.println("driving");
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
    //    Serial.println("right back");
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);//set motor direction to backward
    setMotorSpeed(RIGHT_MOTOR, map(rightStick, stickHalf, stickMax, 0, 30));//and set the motor speed
  }
  else if (rightStick >= stickHalf - 5) //if the left stick is pushed forward
  {
    //    Serial.println("right forward");
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
  //Serial1.println(angle);
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
  delayMicroseconds(1000 * 1300);
  setMotorSpeed(BOTH_MOTORS, 0);
  gripper.write(20);
  //  STATE = AUTO;
}
