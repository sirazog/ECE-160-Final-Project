#include "TinyIR.h"
#include "SimpleRSLK.h"
#include <ROMI_MOTOR_POWER.h>
#include "Servo.h"

//Ir sensor
#define UP 0
#define DOWN 1
#define LEFT 2
#define RIGHT 3
#define IDLE 4
int STATE = IDLE;
int IRPin = 19;

IRData IRresults;

#define MS 1000 //conversion for miliseconds

void setup()
{
  Serial.begin(57600);
  delayMicroseconds(500 * MS);
  initTinyIRReceiver();//Ir receiver

  pinMode(IRPin, INPUT);
  
  setupRSLK();
  enableMotor(LEFT_MOTOR);
  enableMotor(RIGHT_MOTOR);
}

void loop()
{ 
  //Serial.println("Hello World");
  
  if(decodeIR(&IRresults)){
    Serial.println(IRresults.command);
    translateIR();
  }
}
void translateIR() {
  switch (IRresults.command) {
    case 0xFFA25D:
      Serial.println("POWER");
      break;
    case 0x46:
      //Go Forward
      Serial.println("UP");
      setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
      setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
      setMotorSpeed(LEFT_MOTOR, 255);
      setMotorSpeed(RIGHT_MOTOR, 255);
      break;
    case 0x44:
      //turns Left
      Serial.println("LEFT");
      setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
      setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
      setMotorSpeed(LEFT_MOTOR, 255/2);
      setMotorSpeed(RIGHT_MOTOR, 255);
      break;
    case 0x43:
      //turns Right
      Serial.println("RIGHT");
       setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
      setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
      setMotorSpeed(LEFT_MOTOR, 255);
      setMotorSpeed(RIGHT_MOTOR, 255/2);
      break;
    case 0x15:
      //backwards
      Serial.println("DOWN");
      setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
      setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
      setMotorSpeed(LEFT_MOTOR, 255);
      setMotorSpeed(RIGHT_MOTOR, 255);
      break;
    case 0x40:
      //Stop/reset
      Serial.println("OK");
      setMotorSpeed(LEFT_MOTOR, 0);
      setMotorSpeed(RIGHT_MOTOR, 0);
      break;
    case 0x42:
      Serial.println("*");
      break;
    case 0x4A:
      Serial.println("#");
      break;
    case 0x52:
      Serial.println("0");
      break;
    case 0x16:
    //Spin Left
      Serial.println("1");
      setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
      setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
      setMotorSpeed(LEFT_MOTOR, 255);
      setMotorSpeed(RIGHT_MOTOR, 255);
      break;
    case 0x19:
      Serial.println("2");
      break;
    case 0xD:
      //Spin Right
      Serial.println("3");
      setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
      setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
      setMotorSpeed(LEFT_MOTOR, 255);
      setMotorSpeed(RIGHT_MOTOR, 255);
      break;
    case 0xC:
      Serial.println("4");
      break;
    case 0x18:
      Serial.println("5");
      break;
    case 0x5E:
      Serial.println("6");
      break;
    case 0x8:
      Serial.println("7");
      break;
    case 0x1C:
      Serial.println("8");
      break;
    case 0x5A:
      Serial.println("9");
      break;
    default:
      //sets everything to default
      Serial.println("other button");
      break;
  }
}
