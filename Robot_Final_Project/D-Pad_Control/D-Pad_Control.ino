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
int state = 0;
int error = 0;

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

  if(error == 0){
      Serial.print("Found Controller, configured successful ");
      Serial.print("pressures = ");
    if (pressures)
      Serial.println("true ");
    else
      Serial.println("false");
    Serial.print("rumble = ");
    if (rumble)
      Serial.println("true)");
    else
      Serial.println("false");
      Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
      Serial.println("holding L1 or R1 will print out the analog stick values.");
      Serial.println("Note: Go to www.billporter.info for updates and to report bugs.");
    }  else if(error == 1)
      Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
     
    else if(error == 2)
      Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");

    else if(error == 3)
      Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
    delayMicroseconds(1000*1000);
  }


void loop() {
  // put your main code here, to run repeatedly: 
  ps2x.read_gamepad(false, vibrate);
    /*if (! ps2x.ButtonPressed(PSB_TRIANGLE) && ! ps2x.ButtonPressed(PSB_CROSS) && ! ps2x.ButtonPressed(PSB_CIRCLE) && ! ps2x.ButtonPressed(PSB_SQUARE)) {
      state = 0;
    }  */
    
    if (ps2x.Button(PSB_TRIANGLE)){
    state = 1;
    }
    
    if (ps2x.Button(PSB_CROSS)) {
    state = 2;
    }
    
    if (ps2x.Button(PSB_CIRCLE)) {
    state = 3;
    }
    
    if (ps2x.Button(PSB_SQUARE)) {
    state = 4;
  }

  if(ps2x.Button(PSB_PAD_DOWN))
  {
    state = 0;
  }

  if (ps2x.Button(PSB_L1))//run moveGripper based on which button is pushed
  {
    moveGripper(false);
  }
  else if (ps2x.Button(PSB_R1))
  {
    moveGripper(true);
  }
  
  Serial.println(state);
  alternateDrive();
  delayMicroseconds(500);
}

void alternateDrive() {
   if (state == 0) {
    setMotorSpeed(LEFT_MOTOR, 0);
    setMotorSpeed(RIGHT_MOTOR, 0);
   }
   
   if(state == 2) {      
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);//set motor direction to backward
    setMotorSpeed(LEFT_MOTOR, 255);//and set the motor speed
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);//set motor direction to backward
    setMotorSpeed(RIGHT_MOTOR, 255);//and set the motor speed;
    } 

    if (state == 1) {
      setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);//set motor direction to backward
      setMotorSpeed(LEFT_MOTOR, 255);//and set the motor speed
      setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);//set motor direction to backward
      setMotorSpeed(RIGHT_MOTOR, 255);//and set the motor speed;
    }

    if (state == 3) {
      setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);//set motor direction to backward
      setMotorSpeed(LEFT_MOTOR, 255);//and set the motor speed
      setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);//set motor direction to backward
      setMotorSpeed(RIGHT_MOTOR, 255);//and set the motor speed;
    }

    if (state == 4) {
      setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);//set motor direction to backward
      setMotorSpeed(LEFT_MOTOR, 255);//and set the motor speed
      setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);//set motor direction to backward
      setMotorSpeed(RIGHT_MOTOR, 255);//and set the motor speed;
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
