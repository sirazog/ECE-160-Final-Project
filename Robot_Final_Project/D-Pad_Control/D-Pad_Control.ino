#include "PS2X_lib.h"  //for v1.6
#include "SimpleRSLK.h"
#include <ROMI_MOTOR_POWER.h>
#include "Servo.h"

#define PS2_DAT         14 //P1.7 <-> brown wire
#define PS2_CMD         15 //P1.6 <-> orange wire
#define PS2_SEL         34 //P2.3 <-> yellow wire
#define PS2_CLK         35 //P6.7 <-> blue wire

#define pressures   false
#define rumble      false
#define MS 1000 

byte vibrate = 0;

PS2X ps2x;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  delayMicroseconds(500*MS);

  setupRSLK();
  enableMotor(LEFT_MOTOR);
  enableMotor(RIGHT_MOTOR);

  //gripper.attach(SRV_0);
  //gripper.write(angle);

  ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);//configure the pins
}

void loop() {
  // put your main code here, to run repeatedly: 
  ps2x.read_gamepad(false, vibrate);

  alternateDrive();
}

void alternateDrive() {
   if(ps2x.Button(PSB_PAD_UP)) {      
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);//set motor direction to backward
    setMotorSpeed(LEFT_MOTOR, 255);//and set the motor speed
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);//set motor direction to backward
    setMotorSpeed(RIGHT_MOTOR, 255);//and set the motor speed;
    }

    if (ps2x.Button(PSB_PAD_DOWN)) {
      setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);//set motor direction to backward
      setMotorSpeed(LEFT_MOTOR, 255);//and set the motor speed
      setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);//set motor direction to backward
      setMotorSpeed(RIGHT_MOTOR, 255);//and set the motor speed;
    }

    if (ps2x.Button(PSB_PAD_LEFT)) {
      setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);//set motor direction to backward
      setMotorSpeed(LEFT_MOTOR, 255);//and set the motor speed
      setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);//set motor direction to backward
      setMotorSpeed(RIGHT_MOTOR, 255);//and set the motor speed;
    }

    if (ps2x.Button(PSB_PAD_RIGHT)) {
      setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);//set motor direction to backward
      setMotorSpeed(LEFT_MOTOR, 255);//and set the motor speed
      setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);//set motor direction to backward
      setMotorSpeed(RIGHT_MOTOR, 255);//and set the motor speed;
    }
}
