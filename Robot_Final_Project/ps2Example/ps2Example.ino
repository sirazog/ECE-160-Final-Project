#include "PS2X_lib.h"

//using recommended pinout
#define PS2_DAT         14 //P1.7 <-> brown wire
#define PS2_CMD         15 //P1.6 <-> orange wire
#define PS2_SEL         34 //P2.3 <-> yellow wire (also called attention)
#define PS2_CLK         35 //P6.7 <-> blue wire
#define PS2X_DEBUG
#define PS2X_COM_DEBUG

PS2X psCon;

#define MS 1000 //conversion for miliseconds

void setup()
{
  Serial.begin(57600);
  delay(500 * MS);
  psCon.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);//configure the pins
}

void loop()//everything in loop is just the dualshock code pulled from the example 
{
  psCon.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed

  if (psCon.Button(PSB_START))        //button values will be TRUE as long as button is pressed
    Serial.println("Start is being held");
  if (psCon.Button(PSB_SELECT))
    Serial.println("Select is being held");

  if (psCon.Button(PSB_PAD_UP)) {
    Serial.print("Up held this hard: ");
    Serial.println(psCon.Analog(PSAB_PAD_UP), DEC);
  }
  if (psCon.Button(PSB_PAD_RIGHT)) {
    Serial.print("Right held this hard: ");
    Serial.println(psCon.Analog(PSAB_PAD_RIGHT), DEC);
  }
  if (psCon.Button(PSB_PAD_LEFT)) {
    Serial.print("LEFT held this hard: ");
    Serial.println(psCon.Analog(PSAB_PAD_LEFT), DEC);
  }
  if (psCon.Button(PSB_PAD_DOWN)) {
    Serial.print("DOWN held this hard: ");
    Serial.println(psCon.Analog(PSAB_PAD_DOWN), DEC);
  }

  vibrate = psCon.Analog(PSAB_CROSS);  //this will set the large motor vibrate speed based on how hard you press the blue (X) button
  if (psCon.NewButtonState()) {        //will be TRUE if any button changes state (on to off, or off to on)
    if (psCon.Button(PSB_L3))
      Serial.println("L3 pressed");
    if (psCon.Button(PSB_R3))
      Serial.println("R3 pressed");
    if (psCon.Button(PSB_L2))
      Serial.println("L2 pressed");
    if (psCon.Button(PSB_R2))
      Serial.println("R2 pressed");
    if (psCon.Button(PSB_TRIANGLE))
      Serial.println("Triangle pressed");
  }

  if (psCon.ButtonPressed(PSB_CIRCLE))              //will be TRUE if button was JUST pressed
    Serial.println("Circle just pressed");
  if (psCon.NewButtonState(PSB_CROSS))              //will be TRUE if button was JUST pressed OR released
    Serial.println("X just changed");
  if (psCon.ButtonReleased(PSB_SQUARE))             //will be TRUE if button was JUST released
    Serial.println("Square just released");

  if (psCon.Button(PSB_L1) || psCon.Button(PSB_R1)) { //print analog stick values if L1 or R1 is pressed
    Serial.print("Stick Values:");
    Serial.print(psCon.Analog(PSS_LY), DEC); //Left stick, Y axis. Other options: LX, RY, RX
    Serial.print(",");
    Serial.print(psCon.Analog(PSS_LX), DEC);
    Serial.print(",");
    Serial.print(psCon.Analog(PSS_RY), DEC);
    Serial.print(",");
    Serial.println(psCon.Analog(PSS_RX), DEC);
  }
  
  delayMicroseconds(50*1000);//don't know if we need this
}
