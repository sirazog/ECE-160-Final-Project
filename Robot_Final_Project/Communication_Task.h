#include "PS2X_lib.h"
#include "TinyIR.h"

//using recommended pinout
#define PS2_DAT         14 //P1.7 <-> brown wire
#define PS2_CMD         15 //P1.6 <-> orange wire
#define PS2_SEL         34 //P2.3 <-> yellow wire (also called attention)
#define PS2_CLK         35 //P6.7 <-> blue wire
#define PS2X_DEBUG
#define PS2X_COM_DEBUG

//Ir sensor
#define UP 0
#define DOWN 1
#define LEFT 2
#define RIGHT 3
#define IDLE 4
int STATE = IDLE;

PS2X psCon;

#define MS 1000 //conversion for miliseconds

void setup()
{
  Serial.begin(57600);
  delay(500 * MS);
  initTinyIRReceiver();//Ir receiver
}

void loop()//everything in loop is just the dualshock code pulled from the example 
{
  delayMicroseconds(50*1000);//don't know if we need this
  
  if(decodeIR(&IRresults)){
    //Serial.println(IRresults.command, HEX);
    translateIR();
  }
}
