// M. Kaan Tasbas | mktasbas@gmail.com
// March 2019

/*
 * Arduino Nano sketch for controling the camera rail on the Argus robot
 * 
 * Connections:
 * 
 * Camera    | Left | Right
 * pulse     | D06  | D10  // Active on rising edge, min width 2.5us
 * direction | D05  | D09  // HIGH: inward, LOW: outward
 * opto      | VCC  | VCC  // optocoupler power (+5V);
 * enable    | D04  | D08  // HIGH: enabled, LOW: disabled
 * 
 * Note: 5us delay req after changing direction or enable
 */

#include "argus_stepper.h"

ArgusStepper Right(10, 9, 8);
ArgusStepper Left (6, 5, 4);

void setup() {
  Right.begin();
  Right.setSpeed(30);
  Right.setDirection(INWARD);
  Right.enable();

  Left.begin();
  Left.setSpeed(30);
  Left.setDirection(INWARD);
  Left.enable();

  pinMode(3, INPUT_PULLUP);
}

void loop() {
  static long count = 0;

  if(!digitalRead(3))
  {
    if(Left.isEnabled() && Right.isEnabled())
    {
      Left.sendPulse();
      Right.sendPulse();
    }
  
    if(count++ > 15275) 
    {
      Left.reverseDirection();
      Right.reverseDirection();
      count = 0;
    } 
  }
}
