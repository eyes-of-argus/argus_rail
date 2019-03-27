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

#include <ros.h>
#include <argus_rail/RailStatus.h>
#include <argus_rail/RailCommand.h>

#include "argus_stepper.h"

// Baseline movement is hardcoded at:
//    FULL_MOVE  30100
//    HALF_MOVE  15050
//    RAIL_SPEED 25

ArgusStepper Right(10, 9, 8);
ArgusStepper Left (6, 5, 4);

ros::NodeHandle nh;

argus_rail::RailStatus rail_status;
ros::Publisher rail_pub("/rail_status", &rail_status);

// RailStatus message variables
// header: uint32 seq, time stamp, string frame_id
char frame_id[] = "/rail_status";
uint8_t current_baseline = 3;
bool moving = false;

uint8_t requested_baseline = current_baseline;

void railCommandCb(const argus_rail::RailCommand& rail_command)
{
  requested_baseline = rail_command.requested_baseline;
}

ros::Subscriber<argus_rail::RailCommand> rail_sub("/rail_command", &railCommandCb);

void setup() {
  Right.begin();
  Right.setSpeed(20);
  Right.setDirection(INWARD);
  Right.setPosition(3);

  Left.begin();
  Left.setSpeed(20);
  Left.setDirection(INWARD);
  Left.setPosition(3);

  nh.initNode();
  nh.subscribe(rail_sub);
  nh.advertise(rail_pub);
}

void loop() {

  if(requested_baseline != current_baseline)
  {
    rail_status.baseline = current_baseline;
    rail_status.moving = true;
    rail_status.header.frame_id = frame_id;
    rail_status.header.stamp = nh.now();
    rail_pub.publish(&rail_status);
    
    requestBaseline(Left, Right, requested_baseline, current_baseline);
    current_baseline = requested_baseline;
  }
  
  rail_status.baseline = current_baseline;
  rail_status.moving = false;
  rail_status.header.frame_id = frame_id;
  rail_status.header.stamp = nh.now();
  
  rail_pub.publish(&rail_status);
  
  nh.spinOnce();
  delay(10); // 100 hz
}
