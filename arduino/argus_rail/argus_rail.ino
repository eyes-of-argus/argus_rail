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
#include <std_msgs/UInt8.h>

#include "argus_stepper.h"

ArgusStepper Right(10, 9, 8);
ArgusStepper Left (6, 5, 4);

ros::NodeHandle nh;

argus_rail::RailStatus rail_status;
ros::Publisher rail("/rail_status", &rail_status);

// RailStatus message variables
// header: uint32 seq, time stamp, string frame_id
char frame_id[] = "/rail_status";
uint8_t current_baseline = 3;
bool moving = false;

uint8_t requested_baseline = current_baseline;
int steps_to_baseline = 0;

void setMove(int steps, bool dir)
{
  steps_to_baseline = steps;
  if(dir == INWARD)
  {
    Right.setDirection(INWARD);
    Left.setDirection(INWARD);
  }
  else if(dir == OUTWARD)
  {
    Right.setDirection(OUTWARD);
    Left.setDirection(OUTWARD);
  }
}

void railCommandCb(const std_msgs::UInt8& req_baseline)
{
  // return if rail is already moving or at req position
  if(steps_to_baseline > 0) return;
  requested_baseline = req_baseline.data;
  if(requested_baseline == current_baseline) return;
  
  if(current_baseline == 3 && requested_baseline == 2) setMove(15275+600, INWARD);
  else if(current_baseline == 3 && requested_baseline == 1) setMove(30550, INWARD);
  else if(current_baseline == 2 && requested_baseline == 3) setMove(15275, OUTWARD);
  else if(current_baseline == 2 && requested_baseline == 1) setMove(15275, INWARD);
  else if(current_baseline == 1 && requested_baseline == 3) setMove(30550, OUTWARD);
  else if(current_baseline == 1 && requested_baseline == 2) setMove(15275, OUTWARD);
  
  Left.enable();
  Right.enable();
  
  rail_status.baseline = current_baseline;
  rail_status.moving = (current_baseline != requested_baseline);
  rail_status.steps_to_baseline = steps_to_baseline;
  rail_status.header.frame_id = frame_id;
  rail_status.header.stamp = nh.now();
  rail.publish(&rail_status);
}

ros::Subscriber<std_msgs::UInt8> rail_command("/rail_command", &railCommandCb);

void setup() {
  Right.begin();
  Right.setSpeed(20);
  Right.setDirection(INWARD);
  Right.setPosition(3);
  Right.disable();

  Left.begin();
  Left.setSpeed(20);
  Left.setDirection(INWARD);
  Left.setPosition(3);
  Left.disable();

/*
  for(int i = 0; i < 30550; i++)
  {
    Left.sendPulse();
    Right.sendPulse();
  }
  Left.reverseDirection();
  Right.reverseDirection();
  for(int i = 0; i < 30550; i++)
  {
    Left.sendPulse();
    Right.sendPulse();
  }
*/
  nh.initNode();
  nh.subscribe(rail_command);
  nh.advertise(rail);
}

void loop() {
  
  while(current_baseline != requested_baseline)
  {
    if(steps_to_baseline > 0)
    {
      Left.sendPulse();
      Right.sendPulse();
      steps_to_baseline--;
    }
    else
    {
      current_baseline = requested_baseline;
      Left.disable();
      Right.disable();
    }    
  }

  rail_status.baseline = current_baseline;
  rail_status.moving = (current_baseline != requested_baseline);
  rail_status.steps_to_baseline = steps_to_baseline;
  rail_status.header.frame_id = frame_id;
  rail_status.header.stamp = nh.now();
  rail.publish(&rail_status);
  nh.spinOnce();
  delay(50); // 20 hz
}
