/* 
 * rosserial Subscriber
 * Blinks an LED on callback
 */

#include <ros.h>
#include <argus_rail/RailStatus.h>

ros::NodeHandle  nh;

argus_rail::RailStatus rail_status;
ros::Publisher rail("/rail_status", &rail_status);

// RailStatus message variables
// header: uint32 seq, time stamp, string frame_id
char frame_id[] = "/rail_status";
uint8_t current_baseline = 3;
bool moving = false;

//void messageCb( const argus_rail::RailStatus& rail_status){
//  if(rail_status.moving == true) digitalWrite(13, HIGH);
//  else if(rail_status.moving == false) digitalWrite(13, LOW);
//}
//
//ros::Subscriber<argus_rail::RailStatus> sub("/rail_status", &messageCb );

void setup()
{ 
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(rail);
}

void loop()
{  
  current_baseline++;
  if(current_baseline > 3) current_baseline = 1;
  moving = !moving;
  rail_status.baseline = current_baseline;
  rail_status.moving = moving;
  rail_status.header.frame_id = frame_id;
  rail_status.header.stamp = nh.now();
  rail.publish(&rail_status);
  nh.spinOnce();
  delay(100);
}
