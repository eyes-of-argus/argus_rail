/* 
 * rosserial Subscriber
 * Blinks an LED on callback
 */

#include <ros.h>
#include <argus_rail/RailStatus.h>

ros::NodeHandle  nh;

void messageCb( const argus_rail::RailStatus& rail_status){
  if(rail_status.moving == true) digitalWrite(13, HIGH);
  else if(rail_status.moving == false) digitalWrite(13, LOW);
}

ros::Subscriber<argus_rail::RailStatus> sub("/rail_status", &messageCb );

void setup()
{ 
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{  
  nh.spinOnce();
}
