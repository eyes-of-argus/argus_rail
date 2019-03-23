/* 
 * rosserial Subscriber
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Bool.h>

ros::NodeHandle  nh;

void messageCb( const std_msgs::Bool& led_status){
  if(led_status.data == true) digitalWrite(13, HIGH);
  else if(led_status.data == false) digitalWrite(13, LOW);
}

ros::Subscriber<std_msgs::Bool> sub("/arduino/led", &messageCb );

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
