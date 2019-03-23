#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

def send():
    pub = rospy.Publisher('/arduino/led', Bool, queue_size=5)
    rospy.init_node('arduino_send', anonymous=False)
    rate = rospy.Rate(1) # 1Hz
    led_status = False
    while not rospy.is_shutdown():
        led_status = not led_status
        rospy.loginfo(led_status)
        pub.publish(led_status)
        rate.sleep()

if __name__ == '__main__':
    try:
        send()
    except rospy.ROSInterruptException:
        pass