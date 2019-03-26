#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt8

def send():
    pub = rospy.Publisher('/rail_command', UInt8, queue_size=1)
    rospy.init_node('rail', anonymous=False)
    rate = rospy.Rate(20)  # 1Hz
    rail_command = UInt8()
    rail_command.data = 3
    while not rospy.is_shutdown():
        rail_command.data = input()
        if rail_command.data > 3:
            rail_command.data = 3
        elif rail_command.data < 1:
            rail_command.data = 1
        rospy.loginfo(rail_command)
        pub.publish(rail_command)
        rate.sleep()

if __name__ == '__main__':
    try:
        send()
    except rospy.ROSInterruptException:
        pass
