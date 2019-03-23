#!/usr/bin/env python

import rospy
from argus_rail.msg import RailStatus

def send():
    pub = rospy.Publisher('/rail_status', RailStatus, queue_size=5)
    rospy.init_node('rail', anonymous=False)
    rate = rospy.Rate(1) # 1Hz
    rail = RailStatus()
    rail.moving = False
    rail.baseline = 1
    while not rospy.is_shutdown():
        rail.moving = not rail.moving
        rospy.loginfo(rail)
        pub.publish(rail)
        rate.sleep()

if __name__ == '__main__':
    try:
        send()
    except rospy.ROSInterruptException:
        pass