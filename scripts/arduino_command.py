#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt8

def send():
    pub = rospy.Publisher('/rail_command', UInt8, queue_size=1)
    rospy.init_node('rail', anonymous=False)
    rate = rospy.Rate(1)  # 1Hz
    rail_command = UInt8()
    rail_command.data = 3
    baseline = 3
    run_count = 0
    while not rospy.is_shutdown():
        if(run_count > 0):
            baseline = rail_command.data
            baseline -= 1
            if(baseline < 1):
                baseline = 3
            rail_command.data = baseline
            run_count -= 1
        else:
            run_count = input()

        rospy.loginfo(rail_command)
        pub.publish(rail_command)
        rate.sleep()

if __name__ == '__main__':
    try:
        send()
    except rospy.ROSInterruptException:
        pass
