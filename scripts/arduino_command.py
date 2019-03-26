#!/usr/bin/env python

import rospy
from argus_rail.msg import RailCommand

def send():
    pub = rospy.Publisher('/rail_command', RailCommand, queue_size=1)
    rospy.init_node('rail', anonymous=False)
    rate = rospy.Rate(1)  # 1Hz

    rail_command = RailCommand()
    rail_command.requested_baseline = 3
    direction = True # True = inward, False = outward

    while not rospy.is_shutdown():
        if(direction == True):
            rail_command.requested_baseline -= 1
            if(rail_command.requested_baseline < 1):
                direction = not direction
                rail_command.requested_baseline = 2
        elif(direction == False):
            rail_command.requested_baseline += 1
            if(rail_command.requested_baseline > 3):
                direction = not direction
                rail_command.requested_baseline = 2

        rospy.loginfo(rail_command)
        pub.publish(rail_command)
        rate.sleep()

if __name__ == '__main__':
    try:
        send()
    except rospy.ROSInterruptException:
        pass
