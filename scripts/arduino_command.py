#!/usr/bin/env python

import rospy
from argus_rail.msg import RailCommand
from argus_rail.msg import RailStatus

rail_moving  = False

def railStatusCallback(status):
    global rail_moving
    rail_moving = status.moving
    #rospy.loginfo("Time difference: " + str((rospy.Time.now() - status.header.stamp)))
    # measured time delay is < 0.005s

def send():
    pub = rospy.Publisher('/rail_command', RailCommand, queue_size=1)
    rospy.init_node('rail', anonymous=False)
    rospy.Subscriber("/rail_status", RailStatus, railStatusCallback)
    rate = rospy.Rate(5)  # 5Hz works nice at 5Hz, 10Hz causes shutter, 1Hz is slow

    rail_command = RailCommand()
    rail_command.requested_baseline = 3
    direction = True # True = inward, False = outward

    while not rospy.is_shutdown():
        if(not rail_moving):
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
