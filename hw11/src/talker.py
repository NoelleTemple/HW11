#!/usr/bin/env python

#import dependencies
import rospy
from hw11.msg import Drive

import logging
import time

def talker():
    #set up publisher
    pub = rospy.Publisher('chatter', Drive)
    rospy.init_node('cust_talker', anonymous=True)
    rate = rospy.Rate(1) #1 Hz
    msg = Drive()
    msg.steering = 0.16
    msg.speed = 0.17
    while not rospy.is_shutdown():
        #log data from sensor
        rospy.loginfo(msg)
        #publish data from sensor
        pub.publish(msg)
        #sleep based on Hz from earlier (1 Hz = sleep for 1 second; 2 Hz = sleep for 0.5 seconds; etc)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
