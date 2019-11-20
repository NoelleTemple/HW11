#!/usr/bin/env python

#import dependencies
import rospy
from hw11.msg import Drive
import time

#method will be called when new data is published
def callback(data):
    #log data
    rospy.loginfo(rospy.get_caller_id() + "I heard %f for speed and %f for steering", data.speed, data.steering)
    print("Speed: ", data.speed, " Steering: ", data.steering)
def listener():
    # see ros wiki for more information
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", Drive, callback)
    rospy.spin()

if __name__== '__main__':
    listener()

