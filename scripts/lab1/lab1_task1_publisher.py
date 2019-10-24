#!/usr/bin/env python
# license removed for brevity

#this script is responsible for getting user input and sending it on topic chatter as 
# ROS geometry msg Pose

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import *


#function responsible for getting user input and sending it to subscribers
def talker():
   
    #creating publisher "chatter" which sends POse Message
    pub = rospy.Publisher('chatter', Pose, queue_size=10)
    #ROS node inicialization
    rospy.init_node('talker', anonymous=True)
    #creating a Pose Message
    pose = geometry_msgs.msg.Pose()

    #while script isn't closed it takes user input and publishes it
    while not rospy.is_shutdown():
        print("get x")
        x=float(input())
        print("get y")
        y=float(input())
        print("get theta")
        theta=float(input())
        
        pose.position.x = x
        pose.position.y = y
        pose.orientation.z = theta
        
        pub.publish(pose)
        

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
