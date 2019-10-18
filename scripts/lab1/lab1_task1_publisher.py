#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import *


def talker():
    #pub = rospy.Publisher('chatter', String, queue_size=10)
    pub = rospy.Publisher('chatter', Pose, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    #rate = rospy.Rate(10) # 10hz

    pose    = geometry_msgs.msg.Pose()


    pose.position.x = 0
    pose.position.y = 0
    pose.position.z = 0
        
    pose.orientation.x = 0
    pose.orientation.y = 0
    pose.orientation.z = 0
    pose.orientation.w = 0

    while not rospy.is_shutdown():
        
        hello_str = "hello world %s" % rospy.get_time()
        print("get x")
        x=float(input())
        print("get y")
        y=float(input())
        print("get theta")
        theta=float(input())
        
        pose.position.x = x
        pose.position.y = y
     
        pose.orientation.z = theta
        
        rospy.loginfo(hello_str)
        #pub.publish(hello_str)
        pub.publish(pose)
        

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
