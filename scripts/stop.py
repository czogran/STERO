#!/usr/bin/env python
# license removed for brevity

import rospy
from geometry_msgs.msg import *
from std_msgs.msg import String


def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1)


    vel_msg = Twist()
    velocity_publisher = rospy.Publisher('/mux_vel_nav/cmd_vel', Twist, queue_size=10)


    vel_msg.linear.x=0
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z =0

    velocity_publisher.publish(vel_msg)

  
    

    #rate.sleep()
    velocity_publisher.publish(vel_msg)
    velocity_publisher.publish(vel_msg)

    velocity_publisher.publish(vel_msg)
    rate.sleep()
    velocity_publisher.publish(vel_msg)
    velocity_publisher.publish(vel_msg)

    velocity_publisher.publish(vel_msg)

    hello_str ="robot should be stopped"
    rospy.loginfo(hello_str)

def main():
	print("start")
	rospy.loginfo("start")
	talker()


	

if __name__=='__main__':
	main()

