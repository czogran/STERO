#!/usr/bin/env python
# license removed for brevity

import rospy
from geometry_msgs.msg import *
from std_msgs.msg import String
import math

velocity_x=0.5
velocity_y=0.5
velocity_theta=0.5

velocity_publisher = rospy.Publisher('/mux_vel_nav/cmd_vel', Twist, queue_size=10)


def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", Pose, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def callback(data):
    print("\n")
    rospy.loginfo("I have new message")
    talker(data) 
  

def talker(data):
 
    vel_msg = Twist()
    stop_msg = Twist()

    distance=math.sqrt(pow(data.position.x,2)+pow(data.position.y,2))
    cos_x=1
    
    if(distance>0):
        cos_x=data.position.x/distance
    
    theta_start=math.acos(cos_x)
    
    if(data.position.y>=0):
        vel_msg.angular.z=velocity_theta
    else:
        vel_msg.angular.z=-velocity_theta
    
    velocity_publisher.publish(stop_msg)
    rospy.sleep(0.1)
    velocity_publisher.publish(vel_msg)
    
    sleep_theta_start=theta_start/velocity_theta
    
    print("    theta: %f" %theta_start +" sleep theta start: %f" % sleep_theta_start + " angular: %f" % vel_msg.angular.z)
    print(vel_msg)
    
    rospy.sleep(sleep_theta_start)
   
   
    #forward   
    vel_msg.linear.x=velocity_x
    vel_msg.angular.z=0
    
    velocity_publisher.publish(stop_msg)
    rospy.sleep(0.1)
    velocity_publisher.publish(vel_msg)

    sleep_x=distance/velocity_x
    
    print("    forward: %f"% distance +" sleep: %f" % sleep_x)
    print(vel_msg)
    
    rospy.sleep(sleep_x)
    
    
    #end turn
    vel_msg.linear.x=0
    
    if(data.orientation.z>=0):
        vel_msg.angular.z=velocity_theta
    else:
        vel_msg.angular.z=-velocity_theta
    
    velocity_publisher.publish(stop_msg)
    rospy.sleep(0.1)
    velocity_publisher.publish(vel_msg)

    sleep_theta=abs(data.orientation.z)/(velocity_theta)
    print("    end turn: %f"% data.orientation.z +" sleep: %f" % sleep_theta)
    print(vel_msg)
    rospy.sleep(sleep_theta)
    
    
    #end of movement
    vel_msg=stop_msg
    velocity_publisher.publish(stop_msg)
    print("end")
    

def main():
	print("start")
	rospy.loginfo("start")
	listener()
	talker()
	

if __name__=='__main__':
	main()

