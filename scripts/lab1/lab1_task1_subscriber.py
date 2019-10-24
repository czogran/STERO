#!/usr/bin/env python
# license removed for brevity

#this scipt subscribes publisher "chatter" which was created in lab_task1_publisher.py
#when it receives message it counts times for each move and sends approperiate messeges to
#elketron robot on Topic :/mux_vel_nav/cmd_vel'
#messages are in Twist Message

import rospy
from geometry_msgs.msg import *
from std_msgs.msg import String
import math

#declaration of basic velocities for a robot
velocity_x=0.5
velocity_y=0.5
velocity_theta=0.5

#declaration of publisher which sends Twist- speed of every movement to robot
velocity_publisher = rospy.Publisher('/mux_vel_nav/cmd_vel', Twist, queue_size=10)


#this function is a declaration of subscriber on topic "chatter"
#which give a order of movement in x,y axis and theta angle
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", Pose, talker)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
  
#definition of callback function for listener on topic:chatter
def talker(data):
 
    vel_msg = Twist()
    stop_msg = Twist()
    
    #calculation of distance wchich robot should drive
    distance=math.sqrt(pow(data.position.x,2)+pow(data.position.y,2))
    
    #calculation of angle which robot should turn to drive in direction
    #of a given poiny
    cos_x=1
    if(distance>0):
        cos_x=data.position.x/distance
    theta_start=math.acos(cos_x)
    
    #setting turn velocity, it depends from in which direction is smaller
    #angle to overcome
    if(data.position.y>=0):
        vel_msg.angular.z=velocity_theta
    else:
        vel_msg.angular.z=-velocity_theta
    
    
    #turnning of robot
    rospy.sleep(0.1)
    velocity_publisher.publish(vel_msg)
    
    #calculation of how long robot should turn
    sleep_theta_start=theta_start/velocity_theta
    
    print("    theta: %f" %theta_start +" sleep theta start: %f" % sleep_theta_start + " angular: %f" % vel_msg.angular.z)
    print(vel_msg)
    
    #sleeping as long as robot is turnning to a given angle
    rospy.sleep(sleep_theta_start)
   
   
    #forward  movement of robot 
    vel_msg.linear.x=velocity_x
    vel_msg.angular.z=0
    
    #stopping robot
    #it should wait a while to stop completly, then new velocity is given
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
    
    
    #end of movement- stopping a robot after all moves
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

