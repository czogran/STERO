#!/usr/bin/env python
# license removed for brevity

#this script is responsible for sending Ros Path Messages which gives a
#relative destination of robot movement


import rospy
from geometry_msgs.msg import *
from nav_msgs.msg import *
from std_msgs.msg import String, Header

#function with creates a path for turning robot in place
def forward():
    path=Path()
    path.poses.append(PoseStamped(Header(),Pose(Point(1,0,0),Quaternion(0,0,0,0))))
    return path

#function with creates a path for turning robot in place
def turn():
    path=Path()
    path.poses.append(PoseStamped(Header(),Pose(Point(0,0,0),Quaternion(0,0,6.28,0))))
    return path
    
#function for a path during which robot turns, drives a given distance, turns back and come back to start point   
def here_and_back():
    path=Path()
    path.poses.append(PoseStamped(Header(),Pose(Point(-1,0,0),Quaternion(0,0,0,0))))
    path.poses.append(PoseStamped(Header(),Pose(Point(-1,0,0),Quaternion(0,0,0,0))))
 
    return path

#fucntion for moving a robot in a path of a shape of square, looking at robot it makes a left side square     
def square_left():
    path=Path()
    path.poses.append(PoseStamped(Header(),Pose(Point(0,1,0),Quaternion(0,0,0,0))))
    path.poses.append(PoseStamped(Header(),Pose(Point(0,1,0),Quaternion(0,0,0,0))))
    path.poses.append(PoseStamped(Header(),Pose(Point(0,1,0),Quaternion(0,0,0,0))))
    path.poses.append(PoseStamped(Header(),Pose(Point(0,1,0),Quaternion(0,0,0,0))))
    return path
    
def square_right():
    path=Path()
    path.poses.append(PoseStamped(Header(),Pose(Point(0,-1,0),Quaternion(0,0,0,0))))
    path.poses.append(PoseStamped(Header(),Pose(Point(0,-1,0),Quaternion(0,0,0,0))))
    path.poses.append(PoseStamped(Header(),Pose(Point(0,-1,0),Quaternion(0,0,0,0))))
    path.poses.append(PoseStamped(Header(),Pose(Point(0,-1,0),Quaternion(0,0,0,0))))
    return path

def main():
    
    print("start")
    path=here_and_back()
    pub = rospy.Publisher('chatter', Path, queue_size=10)
    rospy.init_node('talker', anonymous=True)

    pub.publish(path)
    print("end")
	
 
	

if __name__=='__main__':
	main()
    

