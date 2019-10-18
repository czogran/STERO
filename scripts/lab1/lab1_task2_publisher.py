#!/usr/bin/env python
# license removed for brevity

import rospy
from geometry_msgs.msg import *
from nav_msgs.msg import *
from std_msgs.msg import String, Header
import math



def here_and_back():
    path=Path()
    path.poses.append(PoseStamped(Header(),Pose(Point(-1,0,0),Quaternion(0,0,0,0))))
    path.poses.append(PoseStamped(Header(),Pose(Point(-1,0,0),Quaternion(0,0,0,0))))
 
    return path
    
def square():
    path=Path()
    path.poses.append(PoseStamped(Header(),Pose(Point(0,1,0),Quaternion(0,0,0,0))))
    path.poses.append(PoseStamped(Header(),Pose(Point(0,1,0),Quaternion(0,0,0,0))))
    path.poses.append(PoseStamped(Header(),Pose(Point(0,1,0),Quaternion(0,0,0,0))))
    path.poses.append(PoseStamped(Header(),Pose(Point(0,1,0),Quaternion(0,0,0,0))))
    return path

def main():
    
    print("start")
    path=square()
    pub = rospy.Publisher('chatter', Path, queue_size=10)
    rospy.init_node('talker', anonymous=True)

    pub.publish(path)
    print("end")
	
 
	

if __name__=='__main__':
	main()
    

