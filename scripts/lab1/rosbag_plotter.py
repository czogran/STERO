#!/usr/bin/env python
# license removed for brevity

import rospy
from geometry_msgs.msg import *
from nav_msgs.msg import *
from std_msgs.msg import String
import rosbag
import math
import matplotlib.pyplot as plt
import os
from tf.transformations import euler_from_quaternion, quaternion_from_euler



def count_orientation(msg):
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    
    return yaw


def with_laser(bag_package,chart_title):
    script_dir = os.path.dirname(__file__)
    rel_path = "bag_files/"+bag_package
    abs_file_path = os.path.join(script_dir, rel_path)
 
    bag  = rosbag.Bag(abs_file_path)# +bag_package)
    
    time=list()
    gazebo_msgs_x=list()
    gazebo_msgs_y=list()
    gazebo_msgs_theta=list()
    
    odom_time=list()
    odom_msgs_x=list()
    odom_msgs_y=list()
    odom_msgs_theta=list()
    
    pose2D_time=list()
    pose2D_msgs_x=list()
    pose2D_msgs_y=list()
    pose2D_msgs_theta=list()
    
    for topic, msg, t in bag.read_messages(topics=['/gazebo_odom']): 
        time.append(t.to_time())
       # print(t)
        gazebo_msgs_x.append(msg.pose.pose.position.x)
        gazebo_msgs_y.append(msg.pose.pose.position.y)
        gazebo_msgs_theta.append(count_orientation(msg))
        
    for topic1, msg1, t1 in bag.read_messages(topics=['/elektron/mobile_base_controller/odom']): #'elektron/mobile_base_controller/odom']):
        odom_time.append(t1.to_time())
        odom_msgs_x.append(msg1.pose.pose.position.x)
        odom_msgs_y.append(msg1.pose.pose.position.y)
        odom_msgs_theta.append(count_orientation(msg1))
        #print("               ssssssssss")
    for topic1, msg1, t1 in bag.read_messages(topics=['/pose2D']): 
     #   print(t1)
        pose2D_time.append(t1.to_time())
        pose2D_msgs_x.append(msg1.x)
        pose2D_msgs_y.append(msg1.y)
        pose2D_msgs_theta.append(msg1.theta)

  
    bag.close()
    
    error_x=list()
    error_y=list()
    error_theta=list()
    
    error_time=list()
    
    pos_error_x=list()
    pos_error_y=list()
    pos_error_theta=list()
    
    pos_error_time=list()
   # print(math.floor(len(odom_msgs_x)/2))
    #print(len(gazebo_msgs_x))
    error_lenght=min(len(gazebo_msgs_x),int(math.floor(len(odom_msgs_x)/2)))
    for x in range(error_lenght):
         error_x.append(gazebo_msgs_x[x]-odom_msgs_x[x*2])
         error_y.append(gazebo_msgs_y[x]-odom_msgs_y[x*2])
         error_theta.append(gazebo_msgs_theta[x]-odom_msgs_theta[x*2])
         
         #print(gazebo_msgs_theta[x])
         #print(odom_msgs_theta[x*2])
         #print(gazebo_msgs_theta[x]-odom_msgs_theta[x*2])
         error_time.append(time[x])
         
    for x in range(len(pose2D_time)):
         i=0
         while(time[i]<=pose2D_time[x] and i<len(pose2D_time)):
             i+=1
             
         pos_error_x.append(gazebo_msgs_x[i]-pose2D_msgs_x[x])
         pos_error_y.append(gazebo_msgs_y[i]-pose2D_msgs_y[x])
         pos_error_theta.append(gazebo_msgs_theta[i]-pose2D_msgs_theta[x])
    
        
    
    plt.figure()
    plt.plot(odom_time,odom_msgs_x[:],label='x ODOM position of ELEKTRON')
    plt.plot(time,gazebo_msgs_x[:],label='x GAZEBO position of ELEKTRON')
    plt.plot(pose2D_time,pose2D_msgs_x[:],label='x LASER position of ELEKTRON')
    #plt.plot(error_time,error_x,label='x error: GAZEBO- ODOM')
   
    plt.xlabel('time')
    plt.ylabel('position')
    
    plt.legend(loc=0)
    plt.title("ELEKTRON odometry in time with laser")
   # plt.show()
    plt.tight_layout()
    plt.grid()
    plt.savefig(chart_title+"LaserX.png")

    
    plt.figure()
    plt.plot(time,gazebo_msgs_y[:],label='y GAZEBO position of ELEKTRON')
    plt.plot(odom_time,odom_msgs_y[:],label='y ODOM position of ELEKTRON')
    plt.plot(pose2D_time,pose2D_msgs_y,label='y LASER position of ELEKTRON')

    plt.ylabel('position')
    plt.xlabel('time')
    plt.legend(loc=0)
    plt.grid()
    plt.title("ELEKTRON laser odometry in time")
#plt.show()
    plt.savefig(chart_title+"LaserY.png")
    
    plt.figure()
    plt.plot(time,gazebo_msgs_theta[:],label='theta GAZEBO orientation of ELEKTRON')
    plt.plot(odom_time,odom_msgs_theta[:],label='theta ODOM orintation of ELEKTRON')
    plt.plot(pose2D_time,pose2D_msgs_theta,label='theta LASER position of ELEKTRON')
    plt.ylabel('angle')
    plt.xlabel('time')
    plt.legend(loc=0)
    plt.grid()
    plt.title("ELEKTRON laser odometry in time")
  #  plt.show()
    plt.savefig(chart_title+"LaserTheta.png")
    
    plt.figure()
    plt.plot(error_time,error_x,label='x error: GAZEBO- ODOM')
    plt.plot(error_time,error_y,label='y error: GAZEBO- ODOM')
    plt.plot(error_time,error_theta,label='theta error: GAZEBO- ODOM')
   
    
    plt.ylabel('angle')
    plt.xlabel('time')
    plt.legend(loc=0)
    plt.grid()

    plt.title("ELEKTRON laser odometry elektron | odometry errors in time")
   # plt.show()
    plt.savefig(chart_title+"LaserERROROdom.png")
    
    plt.figure()
    
    plt.plot(pose2D_time,pos_error_theta,label='theta error: GAZEBO- LASER')
    plt.plot(pose2D_time,pos_error_x,label='x error: GAZEBO- LASER')
    plt.plot(pose2D_time,pos_error_y,label='y error: GAZEBO- LASER')
    
    plt.ylabel('angle')
    plt.xlabel('time')
    plt.legend(loc=0)
    plt.grid()

    plt.title("ELEKTRON laser odometry errors in time")
   # plt.show()
    plt.savefig(chart_title+"LaserERRORLaser.png")
    
    

def no_laser(bag_package,chart_title):
    script_dir = os.path.dirname(__file__)
    rel_path = "bag_files/"+bag_package
    abs_file_path = os.path.join(script_dir, rel_path)
 
    bag  = rosbag.Bag(abs_file_path)# +bag_package)
    
    time=list()
    gazebo_msgs_x=list()
    gazebo_msgs_y=list()
    gazebo_msgs_theta=list()
    
    odom_time=list()
    odom_msgs_x=list()
    odom_msgs_y=list()
    odom_msgs_theta=list()
    for topic, msg, t in bag.read_messages(topics=['/gazebo_odom']): 
        time.append(t.to_time())
        gazebo_msgs_x.append(msg.pose.pose.position.x)
        gazebo_msgs_y.append(msg.pose.pose.position.y)
        gazebo_msgs_theta.append(count_orientation(msg))
        
    for topic1, msg1, t1 in bag.read_messages(topics=['/elektron/mobile_base_controller/odom']): #'elektron/mobile_base_controller/odom']):
        odom_time.append(t1.to_time())
        odom_msgs_x.append(msg1.pose.pose.position.x)
        odom_msgs_y.append(msg1.pose.pose.position.y)
        odom_msgs_theta.append(count_orientation(msg1))

  
    bag.close()
    
    error_x=list()
    error_y=list()
    error_theta=list()
    error_time=list()
   # print(math.floor(len(odom_msgs_x)/2))
    #print(len(gazebo_msgs_x))
    error_lenght=min(len(gazebo_msgs_x),int(math.floor(len(odom_msgs_x)/2)))
    for x in range(error_lenght):
         error_x.append(gazebo_msgs_x[x]-odom_msgs_x[x*2])
         error_y.append(gazebo_msgs_y[x]-odom_msgs_y[x*2])
         error_theta.append(gazebo_msgs_theta[x]-odom_msgs_theta[x*2])
         
         #print(gazebo_msgs_theta[x])
         #print(odom_msgs_theta[x*2])
         #print(gazebo_msgs_theta[x]-odom_msgs_theta[x*2])
         error_time.append(time[x])
    
    plt.figure()
    plt.plot(odom_time,odom_msgs_x[:],label='x ODOM position of ELEKTRON')
    plt.plot(time,gazebo_msgs_x[:],label='x GAZEBO position of ELEKTRON')
    #plt.plot(error_time,error_x,label='x error: GAZEBO- ODOM')
   
    plt.xlabel('time')
    plt.ylabel('position')
    
    plt.legend(loc=0)
    plt.title("ELEKTRON odometry in time")
   # plt.show()
    plt.tight_layout()
    plt.grid()
    plt.savefig(chart_title+"X.png")

    
    plt.figure()
    plt.plot(time,gazebo_msgs_y[:],label='y GAZEBO position of ELEKTRON')
    plt.plot(odom_time,odom_msgs_y[:],label='y ODOM position of ELEKTRON')
    plt.ylabel('position')
    plt.xlabel('time')
    plt.legend(loc=0)
    plt.grid()
    plt.title("ELEKTRON odometry in time")
#plt.show()
    plt.savefig(chart_title+"Y.png")
    
    plt.figure()
    plt.plot(time,gazebo_msgs_theta[:],label='theta GAZEBO orientation of ELEKTRON')
    plt.plot(odom_time,odom_msgs_theta[:],label='theta ODOM orintation of ELEKTRON')
    plt.ylabel('angle')
    plt.xlabel('time')
    plt.legend(loc=0)
    plt.grid()
    plt.title("ELEKTRON odometry in time")
  #  plt.show()
    plt.savefig(chart_title+"THETA.png")
    
    plt.figure()
    plt.plot(error_time,error_x,label='x error: GAZEBO- ODOM')
    plt.plot(error_time,error_y,label='y error: GAZEBO- ODOM')
    plt.plot(error_time,error_theta,label='theta error: GAZEBO- ODOM')
    
    plt.ylabel('angle')
    plt.xlabel('time')
    plt.legend(loc=0)
    plt.grid()

    plt.title("ELEKTRON odometry in time")
   # plt.show()
    plt.savefig(chart_title+"ERROR.png")
    
    
    
    


def main():
    
    bag= raw_input("Enter bag Name:  ")
    chart_title= raw_input("Enter chart title:  ")
    with_laser(bag,chart_title)

    
    
    
    

if __name__=='__main__':
    main()

