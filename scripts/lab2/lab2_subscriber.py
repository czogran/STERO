#!/usr/bin/env python
# license removed for brevity

#this script is responsible for releasing a Path given on
#chatter Topic with ROS path message
#it switches between ROS subscribers
#it means that it unregisters from one and starts listen for another one
#on the same topic but with different callback function
#it has three separate phases of movement
#in first it turns in dirresction of a given point 
#(points are given in relative way- 1 , 1 , 0 it means that it moves one unit in x axis and on on y axis)
#it second phase it goes to this point
#and in fourth it turns a given angle
#when a message comes it runs in "loop":
#talker inicialize odom listener and starts function move_beginner
#which starts listner for beginning turn(turn function is a callback)
#when turn ends it calls forward function
#which goes to last_turn
#after turning to given position there is a try to get new pose from path
#when it is accomplished with succes it goes back to move_beginner

import rospy
from geometry_msgs.msg import *
from nav_msgs.msg import *
from nav_msgs.srv import *
from std_msgs.msg import *
import math
#from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf.transformations import *

#declaration of robot's velocities
velocity_x=0.5
velocity_y=0.5
velocity_theta=0.5

#declaration of variables used for calculation of an angle which robot has turned
angle_moved=0
current_orientation=0
previous_orientation=0

#declaration of variables used for calculation of a distance which robot has to drive
distance=0
distance_moved=0
previous_position=Point()
current_position=Point()

#publisher a velocities to robot
velocity_publisher = rospy.Publisher('/mux_vel_nav/cmd_vel', Twist, queue_size=10)

#data of a task(pose) which robot should do
move_pose = geometry_msgs.msg.Pose()
end_turn=geometry_msgs.msg.Quaternion()

#a givenpath of a robot
poses=PoseStamped()


#subscriber to odometry message from robot
global odom_subscriber

global vel_msg
vel_msg=Twist()

#function used for calculation of beginning settings of a robot
#it calculate a distance which robot should come
#and angles which it should turn
#it alse inicialize a movement of a robot
def move_beginner(pose):
    global move_pose
    global odom_subscriber
    global angle_moved
    global previous_orientation
    global current_orientation
    global distance
    global distance_moved
    global previous_position
    global current_position
    global end_turn
    global vel_msg
    
    angle_moved=0
    distance_moved=0
    
    turn_odom=rospy.wait_for_message('/elektron/mobile_base_controller/odom',Odometry)
    print(turn_odom)
    print("\n\n")
    print("aaaaaaaaaaaaaa")
    print(pose)
#rospy.sleep(4)
    
    orientation_q = turn_odom.pose.pose.orientation

    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    current_orientation=abs(yaw)
    previous_orientation=abs(yaw)
    
    current_position=turn_odom.pose.pose.position
    previous_position=turn_odom.pose.pose.position

    odom_subscriber=rospy.Subscriber('/elektron/mobile_base_controller/odom',Odometry, turn)

    delta_x=pose.position.x-turn_odom.pose.pose.position.x
    delta_y=pose.position.y-turn_odom.pose.pose.position.y
    distance=math.sqrt(pow(delta_x,2)+pow(delta_y,2))
    
    cos_x=1
    if(distance>0):
        cos_x=abs(delta_x)/distance
    
    vel_msg=Twist()
    
    theta_start=(math.atan2(delta_y,delta_x)-yaw)
    
    if(theta_start<0):
        vel_msg.angular.z=-velocity_theta
    else:
        vel_msg.angular.z=velocity_theta
        
    if (abs(theta_start)>6.28-abs(theta_start)):
         theta_start=6.28-abs(theta_start)
         vel_msg.angular.z=-vel_msg.angular.z
   
    end_turn.z=pose.orientation.z
    
    
    
   
    
    print "angle",math.atan2(delta_y,delta_x)
    print "yaw", yaw
    print "delta x", delta_x
    print "delta y", delta_y
    
    print "distance", distance
    print "theta " ,theta_start
    #rospy.sleep(5)
    move_pose.orientation.z=abs(theta_start)
    velocity_publisher.publish(vel_msg)    

#function responsible for last turn of a robot for a given angle
def last_turn(data):
    global end_turn
    global velocity_publisher
    global odom_subscriber
    global angle_moved
    global previous_orientation
    global current_orientation
    global vel_msg

    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    current_orientation=abs(yaw)
    angle_moved+= abs(current_orientation-previous_orientation)
    previous_orientation=current_orientation
    
    velocity_publisher.publish(vel_msg)

    #print("end turn data")
    #print(move_pose.orientation.z)
    #print(data.pose.pose.orientation.z)
    #print("yaw: %f" %yaw)
    #print("angle moved: %f"%angle_moved)
    
  
    if(angle_moved>=end_turn.z):
        print("end of ending turn movement")
        velocity_publisher.publish(Twist())
        rospy.sleep(0.2)
        velocity_publisher.publish(Twist())
        
        odom_subscriber.unregister()
        
        try:
            move_beginner(poses.pop(0).pose)
        except:
            print("complete end of movement")
            pass

#function responsible for moving robot forward for a distance calculated
#in function move_beginner()
def forward(data):
    global distance
    global distance_moved
    global previous_position
    global current_position
    global odom_subscriber
    global angle_moved
    global vel_msg
    
    
    current_position=data.pose.pose.position
    diff_x=current_position.x-previous_position.x
    diff_y=current_position.y-previous_position.y
    distance_moved +=math.sqrt(pow(diff_x,2)+pow(diff_y,2))
    previous_position=current_position
    
    velocity_publisher.publish(vel_msg)
    
   # print("\nforward data")
    #print("distance: %f" % distance)
    #rint("distance moved: %f" % distance_moved)
   
    
    if(distance_moved>=distance):
        print("end of forward movement")
       
        velocity_publisher.publish(Twist())
        
        odom_subscriber.unregister()
        rospy.sleep(0.2)
        velocity_publisher.publish(Twist())
        angle_moved=0
        odom_subscriber=rospy.Subscriber('/elektron/mobile_base_controller/odom',Odometry, last_turn)
        if(end_turn<0):
            vel_msg=Twist(Vector3(0, 0, 0),Vector3(0,0,-velocity_theta))
            velocity_publisher.publish(Twist(Vector3(0, 0, 0),Vector3(0,0,-velocity_theta)))
        else:
            vel_msg=Twist(Vector3(0, 0, 0),Vector3(0,0,velocity_theta))
            velocity_publisher.publish(Twist(Vector3(0, 0, 0),Vector3(0,0,velocity_theta)))

        
#function responsible for turning a robot in direction of a given point
def turn(data):
    global move_pose
    global velocity_publisher
    global odom_subscriber
    global angle_moved
    global previous_orientation
    global current_orientation
    global vel_msg

    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    current_orientation=abs(yaw)
    angle_moved+= abs(current_orientation-previous_orientation)
    previous_orientation=current_orientation
    
    velocity_publisher.publish(vel_msg)

    #print("")
    #print("turn data")
    #print(move_pose.orientation.z)
    #print(data.pose.pose.orientation.z)
    #print("yaw: %f" %yaw)
    #print("angle moved: %f"%angle_moved)
    
  
    if(angle_moved>=move_pose.orientation.z):
        print("end of turn movement")
        velocity_publisher.publish(Twist())
        rospy.sleep(0.2)
        velocity_publisher.publish(Twist())
         
        vel_msg= Twist(Vector3(velocity_x, 0, 0),Vector3(0,0,0))
        velocity_publisher.publish(Twist(Vector3(velocity_x, 0, 0),Vector3(0,0,0)))
        
        odom_subscriber.unregister()
        odom_subscriber=rospy.Subscriber('/elektron/mobile_base_controller/odom',Odometry, forward)


def talker(data):
    global poses
    poses=data
    move_beginner(poses.pop(0).pose)
   

def main():
    rospy.init_node('listener23', anonymous=True)
    #odom_subscriber.unregister()
    print("start")
    pos=rospy.wait_for_message('/gazebo_odom',Odometry)
    rospy.wait_for_service('/global_planner/planner/make_plan')
    get_plan = rospy.ServiceProxy('/global_planner/planner/make_plan', GetPlan)
    req = GetPlanRequest()
    point=Point(3,-5,0)
    start_point=pos.pose.pose.position

    req.goal.header.frame_id='map'
    req.start.header.frame_id='map'
    
    req.start.pose.position=start_point
    req.goal.pose.position=point
    req.tolerance =0
    resp = get_plan(req.start, req.goal, req.tolerance)
   # print(resp)
    #my_path = rospy.ServiceProxy('/global_planner/planner/make_plan',GetPlan)

    poses=list()
    
    for x in range(10,len(resp.plan.poses),10):
        poses.append(resp.plan.poses[x])
    print(len(resp.plan.poses))
    poses.append(resp.plan.poses[len(resp.plan.poses)-1])
    

    print("start")
   # print(poses)
    talker(poses)
  
    rospy.spin()

if __name__=='__main__':
	main()

