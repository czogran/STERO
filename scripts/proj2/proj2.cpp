#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "costmap_2d/costmap_2d_ros.h"
#include "tf2_ros/buffer.h"
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <base_local_planner/costmap_model.h>
#include <global_planner/planner_core.h>
#include <nav_core/base_global_planner.h>
#include <string>
#include <vector>
#include "stero_mobile_init/goal.h"
#include <nav_msgs/Odometry.h>
#include <dwa_local_planner/dwa_planner_ros.h>

#include <rotate_recovery/rotate_recovery.h>


geometry_msgs::PoseStamped GetLocalPose();
bool getGoal();
void setLocalPlan();
void setGloabalPlan();
geometry_msgs::PoseStamped goal;

 ros::Publisher chatter_pub;


//base_local_planner::CostmapModel::CostmapModel localPlanner;
global_planner::GlobalPlanner globalPlanner1;
tf2_ros::Buffer bf;
std::vector <  geometry_msgs::PoseStamped > local_plan;
std::vector <  geometry_msgs::PoseStamped > plan;

costmap_2d::Costmap2DROS *global_costmap;
costmap_2d::Costmap2DROS *local_costmap;

//base_local_planner::CostmapModel::CostmapModel *localPlanner;
boost::shared_ptr<dwa_local_planner::DWAPlannerROS> localPlanner;
//dwa_local_planner::DWAPlannerROS *localPlanner;
rotate_recovery::RotateRecovery rr;

int countVelocities;

bool getGoal(stero_mobile_init::goal::Request &req,
               stero_mobile_init::goal::Response &res )
{
    try{
     ROS_INFO_STREAM("x destination "<<req.x);
     ROS_INFO_STREAM("y destination "<<req.y);
     goal.pose.position.x=req.x;
     goal.pose.position.y=req.y;
     goal.header.frame_id="map";
     setGloabalPlan();
     res.success=true;
      return true;
  }
  catch(const std::exception& e)
  {
      res.success=false;
            return false;

  }
}

geometry_msgs::PoseStamped GetLocalPose()
{
	geometry_msgs::PoseStamped pose;
	boost::shared_ptr<nav_msgs::Odometry const> sharedEdge;
    sharedEdge=ros::topic::waitForMessage<nav_msgs::Odometry>("/elektron/mobile_base_controller/odom");
	nav_msgs::Odometry odo=*sharedEdge;
	pose.pose=odo.pose.pose;
	return pose;
}

//global is working
void setGloabalPlan()
{
   

    geometry_msgs::PoseStamped start;
    start=GetLocalPose();

    start.header.frame_id="map";
    
    try
    {
    bool status;
    status=globalPlanner1.makePlan(start, goal, plan);
    
   
        globalPlanner1.publishPlan(plan);
       
         ROS_INFO_STREAM("start "<<start);
        ROS_INFO_STREAM("goal "<<goal);

        geometry_msgs::TransformStamped transform = bf.lookupTransform("map", "base_link", ros::Time(), ros::Duration(10));
        ROS_INFO_STREAM("transform "<<transform);
        ROS_INFO_STREAM("plan maded");
     
        setLocalPlan();
    }
      catch(std::exception ex)
      {
      	        ROS_INFO_STREAM("error in path making ");

      }   
}



void setLocalPlan()
{
        countVelocities=0;
       
        geometry_msgs::PoseStamped check;

        geometry_msgs::Twist cmd_vel;
        bool status;
        status=localPlanner->setPlan(plan);
       
        ROS_INFO_STREAM("lcoal velocity maded"<<cmd_vel);
     
    bool compute_velocities;
    while(true)
     {
         compute_velocities=localPlanner->computeVelocityCommands(cmd_vel);

         if(!compute_velocities)
         {
         	 ROS_INFO_STREAM("failed to calculate velocities");

         }
         if(abs(cmd_vel.linear.x)<0.02 and abs(cmd_vel.angular.z)<0.02)
         {
         	ROS_INFO_STREAM("zero speed "<<countVelocities);

         	countVelocities++;

         
    			check=GetLocalPose();
    			if(abs(goal.pose.position.x-check.pose.position.x)<0.3 and abs(goal.pose.position.y-check.pose.position.y)<0.3)
    			{
    				//geometry_msgs::Twist stop;
    			    //chatter_pub.publish(cmd_vel);
    			    ROS_INFO_STREAM("sucess!!!!!!!!!!!!");
    			    break;
    			}
    			else
    			{
    				 ROS_INFO_STREAM("recoveryyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyy");
                   //      localPlanner->initialize("localPlanner",&bf, local_costmap); 
                     //    localPlanner->setPlan(plan);

                    cmd_vel.angular.z=0.2;
    				//rr.runBehavior();
                    				ros::Duration(0.5).sleep();   
                    				         	 countVelocities=0;
          

    			}
         	
        	
        	//status=localPlanner->setPlan(plan);
         }

         else
         {
         	 countVelocities=0;

         }

        check=GetLocalPose();
    	if(abs(goal.pose.position.x-check.pose.position.x)<0.3 and abs(goal.pose.position.y-check.pose.position.y)<0.3)
    	{
    				geometry_msgs::Twist stop;
    			    chatter_pub.publish(stop);
    			    ROS_INFO_STREAM("no obstacles sucess!!!!!!!!!!!!");
    			    break;
    	}

         //if(compute_velocities)
            
                chatter_pub.publish(cmd_vel);
                ROS_INFO_STREAM("sending velocity"<<cmd_vel);
                local_costmap->updateMap();

				ros::Duration(0.1).sleep();             
             //else
             
                // rr.runBehavior();
                // status=localPlanner->setPlan(plan);
             
            

             ros::spinOnce();           

     }
    
}

// Standard C++ entry point
int main(int argc, char** argv) {
   // Announce this program to the ROS master as a "node" called "hello_world_node"
    ros::init(argc, argv, "proj2");
   // Start the node resource managers (communication, time, etc)
  // ros::start();
   
   
   ros::NodeHandle n;
   

  
  ROS_INFO_STREAM("before service");
   ros::ServiceServer service = n.advertiseService("get_goal_service", getGoal);
   ROS_INFO_STREAM("after service");
   
    chatter_pub = n.advertise<  geometry_msgs::Twist>("/mux_vel_nav/cmd_vel", 10);


     tf2_ros::TransformListener trf(bf);
    
    

   
       local_costmap=new costmap_2d::Costmap2DROS ("local_costmap", bf);
       local_costmap->start();

    global_costmap=new costmap_2d::Costmap2DROS ("global_costmap", bf);
   
   
    localPlanner.reset(new dwa_local_planner::DWAPlannerROS );
    //localPlanner=new dwa_local_planner::DWAPlannerROS ;
    localPlanner->initialize("localPlanner",&bf, local_costmap); 
    globalPlanner1.initialize("globalPlanner1",global_costmap);
    
    
    rr.initialize("my_rotate_recovery", &bf, global_costmap, local_costmap);

    //localPlanner=&dp;
//
ROS_INFO_STREAM("Hello, world!");
// Process ROS callbacks until receiving a SIGINT (ctrl-c)
ros::spin();
// Stop the node's resources
ros::shutdown();
// Exit tranquilly
return 0;
}
