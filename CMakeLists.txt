cmake_minimum_required(VERSION 3.5)
project(stero_mobile_init)
set(CMAKE_CXX_STANDARD 11)
#find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
 # eigen_conversions
  roscpp
  rospy
  std_msgs
  message_generation
  dwa_local_planner
  sensor_msgs
  std_srvs
  trajectory_msgs
  tf
  nav_msgs
  nav_core
  geometry_msgs
  base_local_planner
  global_planner
  costmap_2d
  map_server
  tf2_ros
  rotate_recovery
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_service_files(
  FILES
  goal.srv
)
generate_messages(
  DEPENDENCIES
  nav_msgs
  map_msgs
  std_msgs
)




catkin_package(
#   INCLUDE_DIRS include
 #  LIBRARIES ${PROJECT_NAME}
#   CATKIN_DEPENDS roscpp nodelet
 CATKIN_DEPENDS message_runtime 
   DEPENDS elektron
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  
  
)

add_executable(proj2 scripts/proj2/proj2.cpp)
target_link_libraries(proj2 ${catkin_LIBRARIES})

install (DIRECTORY worlds/ DESTINATION share/${CMAKE_PROJECT_NAME}/worlds 
         FILES_MATCHING PATTERN "*.world")
install (DIRECTORY launch/ DESTINATION share/${CMAKE_PROJECT_NAME}/launch 
         FILES_MATCHING PATTERN "*.launch*")
install (DIRECTORY config/ DESTINATION share/${CMAKE_PROJECT_NAME}/config 
         FILES_MATCHING PATTERN "*.yaml")
