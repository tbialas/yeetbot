cmake_minimum_required(VERSION 2.8.3)
project(yeetbot_navigation)

#freenect_stack needs to be compiled manually
#libfreenect needs to be installed manually, check that ROS points to the right dir
find_package(catkin REQUIRED COMPONENTS geometry_msgs sensor_msgs std_msgs robot_state_publisher urdf tf move_base costmap_2d robot_localization pointcloud_to_laserscan map_server)

catkin_package(
    CATKIN_DEPENDS robot_state_publisher urdf geometry_msgs sensor_msgs std_msgs
)

