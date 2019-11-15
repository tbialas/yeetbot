cmake_minimum_required(VERSION 2.8.3)
project(yeetbot_navigation)

find_package(catkin REQUIRED COMPONENTS geometry_msgs sensor_msgs std_msgs robot_state_publisher urdf)

catkin_package(
    CATKIN_DEPENDS robot_state_publisher urdf geometry_msgs sensor_msgs std_msgs
)

