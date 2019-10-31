cmake_minimum_required(VERSION 2.8.3)
project(yeetbot_localisation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
)

catkin_package()

# TODO: Add actual compiling stuff here if we're using C++
# TODO: Add the python CMake stuff if we're using Python
