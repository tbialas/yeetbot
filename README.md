# YEETBot-3000
## Yiannis' Electrically Enhanced Toolbox Bot

Github repository for our group for the 4th year Human Centred Robotics course at Imperial College London (EEE Dept.)

### Team Members
- Tomasz Białas (tjb16)
- Nick Hafner (nrh16)
- Ben Biggs (bb2515)
- Tom Poskitt (tjp16)
- Archit Sharma (as10216)
- Sergey Zhelyabovskiy (sz4316)

## Robot Purpose

YEETBot3000 is a robot designed to interact with humans who are using the ICRS (Imperial College Robotics Society) lab. A particular problem the ICRS committee have is that many people do not return tools to the correct places, instead opting to leave them out (or worse, lose them). YEETBot3000 aims to discover whether a human centred robot can improve the return rate of tools, and how much of an effect different styles of robot-human interaction have on tool return rate.

## Requirements

Required Software:
 - [ROS Melodic](http://wiki.ros.org/melodic)
 - [rosaria](http://wiki.ros.org/ROSARIA/Tutorials/How%20to%20use%20ROSARIA)
 - [apriltag_ros](http://wiki.ros.org/apriltag_ros)
 - [libfreenect](https://openkinect.org/wiki/Main_Page)
 - [freenect_launch](http://wiki.ros.org/freenect_launch)
 - [tensorflow](https://www.tensorflow.org/)
 - [opencv](https://opencv.org/)
 - [ROS Navigation Stack](http://wiki.ros.org/navigation)

(ROS dependencies can be installed via `rosdep install --from-path src --ignore-src -r -y` when run from the root of your catkin workspace)

##### yeet

### Packages

#### yeetbot_arduino 
- contains the script running on the Arduino that controls the operation of the drawers and detection of the RFID tags. 
- contains a node that communicates via serial with the arduino and publishes the drawer information (read by the controller).
- contains PCB gerber and circuit diagram files for the set of drawers.

#### yeetbot_bringup
- contains the launch files to be run on all the computers integrated in YEETBot's ROS network.

#### yeetbot_gui 
- contains the assets and nodes required to run the GUI on a laptop using the Qt library.
- Subscribes to state machine topics and diplays text, interaction options and custom animations.

#### yeetbot_humantracker
- Uses a single pass detector from the Tensorflow Object Detection API for the human detection.
- Uses the reference implementation of the SORT algorithm for human tracking.
- Uses a custom algorithm to stitch the tracked humans together to maintain continuous localisations of the humans through time.
- The nodes publish the positions to be available for the controller.

#### yeetbot_localisation
- Uses the april_tags package to obtain the pose of the robot from the Kinect cameras in the lab.
- Uses an Extended Kalman Filter to fuse the noisey position data from both cameras and publishes the baselink to map transform.

#### yeetbot_mastercontroller
- Contains the main state machine that handles human interaction, robot behaviour and database management.
- Contains all the interfaces used to publish/subscribe to all yeetbot sub-functions.
- All custom message types are contained in yeetbot_msgs.

#### yeetbot_natural_language
- Uses the MATRIX Voice microphone in combination with the ODAS library for detecting direction of arrival.
- Uses pre-existing model and wake word to trigger the direction of arrival to be published to the master controller.
- Uses Google Cloud STT to convert speech to text and Google TTS to play the messages displayed on the GUI.

#### yeetbot_navigation
- Uses the ROS Navigation stack with the global planner configured to use Dijkstra's algorithm for path planning and the base local planner for obstacle avoidance.
- Contains the URDF of the robot which uses robot_state_publisher to publishes the positions of the apriltags, on-board Kinect, Hokuyo LIDAR relative to the model of the Pioneer base with drawers attached.
