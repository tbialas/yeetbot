cmake_minimum_required(VERSION 2.8.3)
project(yeetbot_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  std_msgs
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  YEETBotState.msg
  YEETBotUserChoices.msg
  YEETBotUserResponse.msg
  YEETBotItemStates.msg
  YEETBotDrawerStates.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)
