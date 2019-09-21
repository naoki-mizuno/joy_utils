cmake_minimum_required(VERSION 2.8.3)
project(joy_utils)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  sensor_msgs
)

# add_message_files(
#   DIRECTORY msg
#   FILES NormalizedJoy.msg
# )

catkin_python_setup()

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   sensor_msgs
# )

catkin_package()
