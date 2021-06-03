cmake_minimum_required(VERSION 3.0.2)
project(traffic_count)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp 
  rospy
  std_msgs
  cv_bridge
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  node/image_publisher_node.py  
  node/traffic_count_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  launch/traffic_count.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
