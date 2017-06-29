cmake_minimum_required(VERSION 2.8.3)
project(borg_nao)

add_definitions("-Wall")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  sensor_msgs
  alice_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_message_files (
#  FILES
#  KinectLine.msg
#)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

catkin_package(
  CATKIN_DEPENDS
)


## Declare a cpp executable
add_executable(colorblob_node src/main.cc     
	src/colorblob/colorblob1.cc
	src/colorblob/newImageCB.cc
	src/colorblob/onMouse.cc
	src/colorblob/visualMode.cc
	src/colorblob/updateVisualState.cc
	src/colorblob/findBlobs.cc
	src/colorblob/run.cc
	src/colorblob/writeToMemory.cc
	src/colorblob/sortObservations.cc
)


target_link_libraries(
  colorblob_node
  ${catkin_LIBRARIES}
)
