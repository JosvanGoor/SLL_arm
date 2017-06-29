cmake_minimum_required(VERSION 2.8.3)
project(object_recognition)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  std_msgs
)
find_library(JSONCPP_LIBRARY NAMES jsoncpp)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES camtest
  CATKIN_DEPENDS roscpp pcl_ros sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(camtest
#   src/${PROJECT_NAME}/main.cc
# )

## Declare a cpp executable
add_executable(object_recognition_node src/main.cc src/objectRecognizer/jsonParser.cc src/objectRecognizer/filterView.cc src/objectRecognizer/getSize.cc src/objectRecognizer/classify.cc src/objectRecognizer/extractFeatures.cc src/objectRecognizer/colorClusters.cc src/objectRecognizer/objectRecognizer.cc src/objectRecognizer/processpcl.cc src/objectRecognizer/cleanUpImage.cc src/planarRemover/removePlanarSurface.cc src/clusterObjects/cluster.cc src/clusterObjects/clusterObjects.cc src/objectRecognizer/run.cc src/objectRecognizer/writeResults.cc src/objectRecognizer/sample.cc src/objectRecognizer/publishize.cc src/objectRecognizer/splitClusters.cc src/objectRecognizer/computeNormals.cc src/objectRecognizer/estimateDescriptors.cc src/objectRecognizer/loadDB.cc)
target_link_libraries(object_recognition_node ${catkin_LIBRARIES} jsoncpp)
add_dependencies(object_recognition_node object_recognition_generate_messages_cpp)


