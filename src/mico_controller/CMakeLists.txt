cmake_minimum_required(VERSION 2.8.3)
project(mico_controller)


find_package(catkin REQUIRED roscpp actionlib actionlib_msgs)

link_directories(${PROJECT_SOURCE_DIR}/lib/${CMAKE_LIBRARY_ARCHITECTURE})


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp

)

###########
## Build ##
###########

# Copy over Kinova .so binary blobs to /usr/lib
install(FILES
  lib/${CMAKE_LIBRARY_ARCHITECTURE}/Kinova.API.CommLayerUbuntu.so
  lib/${CMAKE_LIBRARY_ARCHITECTURE}/Kinova.API.USBCommandLayerUbuntu.so
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
#dd_dependencies(mico_controller_node ${catkin_EXPORTED_TARGETS} actionlib_msgs)

## Declare a C++ executable
add_executable(mico_controller_node src/main.cc src/micoArm.cc src/trajectory.cc)
#add_executable(nn_tester_node src/nnTester.cc src/micoArm.cc)

## Specify libraries to link a library or executable target against
 target_link_libraries(mico_controller_node
   ${catkin_LIBRARIES} 
   -ldl
 )

