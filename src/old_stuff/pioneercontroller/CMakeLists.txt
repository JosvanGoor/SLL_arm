cmake_minimum_required (VERSION 2.8)
project(pioneercontroller)

find_package(catkin REQUIRED COMPONENTS 
        roscpp 
        geometry_msgs 
        nav_msgs
        std_msgs
        tf
)

set (ARIA_PATH
    /usr/local/Aria/lib
)

find_library(ARIA_LIB Aria ${ARIA_PATH})
find_package(Threads)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pioneercontrollerlib motioncontroller commandparser
  CATKIN_DEPENDS roscpp tf geometry_msgs nav_msgs std_msgs
#  DEPENDS system_lib
)


include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(/usr/local/Aria/include)

add_definitions("-std=c++0x -Wall -static -pthread")

add_library(commandparser
    src/commandparser/constructor.cc
    src/commandparser/parsetwistcommand.cc
    src/commandparser/destructor.cc
    src/commandparser/parsecommand.cc
)

add_library(motioncontroller
    src/motioncontroller/constructor.cc
    src/motioncontroller/handlecommand.cc
    src/motioncontroller/handlecommanddouble.cc
    src/motioncontroller/drive_turn.cc
    src/motioncontroller/start.cc
    src/motioncontroller/handle_signal.cc
    src/motioncontroller/staticdata.cc
    src/motioncontroller/set_options.cc
    src/motioncontroller/set_cmd_vel.cc
    src/motioncontroller/connect_robot.cc
    src/motioncontroller/on_disconnected.cc
    src/motioncontroller/stop.cc
    src/motioncontroller/on_connected.cc
    src/motioncontroller/destructor.cc
    src/motioncontroller/disconnect_robot.cc
    src/motioncontroller/set_speeds.cc
    src/motioncontroller/runner.cc
    src/motioncontroller/get_odometry.cc
    src/motioncontroller/tdiff.cc
    src/motioncontroller/get_status_string.cc
    src/motioncontroller/on_conn_fail.cc
    src/motioncontroller/drive_forward.cc
)

add_library(mainlib
    src/pioneercontroller/constructor.cc
    src/pioneercontroller/publish_odometry.cc
    src/pioneercontroller/publish_status.cc
    src/pioneercontroller/run.cc
    src/pioneercontroller/twist_message_received.cc
    src/pioneercontroller/message_received.cc
)

add_executable(
    pioneercontroller
    src/main.cc
)

add_executable(
    battery
    src/battery.cc
)
target_link_libraries(mainlib ${catkin_LIBRARIES})
target_link_libraries(
    pioneercontroller
    ${catkin_LIBRARIES}
    mainlib
    motioncontroller
    commandparser
    ${CMAKE_THREAD_LIBS_INIT}
    ${ARIA_LIB}
    dl
    pthread
    rt
)

target_link_libraries(
    battery
    ${catkin_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    ${ARIA_LIB}
    dl
    pthread
    rt
)
