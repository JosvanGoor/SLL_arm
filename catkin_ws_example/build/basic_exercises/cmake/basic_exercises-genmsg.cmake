# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "basic_exercises: 8 messages, 1 services")

set(MSG_I_FLAGS "-Ibasic_exercises:/home/student/catkin_ws/src/basic_exercises/msg;-Ibasic_exercises:/home/student/catkin_ws/devel/share/basic_exercises/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/indigo/share/sensor_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/indigo/share/actionlib_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(basic_exercises_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/student/catkin_ws/src/basic_exercises/msg/resultbag.msg" NAME_WE)
add_custom_target(_basic_exercises_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "basic_exercises" "/home/student/catkin_ws/src/basic_exercises/msg/resultbag.msg" "std_msgs/Header:sensor_msgs/Image"
)

get_filename_component(_filename "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountActionFeedback.msg" NAME_WE)
add_custom_target(_basic_exercises_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "basic_exercises" "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountActionFeedback.msg" "actionlib_msgs/GoalStatus:actionlib_msgs/GoalID:std_msgs/Header:basic_exercises/pixelcountFeedback"
)

get_filename_component(_filename "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountAction.msg" NAME_WE)
add_custom_target(_basic_exercises_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "basic_exercises" "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountAction.msg" "basic_exercises/pixelcountGoal:actionlib_msgs/GoalStatus:basic_exercises/pixelcountResult:actionlib_msgs/GoalID:basic_exercises/pixelcountActionGoal:std_msgs/Header:basic_exercises/pixelcountActionResult:sensor_msgs/Image:basic_exercises/pixelcountActionFeedback:basic_exercises/pixelcountFeedback"
)

get_filename_component(_filename "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountActionGoal.msg" NAME_WE)
add_custom_target(_basic_exercises_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "basic_exercises" "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountActionGoal.msg" "actionlib_msgs/GoalID:std_msgs/Header:basic_exercises/pixelcountGoal:sensor_msgs/Image"
)

get_filename_component(_filename "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountGoal.msg" NAME_WE)
add_custom_target(_basic_exercises_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "basic_exercises" "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountGoal.msg" "std_msgs/Header:sensor_msgs/Image"
)

get_filename_component(_filename "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountFeedback.msg" NAME_WE)
add_custom_target(_basic_exercises_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "basic_exercises" "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountFeedback.msg" ""
)

get_filename_component(_filename "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountActionResult.msg" NAME_WE)
add_custom_target(_basic_exercises_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "basic_exercises" "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountActionResult.msg" "actionlib_msgs/GoalStatus:basic_exercises/pixelcountResult:actionlib_msgs/GoalID:std_msgs/Header"
)

get_filename_component(_filename "/home/student/catkin_ws/src/basic_exercises/srv/rgb2grey.srv" NAME_WE)
add_custom_target(_basic_exercises_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "basic_exercises" "/home/student/catkin_ws/src/basic_exercises/srv/rgb2grey.srv" "std_msgs/Header:sensor_msgs/Image"
)

get_filename_component(_filename "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountResult.msg" NAME_WE)
add_custom_target(_basic_exercises_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "basic_exercises" "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountResult.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(basic_exercises
  "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/basic_exercises
)
_generate_msg_cpp(basic_exercises
  "/home/student/catkin_ws/src/basic_exercises/msg/resultbag.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/basic_exercises
)
_generate_msg_cpp(basic_exercises
  "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/basic_exercises
)
_generate_msg_cpp(basic_exercises
  "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountAction.msg"
  "${MSG_I_FLAGS}"
  "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountGoal.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountResult.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountActionGoal.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountActionResult.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/Image.msg;/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountActionFeedback.msg;/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/basic_exercises
)
_generate_msg_cpp(basic_exercises
  "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountGoal.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/basic_exercises
)
_generate_msg_cpp(basic_exercises
  "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/basic_exercises
)
_generate_msg_cpp(basic_exercises
  "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountResult.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/basic_exercises
)
_generate_msg_cpp(basic_exercises
  "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/basic_exercises
)

### Generating Services
_generate_srv_cpp(basic_exercises
  "/home/student/catkin_ws/src/basic_exercises/srv/rgb2grey.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/basic_exercises
)

### Generating Module File
_generate_module_cpp(basic_exercises
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/basic_exercises
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(basic_exercises_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(basic_exercises_generate_messages basic_exercises_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/student/catkin_ws/src/basic_exercises/msg/resultbag.msg" NAME_WE)
add_dependencies(basic_exercises_generate_messages_cpp _basic_exercises_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountActionFeedback.msg" NAME_WE)
add_dependencies(basic_exercises_generate_messages_cpp _basic_exercises_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountAction.msg" NAME_WE)
add_dependencies(basic_exercises_generate_messages_cpp _basic_exercises_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountActionGoal.msg" NAME_WE)
add_dependencies(basic_exercises_generate_messages_cpp _basic_exercises_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountGoal.msg" NAME_WE)
add_dependencies(basic_exercises_generate_messages_cpp _basic_exercises_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountFeedback.msg" NAME_WE)
add_dependencies(basic_exercises_generate_messages_cpp _basic_exercises_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountActionResult.msg" NAME_WE)
add_dependencies(basic_exercises_generate_messages_cpp _basic_exercises_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/src/basic_exercises/srv/rgb2grey.srv" NAME_WE)
add_dependencies(basic_exercises_generate_messages_cpp _basic_exercises_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountResult.msg" NAME_WE)
add_dependencies(basic_exercises_generate_messages_cpp _basic_exercises_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(basic_exercises_gencpp)
add_dependencies(basic_exercises_gencpp basic_exercises_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS basic_exercises_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(basic_exercises
  "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/basic_exercises
)
_generate_msg_lisp(basic_exercises
  "/home/student/catkin_ws/src/basic_exercises/msg/resultbag.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/basic_exercises
)
_generate_msg_lisp(basic_exercises
  "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/basic_exercises
)
_generate_msg_lisp(basic_exercises
  "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountAction.msg"
  "${MSG_I_FLAGS}"
  "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountGoal.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountResult.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountActionGoal.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountActionResult.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/Image.msg;/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountActionFeedback.msg;/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/basic_exercises
)
_generate_msg_lisp(basic_exercises
  "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountGoal.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/basic_exercises
)
_generate_msg_lisp(basic_exercises
  "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/basic_exercises
)
_generate_msg_lisp(basic_exercises
  "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountResult.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/basic_exercises
)
_generate_msg_lisp(basic_exercises
  "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/basic_exercises
)

### Generating Services
_generate_srv_lisp(basic_exercises
  "/home/student/catkin_ws/src/basic_exercises/srv/rgb2grey.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/basic_exercises
)

### Generating Module File
_generate_module_lisp(basic_exercises
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/basic_exercises
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(basic_exercises_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(basic_exercises_generate_messages basic_exercises_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/student/catkin_ws/src/basic_exercises/msg/resultbag.msg" NAME_WE)
add_dependencies(basic_exercises_generate_messages_lisp _basic_exercises_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountActionFeedback.msg" NAME_WE)
add_dependencies(basic_exercises_generate_messages_lisp _basic_exercises_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountAction.msg" NAME_WE)
add_dependencies(basic_exercises_generate_messages_lisp _basic_exercises_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountActionGoal.msg" NAME_WE)
add_dependencies(basic_exercises_generate_messages_lisp _basic_exercises_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountGoal.msg" NAME_WE)
add_dependencies(basic_exercises_generate_messages_lisp _basic_exercises_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountFeedback.msg" NAME_WE)
add_dependencies(basic_exercises_generate_messages_lisp _basic_exercises_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountActionResult.msg" NAME_WE)
add_dependencies(basic_exercises_generate_messages_lisp _basic_exercises_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/src/basic_exercises/srv/rgb2grey.srv" NAME_WE)
add_dependencies(basic_exercises_generate_messages_lisp _basic_exercises_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountResult.msg" NAME_WE)
add_dependencies(basic_exercises_generate_messages_lisp _basic_exercises_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(basic_exercises_genlisp)
add_dependencies(basic_exercises_genlisp basic_exercises_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS basic_exercises_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(basic_exercises
  "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/basic_exercises
)
_generate_msg_py(basic_exercises
  "/home/student/catkin_ws/src/basic_exercises/msg/resultbag.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/basic_exercises
)
_generate_msg_py(basic_exercises
  "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/basic_exercises
)
_generate_msg_py(basic_exercises
  "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountAction.msg"
  "${MSG_I_FLAGS}"
  "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountGoal.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountResult.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountActionGoal.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountActionResult.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/Image.msg;/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountActionFeedback.msg;/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/basic_exercises
)
_generate_msg_py(basic_exercises
  "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountGoal.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/basic_exercises
)
_generate_msg_py(basic_exercises
  "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/basic_exercises
)
_generate_msg_py(basic_exercises
  "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountResult.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/basic_exercises
)
_generate_msg_py(basic_exercises
  "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/basic_exercises
)

### Generating Services
_generate_srv_py(basic_exercises
  "/home/student/catkin_ws/src/basic_exercises/srv/rgb2grey.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/basic_exercises
)

### Generating Module File
_generate_module_py(basic_exercises
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/basic_exercises
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(basic_exercises_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(basic_exercises_generate_messages basic_exercises_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/student/catkin_ws/src/basic_exercises/msg/resultbag.msg" NAME_WE)
add_dependencies(basic_exercises_generate_messages_py _basic_exercises_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountActionFeedback.msg" NAME_WE)
add_dependencies(basic_exercises_generate_messages_py _basic_exercises_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountAction.msg" NAME_WE)
add_dependencies(basic_exercises_generate_messages_py _basic_exercises_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountActionGoal.msg" NAME_WE)
add_dependencies(basic_exercises_generate_messages_py _basic_exercises_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountGoal.msg" NAME_WE)
add_dependencies(basic_exercises_generate_messages_py _basic_exercises_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountFeedback.msg" NAME_WE)
add_dependencies(basic_exercises_generate_messages_py _basic_exercises_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountActionResult.msg" NAME_WE)
add_dependencies(basic_exercises_generate_messages_py _basic_exercises_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/src/basic_exercises/srv/rgb2grey.srv" NAME_WE)
add_dependencies(basic_exercises_generate_messages_py _basic_exercises_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/devel/share/basic_exercises/msg/pixelcountResult.msg" NAME_WE)
add_dependencies(basic_exercises_generate_messages_py _basic_exercises_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(basic_exercises_genpy)
add_dependencies(basic_exercises_genpy basic_exercises_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS basic_exercises_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/basic_exercises)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/basic_exercises
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(basic_exercises_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(basic_exercises_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(basic_exercises_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/basic_exercises)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/basic_exercises
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(basic_exercises_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(basic_exercises_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(basic_exercises_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/basic_exercises)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/basic_exercises\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/basic_exercises
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(basic_exercises_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(basic_exercises_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(basic_exercises_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
