# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "mujoco_ros_msgs: 7 messages, 0 services")

set(MSG_I_FLAGS "-Imujoco_ros_msgs:/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(mujoco_ros_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/JointInit.msg" NAME_WE)
add_custom_target(_mujoco_ros_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mujoco_ros_msgs" "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/JointInit.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/JointSet.msg" NAME_WE)
add_custom_target(_mujoco_ros_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mujoco_ros_msgs" "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/JointSet.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/JointState.msg" NAME_WE)
add_custom_target(_mujoco_ros_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mujoco_ros_msgs" "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/JointState.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SensorState.msg" NAME_WE)
add_custom_target(_mujoco_ros_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mujoco_ros_msgs" "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SensorState.msg" "mujoco_ros_msgs/SensorBase:std_msgs/Header"
)

get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SensorBase.msg" NAME_WE)
add_custom_target(_mujoco_ros_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mujoco_ros_msgs" "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SensorBase.msg" ""
)

get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SimstatusM2C.msg" NAME_WE)
add_custom_target(_mujoco_ros_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mujoco_ros_msgs" "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SimstatusM2C.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SimStatus.msg" NAME_WE)
add_custom_target(_mujoco_ros_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mujoco_ros_msgs" "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SimStatus.msg" "mujoco_ros_msgs/SensorBase:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(mujoco_ros_msgs
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/JointInit.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mujoco_ros_msgs
)
_generate_msg_cpp(mujoco_ros_msgs
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/JointSet.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mujoco_ros_msgs
)
_generate_msg_cpp(mujoco_ros_msgs
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/JointState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mujoco_ros_msgs
)
_generate_msg_cpp(mujoco_ros_msgs
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SensorState.msg"
  "${MSG_I_FLAGS}"
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SensorBase.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mujoco_ros_msgs
)
_generate_msg_cpp(mujoco_ros_msgs
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SensorBase.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mujoco_ros_msgs
)
_generate_msg_cpp(mujoco_ros_msgs
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SimstatusM2C.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mujoco_ros_msgs
)
_generate_msg_cpp(mujoco_ros_msgs
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SimStatus.msg"
  "${MSG_I_FLAGS}"
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SensorBase.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mujoco_ros_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(mujoco_ros_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mujoco_ros_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(mujoco_ros_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(mujoco_ros_msgs_generate_messages mujoco_ros_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/JointInit.msg" NAME_WE)
add_dependencies(mujoco_ros_msgs_generate_messages_cpp _mujoco_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/JointSet.msg" NAME_WE)
add_dependencies(mujoco_ros_msgs_generate_messages_cpp _mujoco_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/JointState.msg" NAME_WE)
add_dependencies(mujoco_ros_msgs_generate_messages_cpp _mujoco_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SensorState.msg" NAME_WE)
add_dependencies(mujoco_ros_msgs_generate_messages_cpp _mujoco_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SensorBase.msg" NAME_WE)
add_dependencies(mujoco_ros_msgs_generate_messages_cpp _mujoco_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SimstatusM2C.msg" NAME_WE)
add_dependencies(mujoco_ros_msgs_generate_messages_cpp _mujoco_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SimStatus.msg" NAME_WE)
add_dependencies(mujoco_ros_msgs_generate_messages_cpp _mujoco_ros_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mujoco_ros_msgs_gencpp)
add_dependencies(mujoco_ros_msgs_gencpp mujoco_ros_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mujoco_ros_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(mujoco_ros_msgs
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/JointInit.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mujoco_ros_msgs
)
_generate_msg_eus(mujoco_ros_msgs
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/JointSet.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mujoco_ros_msgs
)
_generate_msg_eus(mujoco_ros_msgs
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/JointState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mujoco_ros_msgs
)
_generate_msg_eus(mujoco_ros_msgs
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SensorState.msg"
  "${MSG_I_FLAGS}"
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SensorBase.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mujoco_ros_msgs
)
_generate_msg_eus(mujoco_ros_msgs
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SensorBase.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mujoco_ros_msgs
)
_generate_msg_eus(mujoco_ros_msgs
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SimstatusM2C.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mujoco_ros_msgs
)
_generate_msg_eus(mujoco_ros_msgs
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SimStatus.msg"
  "${MSG_I_FLAGS}"
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SensorBase.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mujoco_ros_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(mujoco_ros_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mujoco_ros_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(mujoco_ros_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(mujoco_ros_msgs_generate_messages mujoco_ros_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/JointInit.msg" NAME_WE)
add_dependencies(mujoco_ros_msgs_generate_messages_eus _mujoco_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/JointSet.msg" NAME_WE)
add_dependencies(mujoco_ros_msgs_generate_messages_eus _mujoco_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/JointState.msg" NAME_WE)
add_dependencies(mujoco_ros_msgs_generate_messages_eus _mujoco_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SensorState.msg" NAME_WE)
add_dependencies(mujoco_ros_msgs_generate_messages_eus _mujoco_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SensorBase.msg" NAME_WE)
add_dependencies(mujoco_ros_msgs_generate_messages_eus _mujoco_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SimstatusM2C.msg" NAME_WE)
add_dependencies(mujoco_ros_msgs_generate_messages_eus _mujoco_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SimStatus.msg" NAME_WE)
add_dependencies(mujoco_ros_msgs_generate_messages_eus _mujoco_ros_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mujoco_ros_msgs_geneus)
add_dependencies(mujoco_ros_msgs_geneus mujoco_ros_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mujoco_ros_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(mujoco_ros_msgs
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/JointInit.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mujoco_ros_msgs
)
_generate_msg_lisp(mujoco_ros_msgs
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/JointSet.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mujoco_ros_msgs
)
_generate_msg_lisp(mujoco_ros_msgs
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/JointState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mujoco_ros_msgs
)
_generate_msg_lisp(mujoco_ros_msgs
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SensorState.msg"
  "${MSG_I_FLAGS}"
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SensorBase.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mujoco_ros_msgs
)
_generate_msg_lisp(mujoco_ros_msgs
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SensorBase.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mujoco_ros_msgs
)
_generate_msg_lisp(mujoco_ros_msgs
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SimstatusM2C.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mujoco_ros_msgs
)
_generate_msg_lisp(mujoco_ros_msgs
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SimStatus.msg"
  "${MSG_I_FLAGS}"
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SensorBase.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mujoco_ros_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(mujoco_ros_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mujoco_ros_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(mujoco_ros_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(mujoco_ros_msgs_generate_messages mujoco_ros_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/JointInit.msg" NAME_WE)
add_dependencies(mujoco_ros_msgs_generate_messages_lisp _mujoco_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/JointSet.msg" NAME_WE)
add_dependencies(mujoco_ros_msgs_generate_messages_lisp _mujoco_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/JointState.msg" NAME_WE)
add_dependencies(mujoco_ros_msgs_generate_messages_lisp _mujoco_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SensorState.msg" NAME_WE)
add_dependencies(mujoco_ros_msgs_generate_messages_lisp _mujoco_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SensorBase.msg" NAME_WE)
add_dependencies(mujoco_ros_msgs_generate_messages_lisp _mujoco_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SimstatusM2C.msg" NAME_WE)
add_dependencies(mujoco_ros_msgs_generate_messages_lisp _mujoco_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SimStatus.msg" NAME_WE)
add_dependencies(mujoco_ros_msgs_generate_messages_lisp _mujoco_ros_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mujoco_ros_msgs_genlisp)
add_dependencies(mujoco_ros_msgs_genlisp mujoco_ros_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mujoco_ros_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(mujoco_ros_msgs
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/JointInit.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mujoco_ros_msgs
)
_generate_msg_nodejs(mujoco_ros_msgs
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/JointSet.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mujoco_ros_msgs
)
_generate_msg_nodejs(mujoco_ros_msgs
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/JointState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mujoco_ros_msgs
)
_generate_msg_nodejs(mujoco_ros_msgs
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SensorState.msg"
  "${MSG_I_FLAGS}"
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SensorBase.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mujoco_ros_msgs
)
_generate_msg_nodejs(mujoco_ros_msgs
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SensorBase.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mujoco_ros_msgs
)
_generate_msg_nodejs(mujoco_ros_msgs
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SimstatusM2C.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mujoco_ros_msgs
)
_generate_msg_nodejs(mujoco_ros_msgs
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SimStatus.msg"
  "${MSG_I_FLAGS}"
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SensorBase.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mujoco_ros_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(mujoco_ros_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mujoco_ros_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(mujoco_ros_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(mujoco_ros_msgs_generate_messages mujoco_ros_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/JointInit.msg" NAME_WE)
add_dependencies(mujoco_ros_msgs_generate_messages_nodejs _mujoco_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/JointSet.msg" NAME_WE)
add_dependencies(mujoco_ros_msgs_generate_messages_nodejs _mujoco_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/JointState.msg" NAME_WE)
add_dependencies(mujoco_ros_msgs_generate_messages_nodejs _mujoco_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SensorState.msg" NAME_WE)
add_dependencies(mujoco_ros_msgs_generate_messages_nodejs _mujoco_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SensorBase.msg" NAME_WE)
add_dependencies(mujoco_ros_msgs_generate_messages_nodejs _mujoco_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SimstatusM2C.msg" NAME_WE)
add_dependencies(mujoco_ros_msgs_generate_messages_nodejs _mujoco_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SimStatus.msg" NAME_WE)
add_dependencies(mujoco_ros_msgs_generate_messages_nodejs _mujoco_ros_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mujoco_ros_msgs_gennodejs)
add_dependencies(mujoco_ros_msgs_gennodejs mujoco_ros_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mujoco_ros_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(mujoco_ros_msgs
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/JointInit.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mujoco_ros_msgs
)
_generate_msg_py(mujoco_ros_msgs
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/JointSet.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mujoco_ros_msgs
)
_generate_msg_py(mujoco_ros_msgs
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/JointState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mujoco_ros_msgs
)
_generate_msg_py(mujoco_ros_msgs
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SensorState.msg"
  "${MSG_I_FLAGS}"
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SensorBase.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mujoco_ros_msgs
)
_generate_msg_py(mujoco_ros_msgs
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SensorBase.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mujoco_ros_msgs
)
_generate_msg_py(mujoco_ros_msgs
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SimstatusM2C.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mujoco_ros_msgs
)
_generate_msg_py(mujoco_ros_msgs
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SimStatus.msg"
  "${MSG_I_FLAGS}"
  "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SensorBase.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mujoco_ros_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(mujoco_ros_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mujoco_ros_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(mujoco_ros_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(mujoco_ros_msgs_generate_messages mujoco_ros_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/JointInit.msg" NAME_WE)
add_dependencies(mujoco_ros_msgs_generate_messages_py _mujoco_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/JointSet.msg" NAME_WE)
add_dependencies(mujoco_ros_msgs_generate_messages_py _mujoco_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/JointState.msg" NAME_WE)
add_dependencies(mujoco_ros_msgs_generate_messages_py _mujoco_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SensorState.msg" NAME_WE)
add_dependencies(mujoco_ros_msgs_generate_messages_py _mujoco_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SensorBase.msg" NAME_WE)
add_dependencies(mujoco_ros_msgs_generate_messages_py _mujoco_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SimstatusM2C.msg" NAME_WE)
add_dependencies(mujoco_ros_msgs_generate_messages_py _mujoco_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yilangliu/a1_real_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SimStatus.msg" NAME_WE)
add_dependencies(mujoco_ros_msgs_generate_messages_py _mujoco_ros_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mujoco_ros_msgs_genpy)
add_dependencies(mujoco_ros_msgs_genpy mujoco_ros_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mujoco_ros_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mujoco_ros_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mujoco_ros_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(mujoco_ros_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(mujoco_ros_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mujoco_ros_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mujoco_ros_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(mujoco_ros_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(mujoco_ros_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mujoco_ros_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mujoco_ros_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(mujoco_ros_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(mujoco_ros_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mujoco_ros_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mujoco_ros_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(mujoco_ros_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(mujoco_ros_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mujoco_ros_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mujoco_ros_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mujoco_ros_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(mujoco_ros_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(mujoco_ros_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
