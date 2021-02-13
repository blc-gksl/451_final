# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "tgr_msgs: 7 messages, 0 services")

set(MSG_I_FLAGS "-Itgr_msgs:/home/berkay/finalprj_ws/src/tgr_msgs/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(tgr_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/VehicleMode.msg" NAME_WE)
add_custom_target(_tgr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tgr_msgs" "/home/berkay/finalprj_ws/src/tgr_msgs/msg/VehicleMode.msg" ""
)

get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/CameraSpecs.msg" NAME_WE)
add_custom_target(_tgr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tgr_msgs" "/home/berkay/finalprj_ws/src/tgr_msgs/msg/CameraSpecs.msg" "geometry_msgs/Point"
)

get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/PixelPose.msg" NAME_WE)
add_custom_target(_tgr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tgr_msgs" "/home/berkay/finalprj_ws/src/tgr_msgs/msg/PixelPose.msg" ""
)

get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/PixelPoses.msg" NAME_WE)
add_custom_target(_tgr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tgr_msgs" "/home/berkay/finalprj_ws/src/tgr_msgs/msg/PixelPoses.msg" "tgr_msgs/PixelPose"
)

get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/Int.msg" NAME_WE)
add_custom_target(_tgr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tgr_msgs" "/home/berkay/finalprj_ws/src/tgr_msgs/msg/Int.msg" ""
)

get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/MotorVal.msg" NAME_WE)
add_custom_target(_tgr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tgr_msgs" "/home/berkay/finalprj_ws/src/tgr_msgs/msg/MotorVal.msg" ""
)

get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/Line.msg" NAME_WE)
add_custom_target(_tgr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tgr_msgs" "/home/berkay/finalprj_ws/src/tgr_msgs/msg/Line.msg" "geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(tgr_msgs
  "/home/berkay/finalprj_ws/src/tgr_msgs/msg/VehicleMode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tgr_msgs
)
_generate_msg_cpp(tgr_msgs
  "/home/berkay/finalprj_ws/src/tgr_msgs/msg/Int.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tgr_msgs
)
_generate_msg_cpp(tgr_msgs
  "/home/berkay/finalprj_ws/src/tgr_msgs/msg/PixelPose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tgr_msgs
)
_generate_msg_cpp(tgr_msgs
  "/home/berkay/finalprj_ws/src/tgr_msgs/msg/PixelPoses.msg"
  "${MSG_I_FLAGS}"
  "/home/berkay/finalprj_ws/src/tgr_msgs/msg/PixelPose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tgr_msgs
)
_generate_msg_cpp(tgr_msgs
  "/home/berkay/finalprj_ws/src/tgr_msgs/msg/CameraSpecs.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tgr_msgs
)
_generate_msg_cpp(tgr_msgs
  "/home/berkay/finalprj_ws/src/tgr_msgs/msg/MotorVal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tgr_msgs
)
_generate_msg_cpp(tgr_msgs
  "/home/berkay/finalprj_ws/src/tgr_msgs/msg/Line.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tgr_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(tgr_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tgr_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(tgr_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(tgr_msgs_generate_messages tgr_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/VehicleMode.msg" NAME_WE)
add_dependencies(tgr_msgs_generate_messages_cpp _tgr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/CameraSpecs.msg" NAME_WE)
add_dependencies(tgr_msgs_generate_messages_cpp _tgr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/PixelPose.msg" NAME_WE)
add_dependencies(tgr_msgs_generate_messages_cpp _tgr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/PixelPoses.msg" NAME_WE)
add_dependencies(tgr_msgs_generate_messages_cpp _tgr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/Int.msg" NAME_WE)
add_dependencies(tgr_msgs_generate_messages_cpp _tgr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/MotorVal.msg" NAME_WE)
add_dependencies(tgr_msgs_generate_messages_cpp _tgr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/Line.msg" NAME_WE)
add_dependencies(tgr_msgs_generate_messages_cpp _tgr_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tgr_msgs_gencpp)
add_dependencies(tgr_msgs_gencpp tgr_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tgr_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(tgr_msgs
  "/home/berkay/finalprj_ws/src/tgr_msgs/msg/VehicleMode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tgr_msgs
)
_generate_msg_eus(tgr_msgs
  "/home/berkay/finalprj_ws/src/tgr_msgs/msg/Int.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tgr_msgs
)
_generate_msg_eus(tgr_msgs
  "/home/berkay/finalprj_ws/src/tgr_msgs/msg/PixelPose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tgr_msgs
)
_generate_msg_eus(tgr_msgs
  "/home/berkay/finalprj_ws/src/tgr_msgs/msg/PixelPoses.msg"
  "${MSG_I_FLAGS}"
  "/home/berkay/finalprj_ws/src/tgr_msgs/msg/PixelPose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tgr_msgs
)
_generate_msg_eus(tgr_msgs
  "/home/berkay/finalprj_ws/src/tgr_msgs/msg/CameraSpecs.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tgr_msgs
)
_generate_msg_eus(tgr_msgs
  "/home/berkay/finalprj_ws/src/tgr_msgs/msg/MotorVal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tgr_msgs
)
_generate_msg_eus(tgr_msgs
  "/home/berkay/finalprj_ws/src/tgr_msgs/msg/Line.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tgr_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(tgr_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tgr_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(tgr_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(tgr_msgs_generate_messages tgr_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/VehicleMode.msg" NAME_WE)
add_dependencies(tgr_msgs_generate_messages_eus _tgr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/CameraSpecs.msg" NAME_WE)
add_dependencies(tgr_msgs_generate_messages_eus _tgr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/PixelPose.msg" NAME_WE)
add_dependencies(tgr_msgs_generate_messages_eus _tgr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/PixelPoses.msg" NAME_WE)
add_dependencies(tgr_msgs_generate_messages_eus _tgr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/Int.msg" NAME_WE)
add_dependencies(tgr_msgs_generate_messages_eus _tgr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/MotorVal.msg" NAME_WE)
add_dependencies(tgr_msgs_generate_messages_eus _tgr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/Line.msg" NAME_WE)
add_dependencies(tgr_msgs_generate_messages_eus _tgr_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tgr_msgs_geneus)
add_dependencies(tgr_msgs_geneus tgr_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tgr_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(tgr_msgs
  "/home/berkay/finalprj_ws/src/tgr_msgs/msg/VehicleMode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tgr_msgs
)
_generate_msg_lisp(tgr_msgs
  "/home/berkay/finalprj_ws/src/tgr_msgs/msg/Int.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tgr_msgs
)
_generate_msg_lisp(tgr_msgs
  "/home/berkay/finalprj_ws/src/tgr_msgs/msg/PixelPose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tgr_msgs
)
_generate_msg_lisp(tgr_msgs
  "/home/berkay/finalprj_ws/src/tgr_msgs/msg/PixelPoses.msg"
  "${MSG_I_FLAGS}"
  "/home/berkay/finalprj_ws/src/tgr_msgs/msg/PixelPose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tgr_msgs
)
_generate_msg_lisp(tgr_msgs
  "/home/berkay/finalprj_ws/src/tgr_msgs/msg/CameraSpecs.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tgr_msgs
)
_generate_msg_lisp(tgr_msgs
  "/home/berkay/finalprj_ws/src/tgr_msgs/msg/MotorVal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tgr_msgs
)
_generate_msg_lisp(tgr_msgs
  "/home/berkay/finalprj_ws/src/tgr_msgs/msg/Line.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tgr_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(tgr_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tgr_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(tgr_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(tgr_msgs_generate_messages tgr_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/VehicleMode.msg" NAME_WE)
add_dependencies(tgr_msgs_generate_messages_lisp _tgr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/CameraSpecs.msg" NAME_WE)
add_dependencies(tgr_msgs_generate_messages_lisp _tgr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/PixelPose.msg" NAME_WE)
add_dependencies(tgr_msgs_generate_messages_lisp _tgr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/PixelPoses.msg" NAME_WE)
add_dependencies(tgr_msgs_generate_messages_lisp _tgr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/Int.msg" NAME_WE)
add_dependencies(tgr_msgs_generate_messages_lisp _tgr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/MotorVal.msg" NAME_WE)
add_dependencies(tgr_msgs_generate_messages_lisp _tgr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/Line.msg" NAME_WE)
add_dependencies(tgr_msgs_generate_messages_lisp _tgr_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tgr_msgs_genlisp)
add_dependencies(tgr_msgs_genlisp tgr_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tgr_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(tgr_msgs
  "/home/berkay/finalprj_ws/src/tgr_msgs/msg/VehicleMode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tgr_msgs
)
_generate_msg_nodejs(tgr_msgs
  "/home/berkay/finalprj_ws/src/tgr_msgs/msg/Int.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tgr_msgs
)
_generate_msg_nodejs(tgr_msgs
  "/home/berkay/finalprj_ws/src/tgr_msgs/msg/PixelPose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tgr_msgs
)
_generate_msg_nodejs(tgr_msgs
  "/home/berkay/finalprj_ws/src/tgr_msgs/msg/PixelPoses.msg"
  "${MSG_I_FLAGS}"
  "/home/berkay/finalprj_ws/src/tgr_msgs/msg/PixelPose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tgr_msgs
)
_generate_msg_nodejs(tgr_msgs
  "/home/berkay/finalprj_ws/src/tgr_msgs/msg/CameraSpecs.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tgr_msgs
)
_generate_msg_nodejs(tgr_msgs
  "/home/berkay/finalprj_ws/src/tgr_msgs/msg/MotorVal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tgr_msgs
)
_generate_msg_nodejs(tgr_msgs
  "/home/berkay/finalprj_ws/src/tgr_msgs/msg/Line.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tgr_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(tgr_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tgr_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(tgr_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(tgr_msgs_generate_messages tgr_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/VehicleMode.msg" NAME_WE)
add_dependencies(tgr_msgs_generate_messages_nodejs _tgr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/CameraSpecs.msg" NAME_WE)
add_dependencies(tgr_msgs_generate_messages_nodejs _tgr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/PixelPose.msg" NAME_WE)
add_dependencies(tgr_msgs_generate_messages_nodejs _tgr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/PixelPoses.msg" NAME_WE)
add_dependencies(tgr_msgs_generate_messages_nodejs _tgr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/Int.msg" NAME_WE)
add_dependencies(tgr_msgs_generate_messages_nodejs _tgr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/MotorVal.msg" NAME_WE)
add_dependencies(tgr_msgs_generate_messages_nodejs _tgr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/Line.msg" NAME_WE)
add_dependencies(tgr_msgs_generate_messages_nodejs _tgr_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tgr_msgs_gennodejs)
add_dependencies(tgr_msgs_gennodejs tgr_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tgr_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(tgr_msgs
  "/home/berkay/finalprj_ws/src/tgr_msgs/msg/VehicleMode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tgr_msgs
)
_generate_msg_py(tgr_msgs
  "/home/berkay/finalprj_ws/src/tgr_msgs/msg/Int.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tgr_msgs
)
_generate_msg_py(tgr_msgs
  "/home/berkay/finalprj_ws/src/tgr_msgs/msg/PixelPose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tgr_msgs
)
_generate_msg_py(tgr_msgs
  "/home/berkay/finalprj_ws/src/tgr_msgs/msg/PixelPoses.msg"
  "${MSG_I_FLAGS}"
  "/home/berkay/finalprj_ws/src/tgr_msgs/msg/PixelPose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tgr_msgs
)
_generate_msg_py(tgr_msgs
  "/home/berkay/finalprj_ws/src/tgr_msgs/msg/CameraSpecs.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tgr_msgs
)
_generate_msg_py(tgr_msgs
  "/home/berkay/finalprj_ws/src/tgr_msgs/msg/MotorVal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tgr_msgs
)
_generate_msg_py(tgr_msgs
  "/home/berkay/finalprj_ws/src/tgr_msgs/msg/Line.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tgr_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(tgr_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tgr_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(tgr_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(tgr_msgs_generate_messages tgr_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/VehicleMode.msg" NAME_WE)
add_dependencies(tgr_msgs_generate_messages_py _tgr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/CameraSpecs.msg" NAME_WE)
add_dependencies(tgr_msgs_generate_messages_py _tgr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/PixelPose.msg" NAME_WE)
add_dependencies(tgr_msgs_generate_messages_py _tgr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/PixelPoses.msg" NAME_WE)
add_dependencies(tgr_msgs_generate_messages_py _tgr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/Int.msg" NAME_WE)
add_dependencies(tgr_msgs_generate_messages_py _tgr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/MotorVal.msg" NAME_WE)
add_dependencies(tgr_msgs_generate_messages_py _tgr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/berkay/finalprj_ws/src/tgr_msgs/msg/Line.msg" NAME_WE)
add_dependencies(tgr_msgs_generate_messages_py _tgr_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tgr_msgs_genpy)
add_dependencies(tgr_msgs_genpy tgr_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tgr_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tgr_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tgr_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(tgr_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(tgr_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tgr_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tgr_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(tgr_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(tgr_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tgr_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tgr_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(tgr_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(tgr_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tgr_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tgr_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(tgr_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(tgr_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tgr_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tgr_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tgr_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(tgr_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(tgr_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
