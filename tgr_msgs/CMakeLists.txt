cmake_minimum_required(VERSION 3.0.2)
project(tgr_msgs)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Line.msg
  MotorVal.msg
  Int.msg
  CameraSpecs.msg
  PixelPose.msg
  PixelPoses.msg
  VehicleMode.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs std_msgs message_runtime

)







