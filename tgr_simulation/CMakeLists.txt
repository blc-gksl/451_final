cmake_minimum_required(VERSION 3.0.2)
project(tgr_simulation)
add_compile_options(-std=c++11)
add_compile_options(-O2)
add_compile_options(-pthread)
find_package(catkin REQUIRED COMPONENTS
		roscpp
		rospy
		rviz
		std_msgs
		tf
		urdf
		xacro
		geometry_msgs
		message_generation
		message_runtime
		gazebo_ros
		image_transport
		sensor_msgs
		nav_msgs
		tgr_msgs
		)
find_package(gazebo REQUIRED)
include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS})
include_directories(
		include
		${PYTHON_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
catkin_package( CATKIN_DEPENDS message_runtime )

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
		DEPENDS
		roscpp
		gazebo_ros
)
add_executable(objSpawner src/objSpawner.cpp include/gazebo_cube_spawner.h)
target_link_libraries(objSpawner ${catkin_LIBRARIES}${GAZEBO_LIBRARIES})
add_executable(readdemo src/readdemo.cpp)
target_link_libraries(readdemo ${catkin_LIBRARIES}${GAZEBO_LIBRARIES})
add_executable(main src/main.cpp include/common.h include/common.cpp)
target_link_libraries(main ${catkin_LIBRARIES}${GAZEBO_LIBRARIES})