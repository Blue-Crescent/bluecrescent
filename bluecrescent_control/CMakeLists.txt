cmake_minimum_required(VERSION 2.8.3)
project(bluecrescent_control)


FIND_LIBRARY(WIRINGPI_LIBRARY wiringPi /usr/local/lib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  combined_robot_hw
  hardware_interface
  transmission_interface
  controller_manager
  actionlib
  actionlib_msgs
  trajectory_msgs
  control_msgs
  sensor_msgs
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES bluecrescent_control
  CATKIN_DEPENDS roscpp hardware_interface transmission_interface controller_manager
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(bluecrescent_control 
	src/bluecrescent_control.cpp
       	src/bluecrescent_hw_head.cpp
	src/bluecrescent_hw_rarm.cpp
	#       	src/bluecrescent_hw_larm.cpp
	
	)
add_dependencies(bluecrescent_control wiringPi)

target_link_libraries(bluecrescent_control
  ${catkin_LIBRARIES}
  ${WIRINGPI_LIBRARY}
)

install(TARGETS bluecrescent_control
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
