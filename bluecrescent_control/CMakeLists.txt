cmake_minimum_required(VERSION 2.8.3)
project(bluecrescent_control)

find_package(WiringPi REQUIRED)
#find_package(Threads REQUIRED)

find_library(WIRINGPI_LIBRARY wiringPi /home/ubuntu/lib/WiringNP/wiringPi)

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
 LIBRARIES bluecrescnt_control
 CATKIN_DEPENDS roscpp hardware_interface combined_robot_hw transmission_interface controller_manager
 # DEPENDS system_lib
)

add_library(${PROJECT_NAME} 
	src/HEADHW.cpp
	src/LARMHW.cpp
	src/RARMHW.cpp
	)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${WIRINGPI_INCLUDE_DIRS}
)

add_executable(bluecrescent_app src/bluecrescent_app.cpp)
target_link_libraries(bluecrescent_app
  ${catkin_LIBRARIES}
  ${WIRINGPI_LIBRARIES}
  -lwiringPi
  -lpthread
)
add_dependencies(bluecrescent_app wiringPi)



install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS bluecrescent_app
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES bluecrescent_control_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
