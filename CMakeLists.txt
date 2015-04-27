cmake_minimum_required(VERSION 2.8.3)
project(roboteq_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  device_driver_base
  roscpp
  hardware_interface
)


catkin_package(
  INCLUDE_DIRS include src/roboteq_lib
  LIBRARIES roboteq_motor_controller roboteq_controller_hw
  CATKIN_DEPENDS 
  device_driver_base
  roscpp
  hardware_interface
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  src/roboteq_lib
  ${catkin_INCLUDE_DIRS}
)


add_library(roboteq_lib src/roboteq_lib/RoboteqDevice.cpp)

add_library(roboteq_motor_controller src/roboteq_motor_controller.cpp)
target_link_libraries(roboteq_motor_controller roboteq_lib  ${catkin_LIBRARIES})

add_library(roboteq_controller_hw src/roboteq_controller_hw.cpp)
target_link_libraries(roboteq_controller_hw roboteq_motor_controller ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS roboteq_motor_controller roboteq_controller_hw
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
install(DIRECTORY src/roboteq_lib
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
