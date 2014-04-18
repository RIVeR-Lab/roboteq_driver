cmake_minimum_required(VERSION 2.8.3)
project(roboteq_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  device_driver_base
  roscpp
  rospy
)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES hd_driver
#  DEPENDS system_lib
  CATKIN_DEPENDS 
  device_driver_base
  roscpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(include src/roboteq_lib)


add_library(${PROJECT_NAME}RoboteqLib src/roboteq_lib/RoboteqDevice.cpp src/roboteq_motor_controller.cpp)
target_link_libraries(${PROJECT_NAME}RoboteqLib ${catkin_LIBRARIES})

add_library(roboteq_manager_lib src/roboteq_manager_lib.cpp)
target_link_libraries(roboteq_manager_lib ${catkin_LIBRARIES})


add_executable(roboteq_manager src/roboteq_manager.cpp)
target_link_libraries(roboteq_manager ${PROJECT_NAME}RoboteqLib ${catkin_LIBRARIES})
