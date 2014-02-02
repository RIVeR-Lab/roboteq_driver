cmake_minimum_required(VERSION 2.8.3)
project(roboteq_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  device_driver_base
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  dynamic_reconfigure
  message_generation
)
generate_dynamic_reconfigure_options(
  cfg/Roboteq.cfg
)
add_dependencies(${PROJECT_NAME}_gencfg device_driver_base_generate_messages_py)

add_message_files(DIRECTORY msg FILES RoboteqGroupInfo.msg RoboteqGroupMotorControl.msg RoboteqMotorControl.msg RoboteqMotorInfo.msg)
generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs device_driver_base)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES hd_driver
#  DEPENDS system_lib
  CATKIN_DEPENDS 
  actionlib
  device_driver_base
  robot_base_msgs
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  dynamic_reconfigure
  message_runtime
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
add_dependencies(roboteq_manager_lib ${PROJECT_NAME}_gencfg)
add_dependencies(roboteq_manager_lib ${PROJECT_NAME}_generate_messages_cpp)


add_executable(roboteq_manager src/roboteq_manager.cpp)
target_link_libraries(roboteq_manager ${PROJECT_NAME}RoboteqLib ${catkin_LIBRARIES})

add_executable(drive_motors src/util/drive_motors.cpp)
target_link_libraries(drive_motors ${PROJECT_NAME}RoboteqLib ${catkin_LIBRARIES})

add_executable(simple_twist_client src/util/simple_twist_client.cpp)
target_link_libraries(simple_twist_client roboteq_manager_lib ${catkin_LIBRARIES})
