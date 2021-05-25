cmake_minimum_required(VERSION 3.0.2)
project(controller)

add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  moveit_kinematics 
  moveit_visual_tools
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS 
 LIBRARIES 
  controller
 CATKIN_DEPENDS 
  rospy
  std_msgs
  moveit_kinematics 
  moveit_visual_tools
 DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include/controller
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/controller_node.cpp src/RvizController.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
