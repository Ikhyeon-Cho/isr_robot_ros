cmake_minimum_required(VERSION 3.0.2)
project(isr_m4_tools)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
)

catkin_package(
  INCLUDE_DIRS 
  CATKIN_DEPENDS 
    roscpp
    rospy
)

###########
## Build ##
###########
set(CMAKE_BUILD_TYPE Release)

include_directories(
  ${catkin_INCLUDE_DIRS}
)