cmake_minimum_required(VERSION 3.0.2)
project(isr_m3_description)
add_compile_options(-std=c++17)

catkin_package(
  INCLUDE_DIRS 
)

###########
## Build ##
###########
set(CMAKE_BUILD_TYPE Release)

include_directories(
  ${catkin_INCLUDE_DIRS}
)