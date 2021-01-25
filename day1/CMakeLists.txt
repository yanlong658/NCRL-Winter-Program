cmake_minimum_required(VERSION 3.0.2)
project(day1)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  visualization_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES day1
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
DEPENDS EIGEN3
)

#inlude myself library
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(Integration 
src/Integration.cpp
src/ncrl.cpp)
target_link_libraries(Integration ${catkin_LIBRARIES} )

