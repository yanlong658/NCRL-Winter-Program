cmake_minimum_required(VERSION 3.0.2)
project(day3)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES day3
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
DEPENDS EIGEN3 PCL
)

#inlude myself library
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

add_executable(icpmatch src/icpmatch.cpp)
target_link_libraries(icpmatch ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})
