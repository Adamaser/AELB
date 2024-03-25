cmake_minimum_required(VERSION 3.0.2)
project(marker_visual)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES marker_visual
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories("/usr/include/eigen3")

include_directories(
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(marker_visual 
               src/marker_visual.cpp)

target_link_libraries(marker_visual ${catkin_LIBRARIES})
