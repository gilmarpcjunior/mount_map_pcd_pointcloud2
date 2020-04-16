cmake_minimum_required(VERSION 2.8.3)
project(mount_map_txt_pointcloud2)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(PCL REQUIRED QUIET)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mount_map_txt_pcd
#  CATKIN_DEPENDS geometry_msgs pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(converter_txt_to_pointclou2 src/converter_txt_to_pointclou2.cpp)
target_link_libraries(converter_txt_to_pointclou2 ${catkin_LIBRARIES} ${PCL_LIBRARIES})
