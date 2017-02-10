cmake_minimum_required(VERSION 2.8.3)
project(security_area)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  pcl_ros
  std_msgs
  std_srvs
  tf
  cmake_modules
)

####################################################
# Dependencies:
find_package(PkgConfig)

# Dependencies - PCL:
find_package(PCL REQUIRED)

add_definitions(-std=c++11)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

# security_area node
add_executable(security_area src/security_area_node.cpp)
target_link_libraries(security_area ${PCL_LIBRARIES} ${catkin_LIBRARIES})
