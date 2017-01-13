cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules/")


project(security_area)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  auv_msgs
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

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS})

# local_planner node
add_executable(security_area
  src/security_area_node.cpp)
target_link_libraries(security_area
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES})
add_dependencies(security_area ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


