cmake_minimum_required(VERSION 2.8.3)
project(local_planner)
set(CMAKE_CXX_STANDARD 11)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules/")

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    std_msgs
    pcl_ros
    pcl_conversions
    sensor_msgs
    cmake_modules
)

add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
  CATKIN_DEPENDS std_msgs
)

include_directories(
  "${PROJECT_SOURCE_DIR}/src/"
  "${PROJECT_SOURCE_DIR}/include/"
  ${catkin_INCLUDE_DIRS}
)

add_executable(local_planner src/local_planner_node.cpp)

target_link_libraries(local_planner
  ${catkin_LIBRARIES}
)