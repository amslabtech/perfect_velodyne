cmake_minimum_required(VERSION 2.8.3)
project(perfect_velodyne)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  velodyne_msgs
  visualization_msgs
  # OpenMP
)

find_package(cmake_modules)
find_package(PCL 1.8.0 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Eigen3 REQUIRED COMPONENTS system)
include_directories(${Eigen_INCLUDE_DIRS})

find_package(OpenMP)
if (OPENMP_FOUND)
	set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS geometry_msgs pcl_ros roscpp sensor_msgs std_msgs velodyne_msgs visualization_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME} SHARED
   src/perfect_velodyne.cpp
)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Eigen_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_executable(perfect_velodyne_node src/perfect_velodyne_node.cpp)
target_link_libraries(perfect_velodyne_node ${PROJECT_NAME})
