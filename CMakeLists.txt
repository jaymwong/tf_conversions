cmake_minimum_required(VERSION 2.8.3)
project(transform_conversions)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
	COMPONENTS
		geometry_msgs
)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

catkin_python_setup()

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES transform_conversions
	CATKIN_DEPENDS roscpp
	DEPENDS EIGEN3
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
	include
)

add_library(transform_conversions
	src/conversions.cpp
)

add_dependencies(transform_conversions
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(transform_conversions
  ${catkin_LIBRARIES}
)
