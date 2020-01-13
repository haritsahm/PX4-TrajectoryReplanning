cmake_minimum_required(VERSION 2.8.3)
project(local_planner)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin_simple REQUIRED)
find_package(octomap REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system chrono)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${OCTOMAP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

catkin_simple()

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
cs_add_executable(${PROJECT_NAME}_node src/local_planner_node.cpp)
#  src/trajectory_generator.cpp
#  src/ring_buffer.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
   ${OCTOMAP_LIBRARIES}
   ${Eigen3_LIBRARIES}
)

cs_install()
cs_export()
