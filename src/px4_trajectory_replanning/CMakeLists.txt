cmake_minimum_required(VERSION 2.8.3)
project(px4_trajectory_replanning)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
add_definitions(-std=c++11 -g -O3)


find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(SuiteSparse REQUIRED)
find_package(Boost COMPONENTS thread chrono REQUIRED)

## Compile as C++11, supported in ROS Kinetic and newer

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${CHOLMOD_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)

catkin_simple()

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

cs_add_executable(px4_replanning_rrt_node src/px4_replanning_rrt.cpp)
cs_add_executable(px4_mavros_controller_node src/px4_mavros_controller.cpp)
cs_add_executable(px4_bspline_position_controller_node src/bspline_position_controller_node.cpp)

target_link_libraries(px4_replanning_rrt_node ${CHOLMOD_LIBRARY} ${Eigen3_LIBS} nlopt Boost::thread)


## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(px4_replanning_rrt_node ${CHOLMOD_LIBRARY} ${Eigen3_LIBS} nlopt Boost::thread Boost::chrono)

cs_install()
cs_export()
