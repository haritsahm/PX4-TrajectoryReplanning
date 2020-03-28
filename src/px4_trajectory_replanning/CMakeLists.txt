cmake_minimum_required(VERSION 2.8.3)
project(px4_trajectory_replanning)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
add_definitions(-std=c++11 -g -O3 -fPIC)

#set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
#SET(CMAKE_AUTOUIC ON)

set(Qt5_DIR "/opt/qt511/lib/cmake/Qt5")
set(yaml-cpp_DIR "/usr/local/lib/cmake/yaml-cpp")
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Core REQUIRED)


find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(SuiteSparse REQUIRED)
find_package(Boost COMPONENTS thread chrono REQUIRED)
find_package(yaml-cpp REQUIRED)

set(controller_SRCS
    src/ui/controller.cpp
    src/ui/qnode.cpp
    src/ui/px4_controller_node.cpp
)

set(controller_HDRS
    include/px4_trajectory_replanning/ui/controller.h
    include/px4_trajectory_replanning/ui/qnode.h
)

set(controller_UIS
    ui/offboardcontroller.ui
)

if(NOT EXISTS "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}")
  file(MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}")
endif()

qt5_wrap_cpp(controller_MOCS ${controller_HDRS})

# ensure generated header files are being created in the devel space
set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}")
qt5_wrap_ui(controller_UIS_H ${controller_UIS})
set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")

## Compile as C++11, supported in ROS Kinetic and newer

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  include/ui
  ${CHOLMOD_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

catkin_simple()

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

cs_add_executable(px4_replanning_rrt_node src/px4_replanning_rrt.cpp)
cs_add_executable(px4_mavros_controller_node src/px4_mavros_controller.cpp)
cs_add_executable(px4_position_controller_node src/px4_position_controller_node.cpp)
cs_add_executable(px4_controller_node ${controller_SRCS} ${controller_MOCS} ${controller_UIS_H})
## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(px4_replanning_rrt_node ${CHOLMOD_LIBRARY} ${Eigen3_LIBS} nlopt Boost::thread Boost::chrono ${YAML_CPP_LIBRARIES})
target_link_libraries(px4_position_controller_node ${CHOLMOD_LIBRARY} Eigen3::Eigen nlopt Boost::thread Boost::chrono ${YAML_CPP_LIBRARIES})
target_link_libraries(px4_controller_node Boost::thread Boost::chrono Qt5::Widgets Qt5::Gui Qt5::Core ${YAML_CPP_LIBRARIES})

cs_install()
cs_export()
