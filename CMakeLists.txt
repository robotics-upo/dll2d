cmake_minimum_required(VERSION 3.0.2)
project(dll2d)

## Compile as C++14
add_compile_options(-O3 )
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
  geometry_msgs
  nav_msgs
)

# Ceres solver
find_package(Ceres REQUIRED)

# ANN Library
find_library(ANN_LIB NAMES ann)
IF(${ANN_LIB} STREQUAL "ANN_LIB-NOTFOUND")
  message(FATAL_ERROR "ANN library not found!\nTry 'sudo apt-get install libann-dev'\n")
ELSE()
  message(STATUS "ANN = ${ANN_LIB}")
ENDIF()


###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp sensor_msgs tf
   DEPENDS ann
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/dll2d.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(dlo2d_node src/dlo2d_node.cpp)
add_executable(dll2d_node src/dll2d_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(dlo2d_node ${dll2d_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(dll2d_node ${dll2d_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(dlo2d_node
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ann
)
target_link_libraries(dll2d_node
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ann
)
