cmake_minimum_required(VERSION 2.8.3)
project(fms_routes)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  robotnik_msgs
  robotnik_fms_msgs
  geometry_msgs
  message_generation
  robot_local_control_msgs
  visualization_msgs
  diagnostic_updater
  diagnostic_msgs
  tf
  jsonization
)


find_package(PkgConfig REQUIRED tf)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs robotnik_msgs geometry_msgs robot_local_control_msgs
)

catkin_package(
   DEPENDS XERCES)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable
add_executable(fms_routes_node
                        include/fms_routes/timer.h
                        include/fms_routes/Dijkstra.h
                        include/fms_routes/Graph.h
                        src/fms_graph_node.cpp
                        src/Dijkstra.cc src/Graph.cc)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(fms_routes_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}  robot_local_control_msgs_generate_msgs_cpp robotnik_msgs_generate_msgs_cpp robotnik_fms_msgs_generate_msgs_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(fms_routes_node
   ${catkin_LIBRARIES}
   xerces-c
)

install(TARGETS fms_routes_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)
install(DIRECTORY launch config
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
