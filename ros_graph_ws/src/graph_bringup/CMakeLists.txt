cmake_minimum_required(VERSION 2.8.3)
project(graph_bringup)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  
)


catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
