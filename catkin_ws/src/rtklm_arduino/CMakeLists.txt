cmake_minimum_required(VERSION 3.0.2)
project(rtklm_arduino)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  serial
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES rtklm_arduino
#   CATKIN_DEPENDS controller_manager hardware_interface serial
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(rtklm_arduino
  src/rtklm_arduino_node.cpp
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(rtklm_arduino_node
  src/rtklm_arduino_node.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(rtklm_arduino_node ${PROJECT_NAME})

## Specify libraries to link a library or executable target against
target_link_libraries(rtklm_arduino_node
  ${PROJECT_NAME} ${catkin_LIBRARIES}
)