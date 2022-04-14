# Copyright (c) 2021
# Commonwealth Scientific and Industrial Research Organisation (CSIRO)
# ABN 41 687 119 230
#
# Author: Kazys Stepanas
# This is a very rought CMake script which I could label as "badly written" to get a job done.

cmake_minimum_required(VERSION 3.10)
project(octomappop_ros)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Find external depedencies.
# Note: this project fails to build if find_package(ohm) comes first. The new CUDA configuration is suspect.
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  ohmdataros
  roscpp
  sensor_msgs
  tf2_ros
  visualization_msgs
)

find_package(ohm CONFIG REQUIRED)
find_package(octomap CONFIG REQUIRED)
find_package(octomappop CONFIG REQUIRED)

# The catkin_package macro generates cmake config files for your package
# Declare things to be passed to dependent projects
# INCLUDE_DIRS: uncomment this if your package contains header files
# LIBRARIES: libraries you create in this project that dependent projects also need
# CATKIN_DEPENDS: catkin_packages dependent projects also need
# DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  # Use the directory up from this one to support "#include <package/header.h>"
  # INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/..
#  LIBRARIES cat
#  CATKIN_DEPENDS blah
#  DEPENDS system_lib
)

set(SOURCES
  octopop_ros.cpp
  OctopopRos.h
)

add_executable(octopop_ros ${SOURCES})

if(MSVC)
  set_target_properties(octopop_ros PROPERTIES DEBUG_POSTFIX "_d")
endif(MSVC)

target_include_directories(octopop_ros SYSTEM
  PRIVATE
    "${OCTOMAP_INCLUDE_DIRS}"
    "${catkin_INCLUDE_DIRS}"
)

target_link_libraries(octopop_ros
  PUBLIC
  ohm::ohmapp
  octomappop::octomappop
  ${OCTOMAP_LIBRARIES}
  ${catkin_LIBRARIES}
)
target_compile_definitions(octopop_ros PUBLIC "-DOHMPOP_CPU")

# Setup installation.
# Binary installation.
install(TARGETS octopop_ros
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  INCLUDES DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})
