# Copyright (c) 2018
# Commonwealth Scientific and Industrial Research Organisation (CSIRO)
# ABN 41 687 119 230
#
# Author: Kazys Stepanas
cmake_minimum_required(VERSION 3.10)
project(ohmdataros)

set(PACKAGE_VERSION 0.0.0)
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/package.xml")
  file(READ package.xml PACKAGE_XML)
  string(REGEX REPLACE ".*<version>[\\t ]*(.*)[\\t ]*<\\/version>.*" "\\1" PACKAGE_VERSION "${PACKAGE_XML}")
endif(EXISTS "${CMAKE_CURRENT_LIST_DIR}/package.xml")
set(ohmdataros_VERSION "${PACKAGE_VERSION}")

# C++ standards setup.
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
# Ensure -fPIC is added.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT MSVC)
  add_compile_options(
    # "-pedantic"
    "-Wall"
    "-Wextra"
    "-Werror=return-type"
    "-Werror=vla"
    "-Wno-unused-function"
    "-Wno-missing-braces"
    "-Wno-unknown-pragmas"
    "-Wno-parentheses"
    "-Wno-ignored-attributes"
  )
endif(NOT MSVC)

# Find external depedencies.
# Note: this project fails to build if find_package(ohm) comes first. The new CUDA configuration is suspect.
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  tf2_ros
  visualization_msgs
)

find_package(ohm REQUIRED)

##################################
# catkin specific configuration ##
##################################
# The catkin_package macro generates cmake config files for your package
# Declare things to be passed to dependent projects
# INCLUDE_DIRS: uncomment this if your package contains header files
# LIBRARIES: libraries you create in this project that dependent projects also need
# CATKIN_DEPENDS: catkin_packages dependent projects also need
# DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  # Use the directory up from this one to support "#include <package/header.h>"
  INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/..
  LIBRARIES ohmdataros
  # CATKIN_DEPENDS
#  DEPENDS system_lib
)

##################################
# Build setup
##################################

# Configure the project config header.
set(GENERATED_HEADER_PREFIX "${CATKIN_DEVEL_PREFIX}/include")
configure_file(DataRosConfig.in.h "${GENERATED_HEADER_PREFIX}/ohmdataros/DataRosConfig.h")

# Project source files. There are two options here:
# 1. Populate the following varibles inline here:
#   - SOURCES for source files to be built and header files associated with built sources.
#   - PUBLIC_HEADERS for all headers to be marshalled as part of the public API.
#     May double up with SOURCES
# 2. Create a file alongside CMakeLists.txt called "sourcelist.cmake" and populate
#    the same varaibles in that file instead, then use "incldue(sourcelist.cmake)" here.
#include(sourcelist.cmake)
# For executables we don't need to concern outselves with PUBLIC_HEADERS as we can assume noone will link to the
# executable. Cases where linking to the executable is requried (e.g., plugins) are beyond the scope of this exercise.
set(SOURCES
  DataRosConfig.in.h
  MapPublish.cpp
  MapPublish.h
  PointCloud2Helper.cpp
  PointCloud2Helper.h
  RosDataSource.cpp
  RosDataSource.h
)

set(PUBLIC_HEADERS
  MapPublish.h
  PointCloud2Helper.h
  RosDataSource.h
)

# Build generated header list.
set(GENERATED_FILES
  "${GENERATED_HEADER_PREFIX}/ohmdataros/DataRosConfig.h"
)

list(APPEND PUBLIC_HEADERS ${GENERATED_FILES})

# Generate the executable.
add_library(ohmdataros ${SOURCES} ${GENERATED_FILES})

# Add include directories to the target
target_include_directories(ohmdataros
  PUBLIC
    # Include path for generated files during build.
    $<BUILD_INTERFACE:${GENERATED_HEADER_PREFIX}>
    # Add parent directory to support include pattern: #include <project_dir/header.h>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/..>
    $<INSTALL_INTERFACE:${CATKIN_PACKAGE_INCLUDE_DESTINATION}/..>
  )

# Add catkin include directories and system include directories.
# Always add ${catkin_INCLUDE_DIRS} with the SYSTEM argument
# These dependenties should be private as much as possible.
target_include_directories(ohmdataros SYSTEM
  PRIVATE
    "${catkin_INCLUDE_DIRS}"
  )

# Link dependencies.
# Properly defined targets will also have their include directories and those of dependencies added by this command.
target_link_libraries(ohmdataros ${catkin_LIBRARIES} ohm::ohmapp ohm::ohmutil)

source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})

# Setup installation.
# Binary installation.
install(TARGETS ohmdataros
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  INCLUDES DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install public header files.
install(FILES ${PUBLIC_HEADERS} DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
