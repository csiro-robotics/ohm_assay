# Copyright (c) 2018
# Commonwealth Scientific and Industrial Research Organisation (CSIRO)
# ABN 41 687 119 230
#
# Author: Kazys Stepanas
cmake_minimum_required(VERSION 3.10)
project(ohmpop_ros)

set(PACKAGE_VERSION 0.0.0)
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/package.xml")
  file(READ package.xml PACKAGE_XML)
  string(REGEX REPLACE ".*<version>[\\t ]*(.*)[\\t ]*<\\/version>.*" "\\1" PACKAGE_VERSION "${PACKAGE_XML}")
endif(EXISTS "${CMAKE_CURRENT_LIST_DIR}/package.xml")
set(ohmpopros_VERSION "${PACKAGE_VERSION}")

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
  ohmdataros
  roscpp
  sensor_msgs
  tf2_ros
)

find_package(ohm)

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
#  LIBRARIES cat
#  CATKIN_DEPENDS blah
#  DEPENDS system_lib
)

function(add_ohmpop_ros_node TARGET_NAME )
  cmake_parse_arguments(ARG "" "" "LINK_LIBRARIES;SOURCES" ${ARGN})

  # Generate the executable.
  add_executable(${TARGET_NAME} ${ARG_SOURCES})

  # CMake does not automatically propagate CMAKE_DEBUG_POSTFIX to executables. We do so to avoid confusing link issues
  # which can would when building release and debug exectuables to the same path.
  set_target_properties(${TARGET_NAME} PROPERTIES DEBUG_POSTFIX "${CMAKE_DEBUG_POSTFIX}")

  # Add include directories to the target
  # For executables, all includes can be private (see set(SOURCES ...) above).
  target_include_directories(${TARGET_NAME}
    PRIVATE
      # Include path for generated files during build.
      $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
      $<BUILD_INTERFACE:${CATKIN_DEVEL_PREFIX}/include>
      # Add parent directory to support include pattern: #include <project_dir/header.h>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/..>
    )

  # Add catkin include directories and system include directories.
  # Always add ${catkin_INCLUDE_DIRS} with the SYSTEM argument
  # These dependenties should be private as much as possible.
  target_include_directories(${TARGET_NAME} SYSTEM
    PRIVATE
      "${catkin_INCLUDE_DIRS}"
    )

  # Link dependencies.
  # Properly defined targets will also have their include directories and those of dependencies added by this command.
  target_link_libraries(${TARGET_NAME}
    ${catkin_LIBRARIES}
    ohm::ohmapp
    ${ARG_LINK_LIBRARIES}
  )
endfunction(add_ohmpop_ros_node)

set(SOURCES
  RosArgMap.h
  RosMap.h
  RosOhmCloud.h
  RosPopMain.h
)

# Generate the executable.
set(TARGET_LIST ohmpopcpu_ros)
add_ohmpop_ros_node(ohmpopcpu_ros LINK_LIBRARIES ohm::ohm SOURCES ohmpopmaincpu_ros.cpp ${SOURCES})
if(TARGET ohm::ohmcuda)
  list(APPEND TARGET_LIST ohmpopcuda_ros)
  add_ohmpop_ros_node(ohmpopcuda_ros LINK_LIBRARIES ohm::ohmcuda ohm::ohmappcuda ohm::ohmheightmap
    SOURCES
      ohmpopmaincuda_ros.cpp ${SOURCES}
    )
endif(TARGET ohm::ohmcuda)
if(TARGET ohm::ohmocl)
  list(APPEND TARGET_LIST ohmpopocl_ros)
  add_ohmpop_ros_node(ohmpopocl_ros LINK_LIBRARIES ohm::ohmocl ohm::ohmappocl ohm::ohmheightmap
    SOURCES
      ohmpopmainocl_ros.cpp ${SOURCES}
    )
endif(TARGET ohm::ohmocl)

# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})

# Setup installation.
# Binary installation.
install(TARGETS ${TARGET_LIST}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  INCLUDES DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
