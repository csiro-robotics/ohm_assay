# Copyright (c) 2021
# Commonwealth Scientific and Industrial Research Organisation (CSIRO)
# ABN 41 687 119 230
#
# Author: Kazys Stepanas
# This is a very rought CMake script which I could label as "badly written" to get a job done.

cmake_minimum_required(VERSION 3.10)
project(voxbloxpop)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

find_package(ohm CONFIG REQUIRED)
find_package(voxblox CONFIG REQUIRED)

function(add_voxbloxpop TARGET_NAME)
  add_executable(${TARGET_NAME} ${ARGN})

  set_target_properties(${TARGET_NAME} PROPERTIES FOLDER utils)
  if(MSVC)
    set_target_properties(${TARGET_NAME} PROPERTIES DEBUG_POSTFIX "_d")
  endif(MSVC)

  target_include_directories(${TARGET_NAME} SYSTEM
    PRIVATE
      "${voxblox_INCLUDE_DIRS}"
      "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>"
  )

  target_link_libraries(${TARGET_NAME} PRIVATE ohm::slamio voxbloxpop ${voxblox_LIBRARIES})
  target_compile_definitions(${TARGET_NAME} PUBLIC "-DOHMPOP_CPU")
  install(TARGETS ${TARGET_NAME} # EXPORT voxblox-targets
    DESTINATION bin)
endfunction(add_voxbloxpop)

add_subdirectory(voxbloxpop)
add_subdirectory(voxbloxpopoccupancy)
add_subdirectory(voxbloxpoptsdf)

include(CMakePackageConfigHelpers)

configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Config.in.cmake
  "${CMAKE_CURRENT_BINARY_DIR}/voxbloxpop-config.cmake"
  INSTALL_DESTINATION "lib/cmake/${CMAKE_PROJECT_NAME}"
  NO_SET_AND_CHECK_MACRO
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/voxbloxpop-config-version.cmake"
  VERSION "${Tutorial_VERSION_MAJOR}.${Tutorial_VERSION_MINOR}"
  COMPATIBILITY AnyNewerVersion
)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/voxbloxpop-config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/voxbloxpop-config-version.cmake
  DESTINATION lib/cmake/${CMAKE_PROJECT_NAME}
)
export(EXPORT voxbloxpop-targets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/voxbloxpop-targets.cmake"
  NAMESPACE voxbloxpop::
)

install(EXPORT voxbloxpop-targets
  FILE voxbloxpop-targets.cmake
  NAMESPACE voxbloxpop::
  DESTINATION lib/cmake/${CMAKE_PROJECT_NAME}
)
