# Copyright (c) 2021
# Commonwealth Scientific and Industrial Research Organisation (CSIRO)
# ABN 41 687 119 230
#
# Author: Kazys Stepanas
# This is a very rought CMake script which I could label as "badly written" to get a job done.

find_package(ohm CONFIG REQUIRED)
find_package(octomap CONFIG REQUIRED)

set(SOURCES
  octopop.cpp
)

add_executable(octopop ${SOURCES})

set_target_properties(octopop PROPERTIES FOLDER utils)
if(MSVC)
  set_target_properties(octopop PROPERTIES DEBUG_POSTFIX "_d")
endif(MSVC)

target_include_directories(octopop SYSTEM
  PRIVATE
    "${OCTOMAP_INCLUDE_DIRS}"
    "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>"
)

target_link_libraries(octopop PRIVATE ohm::slamio octomappop ${OCTOMAP_LIBRARIES})
target_compile_definitions(octopop PUBLIC "-DOHMPOP_CPU")
install(TARGETS octopop # EXPORT octomappop-targets
  DESTINATION bin)

source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})
