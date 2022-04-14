# Copyright (c) 2021
# Commonwealth Scientific and Industrial Research Organisation (CSIRO)
# ABN 41 687 119 230
#
# Author: Kazys Stepanas
# This is a very rought CMake script which I could label as "badly written" to get a job done.

set(SOURCES
  OctomapPop.cpp
  OctomapPop.h
)

set(PUBLIC_HEADERS
  OctomapPop.h
)

add_library(octomappop STATIC ${SOURCES})

target_include_directories(octomappop PUBLIC ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(octomappop PUBLIC ohm::ohmapp ohm::ohmutil ${OCTOMAP_LIBRARIES})

install(TARGETS octomappop EXPORT octomappop-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/octomappop
)

install(FILES ${PUBLIC_HEADERS} DESTINATION include/octomappop COMPONENT Devel)

source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})
