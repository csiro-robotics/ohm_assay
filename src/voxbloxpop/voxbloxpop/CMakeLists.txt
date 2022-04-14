# Copyright (c) 2021
# Commonwealth Scientific and Industrial Research Organisation (CSIRO)
# ABN 41 687 119 230
#
# Author: Kazys Stepanas
# This is a very rought CMake script which I could label as "badly written" to get a job done.

set(SOURCES
  VoxbloxPopMap.cpp
  VoxbloxPopMap.h
)

set(PUBLIC_HEADERS
  VoxbloxPopMap.h
)

add_library(voxbloxpop STATIC ${SOURCES})

target_include_directories(voxbloxpop PUBLIC ${voxblox_INCLUDE_DIRS})
target_link_libraries(voxbloxpop PUBLIC ohm::ohmapp ohm::ohmutil ${voxblox_LIBRARIES})

install(TARGETS voxbloxpop EXPORT voxbloxpop-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/voxbloxpop
)

install(FILES ${PUBLIC_HEADERS} DESTINATION include/voxbloxpop COMPONENT Devel)

source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})
