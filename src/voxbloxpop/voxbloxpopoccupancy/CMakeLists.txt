# Copyright (c) 2021
# Commonwealth Scientific and Industrial Research Organisation (CSIRO)
# ABN 41 687 119 230
#
# Author: Kazys Stepanas
# This is a very rought CMake script which I could label as "badly written" to get a job done.

find_package(ohm CONFIG REQUIRED)
find_package(voxblox CONFIG REQUIRED)

set(SOURCES
  voxbloxpopoccupancy.cpp
)

add_voxbloxpop(voxbloxpopoccupancy ${SOURCES} voxbloxpopoccupancy.cpp)

source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})
