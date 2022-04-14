cmake_minimum_required(VERSION 3.10)
project(octomappop)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

find_package(ohm CONFIG REQUIRED)
find_package(octomap CONFIG REQUIRED)

add_subdirectory(octomappop)
add_subdirectory(octopop)

include(CMakePackageConfigHelpers)

configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Config.in.cmake
  "${CMAKE_CURRENT_BINARY_DIR}/octomappop-config.cmake"
  INSTALL_DESTINATION "lib/cmake/${CMAKE_PROJECT_NAME}"
  NO_SET_AND_CHECK_MACRO
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/octomappop-config-version.cmake"
  VERSION "${Tutorial_VERSION_MAJOR}.${Tutorial_VERSION_MINOR}"
  COMPATIBILITY AnyNewerVersion
)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/octomappop-config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/octomappop-config-version.cmake
  DESTINATION lib/cmake/${CMAKE_PROJECT_NAME}
)
export(EXPORT octomappop-targets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/octomappop-targets.cmake"
  NAMESPACE octomappop::
)

install(EXPORT octomappop-targets
  FILE octomappop-targets.cmake
  NAMESPACE octomappop::
  DESTINATION lib/cmake/${CMAKE_PROJECT_NAME}
)
