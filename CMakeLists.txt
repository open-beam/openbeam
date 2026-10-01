# +---------------------------------------------------------------------------+
# |              OpenBeam - C++ Finite Element Analysis library               |
# |                                                                           |
# |   Copyright (C) 2010-2026  Jose Luis Blanco Claraco                       |
# |                                                                           |
# | OpenBeam is free software: you can redistribute it and/or modify          |
# |     it under the terms of the GNU General Public License as published by  |
# |     the Free Software Foundation, either version 3 of the License, or     |
# |     (at your option) any later version.                                   |
# +---------------------------------------------------------------------------+

cmake_minimum_required(VERSION 3.16)

# Version number, from the C++ header (single source of truth):
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/libopenbeam/include/openbeam/openbeam_version.h" _ob_version_h)
foreach(_part MAJOR MINOR PATCH)
  string(REGEX MATCH "OPENBEAM_${_part}_VERSION[ ]+([0-9]+)" _ "${_ob_version_h}")
  set(_ob_${_part} ${CMAKE_MATCH_1})
endforeach()

project(openbeam VERSION ${_ob_MAJOR}.${_ob_MINOR}.${_ob_PATCH} LANGUAGES CXX)
message(STATUS "OpenBeam version: ${PROJECT_VERSION}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# --------------------------
# Build options
# --------------------------
option(OPENBEAM_BUILD_APPS "Build command line applications" ON)
option(OPENBEAM_BUILD_EXAMPLES "Build C++ examples" ON)
include(CTest)  # Defines BUILD_TESTING (default ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wshadow)
endif()

# --------------------------
# Dependencies
# --------------------------
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(mrpt_containers REQUIRED)
find_package(mrpt_core REQUIRED)
find_package(mrpt_expr REQUIRED)
find_package(mrpt_math REQUIRED)
find_package(mrpt_system REQUIRED)
find_package(mrpt_viz REQUIRED)

# Optional: cairomm, for SVG rendering
set(OPENBEAM_HAS_CAIRO 0)
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(CAIROMM QUIET IMPORTED_TARGET cairomm-svg-1.0)
  if(CAIROMM_FOUND)
    set(OPENBEAM_HAS_CAIRO 1)
    message(STATUS "cairomm found (${CAIROMM_VERSION}): SVG rendering enabled")
  endif()
endif()
if(NOT OPENBEAM_HAS_CAIRO)
  message(STATUS "cairomm not found: SVG rendering disabled (optional)")
endif()

# --------------------------
# Targets
# --------------------------
add_subdirectory(libopenbeam)

if(OPENBEAM_BUILD_APPS)
  add_subdirectory(apps)
endif()

if(OPENBEAM_BUILD_EXAMPLES)
  add_subdirectory(examples-code)
endif()

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

# --------------------------
# Install and CMake package config: find_package(openbeam)
# --------------------------
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(
  EXPORT openbeam-targets
  NAMESPACE openbeam::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/openbeam
)
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/openbeam-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/openbeam-config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/openbeam
)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/openbeam-config-version.cmake
  COMPATIBILITY SameMinorVersion
)
install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/openbeam-config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/openbeam-config-version.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/openbeam
)
install(FILES package.xml DESTINATION share/openbeam)
install(DIRECTORY examples-structures/ DESTINATION share/openbeam/examples-structures)
install(FILES data/beam_profiles.txt DESTINATION share/openbeam)
