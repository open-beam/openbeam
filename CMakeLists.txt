#  +---------------------------------------------------------------------------+
#  |              OpenBeam - C++ Finite Element Analysis library               |
#  |                                                                           |
#  |   Copyright (C) 2010-2011  Jose Luis Blanco Claraco                       |
#  |                              University of Malaga                         |
#  |                                                                           |
#  | OpenBeam is free software: you can redistribute it and/or modify          |
#  |     it under the terms of the GNU General Public License as published by  |
#  |     the Free Software Foundation, either version 3 of the License, or     |
#  |     (at your option) any later version.                                   |
#  |                                                                           |
#  | OpenBeam is distributed in the hope that it will be useful,               |
#  |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
#  |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
#  |     GNU General Public License for more details.                          |
#  |                                                                           |
#  |     You should have received a copy of the GNU General Public License     |
#  |     along with OpenBeam.  If not, see <http://www.gnu.org/licenses/>.     |
#  |                                                                           |
#  +---------------------------------------------------------------------------+ 

PROJECT(OpenBeam)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

#  Loads the current version number:
FILE(READ "${CMAKE_CURRENT_SOURCE_DIR}/include/openbeam/openbeam_version.h" STR_OPENBEAM_VERSION_FILE)
STRING(REGEX MATCH "OPENBEAM_MAJOR_VERSION[ ]+[0-9]+" CMAKE_OPENBEAM_VERSION_NUMBER_MAJOR "${STR_OPENBEAM_VERSION_FILE}")
STRING(REGEX MATCH "[0-9]+" CMAKE_OPENBEAM_VERSION_NUMBER_MAJOR "${CMAKE_OPENBEAM_VERSION_NUMBER_MAJOR}")
STRING(REGEX MATCH "OPENBEAM_MINOR_VERSION[ ]+[0-9]+" CMAKE_OPENBEAM_VERSION_NUMBER_MINOR "${STR_OPENBEAM_VERSION_FILE}")
STRING(REGEX MATCH "[0-9]+" CMAKE_OPENBEAM_VERSION_NUMBER_MINOR "${CMAKE_OPENBEAM_VERSION_NUMBER_MINOR}")
STRING(REGEX MATCH "OPENBEAM_PATCH_VERSION[ ]+[0-9]+" CMAKE_OPENBEAM_VERSION_NUMBER_PATCH "${STR_OPENBEAM_VERSION_FILE}")
STRING(REGEX MATCH "[0-9]+" CMAKE_OPENBEAM_VERSION_NUMBER_PATCH "${CMAKE_OPENBEAM_VERSION_NUMBER_PATCH}")

SET(OPENBEAM_VERSION "${CMAKE_OPENBEAM_VERSION_NUMBER_MAJOR}.${CMAKE_OPENBEAM_VERSION_NUMBER_MINOR}.${CMAKE_OPENBEAM_VERSION_NUMBER_PATCH}")
MESSAGE(STATUS "Openbeam version: ${OPENBEAM_VERSION}")

# --------------------------
# Build options
# --------------------------
IF (UNIX)
	SET(DEFAULT_SHARED_LIBS ON)
ELSE(UNIX)
	SET(DEFAULT_SHARED_LIBS OFF)
ENDIF(UNIX)

SET(BUILD_SHARED_LIBS ${DEFAULT_SHARED_LIBS} CACHE BOOL "Build shared libraries (.dll/.so) instead of static ones (.lib/.a)")
SET(BUILD_EXAMPLES ON CACHE BOOL "Build examples?")

# Save all libs and executables in the same place
SET( LIBRARY_OUTPUT_PATH ${${PROJECT_NAME}_BINARY_DIR}/lib CACHE PATH "Output directory for libraries" )
SET( EXECUTABLE_OUTPUT_PATH ${${PROJECT_NAME}_BINARY_DIR}/bin CACHE PATH "Output directory for applications" )


# --------------------------
# Dependencies
# --------------------------
SET(OPENBEAM_LINK_LIBS "")

SET(EIGEN3_INCLUDE_DIR "${OpenBeam_SOURCE_DIR}/3rdparty/eigen3/" CACHE PATH "Path to Eigen3 include directory")
MARK_AS_ADVANCED(EIGEN3_INCLUDE_DIR)

INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

# Cairo
# ----------------------------
SET(OPENBEAM_HAS_CAIRO 0)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${OpenBeam_SOURCE_DIR})
FIND_PACKAGE(Cairo)

# pangommcairo

# Save vars (for some reason they are overwriten by another pkg_check_modules() call):
SET(MY_CAIRO_VERSION  ${CAIRO_VERSION})
SET(MY_CAIRO_LIBRARIES  ${CAIRO_LIBRARIES})
SET(MY_CAIRO_INCLUDE_DIR  ${CAIRO_INCLUDE_DIR})
SET(MY_CAIRO_CFLAGS  ${CAIRO_CFLAGS})

# "cairomm" svg lib
IF(NOT WIN32)
	include(FindPkgConfig)
	pkg_check_modules(CAIROMM cairomm-svg-1.0)
ENDIF(NOT WIN32)
	

IF(CAIRO_FOUND AND CAIROMM_FOUND)
        MESSAGE(STATUS "cairo found: ${MY_CAIRO_VERSION}")
        MESSAGE(STATUS "cairomm found: ${CAIROMM_VERSION}")

        IF($ENV{VERBOSE})
                MESSAGE(STATUS "   CAIRO_LIBRARIES = ${MY_CAIRO_LIBRARIES}")
                MESSAGE(STATUS "   CAIRO_INCLUDE_DIR = ${MY_CAIRO_INCLUDE_DIR}")
                MESSAGE(STATUS "   CAIRO_CFLAGS = ${MY_CAIRO_CFLAGS}")

                MESSAGE(STATUS "   CAIROMM_LIBRARIES = ${CAIROMM_LIBRARIES}")
                MESSAGE(STATUS "   CAIROMM_INCLUDE_DIR = ${CAIROMM_INCLUDE_DIR}")
                MESSAGE(STATUS "   CAIROMM_CFLAGS = ${CAIROMM_CFLAGS}")
        ENDIF($ENV{VERBOSE})
        
        INCLUDE_DIRECTORIES(${MY_CAIRO_INCLUDE_DIR})
        INCLUDE_DIRECTORIES(${CAIROMM_INCLUDE_DIR})

	ADD_DEFINITIONS(${MY_CAIRO_CFLAGS})
	ADD_DEFINITIONS(${CAIROMM_CFLAGS})
        
        SET(OPENBEAM_LINK_LIBS ${OPENBEAM_LINK_LIBS} ${MY_CAIRO_LIBRARIES} ${CAIROMM_LIBRARIES})

	SET(OPENBEAM_HAS_CAIRO 1)
ELSE(CAIRO_FOUND AND CAIROMM_FOUND)
        MESSAGE(STATUS " Cairo and/or cairomm were *not* found: These packages are optional, but SVG rendering will be not available.")
ENDIF(CAIRO_FOUND AND CAIROMM_FOUND)

ADD_DEFINITIONS(-DOPENBEAM_HAS_CAIRO=${OPENBEAM_HAS_CAIRO})

# Qt5
# --------------------------------
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

if(POLICY CMP0020)
  cmake_policy(SET CMP0020 NEW)
endif()

set(OPENBEAM_HAS_QT5Svg 0)

# Find includes in corresponding build directories
find_package(Qt5Svg QUIET)  # Find the QtWidgets library
IF (NOT Qt5Svg_FOUND)
	MESSAGE(STATUS "*WARNING* Qt5Svg not found. Cannot build this target. \n  Add the installation prefix of Qt5Widgets to CMAKE_PREFIX_PATH \n  or set Qt5Svg_DIR to a directory containing one of the above files, typically: 'C:\\Qt\\5.X\\msvcXXXX_64\\lib\\cmake\\Qt5Svg'")
ELSE()
	INCLUDE_DIRECTORIES(${Qt5Svg_INCLUDE_DIRS})
	add_definitions(${Qt5Svg_DEFINITIONS})
	set(OPENBEAM_HAS_QT5Svg 1)
ENDIF()
ADD_DEFINITIONS(-DOPENBEAM_HAS_QT5Svg=${OPENBEAM_HAS_QT5Svg})

# --------------------------
# Global compiler flags
# --------------------------
INCLUDE_DIRECTORIES("${${PROJECT_NAME}_SOURCE_DIR}/include/")
INCLUDE_DIRECTORIES("${${PROJECT_NAME}_SOURCE_DIR}/internationalization/")

# Force usage of UNICODE projects, which is not the default in MSVC:
IF(MSVC)
	add_definitions(-DUNICODE -D_UNICODE -D_CRT_NONSTDC_NO_WARNINGS)
ENDIF(MSVC)

# Set optimized building:
IF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mtune=native")
ENDIF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")


# --------------------------
#       The beef
# --------------------------
add_subdirectory(src)
add_subdirectory(apps)

IF(BUILD_EXAMPLES)
	add_subdirectory(examples-code)
ENDIF(BUILD_EXAMPLES)


# -------------------------------------------------------------------------------------------
#  ${BIN_DIR}/OpenBeamConfig.cmake           -> For use *without* "make install"
# -------------------------------------------------------------------------------------------

# Set CMAKE_INCLUDE_DIRS_CONFIGCMAKE to the list of include directories:
set(CMAKE_INCLUDE_DIRS_CONFIGCMAKE "\"${CMAKE_CURRENT_SOURCE_DIR}/include\" \"${EIGEN3_INCLUDE_DIR}\"")
set(CMAKE_LIB_DIRS_CONFIGCMAKE "${LIBRARY_OUTPUT_PATH}")

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/OpenBeamConfig.cmake.in" "${CMAKE_BINARY_DIR}/OpenBeamConfig.cmake" IMMEDIATE @ONLY)


