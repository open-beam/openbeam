#  +---------------------------------------------------------------------------+
#  |              OpenBeam - C++ Finite Element Analysis library               |
#  |                                                                           |
#  | Copyright (C) 2010-2011  Jose Luis Blanco Claraco, University of Malaga   |
#  |               2012-2022  Jose Luis Blanco Claraco, University of Almeria  |
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

project(OpenBeam)
cmake_minimum_required(VERSION 3.1)
set (CMAKE_CXX_STANDARD 14)

if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)
endif()

#  Loads the current version number:
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/libopenbeam/include/openbeam/openbeam_version.h" STR_OPENBEAM_VERSION_FILE)
string(REGEX MATCH "OPENBEAM_MAJOR_VERSION[ ]+[0-9]+" CMAKE_OPENBEAM_VERSION_NUMBER_MAJOR "${STR_OPENBEAM_VERSION_FILE}")
string(REGEX MATCH "[0-9]+" CMAKE_OPENBEAM_VERSION_NUMBER_MAJOR "${CMAKE_OPENBEAM_VERSION_NUMBER_MAJOR}")
string(REGEX MATCH "OPENBEAM_MINOR_VERSION[ ]+[0-9]+" CMAKE_OPENBEAM_VERSION_NUMBER_MINOR "${STR_OPENBEAM_VERSION_FILE}")
string(REGEX MATCH "[0-9]+" CMAKE_OPENBEAM_VERSION_NUMBER_MINOR "${CMAKE_OPENBEAM_VERSION_NUMBER_MINOR}")
string(REGEX MATCH "OPENBEAM_PATCH_VERSION[ ]+[0-9]+" CMAKE_OPENBEAM_VERSION_NUMBER_PATCH "${STR_OPENBEAM_VERSION_FILE}")
string(REGEX MATCH "[0-9]+" CMAKE_OPENBEAM_VERSION_NUMBER_PATCH "${CMAKE_OPENBEAM_VERSION_NUMBER_PATCH}")

set(OPENBEAM_VERSION "${CMAKE_OPENBEAM_VERSION_NUMBER_MAJOR}.${CMAKE_OPENBEAM_VERSION_NUMBER_MINOR}.${CMAKE_OPENBEAM_VERSION_NUMBER_PATCH}")
message(STATUS "Openbeam version: ${OPENBEAM_VERSION}")

# --------------------------
# Build options
# --------------------------
if (UNIX AND NOT (CMAKE_SYSTEM_NAME STREQUAL "Emscripten"))
	set(DEFAULT_SHARED_LIBS ON)
else()
	set(DEFAULT_SHARED_LIBS OFF)
endif()

if (MSVC)
	add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()

set(BUILD_SHARED_LIBS ${DEFAULT_SHARED_LIBS} CACHE BOOL "Build shared libraries (.dll/.so) instead of static ones (.lib/.a)")
set(BUILD_EXAMPLES ON CACHE BOOL "Build examples?")

# Save all libs and executables in the same place
set(LIBRARY_OUTPUT_PATH ${${PROJECT_NAME}_BINARY_DIR}/lib CACHE PATH "Output directory for libraries" )
set(EXECUTABLE_OUTPUT_PATH ${${PROJECT_NAME}_BINARY_DIR}/bin CACHE PATH "Output directory for applications" )


# --------------------------
# Dependencies
# --------------------------
set(OPENBEAM_LINK_LIBS "")

# Cairo
# ----------------------------
set(OPENBEAM_HAS_CAIRO 0)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${OpenBeam_SOURCE_DIR})
find_package(Cairo)

# MRPT for opengl rendering:
find_package(mrpt-opengl REQUIRED)
find_package(mrpt-gui REQUIRED)

if (CMAKE_SYSTEM_NAME STREQUAL "Emscripten")
	list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)

	find_package(Threads)   # Threads::Threads
	find_package(Filesystem MODULE)

	add_library(mrpt_libfyaml STATIC IMPORTED GLOBAL)
	add_dependencies(mrpt_libfyaml mrpt_liblibfyaml)
	
	find_library(LIBFYAML_LIB fyaml)
	find_path(LIBFYAML_INCLUDES fyaml.h)
	set_target_properties(mrpt_libfyaml PROPERTIES IMPORTED_LOCATION ${LIBFYAML_LIB})
	set_target_properties(mrpt_libfyaml PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${LIBFYAML_INCLUDES})
	
	set(MRPT_STATIC_LINK_DEPS
		Threads::Threads
		CXX::Filesystem
		mrpt_libfyaml
	)
endif()


# pangommcairo

# Save vars (for some reason they are overwriten by another pkg_check_modules() call):
set(MY_CAIRO_VERSION  ${Cairo_VERSION})
set(MY_CAIRO_LIBRARIES  ${Cairo_LIBRARIES})
set(MY_CAIRO_INCLUDE_DIRS  ${Cairo_INCLUDE_DIRS})
set(MY_CAIRO_CFLAGS  ${Cairo_CFLAGS})

# "cairomm" svg lib
if(NOT WIN32)
	include(FindPkgConfig)
	pkg_check_modules(CAIROMM cairomm-svg-1.0)
endif()


if(CAIRO_FOUND AND CAIROMM_FOUND)
        message(STATUS "cairo found: ${MY_CAIRO_VERSION}")
        message(STATUS "cairomm found: ${CAIROMM_VERSION}")

        if($ENV{VERBOSE})
                message(STATUS "   CAIRO_LIBRARIES = ${MY_CAIRO_LIBRARIES}")
                message(STATUS "   CAIRO_INCLUDE_DIR = ${MY_CAIRO_INCLUDE_DIRS}")
                message(STATUS "   CAIRO_CFLAGS = ${MY_CAIRO_CFLAGS}")

                message(STATUS "   CAIROMM_LIBRARIES = ${CAIROMM_LIBRARIES}")
                message(STATUS "   CAIROMM_INCLUDE_DIRS = ${CAIROMM_INCLUDE_DIRS}")
                message(STATUS "   CAIROMM_CFLAGS = ${CAIROMM_CFLAGS}")
        endif()

        include_directories(${MY_CAIRO_INCLUDE_DIRS})
        include_directories(${CAIROMM_INCLUDE_DIRS})

	add_definitions(${MY_CAIRO_CFLAGS})
	add_definitions(${CAIROMM_CFLAGS})

        set(OPENBEAM_LINK_LIBS ${OPENBEAM_LINK_LIBS} ${MY_CAIRO_LIBRARIES} ${CAIROMM_LIBRARIES})

	set(OPENBEAM_HAS_CAIRO 1)
else()
        message(STATUS " Cairo and/or cairomm were *not* found: These packages are optional, but SVG rendering will be not available.")
endif()

add_definitions(-DOPENBEAM_HAS_CAIRO=${OPENBEAM_HAS_CAIRO})

# Qt5
# --------------------------------
if (NOT (CMAKE_SYSTEM_NAME STREQUAL "Emscripten"))
    # Instruct CMake to run moc automatically when needed.
    set(CMAKE_AUTOMOC ON)
    
    if(POLICY CMP0020)
        cmake_policy(SET CMP0020 NEW)
    endif()
    
    set(OPENBEAM_HAS_QT5Svg 0)
    
    # Find includes in corresponding build directories
    find_package(Qt5Svg QUIET)  # Find the QtWidgets library
    if (NOT Qt5Svg_FOUND)
        message(STATUS "*WARNING* Qt5Svg not found. Cannot build this target. \n  Add the installation prefix of Qt5Widgets to CMAKE_PREFIX_PATH \n  or set Qt5Svg_DIR to a directory containing one of the above files, typically: 'C:\\Qt\\5.X\\msvcXXXX_64\\lib\\cmake\\Qt5Svg'")
    else()
        include_directories(${Qt5Svg_INCLUDE_DIRS})
        add_definitions(${Qt5Svg_DEFINITIONS})
        set(OPENBEAM_HAS_QT5Svg 1)
    endif()
    add_definitions(-DOPENBEAM_HAS_QT5Svg=${OPENBEAM_HAS_QT5Svg})
endif()

# --------------------------
# Global compiler flags
# --------------------------
include_directories("${${PROJECT_NAME}_SOURCE_DIR}/libopenbeam/include/")
include_directories("${${PROJECT_NAME}_SOURCE_DIR}/libopenbeam/internationalization/")

# Force usage of UNICODE projects, which is not the default in MSVC:
if(MSVC)
	add_definitions(-DUNICODE -D_UNICODE -D_CRT_NONSTDC_NO_WARNINGS)
endif()

# Set optimized building:
if(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif()

# --------------------------
# The library
# --------------------------
add_subdirectory(libopenbeam)

# --------------------------
# Apps
# --------------------------
add_subdirectory(apps)

# --------------------------
# Examples
# --------------------------
if(BUILD_EXAMPLES)
        add_subdirectory(examples-code)
endif()

# --------------------------
# web apps:
# --------------------------
if (CMAKE_SYSTEM_NAME STREQUAL "Emscripten")
        add_subdirectory(web-apps)

        add_custom_target(web
                DEPENDS web_apps_all
                COMMAND ${CMAKE_COMMAND} 
                -DWEB_OUTPUT_DIR=${CMAKE_BINARY_DIR}/html
                -DOPENBEAM_SOURCE_DIR=${CMAKE_SOURCE_DIR}
                -DOPENBEAM_BINARY_DIR=${CMAKE_BINARY_DIR}
                -P ${CMAKE_SOURCE_DIR}/script_deploy_web.cmake
        )

        add_custom_target(web-and-serve
                DEPENDS web
                COMMAND python3 -m http.server 8080
                WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/html
        )
endif()

# -------------------------------------------------------------------------------------------
#  ${BIN_DIR}/OpenBeamConfig.cmake           -> For use *without* "make install"
# -------------------------------------------------------------------------------------------
#TODO: Update to use modern cmake exported targets

# Set CMAKE_INCLUDE_DIRS_CONFIGCMAKE to the list of include directories:
set(CMAKE_INCLUDE_DIRS_CONFIGCMAKE "\"${CMAKE_CURRENT_SOURCE_DIR}/include\" ")
set(CMAKE_LIB_DIRS_CONFIGCMAKE "${LIBRARY_OUTPUT_PATH}")

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/OpenBeamConfig.cmake.in" "${CMAKE_BINARY_DIR}/OpenBeamConfig.cmake" IMMEDIATE @ONLY)
