#cmake_minimum_required(VERSION 2.8.11)
project(OpenBeamStudio)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Qt5Widgets QUIET)  # Find the QtWidgets library
if (NOT Qt5Widgets_FOUND)
	message(STATUS "*WARNING* Qt5 not found. Cannot build this target. \n  Add the installation prefix of Qt5Widgets to CMAKE_PREFIX_PATH \n  or set Qt5Widgets_DIR to a directory containing one of the above files, typically: 'C:\\Qt\\5.X\\msvcXXXX_64\\lib\\cmake\\Qt5Widgets'")
else()
	add_definitions(${Qt5Widgets_DEFINITIONS})
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")

	include_directories("${OpenBeam_SOURCE_DIR}/3rdparty/")

	set(FORMS
		mainwindow.ui
	)
	qt5_wrap_ui(FORMS_HEADERS ${FORMS})

	set(RESOURCES
		openbeamstudio.qrc
	)
	qt5_add_resources(RESOURCES_RCC ${RESOURCES})

	# Tell CMake to create the helloworld executable
	add_executable(OpenBeamStudio WIN32 
		main.cpp
		mainwindow.cpp mainwindow.h
		highlighter.cpp highlighter.h
		${FORMS_HEADERS}
		${RESOURCES_RCC}
		)

	# Use the Widgets module from Qt 5.
	target_link_libraries(OpenBeamStudio
		${Qt5Widgets_LIBRARIES}
		${Qt5Svg_LIBRARIES}
		openbeam)

	# for builds inside the main SDK
	if(TARGET openbeam)
		add_dependencies(OpenBeamStudio openbeam)
	endif()
endif() # if qt not found
