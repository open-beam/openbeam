project(openbeam)

file(GLOB SRCS_LIST "${openbeam_SOURCE_DIR}/src/*.cpp")
file(GLOB HRDS_LIST "${openbeam_SOURCE_DIR}/include/openbeam/*.h")
file(GLOB INTERNAL_HDRS_LIST "${openbeam_SOURCE_DIR}/src/*.h")

file(GLOB INTERN_HS_LIST "${openbeam_SOURCE_DIR}/internationalization/*.h")
file(GLOB INTERN_CPPS_LIST "${openbeam_SOURCE_DIR}/internationalization/*.cpp")

# Declare the target (a library)
add_library(${PROJECT_NAME}
	${SRCS_LIST}
	${INTERNAL_HDRS_LIST}
	${HRDS_LIST}
	# Internalization code:
	${INTERN_HS_LIST} ${INTERN_CPPS_LIST}
	)

target_link_libraries(${PROJECT_NAME}
	${OPENBEAM_LINK_LIBS}
	${Qt5Svg_LIBRARIES}
	mrpt::opengl
	${MRPT_STATIC_LINK_DEPS}
	)

if (CMAKE_SYSTEM_NAME STREQUAL "Emscripten")
	target_link_options(${PROJECT_NAME} PRIVATE
		-sNO_DISABLE_EXCEPTION_CATCHING
	)
	target_compile_options(${PROJECT_NAME} PRIVATE
		-sNO_DISABLE_EXCEPTION_CATCHING
	)

endif()