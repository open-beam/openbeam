PROJECT(openbeam)

file(GLOB SRCS_LIST "${OpenBeam_SOURCE_DIR}/src/*.cpp")
file(GLOB HRDS_LIST "${OpenBeam_SOURCE_DIR}/include/openbeam/*.h")
file(GLOB INTERNAL_HDRS_LIST "${OpenBeam_SOURCE_DIR}/src/*.h")

file(GLOB INTERN_HS_LIST "${OpenBeam_SOURCE_DIR}/internationalization/*.h")
file(GLOB INTERN_CPPS_LIST "${OpenBeam_SOURCE_DIR}/internationalization/*.cpp")

# Declare the target (a library)
ADD_LIBRARY(openbeam
	${SRCS_LIST}
	${INTERNAL_HDRS_LIST}
	${HRDS_LIST}

	# Internalization code:
	${INTERN_HS_LIST} ${INTERN_CPPS_LIST}
	)
	
target_link_libraries(openbeam ${OPENBEAM_LINK_LIBS})

