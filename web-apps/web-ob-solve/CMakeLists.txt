project(web-ob-solve)

include_directories("${OpenBeam_SOURCE_DIR}/3rdparty/")

add_executable(${PROJECT_NAME} 
    main.cpp
    AppOpenBeam.h
    AppOpenBeam.cpp
    )

target_link_libraries(${PROJECT_NAME}
    openbeam
    ${OPENBEAM_LINK_LIBS}
    )

set_target_properties(${PROJECT_NAME}
    PROPERTIES LINK_FLAGS 
    "-s DEMANGLE_SUPPORT=1 --bind"
)
# --preload-file ${CMAKE_SOURCE_DIR}/examples-structures

target_include_directories(${PROJECT_NAME}
    PRIVATE
    "."
    )
