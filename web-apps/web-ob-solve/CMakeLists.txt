project(web-ob-solve)

include_directories("${OpenBeam_SOURCE_DIR}/3rdparty/")

add_executable(${PROJECT_NAME} 
    main.cpp
    AppOpenBeam.h
    AppOpenBeam.cpp
    )

add_dependencies(web_apps_all ${PROJECT_NAME})

set_target_properties(${PROJECT_NAME}
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${OpenBeam_BINARY_DIR}/bin-js
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

target_compile_options(${PROJECT_NAME} PRIVATE
    -sUSE_SDL=2
)
target_link_options(${PROJECT_NAME} PRIVATE
    -sUSE_SDL=2
    -sUSE_GLFW=3
    -sFULL_ES2=1
    -sFULL_ES3=1
    -sMAX_WEBGL_VERSION=2
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
    "."
    )
