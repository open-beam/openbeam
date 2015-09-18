project(ob_solve)

INCLUDE_DIRECTORIES("${OpenBeam_SOURCE_DIR}/3rdparty/")

add_executable(ob-solve ob-solve_main.cpp)
add_dependencies(ob-solve openbeam)
target_link_libraries(ob-solve openbeam ${OPENBEAM_LINK_LIBS})
