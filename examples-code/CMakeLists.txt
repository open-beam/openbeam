project(examples)


# ------  1 ---------
add_executable(example1 example1.cpp)
add_dependencies(example1 openbeam)
target_link_libraries(example1 openbeam ${MRPT_LIBS})

