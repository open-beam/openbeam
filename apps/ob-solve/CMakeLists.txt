add_executable(ob-solve ob-solve_main.cpp)
target_include_directories(ob-solve PRIVATE ${PROJECT_SOURCE_DIR}/3rdparty)
target_link_libraries(ob-solve PRIVATE openbeam::openbeam)
target_compile_definitions(ob-solve PRIVATE OPENBEAM_HAS_CAIRO=${OPENBEAM_HAS_CAIRO})

install(TARGETS ob-solve RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
