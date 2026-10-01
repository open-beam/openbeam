file(GLOB LIB_SOURCES CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/internationalization/*.cpp
)
file(GLOB LIB_PUBLIC_HEADERS CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/include/openbeam/*.h
)

add_library(openbeam ${LIB_SOURCES} ${LIB_PUBLIC_HEADERS})
add_library(openbeam::openbeam ALIAS openbeam)

set_target_properties(openbeam PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
)

target_include_directories(openbeam
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/internationalization>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(openbeam
  PUBLIC
    Eigen3::Eigen
    mrpt::mrpt_containers
    mrpt::mrpt_core
    mrpt::mrpt_math
    mrpt::mrpt_system
    mrpt::mrpt_viz
  PRIVATE
    mrpt::mrpt_expr
)

target_compile_definitions(openbeam PRIVATE OPENBEAM_HAS_CAIRO=${OPENBEAM_HAS_CAIRO})
if(OPENBEAM_HAS_CAIRO)
  target_link_libraries(openbeam PRIVATE PkgConfig::CAIROMM)
endif()

include(GNUInstallDirs)
install(
  TARGETS openbeam
  EXPORT openbeam-targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/openbeam DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
