find_package(GTest REQUIRED)
include(GoogleTest)

file(GLOB TEST_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*_unittest.cpp)

add_executable(openbeam_tests ${TEST_SOURCES})
target_link_libraries(openbeam_tests PRIVATE openbeam::openbeam GTest::gtest_main)
target_compile_definitions(openbeam_tests PRIVATE
  OPENBEAM_EXAMPLES_DIR="${PROJECT_SOURCE_DIR}/examples-structures"
  OPENBEAM_GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/golden"
)
gtest_discover_tests(openbeam_tests DISCOVERY_TIMEOUT 60)
