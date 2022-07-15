message(STATUS "===================")
message(STATUS " Web deploy script ")
message(STATUS "===================")

message(STATUS "OPENBEAM_SOURCE_DIR : ${OPENBEAM_SOURCE_DIR}")
message(STATUS "WEB_OUTPUT_DIR      : ${WEB_OUTPUT_DIR}")

# Create empty target dir:
file(REMOVE_RECURSE "${WEB_OUTPUT_DIR}/*")
file(MAKE_DIRECTORY ${WEB_OUTPUT_DIR})

# Copy static files:
file(INSTALL
    ${OPENBEAM_SOURCE_DIR}/web-frontend/
    DESTINATION ${WEB_OUTPUT_DIR}
)
file(INSTALL
    ${OPENBEAM_SOURCE_DIR}/examples-structures
    DESTINATION ${WEB_OUTPUT_DIR}
)

# Copy emscripten-built JS files:
file(INSTALL
    ${OPENBEAM_BINARY_DIR}/bin-js/
    DESTINATION ${WEB_OUTPUT_DIR}/js/
)
