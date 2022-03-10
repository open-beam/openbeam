# cli apps:
add_subdirectory(ob-solve)

# gui apps:
add_subdirectory(OpenBeamStudio)

# web apps:
if (CMAKE_SYSTEM_NAME STREQUAL "Emscripten")
add_subdirectory(web-ob-solve)
endif()