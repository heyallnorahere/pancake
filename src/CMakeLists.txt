cmake_minimum_required(VERSION 3.20)

add_subdirectory("robot")
add_subdirectory("swerve")
add_subdirectory("client")
set(PANCAKE_TARGETS swerve swerve-lib robot robot-lib client)

foreach(TARGET ${PANCAKE_TARGETS})
    target_precompile_headers(${TARGET} PRIVATE "${CMAKE_SOURCE_DIR}/include/pancakepch.h")
endforeach()