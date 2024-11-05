cmake_minimum_required(VERSION 3.20)

add_subdirectory("robot")
add_subdirectory("swerve")
set(PANCAKE_TARGETS swerve swerve-lib robot robot-lib)

if(${BUILD_CLIENT})
    add_subdirectory("client")
    list(APPEND PANCAKE_TARGETS client)
endif()

foreach(TARGET ${PANCAKE_TARGETS})
    target_precompile_headers(${TARGET} PRIVATE "${CMAKE_SOURCE_DIR}/include/pancakepch.h")
endforeach()