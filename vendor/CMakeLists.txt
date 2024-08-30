cmake_minimum_required(VERSION 3.20)

if(${BUILD_CLIENT})
    add_subdirectory("SDL")
endif()

if(${CMAKE_SYSTEM_NAME} STREQUAL Linux)
    add_subdirectory("librevfree")
    add_subdirectory("json")
endif()