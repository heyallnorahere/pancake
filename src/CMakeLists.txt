cmake_minimum_required(VERSION 3.20)

if(${BUILD_CLIENT})
    add_subdirectory("client")
endif()

if(${CMAKE_SYSTEM_NAME} STREQUAL Linux)
    add_subdirectory("swerve")
endif()