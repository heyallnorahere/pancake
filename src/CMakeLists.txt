cmake_minimum_required(VERSION 3.20)

if(${BUILD_CLIENT})
    add_subdirectory("client")
endif()

add_subdirectory("robot")
add_subdirectory("swerve")