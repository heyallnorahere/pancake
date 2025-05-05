cmake_minimum_required(VERSION 3.20)

add_executable(hardware_test "${CMAKE_CURRENT_SOURCE_DIR}/main.cpp")
target_link_libraries(hardware_test PRIVATE swerve-lib gtest)

set_target_properties(hardware_test PROPERTIES
    CXX_STANDARD 20)

install(TARGETS hardware_test
    DESTINATION lib/${PROJECT_NAME})