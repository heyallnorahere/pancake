cmake_minimum_required(VERSION 3.20)

file(GLOB_RECURSE ROBOT_SOURCE CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
  "${CMAKE_SOURCE_DIR}/include/pancake-swerve/robot/*.h")

link_libraries(${CPP_TYPESUPPORT_TARGET})

set(ROBOT_MAIN "${CMAKE_CURRENT_SOURCE_DIR}/main.cpp")
list(REMOVE_ITEM ROBOT_SOURCE ${ROBOT_MAIN})

add_library(robot-lib ${ROBOT_SOURCE})
ament_target_dependencies(robot-lib rclcpp std_msgs)

target_include_directories(robot-lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(robot ${ROBOT_MAIN})
target_link_libraries(robot PRIVATE robot-lib)

set_target_properties(robot-lib robot PROPERTIES
  CXX_STANDARD 20)

install(TARGETS robot
  DESTINATION lib/${PROJECT_NAME})