cmake_minimum_required(VERSION 3.20)

file(GLOB_RECURSE ROBOT_SOURCE CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${CMAKE_SOURCE_DIR}/include/pancake-swerve/robot/*.h")

add_executable(robot ${ROBOT_SOURCE})
ament_target_dependencies(robot rclcpp std_msgs)

target_include_directories(robot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

set_target_properties(robot PROPERTIES
    CXX_STANDARD 20)

install(TARGETS robot
  DESTINATION lib/${PROJECT_NAME})

target_link_libraries(robot ${CPP_TYPESUPPORT_TARGET})