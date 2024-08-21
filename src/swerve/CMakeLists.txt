cmake_minimum_required(VERSION 3.20)

file(GLOB_RECURSE SWERVE_SOURCE CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${CMAKE_SOURCE_DIR}/include/pancake-swerve/swerve/*.h")

add_executable(swerve ${SWERVE_SOURCE})
ament_target_dependencies(swerve rclcpp std_msgs)

target_include_directories(swerve PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

set_target_properties(swerve PROPERTIES
    CXX_STANDARD 20)

install(TARGETS swerve
  DESTINATION lib/${PROJECT_NAME})

target_link_libraries(swerve ${CPP_TYPESUPPORT_TARGET} librevfree)