cmake_minimum_required(VERSION 3.20)

file(GLOB_RECURSE CLIENT_SOURCE CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${CMAKE_SOURCE_DIR}/include/pancake-swerve/client/*.h")

add_executable(client ${CLIENT_SOURCE})
ament_target_dependencies(client rclcpp std_msgs)

target_include_directories(client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

set_target_properties(client PROPERTIES
    CXX_STANDARD 14)

install(TARGETS client
  DESTINATION lib/${PROJECT_NAME})

target_link_libraries(client ${CPP_TYPESUPPORT_TARGET})