cmake_minimum_required(VERSION 3.20)

set(SWERVE_MAIN "${CMAKE_CURRENT_SOURCE_DIR}/main.cpp")
file(GLOB_RECURSE SWERVE_SOURCE CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
  "${CMAKE_SOURCE_DIR}/include/pancake-swerve/swerve/*.h")

list(REMOVE_ITEM SWERVE_SOURCE ${SWERVE_MAIN})
add_library(swerve-lib STATIC ${SWERVE_SOURCE})
target_link_libraries(swerve-lib ${CPP_TYPESUPPORT_TARGET} librevfree nlohmann_json)
ament_target_dependencies(swerve-lib rclcpp std_msgs)

target_include_directories(swerve-lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(swerve ${SWERVE_MAIN})
target_link_libraries(swerve PRIVATE swerve-lib)

set_target_properties(swerve swerve-lib PROPERTIES
  CXX_STANDARD 20)

install(TARGETS swerve
  DESTINATION lib/${PROJECT_NAME})