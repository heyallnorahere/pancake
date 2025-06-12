cmake_minimum_required(VERSION 3.8)
project(pancake)

message(STATUS "Target OS: ${CMAKE_SYSTEM_NAME}")
message(STATUS "Target architecture: ${CMAKE_SYSTEM_PROCESSOR}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Input.msg"
  "msg/RobotTransform.msg"
  "msg/RobotVelocity.msg"
  "msg/OdometryState.msg"
  "msg/SwerveRequest.msg"
  "msg/ModuleState.msg"
  "msg/DrivetrainMeta.msg"
  "msg/PID.msg"
  "msg/SVA.msg"
)

rosidl_get_typesupport_target(CPP_TYPESUPPORT_TARGET
  ${PROJECT_NAME} rosidl_typesupport_cpp)

install(DIRECTORY
  launch
  resource
  DESTINATION share/${PROJECT_NAME}/
)

# no shared libs here please
set(SDL_STATIC ON CACHE BOOL "Build static SDL" FORCE)
set(SDL_SHARED OFF CACHE BOOL "Do not build shared SDL" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Just don't build ANY shared libraries" FORCE)

add_subdirectory("vendor")
add_subdirectory("src")
add_subdirectory("test")

ament_package()