cmake_minimum_required(VERSION 3.8)
project(pancake)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Input.msg"
)

rosidl_get_typesupport_target(CPP_TYPESUPPORT_TARGET
  ${PROJECT_NAME} rosidl_typesupport_cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# no shared libs here please
set(SDL_STATIC ON CACHE BOOL "Build static SDL" FORCE)
set(SDL_SHARED OFF CACHE BOOL "Do not build shared SDL" FORCE)

add_subdirectory("vendor/SDL")
add_subdirectory("src/client")

ament_package()
