cmake_minimum_required(VERSION 3.20)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  file(GLOB_RECURSE TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

  foreach(SOURCE ${TEST_SOURCES})
    cmake_path(GET SOURCE FILENAME SOURCE_FILENAME)
    cmake_path(GET SOURCE_FILENAME STEM TEST_NAME)

    ament_add_gtest(test_${TEST_NAME} ${SOURCE})
    ament_target_dependencies(test_${TEST_NAME} rclcpp std_msgs)

    target_link_libraries(test_${TEST_NAME} swerve-lib robot-lib ${CPP_TYPESUPPORT_TARGET})
    set_target_properties(test_${TEST_NAME} PROPERTIES
      CXX_STANDARD 20)
  endforeach()
endif()