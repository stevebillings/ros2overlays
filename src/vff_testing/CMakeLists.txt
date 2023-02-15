cmake_minimum_required(VERSION 3.8)
project(vff_testing)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(vff_test
        src/vff_main.cpp
        src/vff.cpp
        include/vff_testing/vff.hpp)

set(dependencies
  rclcpp
  sensor_msgs
  geometry_msgs
)

ament_target_dependencies(vff_test ${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  set(ament_cmake_copyright_FOUND TRUE)

  find_package(ament_cmake_gtest REQUIRED)
  #find_package(ament_cmake_gmock REQUIRED)
  add_subdirectory(tests)
endif()

install(TARGETS
        vff_test
        DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(${dependencies})
ament_package()
