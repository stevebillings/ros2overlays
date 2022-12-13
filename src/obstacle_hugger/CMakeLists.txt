cmake_minimum_required(VERSION 3.8)
project(obstacle_hugger)

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

add_executable(obstacle_hugger src/switch_fsm/obstacle_hugger.cpp src/laser/laser_analyzer.cpp src/laser/laser_analyzer.h src/laser/laser_analysis.cpp src/laser/laser_analysis.h src/velocity/velocity.cpp src/velocity/velocity.h src/velocity/velocity_calculator.cpp src/velocity/velocity_calculator.h src/switch_fsm/fsm_state.h src/switch_fsm/full_state.cpp src/switch_fsm/full_state.h src/laser/laser_characteristics.cpp src/laser/laser_characteristics.h src/obstacle/nearest_sighting.cpp src/obstacle/nearest_sighting.h src/logger/simple_logger.cpp src/logger/simple_logger.h)

set(dependencies
  rclcpp
  sensor_msgs
  geometry_msgs
)

ament_target_dependencies(obstacle_hugger ${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  set(ament_cmake_copyright_FOUND TRUE)

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(tests)
endif()

install(TARGETS
  obstacle_hugger
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(${dependencies})
ament_package()
