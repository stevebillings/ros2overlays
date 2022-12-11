cmake_minimum_required(VERSION 3.8)
project(obstacle_hugger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(obstacle_hugger src/switch_fsm/obstacle_hugger.cpp src/laser/laser_analyzer.cpp src/laser/laser_analyzer.h src/laser/laser_analysis.cpp src/laser/laser_analysis.h src/velocity/velocity.cpp src/velocity/velocity.h src/velocity/velocity_calculator.cpp src/velocity/velocity_calculator.h src/switch_fsm/fsm_state.h src/switch_fsm/full_state.cpp src/switch_fsm/full_state.h src/laser/laser_characteristics.cpp src/laser/laser_characteristics.h src/obstacle/nearest_sighting.cpp src/obstacle/nearest_sighting.h)
ament_target_dependencies(obstacle_hugger rclcpp sensor_msgs)

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

install(TARGETS
  obstacle_hugger
  DESTINATION lib/${PROJECT_NAME})

ament_package()
