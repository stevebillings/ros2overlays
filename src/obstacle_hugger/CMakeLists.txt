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

add_executable(obstacle_hugger src/obstacle_hugger.cpp src/LaserAnalyzer.cpp src/LaserAnalyzer.h src/LaserAnalysis.cpp src/LaserAnalysis.h src/Velocity.cpp src/Velocity.h src/VelocityCalculator.cpp src/VelocityCalculator.h src/FsmState.h src/FullState.cpp src/FullState.h src/LaserCharacteristics.cpp src/LaserCharacteristics.h src/NearestSighting.cpp src/NearestSighting.h)
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
