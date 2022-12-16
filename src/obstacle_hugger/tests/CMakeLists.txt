#ament_add_gmock(laser_test laser_test.cpp)
ament_add_gtest(laser_test
        laser_test.cpp
        ../src/laser/laser_analyzer.h ../src/laser/laser_analyzer.cpp
        ../src/laser/laser_characteristics.h ../src/laser/laser_characteristics.cpp
        ../src/laser/laser_analysis.h ../src/laser/laser_analysis.cpp
        ../src/obstacle/nearest_sighting.h ../src/obstacle/nearest_sighting.cpp)
ament_add_gtest(velocity_calculator_test
        velocity_calculator_test.cpp
        ../src/laser/laser_characteristics.h ../src/laser/laser_characteristics.cpp
        ../src/laser/laser_analysis.h ../src/laser/laser_analysis.cpp
        ../src/obstacle/nearest_sighting.h ../src/obstacle/nearest_sighting.cpp
        ../src/velocity/velocity.h ../src/velocity/velocity.cpp
        ../src/velocity/velocity_calculator.h ../src/velocity/velocity_calculator.cpp)
ament_target_dependencies(laser_test ${dependencies})
ament_target_dependencies(velocity_calculator_test ${dependencies})