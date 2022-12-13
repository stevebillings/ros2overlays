#ament_add_gtest(laser_test laser_test.cpp)
ament_add_gmock(laser_test laser_test.cpp ../src/laser/laser_analyzer.h ../src/laser/laser_analyzer.cpp
        ../src/laser/laser_characteristics.h ../src/laser/laser_characteristics.cpp
        ../src/logger/simple_logger.h ../src/logger/simple_logger.cpp
        ../src/obstacle/nearest_sighting.h ../src/obstacle/nearest_sighting.cpp)
ament_target_dependencies(laser_test ${dependencies})
