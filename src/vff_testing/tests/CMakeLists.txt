ament_add_gtest(vff_unittest
        vff_unittest.cpp
        ../include/vff_testing/vff.hpp
        ../src/vff.cpp)

ament_target_dependencies(vff_unittest ${dependencies})
