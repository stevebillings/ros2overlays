ament_add_gtest(vff_unittest
        vff_unittest.cpp
        ../include/vff_testing/vff.hpp
        ../src/vff.cpp)

ament_add_gtest(vector_unittest
        vector_unittest.cpp
        ../include/vff_testing/vector.hpp
        ../src/vector.cpp)

ament_target_dependencies(vff_unittest ${dependencies})
ament_target_dependencies(vector_unittest ${dependencies})
