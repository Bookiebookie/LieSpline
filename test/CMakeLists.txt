cmake_minimum_required(VERSION 3.10)

add_executable(test_ceres_spline_helper_old src/test_ceres_spline_helper_old.cpp)
target_link_libraries(test_ceres_spline_helper_old gtest gtest_main Eigen3::Eigen)

enable_testing()

include(GoogleTest)

gtest_add_tests(TARGET test_ceres_spline_helper_old AUTO)
