enable_testing()

add_executable(test_quat test_quat.cpp)
target_link_libraries(test_quat PRIVATE embdC_quat)

add_executable(test_quat_fixed test_quat_fixed.cpp)
target_link_libraries(test_quat_fixed PRIVATE embdC_quat)

# If using actual GTest, we would link GTest::gtest_main
# target_link_libraries(test_quat PRIVATE embdC_quat GTest::gtest_main)

