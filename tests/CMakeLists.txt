enable_testing()

add_executable(test_quat test_quat.cpp)
target_link_libraries(test_quat PRIVATE embdC_quat GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(test_quat)
