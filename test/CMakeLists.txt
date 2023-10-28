enable_testing()

add_executable(
  hello_test
  hello_test.cc
        ../src/pru1/sensor.cpp
)
target_link_libraries(
  hello_test
  GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(hello_test)

