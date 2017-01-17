cmake_minimum_required(VERSION 2.8.3)
project(oomact_tests)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11 -Wall -Wextra)

catkin_add_gtest(oomact_tests
  test/test_main.cpp
  test/TestMicroCalibrationProblems.cpp
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

cs_install()
cs_export()
