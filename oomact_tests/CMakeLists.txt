cmake_minimum_required(VERSION 2.8.3)
project(oomact_tests)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# enable warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  # deprecated-register warnings are cause by old Eigen versions
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-register")
endif()

# enable C++11 support
if(CMAKE_VERSION VERSION_LESS "3.1")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
  set(CMAKE_CXX_STANDARD 11)
endif()

catkin_add_gtest(oomact_tests
  test/test_main.cpp
  test/TestMicroCalibrationProblems.cpp
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

cs_install()
cs_export()
