cmake_minimum_required(VERSION 2.8.3)
project(oomact_toy_examples)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11 -Wall -Wextra)

cs_add_executable(example1
  src/example1.cpp
)

cs_add_executable(example2
  src/example2.cpp
)

cs_install()
cs_export()
