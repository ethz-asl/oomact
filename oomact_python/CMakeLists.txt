cmake_minimum_required(VERSION 2.8.3)
project(oomact_python)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# Set up the python exports.
SET(PY_PROJECT_NAME ${PROJECT_NAME})
SET(PY_PACKAGE_DIR python/oomact)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(
    test/tests.py
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/test
  )
endif()

catkin_python_setup()

cs_install()
cs_export()
