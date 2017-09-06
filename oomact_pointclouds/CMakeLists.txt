cmake_minimum_required(VERSION 2.8.3)
project(oomact_pointclouds)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(libpointmatcher REQUIRED)

include_directories("${libpointmatcher_INCLUDE_DIRS}")
message(STATUS "Using libpointmatcher version ${libpointmatcher_VERSION}")

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

cs_add_library(${PROJECT_NAME}
  src/clouds/CalibrationMatcher.cpp
  src/clouds/CloudBatch.cpp
  src/clouds/CloudsContainer.cpp
  src/clouds/PmFilter.cpp
  src/clouds/PointCloudsPlugin.cpp
  src/error-terms/ErrorTermCloudAssociation.cpp
  src/error-terms/ErrorTermPointToPlain.cpp
  src/laser-scanner/laser3d.cpp
  src/model/sensors/GroundPlanePseudoSensor.cpp
  src/model/sensors/Lidar.cpp
  src/model/sensors/Lidar2d.cpp
  src/model/sensors/Lidar3d.cpp
  src/model/sensors/PointCloudSensor.cpp
  src/model/sensors/StereoCamera.cpp
  src/model/sensors/Velodyne.cpp
  src/tools/PointCloudTools.cpp
)
target_link_libraries(${PROJECT_NAME} ${libpointmatcher_LIBRARIES} ${TBB_LIBRARIES})

catkin_add_gtest(${PROJECT_NAME}_test
  test/test_main.cpp
  #test/clouds/CloudMatchingTest.cpp
  test/error-terms/ErrorTermCloudAssociationTest.cpp
  WORKING_DIRECTORY  ${PROJECT_SOURCE_DIR}/test
)
target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})

cs_install()
cs_export()
