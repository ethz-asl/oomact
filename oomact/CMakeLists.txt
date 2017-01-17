cmake_minimum_required(VERSION 2.8.3)
project(oomact)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11 -Wall -Wextra)

cs_add_library(${PROJECT_NAME}
  src/error-terms/ErrorTermWheel.cpp
  src/error-terms/ErrorTermWheelsZ.cpp
  src/error-terms/ErrorTermLinearVelocity.cpp
  src/error-terms/ErrorTermAngularVelocity.cpp
  src/error-terms/ErrorTermAccelerometer.cpp
  src/error-terms/ErrorTermGyroscope.cpp
  src/error-terms/ErrorTermPose.cpp
  src/error-terms/ErrorTermPosition.cpp
  src/error-terms/ErrorTermTangency.cpp
  src/error-terms/ErrorTermGroup.cpp
  
  src/laser-scanner/laser3d.cpp
  
  src/algo/PredictionWriter.cpp
  src/algo/splinesToFile.cpp
  src/algo/OdometryPath.cpp
  src/model/Module.cpp
  src/model/ModuleList.cpp
  src/model/Model.cpp
  src/model/FrameGraphModel.cpp
  src/model/Sensor.cpp
  src/model/SetActivator.cpp
  src/model/Joint.cpp
  src/model/CalibrationVariable.cpp
  src/model/sensors/AbstractPoseSensor.cpp
  src/model/sensors/MotionCaptureSensor.cpp
  src/model/sensors/PoseSensor.cpp
  src/model/sensors/PositionSensor.cpp
  src/model/sensors/Imu.cpp
  src/model/sensors/WheelOdometry.cpp
  src/model/StateCarrier.cpp
  src/model/PoseTrajectory.cpp
  src/model/fragments/TrajectoryCarrier.cpp
  src/model/fragments/So3R3Trajectory.cpp
  src/model/fragments/PoseCv.cpp
  src/model/priors/CrossPoseCvPrior.cpp
  src/model/fragments/DelayCv.cpp
  src/plan/Printable.cpp
  src/plan/Plan.cpp
  src/plan/PlanFragment.cpp
  src/plan/Executor.cpp
  src/plan/Driver.cpp
  src/plan/Logger.cpp
  src/plan/CalibrationServer.cpp
  src/tools/Interval.cpp
  src/tools/tools.cpp
  src/tools/Parallelizer.cpp
  src/tools/ErrorTermStatistics.cpp
  src/tools/ErrorTermStatisticsWithProblemAndPredictor.cpp

  src/BatchCalibrator.cpp
  src/AbstractCalibrator.cpp
)
target_link_libraries(${PROJECT_NAME} ${TBB_LIBRARIES})

catkin_add_gtest(${PROJECT_NAME}_test
  test/test_main.cpp
  test/error-terms/ErrorTermAccelerometerTest.cpp
  test/error-terms/ErrorTermGyroscopeTest.cpp
  test/error-terms/ErrorTermWheelTest.cpp
  test/error-terms/ErrorTermPoseTest.cpp
  test/error-terms/ConditionalErrorTerm.cpp
  test/plan/PlanTest.cpp
  test/tools/TreeTest.cpp
  test/data/MeasurementsContainerTest.cpp
  test/acceptance/Tools.cpp
  test/acceptance/SimpleModel.cpp
  test/acceptance/SimpleModelTest.cpp
  
  WORKING_DIRECTORY  ${PROJECT_SOURCE_DIR}/test
)

target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})

cs_install()
cs_export()
