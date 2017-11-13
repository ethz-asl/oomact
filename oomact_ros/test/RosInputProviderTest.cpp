#include "aslam/calibration/ros/RosInputProvider.h"

#include <memory>

#include <gtest/gtest.h>
#include <rosbag/bag.h>

#include <aslam/calibration/calibrator/CalibratorI.h>
#include <aslam/calibration/model/FrameGraphModel.h>
#include <aslam/calibration/model/PoseTrajectory.h>
#include <aslam/calibration/model/sensors/PoseSensor.h>
#include <aslam/calibration/tools/SmartPointerTools.h>

#include "bag_tools.h"

using namespace aslam;
using namespace aslam::calibration;
using namespace aslam::calibration::ros;


TEST(RosInputProviderSuite, testEasy) {
  // Create configuration // TODO D support compiler validation for configuration a bit
  auto vs = ValueStoreRef::fromString(
      "model{"
        "Gravity{used=false}, frames=body:world\n"
        "a{frame=body,targetFrame=world,rotation/used=false,translation{used=true,x=0,y=5,z=0},delay/used=false,topic=a}"
        "b{frame=body,targetFrame=world,rotation/used=false,translation{used=false},delay/used=false,topic=a}" // uses the same topic as a!
        "traj{frame=body,referenceFrame=world,McSensor=a,initWithPoseMeasurements=true,splines{knotsPerSecond=5,rotSplineOrder=4,rotFittingLambda=0.001,transSplineOrder=4,transFittingLambda=0.001}}"
      "}"
      "calibrator{"
        "verbose=true\n"
        "timeBaseSensor=a\n"
      "}"
    );

  // Construct objects
  FrameGraphModel m(vs.getChild("model"));
  PoseSensor psA(m, "a"), psB(m, "b");
  PoseTrajectory traj(m, "traj");

  // init model with all sensors :
  m.addModulesAndInit(psA, psB, traj); // TODO B allow easy construction mode (default) : automatic adding of all sensors!

  // Create local pseudo shared pointer to model and create a batchCalibrator for it
  auto spModel = to_local_shared_ptr(m);
  auto c = createBatchCalibrator(vs.getChild("calibrator"), spModel);

  //  Write test data for the pose sensor into the test.bag
  const std::string testBagPath = "test.bag";
  writeMotionCaptureSourceToBag(testBagPath, "a", test::MmcsStraightLine, 1.0);


  RosInputProvider(spModel).feedBag(testBagPath, *c); // Feed the test.bag

  EXPECT_NEAR(5.0, psA.getTranslationToParent()[1], 0.0001); // check the y position of the sensor a BEFORE calibration

  c->calibrate();

  EXPECT_NEAR(0.0, psA.getTranslationToParent()[1], 0.0001); // it should be zero now because it is receiving the same data as b, which has zero transformation and doesn't get calibrated.
}
