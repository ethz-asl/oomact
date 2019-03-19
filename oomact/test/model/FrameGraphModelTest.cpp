#include <aslam/calibration/model/FrameGraphModel.h>

#include <gtest/gtest.h>
#include <sm/eigen/gtest.hpp>

#include <aslam/calibration/model/PoseTrajectory.h>
#include <aslam/calibration/test/Tools.h>
#include <sm/source_file_pos.hpp>
#include <sm/value_store/ValueStore.hpp>

using sm::value_store::ValueStoreRef;

using namespace aslam::calibration;
using namespace aslam::calibration::test;

class MockFrameLink : public Module, public PoseCv {
 public:
  MockFrameLink(Model & model, std::string name, sm::value_store::ValueStoreRef config,
                RelativeKinematicExpression relKin) :
    Module(model, name, config),
    PoseCv(this), relKin(relKin) {}

  RelativeKinematicExpression calcRelativeKinematics() const override {
    return relKin;
  }


 private:
  const RelativeKinematicExpression relKin;
};

TEST(FrameGraphModel, getTransformation) {
  auto config = ValueStoreRef::fromString(
      "Gravity{used=false}"
      "frames=body:world,"
      "body{referenceFrame=world, rotation/used=false,translation/used=false,delay/used=false}"
      "s1{referenceFrame=body, rotation/used=false,translation/used=false,delay/used=false}"
      );

  Eigen::MatrixXd R_w_b(3, 3);
  R_w_b << 0, -1, 0,  1, 0, 0,  0, 0, 1;
  Eigen::Vector3d t_w_b;
  t_w_b << 1, 2, 3;

  FrameGraphModel m(config);
  Sensor s1(m, "s1", config);
  MockFrameLink link(m, "body", config,
                     {
                         aslam::backend::RotationExpression(R_w_b),
                         aslam::backend::EuclideanExpression(t_w_b),
                     });

  m.addModulesAndInit(link, s1);

  const Frame & bodyFrame = m.getFrame("body");
  const Frame & worldFrame = m.getFrame("world");

  auto mAt = m.getAtTime(0.0, 2, {});

  auto T_w_b = mAt.getTransformationToFrom(worldFrame, bodyFrame);
  sm::eigen::assertNear(T_w_b.toRotationExpression().toRotationMatrix(), R_w_b, 1e-9,
                        SM_SOURCE_FILE_POS);
  sm::eigen::assertNear(T_w_b.toEuclideanExpression().evaluate(), t_w_b, 1e-9,
                        SM_SOURCE_FILE_POS);

  auto T_b_w = mAt.getTransformationToFrom(bodyFrame, worldFrame);
  sm::eigen::assertNear(T_b_w.toRotationExpression().toRotationMatrix(), R_w_b.transpose(), 1e-9,
                        SM_SOURCE_FILE_POS);
  sm::eigen::assertNear(T_b_w.toEuclideanExpression().evaluate(), -R_w_b.transpose()*t_w_b, 1e-9,
                        SM_SOURCE_FILE_POS);


  const Frame & s1Frame = m.getFrame("s1");
  auto T_w_s = mAt.getTransformationToFrom(worldFrame, s1Frame);
  sm::eigen::assertNear(T_w_s.toRotationExpression().toRotationMatrix(), R_w_b, 1e-9,
                        SM_SOURCE_FILE_POS);
  sm::eigen::assertNear(T_w_s.toEuclideanExpression().evaluate(), t_w_b, 1e-9,
                        SM_SOURCE_FILE_POS);

  auto T_s_w = mAt.getTransformationToFrom(s1Frame, worldFrame);
  sm::eigen::assertNear(T_s_w.toRotationExpression().toRotationMatrix(), R_w_b.transpose(), 1e-9,
                        SM_SOURCE_FILE_POS);
  sm::eigen::assertNear(T_s_w.toEuclideanExpression().evaluate(), -R_w_b.transpose()*t_w_b, 1e-9,
                        SM_SOURCE_FILE_POS);

}
