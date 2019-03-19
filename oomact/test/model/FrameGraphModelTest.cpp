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

TEST(FrameGraphModel, getTransformationAndGetDerivatives) {
  auto config = ValueStoreRef::fromString(
      "Gravity{used=false}"
      "frames=body:world,"
      "body{referenceFrame=world, rotation/used=false,translation/used=false,delay/used=false}"
      "s1{referenceFrame=body, rotation/used=false,translation/used=false,delay/used=false}"
    );
  FrameGraphModel m(config);
  Sensor s1(m, "s1", config);

  Eigen::MatrixXd R_w_b(3, 3);
  R_w_b << 0, -1, 0,  1, 0, 0,  0, 0, 1;
  Eigen::Vector3d t_w_b;
  t_w_b << 1, 2, 3;

  Eigen::Vector3d omega_w_wb;
  omega_w_wb << 1, 2, 3;

  MockFrameLink link(m, "body", config,
                     {
                         aslam::backend::RotationExpression(R_w_b),
                         aslam::backend::EuclideanExpression(t_w_b),
                         aslam::backend::EuclideanExpression(omega_w_wb)
                     });
  //TODO increase test coverage for all derivatives and more complex graphs

  m.addModulesAndInit(link, s1);

  const Frame & bodyFrame = m.getFrame("body");
  const Frame & worldFrame = m.getFrame("world");

  auto mAt = m.getAtTime(0.0, 2, {});

  auto T_w_b = mAt.getTransformationToFrom(worldFrame, bodyFrame);
  sm::eigen::assertNear(T_w_b.toRotationExpression().toRotationMatrix(), R_w_b, 1e-9,
                        SM_SOURCE_FILE_POS);
  sm::eigen::assertNear(T_w_b.toEuclideanExpression().evaluate(), t_w_b, 1e-9,
                        SM_SOURCE_FILE_POS);
  auto omega_w_wb_exp = mAt.getAngularVelocity(bodyFrame, worldFrame);
  sm::eigen::assertNear(omega_w_wb_exp.evaluate(), omega_w_wb, 1e-9,
                        SM_SOURCE_FILE_POS);

  auto T_b_w = mAt.getTransformationToFrom(bodyFrame, worldFrame);
  sm::eigen::assertNear(T_b_w.toRotationExpression().toRotationMatrix(), R_w_b.transpose(), 1e-9,
                        SM_SOURCE_FILE_POS);
  sm::eigen::assertNear(T_b_w.toEuclideanExpression().evaluate(), -R_w_b.transpose()*t_w_b, 1e-9,
                        SM_SOURCE_FILE_POS);
  auto omega_b_bb_exp = mAt.getAngularVelocity(bodyFrame, bodyFrame);
  sm::eigen::assertNear(omega_b_bb_exp.evaluate(), Eigen::Vector3d::Zero(), 1e-9,
                        SM_SOURCE_FILE_POS);


  const Frame & s1Frame = m.getFrame("s1");
  auto T_w_s = mAt.getTransformationToFrom(worldFrame, s1Frame);
  sm::eigen::assertNear(T_w_s.toRotationExpression().toRotationMatrix(), R_w_b, 1e-9,
                        SM_SOURCE_FILE_POS);
  sm::eigen::assertNear(T_w_s.toEuclideanExpression().evaluate(), t_w_b, 1e-9,
                        SM_SOURCE_FILE_POS);
  auto omega_w_ws_exp = mAt.getAngularVelocity(s1Frame, worldFrame);
  sm::eigen::assertNear(omega_w_ws_exp.evaluate(), omega_w_wb, 1e-9,
                        SM_SOURCE_FILE_POS);

  auto T_s_w = mAt.getTransformationToFrom(s1Frame, worldFrame);
  sm::eigen::assertNear(T_s_w.toRotationExpression().toRotationMatrix(), R_w_b.transpose(), 1e-9,
                        SM_SOURCE_FILE_POS);
  sm::eigen::assertNear(T_s_w.toEuclideanExpression().evaluate(), -R_w_b.transpose()*t_w_b, 1e-9,
                        SM_SOURCE_FILE_POS);
  auto omega_b_bs_exp = mAt.getAngularVelocity(s1Frame, bodyFrame);
  sm::eigen::assertNear(omega_b_bb_exp.evaluate(), Eigen::Vector3d::Zero(), 1e-9,
                        SM_SOURCE_FILE_POS);

}
